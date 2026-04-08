/**
 * CanopenMotorProtocol 实现
 */
#include "canopen_motor_protocol.h"
#include <cstring>
#include <cmath>
#include <QDebug>

namespace dar {

CanopenMotorProtocol::CanopenMotorProtocol(QObject *parent)
    : QObject(parent)
{
}

// ==================== Payload 构建 ====================

QByteArray CanopenMotorProtocol::buildPayload(uint32_t canId, const uint8_t *data, uint8_t dlc)
{
    // 格式: [canId(4 bytes big-endian)] [data(8 bytes)] [dlc(1 byte)] = 13 bytes
    QByteArray payload(13, 0);
    auto *p = reinterpret_cast<uint8_t *>(payload.data());

    p[0] = (canId >> 24) & 0xFF;
    p[1] = (canId >> 16) & 0xFF;
    p[2] = (canId >> 8)  & 0xFF;
    p[3] = canId & 0xFF;

    std::memcpy(&p[4], data, dlc);
    if (dlc < 8) {
        std::memset(&p[4 + dlc], 0, 8 - dlc);
    }
    p[12] = dlc;

    return payload;
}

// ==================== 响应匹配器 ====================

uint32_t CanopenMotorProtocol::getResponseId(uint32_t nodeId)
{
    // 发送: nodeId = 0x601 (RSDO)
    // 接收: responseId = 0x581 (TSDO)
    // 规则: responseId = nodeId - 0x80
    if (nodeId >= 0x601 && nodeId <= 0x67F) {
        return nodeId - 0x80;
    }
    return nodeId;  // fallback
}

std::function<bool(const CanFrame&)>
CanopenMotorProtocol::makeSDOMatcher(uint32_t responseId, uint16_t index, uint8_t subindex, bool isRead)
{
    return [responseId, index, subindex, isRead](const CanFrame &frame) -> bool {
        if (frame.can_id != responseId) return false;
        if (frame.dlc < 4) return false;

        // 检查 index 和 subindex
        uint16_t rxIndex = static_cast<uint16_t>(frame.data[1]) |
                           (static_cast<uint16_t>(frame.data[2]) << 8);
        uint8_t rxSub = frame.data[3];

        if (rxIndex != index || rxSub != subindex) return false;

        // 检查命令字节
        if (frame.data[0] == 0x80) return true;  // 错误响应也要匹配到

        if (isRead) {
            // 读响应: 0x4F(1B), 0x4B(2B), 0x43(4B), 0x42(expedited)
            return (frame.data[0] == 0x4F || frame.data[0] == 0x4B ||
                    frame.data[0] == 0x43 || frame.data[0] == 0x42);
        } else {
            // 写确认: 0x60
            return frame.data[0] == 0x60;
        }
    };
}

// ==================== SDO 请求生成 ====================

CommRequest CanopenMotorProtocol::makeWriteSDO(
    const QString &busId, uint32_t nodeId,
    uint16_t index, uint8_t subindex,
    const uint8_t *data, size_t len,
    Priority priority, bool waitResponse)
{
    CommRequest req;
    req.requestId = nextRequestId_++;
    req.busId = busId;
    req.deviceId = nodeId;
    req.protocolType = ProtocolType::CanopenMotor;
    req.commandType = QStringLiteral("writeSDO_0x%1:%2")
                          .arg(index, 4, 16, QChar('0'))
                          .arg(subindex);
    req.priority = priority;
    req.timeoutMs = 500;
    req.maxRetries = 1;

    // 构造 SDO 写帧
    uint8_t frame[8] = {};
    if (len == 1) {
        frame[0] = 0x2F;
    } else if (len == 2) {
        frame[0] = 0x2B;
    } else if (len == 4) {
        frame[0] = 0x23;
    } else {
        return req;  // 不支持
    }

    frame[1] = index & 0xFF;
    frame[2] = (index >> 8) & 0xFF;
    frame[3] = subindex;
    std::memcpy(&frame[4], data, len);

    req.payload = buildPayload(nodeId, frame, 8);

    if (waitResponse) {
        req.responseMatcher = makeSDOMatcher(getResponseId(nodeId), index, subindex, false);
    }

    return req;
}

CommRequest CanopenMotorProtocol::makeReadSDO(
    const QString &busId, uint32_t nodeId,
    uint16_t index, uint8_t subindex,
    size_t expectedLen, Priority priority)
{
    CommRequest req;
    req.requestId = nextRequestId_++;
    req.busId = busId;
    req.deviceId = nodeId;
    req.protocolType = ProtocolType::CanopenMotor;
    req.commandType = QStringLiteral("readSDO_0x%1:%2")
                          .arg(index, 4, 16, QChar('0'))
                          .arg(subindex);
    req.priority = priority;
    req.timeoutMs = 200;
    req.maxRetries = 0;
    req.expectedResponseLen = expectedLen;

    uint8_t frame[8] = {
        0x40,
        static_cast<uint8_t>(index & 0xFF),
        static_cast<uint8_t>((index >> 8) & 0xFF),
        subindex,
        0x00, 0x00, 0x00, 0x00
    };

    req.payload = buildPayload(nodeId, frame, 8);
    req.responseMatcher = makeSDOMatcher(getResponseId(nodeId), index, subindex, true);

    return req;
}

CommRequest CanopenMotorProtocol::makeNMT(
    const QString &busId, uint8_t command, uint8_t nodeAddr)
{
    CommRequest req;
    req.requestId = nextRequestId_++;
    req.busId = busId;
    req.deviceId = 0;
    req.protocolType = ProtocolType::CanopenMotor;
    req.commandType = QStringLiteral("NMT_0x%1").arg(command, 2, 16, QChar('0'));
    req.priority = Priority::Control;
    req.timeoutMs = 100;

    uint8_t data[2] = {command, nodeAddr};
    req.payload = buildPayload(0x000, data, 2);
    // NMT 无响应
    req.responseMatcher = nullptr;

    return req;
}

// ==================== 电机控制请求 ====================

CommRequest CanopenMotorProtocol::makeEnableRequest(
    const QString &busId, uint32_t nodeId, OperationMode mode)
{
    // 使能是多步骤: 设置模式 -> shutdown -> switchOn -> enableOp
    // 首先发送设置模式指令
    uint8_t modeVal = static_cast<uint8_t>(mode);
    return makeWriteSDO(busId, nodeId, OD_OPERATION_MODE, 0x00, &modeVal, 1, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeDisableRequest(
    const QString &busId, uint32_t nodeId)
{
    uint8_t data[2] = {0x06, 0x00};  // SHUTDOWN
    return makeWriteSDO(busId, nodeId, OD_CONTROL_WORD, 0x00, data, 2, Priority::Critical, false);
}

CommRequest CanopenMotorProtocol::makeFaultResetRequest(
    const QString &busId, uint32_t nodeId)
{
    uint8_t data[2] = {0x80, 0x00};  // FAULT_RESET
    return makeWriteSDO(busId, nodeId, OD_CONTROL_WORD, 0x00, data, 2, Priority::Critical, false);
}

CommRequest CanopenMotorProtocol::makeQuickStopRequest(
    const QString &busId, uint32_t nodeId)
{
    uint8_t data[2] = {0x02, 0x00};  // QUICK_STOP
    return makeWriteSDO(busId, nodeId, OD_CONTROL_WORD, 0x00, data, 2, Priority::Critical, false);
}

CommRequest CanopenMotorProtocol::makeSetVelocityRequest(
    const QString &busId, uint32_t nodeId, int32_t rpm)
{
    // RPM -> encoder pulses/sec
    int32_t encoderVal = static_cast<int32_t>(
        (static_cast<int64_t>(rpm) * ENCODER_RESOLUTION) / 60);

    uint8_t data[4] = {
        static_cast<uint8_t>((encoderVal >> 0)  & 0xFF),
        static_cast<uint8_t>((encoderVal >> 8)  & 0xFF),
        static_cast<uint8_t>((encoderVal >> 16) & 0xFF),
        static_cast<uint8_t>((encoderVal >> 24) & 0xFF)
    };

    return makeWriteSDO(busId, nodeId, OD_TARGET_VELOCITY, 0x00, data, 4, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeSetPositionRequest(
    const QString &busId, uint32_t nodeId, double degrees, bool absolute)
{
    Q_UNUSED(absolute);
    int32_t pulses = static_cast<int32_t>(degrees * PULSES_PER_DEGREE);

    uint8_t data[4] = {
        static_cast<uint8_t>((pulses >> 0)  & 0xFF),
        static_cast<uint8_t>((pulses >> 8)  & 0xFF),
        static_cast<uint8_t>((pulses >> 16) & 0xFF),
        static_cast<uint8_t>((pulses >> 24) & 0xFF)
    };

    return makeWriteSDO(busId, nodeId, OD_TARGET_POSITION, 0x00, data, 4, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeStartPositionMoveRequest(
    const QString &busId, uint32_t nodeId, bool relative)
{
    uint8_t cw = relative ? 0x7F : 0x3F;
    uint8_t data[2] = {cw, 0x00};
    return makeWriteSDO(busId, nodeId, OD_CONTROL_WORD, 0x00, data, 2, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeSetProfileVelocityRequest(
    const QString &busId, uint32_t nodeId, uint32_t rpm)
{
    uint32_t pps = static_cast<uint32_t>((static_cast<uint64_t>(rpm) * ENCODER_RESOLUTION) / 60);
    uint8_t data[4] = {
        static_cast<uint8_t>((pps >> 0)  & 0xFF),
        static_cast<uint8_t>((pps >> 8)  & 0xFF),
        static_cast<uint8_t>((pps >> 16) & 0xFF),
        static_cast<uint8_t>((pps >> 24) & 0xFF)
    };
    return makeWriteSDO(busId, nodeId, OD_PROFILE_VELOCITY, 0x00, data, 4, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeSetProfileAccelerationRequest(
    const QString &busId, uint32_t nodeId, uint32_t acc)
{
    uint32_t pps2 = static_cast<uint32_t>((static_cast<uint64_t>(acc) * ENCODER_RESOLUTION) / 60);
    uint8_t data[4] = {
        static_cast<uint8_t>((pps2 >> 0)  & 0xFF),
        static_cast<uint8_t>((pps2 >> 8)  & 0xFF),
        static_cast<uint8_t>((pps2 >> 16) & 0xFF),
        static_cast<uint8_t>((pps2 >> 24) & 0xFF)
    };
    return makeWriteSDO(busId, nodeId, OD_PROFILE_ACCEL, 0x00, data, 4, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeSetProfileDecelerationRequest(
    const QString &busId, uint32_t nodeId, uint32_t dec)
{
    uint32_t pps2 = static_cast<uint32_t>((static_cast<uint64_t>(dec) * ENCODER_RESOLUTION) / 60);
    uint8_t data[4] = {
        static_cast<uint8_t>((pps2 >> 0)  & 0xFF),
        static_cast<uint8_t>((pps2 >> 8)  & 0xFF),
        static_cast<uint8_t>((pps2 >> 16) & 0xFF),
        static_cast<uint8_t>((pps2 >> 24) & 0xFF)
    };
    return makeWriteSDO(busId, nodeId, OD_PROFILE_DECEL, 0x00, data, 4, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeSetMaxTorqueRequest(
    const QString &busId, uint32_t nodeId, uint16_t permille)
{
    uint8_t data[2] = {
        static_cast<uint8_t>(permille & 0xFF),
        static_cast<uint8_t>((permille >> 8) & 0xFF)
    };
    return makeWriteSDO(busId, nodeId, OD_MAX_TORQUE, 0x00, data, 2, Priority::Control, false);
}

// ==================== 状态读取 ====================

CommRequest CanopenMotorProtocol::makeReadTorqueRequest(
    const QString &busId, uint32_t nodeId)
{
    return makeReadSDO(busId, nodeId, OD_ACTUAL_TORQUE, 0x00, 2, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::makeReadStatusWordRequest(
    const QString &busId, uint32_t nodeId)
{
    return makeReadSDO(busId, nodeId, OD_STATUS_WORD, 0x00, 2, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::makeReadPhaseCurrentRequest(
    const QString &busId, uint32_t nodeId)
{
    return makeReadSDO(busId, nodeId, OD_PHASE_CURRENT, OD_PHASE_CURRENT_SUB, 2, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::makeReadPositionDeviationRequest(
    const QString &busId, uint32_t nodeId)
{
    return makeReadSDO(busId, nodeId, OD_POSITION_DEVIATION, 0x00, 4, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::makeNmtStartRequest(const QString &busId)
{
    return makeNMT(busId, 0x01, 0x00);
}

CommRequest CanopenMotorProtocol::makeNmtPreOpRequest(const QString &busId)
{
    return makeNMT(busId, 0x80, 0x00);
}

CommRequest CanopenMotorProtocol::makeReleaseBrakeRequest(
    const QString &busId, uint32_t nodeId)
{
    uint8_t data[2] = {0x02, 0x00};
    return makeWriteSDO(busId, nodeId, 0x200D, 0x1B, data, 2, Priority::Control, false);
}

CommRequest CanopenMotorProtocol::makeLockBrakeRequest(
    const QString &busId, uint32_t nodeId)
{
    uint8_t data[2] = {0x00, 0x00};
    return makeWriteSDO(busId, nodeId, 0x200D, 0x1B, data, 2, Priority::Control, false);
}

// ==================== 解析响应 ====================

bool CanopenMotorProtocol::parseReadSDOResponse(const CanFrame &frame, uint8_t *data, size_t len)
{
    if (frame.data[0] == 0x80) return false;  // 错误响应

    // 提取数据 (从 byte 4 开始)
    if (len > 4) return false;
    std::memcpy(data, &frame.data[4], len);
    return true;
}

QString CanopenMotorProtocol::decodeStatusWord(uint16_t sw)
{
    if (sw & 0x0008) return QStringLiteral("故障");
    if ((sw & 0x006F) == 0x0027) return QStringLiteral("运行中");
    if ((sw & 0x006F) == 0x0023) return QStringLiteral("已开启");
    if ((sw & 0x006F) == 0x0021) return QStringLiteral("就绪");
    if ((sw & 0x004F) == 0x0040) return QStringLiteral("未准备好");
    if ((sw & 0x006F) == 0x0007) return QStringLiteral("快速停止");
    return QStringLiteral("未知(0x%1)").arg(sw, 4, 16, QChar('0'));
}

bool CanopenMotorProtocol::isInFault(uint16_t statusWord)
{
    return (statusWord & 0x0008) != 0;
}

bool CanopenMotorProtocol::isEnabled(uint16_t statusWord)
{
    return (statusWord & 0x006F) == 0x0027;
}

}  // namespace dar
