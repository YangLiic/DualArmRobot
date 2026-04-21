#include "protocols/ze300_protocol.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace dac {

// ===== 字节工具 =====
int32_t Ze300Protocol::toI32LE(const uint8_t* d) {
    return (int32_t)((uint32_t)d[0] | ((uint32_t)d[1]<<8) | ((uint32_t)d[2]<<16) | ((uint32_t)d[3]<<24));
}
int16_t Ze300Protocol::toI16LE(const uint8_t* d) {
    return (int16_t)((uint16_t)d[0] | ((uint16_t)d[1]<<8));
}
uint16_t Ze300Protocol::toU16LE(const uint8_t* d) {
    return (uint16_t)d[0] | ((uint16_t)d[1]<<8);
}
void Ze300Protocol::putI32LE(uint8_t* dst, int32_t v) {
    dst[0]=(uint8_t)(v&0xFF); dst[1]=(uint8_t)((v>>8)&0xFF);
    dst[2]=(uint8_t)((v>>16)&0xFF); dst[3]=(uint8_t)((v>>24)&0xFF);
}
void Ze300Protocol::putU16LE(uint8_t* dst, uint16_t v) {
    dst[0]=(uint8_t)(v&0xFF); dst[1]=(uint8_t)((v>>8)&0xFF);
}
uint32_t Ze300Protocol::floatToUint(float x, float xMin, float xMax, int bits) {
    if (xMax <= xMin) return 0;
    x = std::max(xMin, std::min(x, xMax));
    float norm = (x - xMin) / (xMax - xMin);
    uint32_t maxInt = (1u << bits) - 1u;
    return (uint32_t)std::round(norm * (float)maxInt);
}
float Ze300Protocol::uintToFloat(uint32_t x, float xMin, float xMax, int bits) {
    if (xMax <= xMin) return xMin;
    uint32_t maxInt = (1u << bits) - 1u;
    return xMin + ((float)x / (float)maxInt) * (xMax - xMin);
}

// ===== CAN ID 计算 =====
uint16_t Ze300Protocol::txCanId(uint16_t devAddr, bool useHostAddr) {
    return useHostAddr ? (0x100 | (devAddr & 0xFF)) : (devAddr & 0xFF);
}
uint16_t Ze300Protocol::mitTxCanId(uint16_t devAddr, bool useHostAddr) {
    return 0x400 | txCanId(devAddr, useHostAddr);
}

// ===== 通用命令构造 =====
CommRequest Ze300Protocol::makeCommand(uint16_t devAddr, bool useHostAddr,
                                       Ze300Cmd cmd, const uint8_t *payload,
                                       size_t payloadLen, Priority pri, int timeoutMs)
{
    CommRequest req;
    req.protocolType = ProtocolType::ZE300Cmd;
    req.canId = txCanId(devAddr, useHostAddr);
    req.responseCanId = devAddr & 0xFF;  // ZE300 responses come from devAddr
    req.ze300ExpectedCmd = static_cast<uint8_t>(cmd);
    req.priority = pri;
    req.timeoutMs = timeoutMs;
    req.expectResponse = true;

    req.payload[0] = static_cast<uint8_t>(cmd);
    size_t n = std::min<size_t>(payloadLen, 7);
    if (payload && n > 0) {
        std::memcpy(&req.payload[1], payload, n);
    }
    req.payloadLen = static_cast<uint8_t>(1 + n);
    return req;
}

// ===== 读取类 =====
CommRequest Ze300Protocol::readQuickStatus(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::ReadQuick, nullptr, 0, Priority::Monitoring, 200);
}
CommRequest Ze300Protocol::readSpeed(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::ReadSpeed, nullptr, 0, Priority::Monitoring, 200);
}
CommRequest Ze300Protocol::readStatus(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::ReadStatus, nullptr, 0, Priority::Monitoring, 200);
}
CommRequest Ze300Protocol::readPosition(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::ReadPosition, nullptr, 0, Priority::Monitoring, 200);
}
CommRequest Ze300Protocol::readMotorInfo(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::ReadMotorInfo, nullptr, 0, Priority::Monitoring, 300);
}
CommRequest Ze300Protocol::readMitLimits(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::MitLimitCfg, nullptr, 0, Priority::Monitoring, 300);
}
CommRequest Ze300Protocol::readMitState(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::MitReadState, nullptr, 0, Priority::Monitoring, 200);
}

// ===== 控制类 =====
CommRequest Ze300Protocol::setSpeedRpm(uint16_t devAddr, bool useHostAddr, float rpm) {
    uint8_t payload[4];
    int32_t raw = (int32_t)std::round(rpm * Ze300Const::RPM_RAW_SCALE);
    putI32LE(payload, raw);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::SpeedControl, payload, 4);
}

CommRequest Ze300Protocol::setAbsolutePositionCount(uint16_t devAddr, bool useHostAddr, int32_t count) {
    uint8_t payload[4];
    putI32LE(payload, count);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::AbsPositionCtrl, payload, 4);
}

CommRequest Ze300Protocol::setRelativePositionCount(uint16_t devAddr, bool useHostAddr, int32_t count) {
    uint8_t payload[4];
    putI32LE(payload, count);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::RelPositionCtrl, payload, 4);
}

CommRequest Ze300Protocol::setTorqueCurrentA(uint16_t devAddr, bool useHostAddr, float currentA) {
    uint8_t payload[4];
    int32_t raw = (int32_t)std::round(currentA * Ze300Const::CURRENT_RAW_SCALE);
    putI32LE(payload, raw);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::TorqueControl, payload, 4);
}

CommRequest Ze300Protocol::goOriginShortest(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::GoOriginShortest);
}

CommRequest Ze300Protocol::setZero(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::SetZero, nullptr, 0, Priority::Control, 500);
}

CommRequest Ze300Protocol::freeOutput(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::FreeOutput, nullptr, 0, Priority::Critical);
}

CommRequest Ze300Protocol::clearFault(uint16_t devAddr, bool useHostAddr) {
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::ClearFault, nullptr, 0, Priority::Critical, 300);
}

CommRequest Ze300Protocol::reboot(uint16_t devAddr, bool useHostAddr) {
    auto req = makeCommand(devAddr, useHostAddr, Ze300Cmd::Reboot, nullptr, 0, Priority::Critical);
    req.expectResponse = false;
    return req;
}

// ===== 抱闸 =====
CommRequest Ze300Protocol::setBrakeClosed(uint16_t devAddr, bool useHostAddr, bool closed) {
    uint8_t payload[1] = { closed ? (uint8_t)0x01 : (uint8_t)0x00 };
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::BrakeControl, payload, 1);
}
CommRequest Ze300Protocol::readBrakeState(uint16_t devAddr, bool useHostAddr) {
    uint8_t payload[1] = { 0xFF };
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::BrakeControl, payload, 1, Priority::Monitoring);
}

// ===== 参数设置 =====
CommRequest Ze300Protocol::setPositionMaxSpeedRpm(uint16_t devAddr, bool useHostAddr, float rpm) {
    uint8_t payload[4];
    int32_t raw = std::max(0, (int32_t)std::round(rpm * Ze300Const::RPM_RAW_SCALE));
    putI32LE(payload, raw);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::SetPosMaxSpeed, payload, 4);
}
CommRequest Ze300Protocol::setMaxCurrentA(uint16_t devAddr, bool useHostAddr, float currentA) {
    uint8_t payload[4];
    int32_t raw = std::max(0, (int32_t)std::round(currentA * Ze300Const::CURRENT_RAW_SCALE));
    putI32LE(payload, raw);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::SetMaxQCurrent, payload, 4);
}
CommRequest Ze300Protocol::setSpeedAccelerationRpmPerSec(uint16_t devAddr, bool useHostAddr, float accRpmS) {
    uint8_t payload[4];
    int32_t raw = std::max(0, (int32_t)std::round(accRpmS * Ze300Const::RPM_RAW_SCALE));
    putI32LE(payload, raw);
    return makeCommand(devAddr, useHostAddr, Ze300Cmd::SetSpeedAccel, payload, 4);
}

// ===== MIT 运控 =====
CommRequest Ze300Protocol::sendMitControl(uint16_t devAddr, bool useHostAddr,
                                          float posRad, float velRadS, float kp, float kd, float torqueNm,
                                          float posMaxRad, float velMaxRadS, float torqueMaxNm)
{
    CommRequest req;
    req.protocolType = ProtocolType::ZE300Cmd;
    req.canId = mitTxCanId(devAddr, useHostAddr);
    req.responseCanId = devAddr & 0xFF;
    req.ze300ExpectedCmd = static_cast<uint8_t>(Ze300Cmd::MitReadState);
    req.priority = Priority::Control;
    req.timeoutMs = 200;
    req.expectResponse = false;  // MIT control is fire-and-forget at frame level

    uint32_t posU16 = floatToUint(posRad, -posMaxRad, posMaxRad, 16);
    uint32_t velU12 = floatToUint(velRadS, -velMaxRadS, velMaxRadS, 12);
    uint32_t kpU12  = floatToUint(kp, 0.0f, 500.0f, 12);
    uint32_t kdU12  = floatToUint(kd, 0.0f, 5.0f, 12);
    uint32_t torU12 = floatToUint(torqueNm, -torqueMaxNm, torqueMaxNm, 12);

    req.payload[0] = (uint8_t)((posU16 >> 8) & 0xFF);
    req.payload[1] = (uint8_t)(posU16 & 0xFF);
    req.payload[2] = (uint8_t)((velU12 >> 4) & 0xFF);
    req.payload[3] = (uint8_t)(((velU12 & 0x0F) << 4) | ((kpU12 >> 8) & 0x0F));
    req.payload[4] = (uint8_t)(kpU12 & 0xFF);
    req.payload[5] = (uint8_t)((kdU12 >> 4) & 0xFF);
    req.payload[6] = (uint8_t)(((kdU12 & 0x0F) << 4) | ((torU12 >> 8) & 0x0F));
    req.payload[7] = (uint8_t)(torU12 & 0xFF);
    req.payloadLen = 8;
    return req;
}

// ===== 解析响应 =====
void Ze300Protocol::parseResponse(const CommResult &result, Ze300Status &status)
{
    if (!result.success || result.dlc < 1) return;
    const uint8_t *data = result.data;
    uint8_t cmd = data[0];
    uint8_t dlc = result.dlc;

    status.online = true;
    status.lastUpdateTime = QDateTime::currentDateTime();

    switch (cmd) {
    case (uint8_t)Ze300Cmd::ReadQCurrent:
    case (uint8_t)Ze300Cmd::TorqueControl:
        if (dlc >= 5) status.qCurrentA = (float)toI32LE(&data[1]) / Ze300Const::CURRENT_RAW_SCALE;
        break;

    case (uint8_t)Ze300Cmd::ReadSpeed:
    case (uint8_t)Ze300Cmd::SpeedControl:
        if (dlc >= 5) status.speedRpm = (float)toI32LE(&data[1]) / Ze300Const::RPM_RAW_SCALE;
        break;

    case (uint8_t)Ze300Cmd::ReadPosition:
    case (uint8_t)Ze300Cmd::AbsPositionCtrl:
    case (uint8_t)Ze300Cmd::RelPositionCtrl:
        if (dlc >= 7) {
            status.singleTurnCount = toU16LE(&data[1]);
            status.multiTurnCount = toI32LE(&data[3]);
            status.singleTurnDeg = status.singleTurnCount * Ze300Const::DEG_PER_COUNT;
            status.totalTurnDeg = status.multiTurnCount * Ze300Const::DEG_PER_COUNT;
        }
        break;

    case (uint8_t)Ze300Cmd::ReadQuick:
        if (dlc >= 8) {
            status.temperatureC = (float)data[1];
            status.qCurrentA = (float)toI16LE(&data[2]) / Ze300Const::CURRENT_RAW_SCALE;
            // 某些 ZE300 固件的 A4 转速字段会滞后，转速统一以 A2 读取值为准。
            status.singleTurnCount = toU16LE(&data[6]);
            status.singleTurnDeg = status.singleTurnCount * Ze300Const::DEG_PER_COUNT;
        }
        break;

    case (uint8_t)Ze300Cmd::ReadStatus:
    case (uint8_t)Ze300Cmd::FreeOutput:
        if (dlc >= 8) {
            status.busVoltageV = (float)toU16LE(&data[1]) / Ze300Const::VOLT_RAW_SCALE;
            status.busCurrentA = (float)toU16LE(&data[3]) / Ze300Const::BUS_CURRENT_SCALE;
            status.temperatureC = (float)data[5];
            status.runMode = data[6];
            status.faultCode = data[7];
            status.fault = (status.faultCode != 0);
        }
        break;

    case (uint8_t)Ze300Cmd::ClearFault:
        if (dlc >= 2) {
            status.faultCode = data[1];
            status.fault = (status.faultCode != 0);
        }
        break;

    case (uint8_t)Ze300Cmd::ReadMotorInfo:
        if (dlc >= 7) {
            status.polePairs = data[1];
            float tc = 0.0f;
            std::memcpy(&tc, &data[2], sizeof(float));
            status.torqueConstant = tc;
            status.gearRatio = data[6];
        }
        break;

    case (uint8_t)Ze300Cmd::BrakeControl:
        if (dlc >= 2) {
            status.brakeState = (data[1] == 0x01);
        }
        break;

    case (uint8_t)Ze300Cmd::MitLimitCfg:
        // MIT limits stored externally in service
        break;

    case (uint8_t)Ze300Cmd::MitReadState:
        // MIT state parsing handled externally with limits context
        break;

    default:
        break;
    }
}

} // namespace dac
