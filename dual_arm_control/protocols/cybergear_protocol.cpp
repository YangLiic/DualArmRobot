#include "protocols/cybergear_protocol.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace dac {

static constexpr float P_MIN = -12.5f, P_MAX = 12.5f;
static constexpr float V_MIN = -30.0f, V_MAX = 30.0f;
static constexpr float T_MIN = -12.0f, T_MAX = 12.0f;

uint32_t CyberGearProtocol::makeExtId(uint8_t cmdId, uint16_t option, uint8_t targetId)
{
    return ((uint32_t)cmdId << 24) | ((uint32_t)option << 8) | (uint32_t)targetId;
}

float CyberGearProtocol::uintToFloat(uint16_t x, float xMin, float xMax)
{
    return (float)x / 65535.0f * (xMax - xMin) + xMin;
}

CommRequest CyberGearProtocol::makeCommand(uint8_t motorId, uint8_t masterId,
                                            uint8_t cmdId, uint16_t option,
                                            const uint8_t *data, size_t len,
                                            uint8_t expectedReplyCmd,
                                            Priority pri, int timeoutMs)
{
    CommRequest req;
    req.protocolType = ProtocolType::CyberGear;
    req.isExtendedFrame = true;
    req.canId = makeExtId(cmdId, option, motorId);
    req.cgExpectedCmdType = expectedReplyCmd;
    req.cgMotorId = motorId;
    req.cgMasterId = masterId;
    req.responseCanId = 0;  // not used for CyberGear (matching via ext ID fields)
    req.priority = pri;
    req.timeoutMs = timeoutMs;
    req.expectResponse = true;

    size_t n = std::min<size_t>(len, 8);
    if (data && n > 0) std::memcpy(req.payload, data, n);
    req.payloadLen = static_cast<uint8_t>(n);
    return req;
}

// ===== 基础控制 =====
CommRequest CyberGearProtocol::enable(uint8_t motorId, uint8_t masterId)
{
    uint8_t data[8] = {0};
    return makeCommand(motorId, masterId, (uint8_t)CgCmd::Enable, masterId,
                       data, 8, (uint8_t)CgCmd::Feedback);
}

CommRequest CyberGearProtocol::stop(uint8_t motorId, uint8_t masterId)
{
    uint8_t data[8] = {0};
    auto req = makeCommand(motorId, masterId, (uint8_t)CgCmd::Stop, masterId,
                           data, 8, (uint8_t)CgCmd::Feedback, Priority::Critical);
    req.expectResponse = false;
    return req;
}

CommRequest CyberGearProtocol::setMechZero(uint8_t motorId, uint8_t masterId)
{
    uint8_t data[8] = {0};
    data[0] = 0x01;
    return makeCommand(motorId, masterId, (uint8_t)CgCmd::SetZero, masterId,
                       data, 8, (uint8_t)CgCmd::Feedback, Priority::Control, 500);
}

// ===== 参数写入 =====
CommRequest CyberGearProtocol::writeFloatParam(uint8_t motorId, uint8_t masterId,
                                                uint16_t addr, float value)
{
    uint8_t data[8] = {0};
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    std::memcpy(&data[4], &value, 4);
    auto req = makeCommand(motorId, masterId, (uint8_t)CgCmd::RamWrite, masterId,
                           data, 8, (uint8_t)CgCmd::Feedback);
    req.expectResponse = false;  // RAM write doesn't always get ack
    return req;
}

CommRequest CyberGearProtocol::writeUint8Param(uint8_t motorId, uint8_t masterId,
                                                uint16_t addr, uint8_t value)
{
    uint8_t data[8] = {0};
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    data[4] = value;
    auto req = makeCommand(motorId, masterId, (uint8_t)CgCmd::RamWrite, masterId,
                           data, 8, (uint8_t)CgCmd::Feedback);
    req.expectResponse = false;
    return req;
}

// ===== 高层便利 =====
CommRequest CyberGearProtocol::setRunMode(uint8_t motorId, uint8_t masterId, CgRunMode mode)
{
    return writeUint8Param(motorId, masterId, CgParam::RunMode, (uint8_t)mode);
}

CommRequest CyberGearProtocol::setSpeedRadS(uint8_t motorId, uint8_t masterId, float radS)
{
    radS = std::max(V_MIN, std::min(radS, V_MAX));
    return writeFloatParam(motorId, masterId, CgParam::SpdRef, radS);
}

CommRequest CyberGearProtocol::setPositionRad(uint8_t motorId, uint8_t masterId, float rad)
{
    rad = std::max(P_MIN, std::min(rad, P_MAX));
    return writeFloatParam(motorId, masterId, CgParam::LocRef, rad);
}

CommRequest CyberGearProtocol::setCurrentLimit(uint8_t motorId, uint8_t masterId, float amps)
{
    amps = std::max(0.0f, std::min(amps, 27.0f));
    return writeFloatParam(motorId, masterId, CgParam::LimitCur, amps);
}

CommRequest CyberGearProtocol::setSpeedLimit(uint8_t motorId, uint8_t masterId, float radS)
{
    radS = std::max(0.0f, std::min(radS, 30.0f));
    return writeFloatParam(motorId, masterId, CgParam::LimitSpd, radS);
}

CommRequest CyberGearProtocol::requestFeedback(uint8_t motorId, uint8_t masterId)
{
    auto req = enable(motorId, masterId);  // Re-enable triggers feedback
    req.priority = Priority::Monitoring;
    req.timeoutMs = 120;
    return req;
}

// ===== 解析反馈帧 =====
void CyberGearProtocol::parseFeedback(uint32_t extCanId, const uint8_t *data,
                                       CyberGearStatus &status)
{
    uint8_t cmdType  = (extCanId >> 24) & 0x3F;
    uint8_t faultBits = (extCanId >> 16) & 0x3F;
    uint8_t stateBits = (extCanId >> 22) & 0x03;
    uint8_t motorId  = (extCanId >> 8) & 0xFF;

    status.motorId = motorId;
    status.online = true;
    status.lastUpdateTime = QDateTime::currentDateTime();
    status.uncalibrated = (faultBits & (1u << 5)) != 0;
    status.hallFault = (faultBits & (1u << 4)) != 0;
    status.magneticFault = (faultBits & (1u << 3)) != 0;
    status.overTempFault = (faultBits & (1u << 2)) != 0;
    status.overCurrentFault = (faultBits & (1u << 1)) != 0;
    status.underVoltageFault = (faultBits & (1u << 0)) != 0;
    status.hasFault = faultBits != 0;
    status.motorState = stateBits <= static_cast<uint8_t>(CgMotorState::Motor)
                      ? static_cast<CgMotorState>(stateBits)
                      : CgMotorState::Unknown;
    status.enabled = (status.motorState == CgMotorState::Motor);

    if (cmdType == (uint8_t)CgCmd::Feedback) {
        uint16_t rawPos  = ((uint16_t)data[0] << 8) | data[1];
        uint16_t rawVel  = ((uint16_t)data[2] << 8) | data[3];
        uint16_t rawTorq = ((uint16_t)data[4] << 8) | data[5];
        uint16_t rawTemp = ((uint16_t)data[6] << 8) | data[7];

        status.positionRad  = uintToFloat(rawPos, P_MIN, P_MAX);
        status.velocityRadS = uintToFloat(rawVel, V_MIN, V_MAX);
        status.torqueNm     = uintToFloat(rawTorq, T_MIN, T_MAX);
        status.temperature  = (float)rawTemp / 10.0f;
    } else if (cmdType == (uint8_t)CgCmd::FaultFeedback) {
        status.hasFault = true;
    }
}

} // namespace dac
