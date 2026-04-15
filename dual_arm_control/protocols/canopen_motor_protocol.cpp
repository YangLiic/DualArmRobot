#include "protocols/canopen_motor_protocol.h"
#include <cstring>

namespace dac {

uint32_t CanopenMotorProtocol::responseId(uint32_t nodeId)
{
    return (nodeId >= 0x80) ? (nodeId - 0x80) : nodeId;
}

// ==================== 辅助打包 ====================

void CanopenMotorProtocol::packU8(uint8_t *p, uint8_t cmd, uint16_t idx, uint8_t sub, uint8_t val)
{
    p[0] = cmd; p[1] = idx & 0xFF; p[2] = (idx >> 8) & 0xFF;
    p[3] = sub; p[4] = val; p[5] = 0; p[6] = 0; p[7] = 0;
}

void CanopenMotorProtocol::packU16(uint8_t *p, uint8_t cmd, uint16_t idx, uint8_t sub, uint16_t val)
{
    p[0] = cmd; p[1] = idx & 0xFF; p[2] = (idx >> 8) & 0xFF;
    p[3] = sub; p[4] = val & 0xFF; p[5] = (val >> 8) & 0xFF; p[6] = 0; p[7] = 0;
}

void CanopenMotorProtocol::packU32(uint8_t *p, uint8_t cmd, uint16_t idx, uint8_t sub, uint32_t val)
{
    p[0] = cmd; p[1] = idx & 0xFF; p[2] = (idx >> 8) & 0xFF;
    p[3] = sub;
    p[4] = val & 0xFF; p[5] = (val >> 8) & 0xFF;
    p[6] = (val >> 16) & 0xFF; p[7] = (val >> 24) & 0xFF;
}

void CanopenMotorProtocol::packI32(uint8_t *p, uint8_t cmd, uint16_t idx, uint8_t sub, int32_t val)
{
    packU32(p, cmd, idx, sub, static_cast<uint32_t>(val));
}

// ==================== SDO 基础 ====================

CommRequest CanopenMotorProtocol::writeSDO(uint32_t nodeId, uint16_t index, uint8_t sub,
                                            const uint8_t *data, size_t len, Priority pri)
{
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = index;
    req.sdoSubindex = sub;
    req.priority = pri;
    req.payloadLen = 8;
    req.expectResponse = false;

    uint8_t cmd = 0x23;
    if (len == 1)      cmd = 0x2F;
    else if (len == 2) cmd = 0x2B;

    req.payload[0] = cmd;
    req.payload[1] = index & 0xFF;
    req.payload[2] = (index >> 8) & 0xFF;
    req.payload[3] = sub;
    std::memcpy(&req.payload[4], data, len);
    if (len < 4) std::memset(&req.payload[4 + len], 0, 4 - len);

    return req;
}

CommRequest CanopenMotorProtocol::readSDO(uint32_t nodeId, uint16_t index, uint8_t sub,
                                           size_t /*expectLen*/, Priority pri)
{
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = index;
    req.sdoSubindex = sub;
    req.priority = pri;
    req.payloadLen = 8;
    req.expectResponse = true;
    req.timeoutMs = 150;

    req.payload[0] = 0x40;
    req.payload[1] = index & 0xFF;
    req.payload[2] = (index >> 8) & 0xFF;
    req.payload[3] = sub;
    std::memset(&req.payload[4], 0, 4);

    return req;
}

// ==================== NMT ====================

CommRequest CanopenMotorProtocol::nmtStart(uint32_t /*nodeId*/)
{
    CommRequest req;
    req.canId = 0x000;
    req.payloadLen = 2;
    req.payload[0] = 0x01;
    req.payload[1] = 0x00;
    req.expectResponse = false;
    req.priority = Priority::Control;
    return req;
}

CommRequest CanopenMotorProtocol::nmtPreOperational(uint32_t /*nodeId*/)
{
    CommRequest req;
    req.canId = 0x000;
    req.payloadLen = 2;
    req.payload[0] = 0x80;
    req.payload[1] = 0x00;
    req.expectResponse = false;
    req.priority = Priority::Control;
    return req;
}

// ==================== 高层便利 ====================

CommRequest CanopenMotorProtocol::setOperationMode(uint32_t nodeId, OperationMode mode)
{
    uint8_t val = static_cast<uint8_t>(mode);
    return writeSDO(nodeId, OD::OperationModeReg, 0x00, &val, 1, Priority::Control);
}

CommRequest CanopenMotorProtocol::sendControlWord(uint32_t nodeId, uint16_t cw)
{
    uint8_t data[2] = { static_cast<uint8_t>(cw & 0xFF), static_cast<uint8_t>((cw >> 8) & 0xFF) };
    return writeSDO(nodeId, OD::ControlWord, 0x00, data, 2, Priority::Control);
}

CommRequest CanopenMotorProtocol::setTargetVelocity(uint32_t nodeId, int32_t rpm, bool inverted)
{
    int32_t actual = inverted ? -rpm : rpm;
    int32_t enc = static_cast<int32_t>((static_cast<int64_t>(actual) * ENCODER_RESOLUTION) / 60);
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = OD::TargetVelocity;
    req.sdoSubindex = 0;
    req.priority = Priority::Control;
    req.payloadLen = 8;
    req.expectResponse = false;
    packI32(req.payload, 0x23, OD::TargetVelocity, 0x00, enc);
    return req;
}

CommRequest CanopenMotorProtocol::setTargetPosition(uint32_t nodeId, double degrees, bool inverted)
{
    double actual = inverted ? -degrees : degrees;
    int32_t pulses = static_cast<int32_t>(actual * PULSES_PER_DEGREE);
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = OD::TargetPosition;
    req.sdoSubindex = 0;
    req.priority = Priority::Control;
    req.payloadLen = 8;
    req.expectResponse = false;
    packI32(req.payload, 0x23, OD::TargetPosition, 0x00, pulses);
    return req;
}

CommRequest CanopenMotorProtocol::setProfileVelocity(uint32_t nodeId, uint32_t rpm)
{
    uint32_t pps = static_cast<uint32_t>((static_cast<uint64_t>(rpm) * ENCODER_RESOLUTION) / 60);
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = OD::ProfileVelocity;
    req.sdoSubindex = 0;
    req.priority = Priority::Control;
    req.payloadLen = 8;
    req.expectResponse = false;
    packU32(req.payload, 0x23, OD::ProfileVelocity, 0x00, pps);
    return req;
}

CommRequest CanopenMotorProtocol::setProfileAcceleration(uint32_t nodeId, uint32_t rpmPerSec)
{
    uint32_t pps2 = static_cast<uint32_t>((static_cast<uint64_t>(rpmPerSec) * ENCODER_RESOLUTION) / 60);
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = OD::ProfileAcceleration;
    req.sdoSubindex = 0;
    req.priority = Priority::Control;
    req.payloadLen = 8;
    req.expectResponse = false;
    packU32(req.payload, 0x23, OD::ProfileAcceleration, 0x00, pps2);
    return req;
}

CommRequest CanopenMotorProtocol::setProfileDeceleration(uint32_t nodeId, uint32_t rpmPerSec)
{
    uint32_t pps2 = static_cast<uint32_t>((static_cast<uint64_t>(rpmPerSec) * ENCODER_RESOLUTION) / 60);
    CommRequest req;
    req.canId = nodeId;
    req.responseCanId = responseId(nodeId);
    req.sdoIndex = OD::ProfileDeceleration;
    req.sdoSubindex = 0;
    req.priority = Priority::Control;
    req.payloadLen = 8;
    req.expectResponse = false;
    packU32(req.payload, 0x23, OD::ProfileDeceleration, 0x00, pps2);
    return req;
}

CommRequest CanopenMotorProtocol::setMaxTorqueLimit(uint32_t nodeId, uint16_t permille)
{
    uint8_t data[2] = { static_cast<uint8_t>(permille & 0xFF), static_cast<uint8_t>((permille >> 8) & 0xFF) };
    return writeSDO(nodeId, OD::MaxTorque, 0x00, data, 2, Priority::Control);
}

CommRequest CanopenMotorProtocol::startPositionMove(uint32_t nodeId, bool relative)
{
    uint16_t cw = relative ? 0x007F : 0x003F;
    return sendControlWord(nodeId, cw);
}

CommRequest CanopenMotorProtocol::clearPositionNewSetpoint(uint32_t nodeId)
{
    return sendControlWord(nodeId, 0x000F);
}

CommRequest CanopenMotorProtocol::readActualTorque(uint32_t nodeId)
{
    return readSDO(nodeId, OD::ActualTorque, 0x00, 2, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::readPhaseCurrent(uint32_t nodeId)
{
    return readSDO(nodeId, OD::PhaseCurrent, OD::PhaseCurrentSub, 2, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::readPositionDeviation(uint32_t nodeId)
{
    return readSDO(nodeId, OD::PositionDeviation, 0x00, 4, Priority::Monitoring);
}

CommRequest CanopenMotorProtocol::releaseBrake(uint32_t nodeId)
{
    uint8_t data[2] = {0x02, 0x00};
    return writeSDO(nodeId, OD::BrakeControl, OD::BrakeControlSub, data, 2, Priority::Control);
}

CommRequest CanopenMotorProtocol::lockBrake(uint32_t nodeId)
{
    uint8_t data[2] = {0x00, 0x00};
    return writeSDO(nodeId, OD::BrakeControl, OD::BrakeControlSub, data, 2, Priority::Control);
}

// ==================== 解析结果 ====================

int16_t CanopenMotorProtocol::parseTorquePermille(const CommResult &r)
{
    if (!r.success || r.dlc < 6) return 0;
    return static_cast<int16_t>(r.data[4] | (r.data[5] << 8));
}

double CanopenMotorProtocol::parsePhaseCurrentAmp(const CommResult &r)
{
    if (!r.success || r.dlc < 6) return 0.0;
    uint16_t ca = static_cast<uint16_t>(r.data[4] | (r.data[5] << 8));
    return static_cast<double>(ca) / 100.0;
}

int32_t CanopenMotorProtocol::parsePositionDeviation(const CommResult &r)
{
    if (!r.success || r.dlc < 8) return 0;
    return static_cast<int32_t>(
        r.data[4] | (r.data[5] << 8) | (r.data[6] << 16) | (r.data[7] << 24));
}

} // namespace dac
