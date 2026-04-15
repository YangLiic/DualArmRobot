#pragma once
#include "models/common_types.h"
#include <cstdint>

namespace dac {

/*
 * CanopenMotorProtocol: CANopen SDO 报文编解码。
 *
 * 将业务层的"使能/速度/位置/扭矩读取"等意图
 * 编码为 CommRequest，将 CommResult 解码为业务数据。
 * 纯工具类，无状态。
 */

namespace OD {
    constexpr uint16_t ControlWord          = 0x6040;
    constexpr uint16_t StatusWord           = 0x6041;
    constexpr uint16_t OperationModeReg     = 0x6060;
    constexpr uint16_t TargetPosition       = 0x607A;
    constexpr uint16_t PositionDeviation    = 0x60F4;
    constexpr uint16_t TargetTorque         = 0x6071;
    constexpr uint16_t MaxTorque            = 0x6072;
    constexpr uint16_t ActualTorque         = 0x6077;
    constexpr uint16_t TorqueRamp           = 0x6087;
    constexpr uint16_t TargetVelocity       = 0x60FF;
    constexpr uint16_t ProfileVelocity      = 0x6081;
    constexpr uint16_t ProfileAcceleration  = 0x6083;
    constexpr uint16_t ProfileDeceleration  = 0x6084;
    constexpr uint16_t ForwardTorqueLimit   = 0x60E0;
    constexpr uint16_t ReverseTorqueLimit   = 0x60E1;
    constexpr uint16_t AverageLoad          = 0x200B;
    constexpr uint8_t  AverageLoadSub       = 0x0D;
    constexpr uint16_t PhaseCurrent         = 0x200B;
    constexpr uint8_t  PhaseCurrentSub      = 0x19;
    constexpr uint16_t BrakeControl         = 0x200D;
    constexpr uint8_t  BrakeControlSub      = 0x1B;
}

constexpr int32_t ENCODER_RESOLUTION = 8388608; // 2^23
constexpr double  PULSES_PER_DEGREE  = ENCODER_RESOLUTION / 360.0;

class CanopenMotorProtocol
{
public:
    // 计算 SDO 响应 CAN ID: 节点>=0x80时 response = node - 0x80
    static uint32_t responseId(uint32_t nodeId);

    // SDO 写请求
    static CommRequest writeSDO(uint32_t nodeId, uint16_t index, uint8_t sub,
                                const uint8_t *data, size_t len, Priority pri = Priority::Control);

    // SDO 读请求
    static CommRequest readSDO(uint32_t nodeId, uint16_t index, uint8_t sub,
                               size_t expectLen, Priority pri = Priority::Monitoring);

    // NMT 命令
    static CommRequest nmtStart(uint32_t nodeId);
    static CommRequest nmtPreOperational(uint32_t nodeId);

    // ============ 高层便利接口 ============

    static CommRequest setOperationMode(uint32_t nodeId, OperationMode mode);
    static CommRequest sendControlWord(uint32_t nodeId, uint16_t cw);
    static CommRequest setTargetVelocity(uint32_t nodeId, int32_t rpm, bool inverted);
    static CommRequest setTargetPosition(uint32_t nodeId, double degrees, bool inverted);
    static CommRequest setProfileVelocity(uint32_t nodeId, uint32_t rpm);
    static CommRequest setProfileAcceleration(uint32_t nodeId, uint32_t rpmPerSec);
    static CommRequest setProfileDeceleration(uint32_t nodeId, uint32_t rpmPerSec);
    static CommRequest setMaxTorqueLimit(uint32_t nodeId, uint16_t permille);
    static CommRequest startPositionMove(uint32_t nodeId, bool relative);
    static CommRequest clearPositionNewSetpoint(uint32_t nodeId);
    static CommRequest readActualTorque(uint32_t nodeId);
    static CommRequest readPhaseCurrent(uint32_t nodeId);
    static CommRequest readPositionDeviation(uint32_t nodeId);
    static CommRequest releaseBrake(uint32_t nodeId);
    static CommRequest lockBrake(uint32_t nodeId);

    // ============ 解析结果 ============
    static int16_t  parseTorquePermille(const CommResult &r);
    static double   parsePhaseCurrentAmp(const CommResult &r);
    static int32_t  parsePositionDeviation(const CommResult &r);

private:
    static void packU8(uint8_t *payload, uint8_t cmd, uint16_t idx, uint8_t sub, uint8_t val);
    static void packU16(uint8_t *payload, uint8_t cmd, uint16_t idx, uint8_t sub, uint16_t val);
    static void packU32(uint8_t *payload, uint8_t cmd, uint16_t idx, uint8_t sub, uint32_t val);
    static void packI32(uint8_t *payload, uint8_t cmd, uint16_t idx, uint8_t sub, int32_t val);
};

} // namespace dac
