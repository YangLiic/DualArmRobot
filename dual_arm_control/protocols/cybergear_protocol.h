#pragma once
#include "models/common_types.h"

namespace dac {

/*
 * CyberGearProtocol: 小米 CyberGear 微电机报文编解码。
 *
 * 使用 29-bit 扩展帧 CAN ID:
 *   cmd_id[28:24] | option[23:8] | target_id[7:0]
 */
class CyberGearProtocol
{
public:
    // ===== 扩展 CAN ID 构造 =====
    static uint32_t makeExtId(uint8_t cmdId, uint16_t option, uint8_t targetId);

    // ===== 构造 CommRequest =====
    static CommRequest makeCommand(uint8_t motorId, uint8_t masterId,
                                   uint8_t cmdId, uint16_t option,
                                   const uint8_t *data, size_t len,
                                   uint8_t expectedReplyCmd,
                                   Priority pri = Priority::Control,
                                   int timeoutMs = 300);

    // 基础控制
    static CommRequest enable(uint8_t motorId, uint8_t masterId);
    static CommRequest stop(uint8_t motorId, uint8_t masterId);
    static CommRequest setMechZero(uint8_t motorId, uint8_t masterId);

    // 参数写入 (RAM)
    static CommRequest writeFloatParam(uint8_t motorId, uint8_t masterId,
                                       uint16_t addr, float value);
    static CommRequest writeUint8Param(uint8_t motorId, uint8_t masterId,
                                       uint16_t addr, uint8_t value);

    // 高层便利
    static CommRequest setRunMode(uint8_t motorId, uint8_t masterId, CgRunMode mode);
    static CommRequest setSpeedRadS(uint8_t motorId, uint8_t masterId, float radS);
    static CommRequest setPositionRad(uint8_t motorId, uint8_t masterId, float rad);
    static CommRequest setCurrentLimit(uint8_t motorId, uint8_t masterId, float amps);
    static CommRequest setSpeedLimit(uint8_t motorId, uint8_t masterId, float radS);

    // 反馈请求 (re-send enable to trigger type 2 feedback)
    static CommRequest requestFeedback(uint8_t motorId, uint8_t masterId);

    // ===== 解析响应 =====
    static void parseFeedback(uint32_t extCanId, const uint8_t *data,
                              CyberGearStatus &status);

private:
    static float uintToFloat(uint16_t x, float xMin, float xMax);
};

} // namespace dac
