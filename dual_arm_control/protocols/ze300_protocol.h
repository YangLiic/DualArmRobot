#pragma once
#include "models/common_types.h"
#include <cstdint>

namespace dac {

/*
 * Ze300Protocol: ZE300 电机报文编解码。
 *
 * 将业务层的"速度/位置/抱闸/MIT/状态读取"等意图
 * 编码为 CommRequest，将 CommResult 解码为 Ze300Status。
 * 纯工具类，无状态。
 */

namespace Ze300Const {
    constexpr float COUNT_PER_REV    = 16384.0f;
    constexpr float DEG_PER_COUNT    = 360.0f / COUNT_PER_REV;
    constexpr float RPM_RAW_SCALE    = 100.0f;
    constexpr float CURRENT_RAW_SCALE = 1000.0f;
    constexpr float VOLT_RAW_SCALE   = 100.0f;
    constexpr float BUS_CURRENT_SCALE = 100.0f;
}

class Ze300Protocol
{
public:
    // ===== CAN ID 计算 =====
    static uint16_t txCanId(uint16_t devAddr, bool useHostAddr);
    static uint16_t mitTxCanId(uint16_t devAddr, bool useHostAddr);

    // ===== 构造 CommRequest =====
    static CommRequest makeCommand(uint16_t devAddr, bool useHostAddr,
                                   Ze300Cmd cmd, const uint8_t *payload = nullptr,
                                   size_t payloadLen = 0, Priority pri = Priority::Control,
                                   int timeoutMs = 300);

    // 读取类
    static CommRequest readQuickStatus(uint16_t devAddr, bool useHostAddr);
    static CommRequest readSpeed(uint16_t devAddr, bool useHostAddr);
    static CommRequest readStatus(uint16_t devAddr, bool useHostAddr);
    static CommRequest readPosition(uint16_t devAddr, bool useHostAddr);
    static CommRequest readMotorInfo(uint16_t devAddr, bool useHostAddr);
    static CommRequest readMitLimits(uint16_t devAddr, bool useHostAddr);
    static CommRequest readMitState(uint16_t devAddr, bool useHostAddr);

    // 控制类
    static CommRequest setSpeedRpm(uint16_t devAddr, bool useHostAddr, float rpm);
    static CommRequest setAbsolutePositionCount(uint16_t devAddr, bool useHostAddr, int32_t count);
    static CommRequest setRelativePositionCount(uint16_t devAddr, bool useHostAddr, int32_t count);
    static CommRequest setTorqueCurrentA(uint16_t devAddr, bool useHostAddr, float currentA);
    static CommRequest goOriginShortest(uint16_t devAddr, bool useHostAddr);
    static CommRequest setZero(uint16_t devAddr, bool useHostAddr);
    static CommRequest freeOutput(uint16_t devAddr, bool useHostAddr);
    static CommRequest clearFault(uint16_t devAddr, bool useHostAddr);
    static CommRequest reboot(uint16_t devAddr, bool useHostAddr);

    // 抱闸
    static CommRequest setBrakeClosed(uint16_t devAddr, bool useHostAddr, bool closed);
    static CommRequest readBrakeState(uint16_t devAddr, bool useHostAddr);

    // 参数设置
    static CommRequest setPositionMaxSpeedRpm(uint16_t devAddr, bool useHostAddr, float rpm);
    static CommRequest setMaxCurrentA(uint16_t devAddr, bool useHostAddr, float currentA);
    static CommRequest setSpeedAccelerationRpmPerSec(uint16_t devAddr, bool useHostAddr, float accRpmS);

    // MIT 运控
    static CommRequest sendMitControl(uint16_t devAddr, bool useHostAddr,
                                      float posRad, float velRadS, float kp, float kd, float torqueNm,
                                      float posMaxRad = 95.5f, float velMaxRadS = 45.0f, float torqueMaxNm = 18.0f);

    // ===== 解析响应 =====
    // 从 CommResult 解析并更新 Ze300Status (累积更新)
    static void parseResponse(const CommResult &result, Ze300Status &status);

private:
    static int32_t toI32LE(const uint8_t* data);
    static int16_t toI16LE(const uint8_t* data);
    static uint16_t toU16LE(const uint8_t* data);
    static void putI32LE(uint8_t* dst, int32_t v);
    static void putU16LE(uint8_t* dst, uint16_t v);
    static uint32_t floatToUint(float x, float xMin, float xMax, int bits);
    static float uintToFloat(uint32_t x, float xMin, float xMax, int bits);
};

} // namespace dac
