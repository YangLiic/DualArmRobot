/**
 * CanopenMotorProtocol - CANopen 电机协议适配器
 *
 * 职责：
 * - 将业务请求编码为 CANopen SDO/NMT 报文
 * - 将底层响应解码为结构化结果
 * - 提供响应匹配规则
 * - 解释状态字、错误码
 *
 * 本类不直接访问串口，而是生成 CommRequest 提交给 CommunicationManager。
 */
#pragma once

#include <QObject>
#include <functional>
#include "../models/common_types.h"

namespace dar {

class CanopenMotorProtocol : public QObject
{
    Q_OBJECT

public:
    explicit CanopenMotorProtocol(QObject *parent = nullptr);

    // ==================== 生成通信请求 ====================

    /**
     * @brief 生成 SDO 写请求
     */
    CommRequest makeWriteSDO(const QString &busId, uint32_t nodeId,
                             uint16_t index, uint8_t subindex,
                             const uint8_t *data, size_t len,
                             Priority priority = Priority::Control,
                             bool waitResponse = true);

    /**
     * @brief 生成 SDO 读请求
     */
    CommRequest makeReadSDO(const QString &busId, uint32_t nodeId,
                            uint16_t index, uint8_t subindex,
                            size_t expectedLen,
                            Priority priority = Priority::Monitoring);

    /**
     * @brief 生成 NMT 命令请求
     */
    CommRequest makeNMT(const QString &busId, uint8_t command, uint8_t nodeAddr = 0x00);

    // ==================== 电机控制请求 ====================

    CommRequest makeEnableRequest(const QString &busId, uint32_t nodeId, OperationMode mode);
    CommRequest makeDisableRequest(const QString &busId, uint32_t nodeId);
    CommRequest makeFaultResetRequest(const QString &busId, uint32_t nodeId);
    CommRequest makeQuickStopRequest(const QString &busId, uint32_t nodeId);

    CommRequest makeSetVelocityRequest(const QString &busId, uint32_t nodeId, int32_t rpm);
    CommRequest makeSetPositionRequest(const QString &busId, uint32_t nodeId,
                                       double degrees, bool absolute = false);
    CommRequest makeStartPositionMoveRequest(const QString &busId, uint32_t nodeId,
                                              bool relative = true);

    CommRequest makeSetProfileVelocityRequest(const QString &busId, uint32_t nodeId, uint32_t rpm);
    CommRequest makeSetProfileAccelerationRequest(const QString &busId, uint32_t nodeId, uint32_t acc);
    CommRequest makeSetProfileDecelerationRequest(const QString &busId, uint32_t nodeId, uint32_t dec);

    CommRequest makeSetMaxTorqueRequest(const QString &busId, uint32_t nodeId, uint16_t permille);

    // ==================== 状态读取请求 ====================

    CommRequest makeReadTorqueRequest(const QString &busId, uint32_t nodeId);
    CommRequest makeReadStatusWordRequest(const QString &busId, uint32_t nodeId);
    CommRequest makeReadPhaseCurrentRequest(const QString &busId, uint32_t nodeId);
    CommRequest makeReadPositionDeviationRequest(const QString &busId, uint32_t nodeId);

    // ==================== NMT ====================
    CommRequest makeNmtStartRequest(const QString &busId);
    CommRequest makeNmtPreOpRequest(const QString &busId);

    // ==================== 抱闸 ====================
    CommRequest makeReleaseBrakeRequest(const QString &busId, uint32_t nodeId);
    CommRequest makeLockBrakeRequest(const QString &busId, uint32_t nodeId);

    // ==================== 解析响应 ====================

    /**
     * @brief 从 SDO 读响应帧中解析数据
     */
    static bool parseReadSDOResponse(const CanFrame &frame, uint8_t *data, size_t len);

    /**
     * @brief 解析状态字
     */
    static QString decodeStatusWord(uint16_t statusWord);

    /**
     * @brief 判断状态字是否表示故障
     */
    static bool isInFault(uint16_t statusWord);

    /**
     * @brief 判断状态字是否表示使能
     */
    static bool isEnabled(uint16_t statusWord);

    /**
     * @brief 获取 SDO 响应 ID
     * 对于 nodeId=0x601, responseId=0x581
     */
    static uint32_t getResponseId(uint32_t nodeId);

    // ==================== 常量 ====================

    static constexpr uint16_t OD_CONTROL_WORD       = 0x6040;
    static constexpr uint16_t OD_STATUS_WORD        = 0x6041;
    static constexpr uint16_t OD_OPERATION_MODE     = 0x6060;
    static constexpr uint16_t OD_TARGET_POSITION    = 0x607A;
    static constexpr uint16_t OD_TARGET_VELOCITY    = 0x60FF;
    static constexpr uint16_t OD_ACTUAL_TORQUE      = 0x6077;
    static constexpr uint16_t OD_MAX_TORQUE         = 0x6072;
    static constexpr uint16_t OD_PROFILE_VELOCITY   = 0x6081;
    static constexpr uint16_t OD_PROFILE_ACCEL      = 0x6083;
    static constexpr uint16_t OD_PROFILE_DECEL      = 0x6084;
    static constexpr uint16_t OD_POSITION_DEVIATION = 0x60F4;
    static constexpr uint16_t OD_PHASE_CURRENT      = 0x200B;
    static constexpr uint8_t  OD_PHASE_CURRENT_SUB  = 0x19;

    static constexpr int32_t  ENCODER_RESOLUTION    = 8388608; // 2^23
    static constexpr double   PULSES_PER_DEGREE     = ENCODER_RESOLUTION / 360.0;

private:
    uint64_t nextRequestId_ = 1;

    // 通用 payload 构建：[canId(4bytes)][data(8bytes)][dlc(1byte)]
    static QByteArray buildPayload(uint32_t canId, const uint8_t *data, uint8_t dlc);

    // 生成 SDO 响应匹配器
    static std::function<bool(const CanFrame&)>
    makeSDOMatcher(uint32_t responseId, uint16_t index, uint8_t subindex, bool isRead);
};

}  // namespace dar
