/**
 * MotorService - 电机业务服务层
 *
 * 职责：
 * - 向 GUI 暴露电机操作接口
 * - 维护电机状态缓存 (MotorState)
 * - 组织初始化/使能/失能流程
 * - 管理轮询任务注册
 * - 把协议结果转为 GUI 可用数据
 * - 支持多节点管理
 */
#pragma once

#include <QObject>
#include <QMap>
#include <QTimer>
#include "../models/common_types.h"
#include "../communication/communication_manager.h"
#include "../protocols/canopen_motor_protocol.h"

namespace dar {

class MotorService : public QObject
{
    Q_OBJECT

public:
    explicit MotorService(CommunicationManager *commMgr, QObject *parent = nullptr);
    ~MotorService();

    // ==================== 节点管理 ====================

    /**
     * @brief 添加电机节点
     */
    void addMotor(const QString &busId, uint32_t nodeId);

    /**
     * @brief 移除电机节点
     */
    void removeMotor(uint32_t nodeId);

    /**
     * @brief 获取所有节点 ID
     */
    QList<uint32_t> motorNodeIds() const;

    /**
     * @brief 获取电机状态
     */
    MotorState motorState(uint32_t nodeId) const;

    /**
     * @brief 获取所有电机状态
     */
    QMap<uint32_t, MotorState> allMotorStates() const;

    // ==================== 单节点控制 ====================

public slots:
    /**
     * @brief 开始通信
     */
    void startCommunication(uint32_t nodeId);

    /**
     * @brief 停止通信
     */
    void stopCommunication(uint32_t nodeId);

    /**
     * @brief 使能电机
     */
    void enableMotor(uint32_t nodeId, OperationMode mode = OperationMode::Velocity);

    /**
     * @brief 失能电机
     */
    void disableMotor(uint32_t nodeId);

    /**
     * @brief 急停
     */
    void quickStopMotor(uint32_t nodeId);

    /**
     * @brief 故障复位
     */
    void faultResetMotor(uint32_t nodeId);

    /**
     * @brief 设置速度 (RPM)
     */
    void setVelocity(uint32_t nodeId, int32_t rpm);

    /**
     * @brief 设置位置 (角度)
     */
    void setPosition(uint32_t nodeId, double degrees, bool absolute = false);

    /**
     * @brief 启动位置运动
     */
    void startPositionMove(uint32_t nodeId, bool relative = true);

    /**
     * @brief 设置运动参数
     */
    void setProfileVelocity(uint32_t nodeId, uint32_t rpm);
    void setProfileAcceleration(uint32_t nodeId, uint32_t acc);
    void setProfileDeceleration(uint32_t nodeId, uint32_t dec);
    void setMaxTorque(uint32_t nodeId, uint16_t permille);

    /**
     * @brief 抱闸控制
     */
    void releaseBrake(uint32_t nodeId);
    void lockBrake(uint32_t nodeId);

    // ==================== 批量操作 ====================

    void enableAllMotors(OperationMode mode = OperationMode::Velocity);
    void disableAllMotors();
    void quickStopAllMotors();
    void faultResetAllMotors();

    // ==================== 监控 ====================

    /**
     * @brief 开始状态监控（启动轮询）
     */
    void startMonitoring(uint32_t nodeId);

    /**
     * @brief 停止状态监控
     */
    void stopMonitoring(uint32_t nodeId);

    /**
     * @brief 扫描在线节点 (0x601 ~ 0x610)
     */
    void scanOnlineNodes();

signals:
    // ==================== 信号 ====================

    void motorStateChanged(uint32_t nodeId, const dar::MotorState &state);
    void motorEnabled(uint32_t nodeId);
    void motorDisabled(uint32_t nodeId);
    void motorFaultOccurred(uint32_t nodeId, uint16_t faultCode, const QString &faultText);
    void collisionTriggered(uint32_t nodeId);
    void motorOnlineChanged(uint32_t nodeId, bool online);
    void logMessage(const dar::LogEntry &entry);
    void scanComplete(QList<uint32_t> onlineNodes);

private slots:
    void onRequestCompleted(const dar::CommResult &result);
    void onFrameReceived(const QString &busId, const dar::CanFrame &frame);

private:
    struct MotorEntry {
        QString busId;
        MotorState state;
        bool monitoring = false;
    };

    void sendEnableSequence(uint32_t nodeId, OperationMode mode, int step = 0);
    void log(LogEntry::Level level, const QString &msg);
    QString busIdForMotor(uint32_t nodeId) const;

    CommunicationManager *commMgr_;
    CanopenMotorProtocol *protocol_;
    QMap<uint32_t, MotorEntry> motors_;

    // 使能状态机跟踪
    struct EnableState {
        OperationMode mode;
        int step;
        uint64_t lastRequestId;
    };
    QMap<uint32_t, EnableState> enableStates_;

    // 请求跟踪
    struct PendingRequest {
        uint32_t nodeId;
        QString commandType;
    };
    QMap<uint64_t, PendingRequest> pendingRequests_;
};

}  // namespace dar
