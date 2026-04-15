#pragma once
#include "models/common_types.h"
#include "communication/communication_manager.h"
#include <QObject>
#include <QMap>

namespace dac {

/*
 * MotorService: 电机业务服务层。
 *
 * 对 GUI 暴露"设备能力"，不暴露底层协议细节。
 * 维护各节点状态缓存。通过 signal 通知 UI 层。
 */
class MotorService : public QObject
{
    Q_OBJECT
public:
    explicit MotorService(CommunicationManager *comm, QObject *parent = nullptr);

    // 节点管理
    void addNode(uint32_t nodeId, const QString &name, bool directionInverted = false);
    void removeNode(uint32_t nodeId);
    QList<uint32_t> nodeIds() const;
    QString nodeName(uint32_t nodeId) const;

    // 状态查询
    MotorState motorState(uint32_t nodeId) const;

    // ============ 控制指令 ============
    void startCommunication(uint32_t nodeId, OperationMode mode);
    void stopCommunication(uint32_t nodeId);
    void enableMotor(uint32_t nodeId, OperationMode mode);
    void disableMotor(uint32_t nodeId);
    void faultReset(uint32_t nodeId);
    void emergencyStop(uint32_t nodeId);

    void setVelocity(uint32_t nodeId, int32_t rpm);
    void setPosition(uint32_t nodeId, double degrees, bool absolute = false);
    void setProfileVelocity(uint32_t nodeId, uint32_t rpm);
    void setMaxTorqueLimit(uint32_t nodeId, uint16_t permille);

    void releaseBrake(uint32_t nodeId);
    void lockBrake(uint32_t nodeId);

    // ============ 碰撞保护 ============
    void enableCollisionProtection(uint32_t nodeId, const CollisionConfig &cfg);
    void disableCollisionProtection(uint32_t nodeId);

    // ============ 批量操作 ============
    void enableAll(OperationMode mode);
    void disableAll();
    void emergencyStopAll();
    void faultResetAll();

    // ============ 轮询控制 ============
    void startMonitoring(uint32_t nodeId, int intervalMs = 100);
    void stopMonitoring(uint32_t nodeId);
    void stopAllMonitoring();

signals:
    void motorStateChanged(uint32_t nodeId, const dac::MotorState &state);
    void torqueUpdated(uint32_t nodeId, int16_t torquePermille);
    void collisionDetected(uint32_t nodeId, int16_t torquePermille);
    void motorEnabled(uint32_t nodeId, bool ok);
    void motorDisabled(uint32_t nodeId);
    void faultOccurred(uint32_t nodeId, const QString &faultText);
    void logMessage(dac::LogLevel level, const QString &msg);

private:
    struct NodeInfo {
        uint32_t nodeId;
        QString  name;
        bool     directionInverted = false;
        MotorState state;
        CollisionConfig collisionCfg;
        int      torquePollTaskId = -1;
        int      consecutiveHits = 0;
    };

    void sendCommand(uint32_t nodeId, const CommRequest &req,
                     std::function<void(const CommResult&)> callback = nullptr);
    void clearMotionCommand(uint32_t nodeId);
    void onTorqueResult(uint32_t nodeId, const CommResult &result);

    CommunicationManager *comm_;
    QMap<uint32_t, NodeInfo> nodes_;
};

} // namespace dac
