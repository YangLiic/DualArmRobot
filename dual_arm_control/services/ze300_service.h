#pragma once
#include "models/common_types.h"
#include "communication/communication_manager.h"
#include <QObject>
#include <QMap>

namespace dac {

/*
 * Ze300Service: ZE300 电机业务服务层。
 *
 * 对 GUI 暴露 ZE300 设备能力，不暴露底层协议细节。
 * 维护各 ZE300 节点状态缓存。通过 signal 通知 UI 层。
 */
class Ze300Service : public QObject
{
    Q_OBJECT
public:
    explicit Ze300Service(CommunicationManager *comm, QObject *parent = nullptr);

    // 节点管理
    void addNode(uint16_t devAddr, const QString &name, bool useHostAddr = true);
    void removeNode(uint16_t devAddr);
    QList<uint16_t> nodeAddrs() const;
    QString nodeName(uint16_t devAddr) const;

    // 状态查询
    Ze300Status motorStatus(uint16_t devAddr) const;

    // ============ 控制指令 ============
    void setSpeedRpm(uint16_t devAddr, float rpm);
    void setAbsolutePositionDeg(uint16_t devAddr, float deg);
    void setRelativePositionDeg(uint16_t devAddr, float deg);
    void setTorqueCurrentA(uint16_t devAddr, float currentA);
    void goOriginShortest(uint16_t devAddr);
    void setZero(uint16_t devAddr);
    void freeOutput(uint16_t devAddr);
    void clearFault(uint16_t devAddr);
    void reboot(uint16_t devAddr);

    // 抱闸
    void setBrakeClosed(uint16_t devAddr, bool closed);
    void readBrakeState(uint16_t devAddr);

    // 参数
    void setPositionMaxSpeedRpm(uint16_t devAddr, float rpm);
    void setMaxCurrentA(uint16_t devAddr, float currentA);
    void setSpeedAccelerationRpmPerSec(uint16_t devAddr, float accRpmS);

    // MIT 运控
    void sendMitControl(uint16_t devAddr, float posRad, float velRadS,
                        float kp, float kd, float torqueNm);

    // ============ 轮询 ============
    void startMonitoring(uint16_t devAddr, int intervalMs = 200);
    void stopMonitoring(uint16_t devAddr);
    void stopAllMonitoring();

    // ============ 批量 ============
    void freeOutputAll();

signals:
    void ze300StateChanged(uint16_t devAddr, const dac::Ze300Status &status);
    void ze300FaultDetected(uint16_t devAddr, uint8_t faultCode);
    void ze300BrakeStateRead(uint16_t devAddr, bool closed);
    void ze300CommandResult(uint16_t devAddr, const QString &cmd, bool ok, const QString &msg);
    void logMessage(dac::LogLevel level, const QString &msg);

private:
    struct NodeInfo {
        uint16_t devAddr = 0;
        QString  name;
        bool     useHostAddr = true;
        Ze300Status status;
        QList<int> pollTaskIds;
    };

    void sendCommand(uint16_t devAddr, const CommRequest &req,
                     std::function<void(const CommResult&)> callback = nullptr);
    void onStatusResult(uint16_t devAddr, const CommResult &result);
    void emitStateUpdate(uint16_t devAddr, const Ze300Status &previousStatus);
    void registerMonitorTask(NodeInfo &node, const CommRequest &req, int intervalMs);
    void requestMonitorSnapshot(uint16_t devAddr);

    CommunicationManager *comm_;
    QMap<uint16_t, NodeInfo> nodes_;

    static constexpr float COUNT_PER_REV = 16384.0f;
    static constexpr float DEG_PER_COUNT = 360.0f / COUNT_PER_REV;
};

} // namespace dac
