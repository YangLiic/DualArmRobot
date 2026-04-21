#pragma once
#include "models/common_types.h"
#include "communication/communication_manager.h"
#include <QObject>
#include <QMap>

namespace dac {

class CyberGearService : public QObject
{
    Q_OBJECT
public:
    explicit CyberGearService(CommunicationManager *comm, QObject *parent = nullptr);

    static constexpr uint8_t kDefaultMasterId = 0x00;
    static constexpr float kDefaultCurrentLimitA = 5.0f;
    static constexpr float kDefaultSpeedLimitRadS = 0.5f;

    void addNode(uint8_t motorId, const QString &name, uint8_t masterId = kDefaultMasterId);
    void removeNode(uint8_t motorId);
    QList<uint8_t> motorIds() const;
    QString nodeName(uint8_t motorId) const;
    CyberGearStatus motorStatus(uint8_t motorId) const;

    // 控制
    void enable(uint8_t motorId);
    void stop(uint8_t motorId);
    void setRunMode(uint8_t motorId, CgRunMode mode);
    void enableSpeedMode(uint8_t motorId);
    void enablePositionMode(uint8_t motorId);
    void setSpeedRadS(uint8_t motorId, float radS);
    void setSpeedRPM(uint8_t motorId, float rpm);
    void setPositionRad(uint8_t motorId, float rad);
    void setPositionDeg(uint8_t motorId, float deg);
    void setCurrentLimit(uint8_t motorId, float amps);
    void setSpeedLimit(uint8_t motorId, float radS);
    void setMechZero(uint8_t motorId);
    void goToZero(uint8_t motorId);

    // 轮询
    void startMonitoring(uint8_t motorId, int intervalMs = 100);
    void stopMonitoring(uint8_t motorId);
    void stopAllMonitoring();

    // 批量
    void stopAll();

signals:
    void cgStateChanged(uint8_t motorId, const dac::CyberGearStatus &status);
    void cgFaultDetected(uint8_t motorId);
    void cgCommandResult(uint8_t motorId, const QString &cmd, bool ok, const QString &msg);
    void logMessage(dac::LogLevel level, const QString &msg);

private:
    struct NodeInfo {
        uint8_t motorId = 0x02;
        uint8_t masterId = kDefaultMasterId;
        QString name;
        CyberGearStatus status;
        float configuredCurrentLimitA = kDefaultCurrentLimitA;
        float configuredSpeedLimitRadS = kDefaultSpeedLimitRadS;
        float lastAppliedSpeedRadS = 0.0f;
        int pollTaskId = -1;
    };

    void sendCmd(uint8_t motorId, const CommRequest &req,
                 std::function<void(const CommResult&)> cb = nullptr);
    void onFeedback(uint8_t motorId, const CommResult &result);
    void applyCachedSpeedLimit(uint8_t motorId);
    void applyCachedLimits(uint8_t motorId);

    CommunicationManager *comm_;
    QMap<uint8_t, NodeInfo> nodes_;
};

} // namespace dac
