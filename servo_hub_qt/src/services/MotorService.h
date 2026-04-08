#pragma once

#include "communication/CommunicationManager.h"
#include "models/MotorTableModel.h"

#include <QHash>
#include <QDateTime>
#include <QObject>

class MotorService : public QObject
{
    Q_OBJECT

public:
    explicit MotorService(CommunicationManager *communicationManager, QObject *parent = nullptr);
    ~MotorService() override;

    void setBusConfig(const BusConfig &config);
    void setMotorConfigs(const QList<MotorConfig> &configs);
    MotorTableModel *tableModel() const;
    QList<quint32> configuredNodeIds() const;
    MotorState motorState(quint32 nodeId) const;

public slots:
    void connectBus();
    void disconnectBus();
    void scanConfiguredNodes();
    void startMonitoring();
    void stopMonitoring();
    void setFocusedNode(quint32 nodeId);

    void enableMotor(quint32 nodeId, bool positionMode);
    void disableMotor(quint32 nodeId);
    void quickStopMotor(quint32 nodeId);
    void faultResetMotor(quint32 nodeId);
    void setVelocity(quint32 nodeId, qint32 rpm);
    void movePosition(quint32 nodeId, double degrees, bool absolute, quint32 profileVelocity, quint32 acceleration, quint32 deceleration);

    void enableAll(bool positionMode);
    void disableAll();
    void quickStopAll();
    void faultResetAll();

signals:
    void logMessage(const QString &message);
    void busConnectionChanged(bool connected, const QString &message);
    void configuredNodesChanged(const QList<quint32> &nodeIds);
    void motorStateChanged(const MotorState &state);

private slots:
    void handleBusResult(const BusResult &result);
    void handleBusConnectionChanged(const QString &busId, bool connected, const QString &message);

private:
    QList<BusRequest> buildFastPollRequests(quint32 nodeId) const;
    QList<BusRequest> buildSlowPollRequests(quint32 nodeId) const;
    QList<BusRequest> buildFocusedPollRequests() const;
    bool shouldProbeOfflineNode(quint32 nodeId, qint64 nowMs, int minIntervalMs) const;
    void scheduleRealtimeSnapshot(quint32 nodeId, int delayMs, RequestPriority priority = RequestPriority::Control);
    QVariantMap baseContext(quint32 nodeId, const QString &kind, const QString &action = {}) const;
    MotorConfig configForNode(quint32 nodeId) const;
    MotorState stateForNode(quint32 nodeId) const;
    void updateState(const MotorState &state);
    void log(const QString &message);
    void submit(const BusRequest &request);
    void refreshNode(quint32 nodeId, RequestPriority priority = RequestPriority::Control);
    void resetMonitoringTasks();

    CommunicationManager *m_communicationManager = nullptr;
    MotorTableModel *m_tableModel = nullptr;
    BusConfig m_busConfig;
    QList<MotorConfig> m_motorConfigs;
    QHash<quint32, MotorState> m_states;
    mutable QHash<quint32, qint64> m_lastOfflineProbeMs;
    quint32 m_focusedNodeId = 0;
    mutable quint32 m_focusPollPhase = 0;
    bool m_busConnected = false;
    bool m_monitoring = false;
};
