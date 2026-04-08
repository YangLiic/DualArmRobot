#include "services/MotorService.h"

#include "protocols/CanOpenProtocol.h"

#include <QDateTime>
#include <QTimer>

namespace {

constexpr qint8 VelocityMode = 3;
constexpr qint8 PositionMode = 1;

QString nodeTaskPrefix(quint32 nodeId)
{
    return QStringLiteral("node-%1").arg(nodeId, 0, 16);
}

} // namespace

MotorService::MotorService(CommunicationManager *communicationManager, QObject *parent)
    : QObject(parent)
    , m_communicationManager(communicationManager)
    , m_tableModel(new MotorTableModel(this))
{
    connect(m_communicationManager, &CommunicationManager::requestCompleted, this, &MotorService::handleBusResult);
    connect(m_communicationManager, &CommunicationManager::busConnectionChanged, this, &MotorService::handleBusConnectionChanged);
    connect(m_communicationManager, &CommunicationManager::logMessage, this, &MotorService::logMessage);

    m_busConfig.busId = QStringLiteral("main-can");
    m_busConfig.devicePath = QStringLiteral("/dev/ttyUSB0");
}

MotorService::~MotorService()
{
    stopMonitoring();
}

void MotorService::setBusConfig(const BusConfig &config)
{
    m_busConfig = config;
    m_communicationManager->configureCanBus(config);
}

void MotorService::setMotorConfigs(const QList<MotorConfig> &configs)
{
    const bool restartMonitoring = m_monitoring;
    if (restartMonitoring) {
        stopMonitoring();
    }

    m_motorConfigs = configs;
    m_states.clear();
    m_lastOfflineProbeMs.clear();

    for (const MotorConfig &config : m_motorConfigs) {
        MotorState state;
        state.nodeId = config.nodeId;
        state.name = config.name;
        state.maxTorquePermille = config.maxTorquePermille;
        state.collisionThresholdPermille = config.collisionThresholdPermille;
        m_states.insert(config.nodeId, state);
    }

    if (!m_motorConfigs.isEmpty()) {
        m_focusedNodeId = m_motorConfigs.first().nodeId;
    } else {
        m_focusedNodeId = 0;
    }
    m_focusPollPhase = 0;

    m_tableModel->setMotorConfigs(configs);
    emit configuredNodesChanged(configuredNodeIds());

    if (restartMonitoring) {
        startMonitoring();
    }
}

MotorTableModel *MotorService::tableModel() const
{
    return m_tableModel;
}

QList<quint32> MotorService::configuredNodeIds() const
{
    QList<quint32> nodeIds;
    for (const MotorConfig &config : m_motorConfigs) {
        nodeIds.append(config.nodeId);
    }
    return nodeIds;
}

MotorState MotorService::motorState(quint32 nodeId) const
{
    return stateForNode(nodeId);
}

void MotorService::connectBus()
{
    m_communicationManager->configureCanBus(m_busConfig);
    m_communicationManager->connectBus(m_busConfig.busId);
}

void MotorService::disconnectBus()
{
    stopMonitoring();
    m_communicationManager->disconnectBus(m_busConfig.busId);
}

void MotorService::scanConfiguredNodes()
{
    for (const MotorConfig &config : m_motorConfigs) {
        refreshNode(config.nodeId, RequestPriority::Control);
    }
}

void MotorService::startMonitoring()
{
    if (m_monitoring) {
        return;
    }

    m_monitoring = true;
    resetMonitoringTasks();
    for (const MotorConfig &config : m_motorConfigs) {
        const QString fastTaskId = QStringLiteral("%1-fast").arg(nodeTaskPrefix(config.nodeId));
        const QString slowTaskId = QStringLiteral("%1-slow").arg(nodeTaskPrefix(config.nodeId));

        m_communicationManager->registerPollingTask(fastTaskId, 100, [this, nodeId = config.nodeId]() {
            return buildFastPollRequests(nodeId);
        });
        m_communicationManager->registerPollingTask(slowTaskId, 500, [this, nodeId = config.nodeId]() {
            return buildSlowPollRequests(nodeId);
        });
    }
    m_communicationManager->registerPollingTask(QStringLiteral("focus-telemetry"), 50, [this]() {
        return buildFocusedPollRequests();
    });
    log(QStringLiteral("已启动状态监控"));
}

void MotorService::stopMonitoring()
{
    if (!m_monitoring) {
        return;
    }

    m_monitoring = false;
    resetMonitoringTasks();
    log(QStringLiteral("已停止状态监控"));
}

void MotorService::setFocusedNode(quint32 nodeId)
{
    if (m_focusedNodeId == nodeId) {
        return;
    }

    m_focusedNodeId = nodeId;
    m_focusPollPhase = 0;
    if (nodeId != 0) {
        scheduleRealtimeSnapshot(nodeId, 0, RequestPriority::Control);
    }
}

void MotorService::enableMotor(quint32 nodeId, bool positionMode)
{
    const qint8 mode = positionMode ? PositionMode : VelocityMode;
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0080, RequestPriority::Critical, CommandType::FaultReset, baseContext(nodeId, QStringLiteral("faultReset"), QStringLiteral("预防性故障复位"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0000, RequestPriority::Critical, CommandType::FaultReset, baseContext(nodeId, QStringLiteral("faultReset"), QStringLiteral("清除复位位"))));
    submit(CanOpenProtocol::makeOperationModeRequest(
        m_busConfig.busId, nodeId, mode, RequestPriority::Control, baseContext(nodeId, QStringLiteral("modeCommand"), QStringLiteral("设置模式"))));
    if (!positionMode) {
        submit(CanOpenProtocol::makeVelocityRequest(
            m_busConfig.busId,
            nodeId,
            0,
            RequestPriority::Critical,
            baseContext(nodeId, QStringLiteral("velocityCommand"), QStringLiteral("使能前清零速度目标"))));
    }
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0006, RequestPriority::Control, CommandType::EnableOperation, baseContext(nodeId, QStringLiteral("enableStep"), QStringLiteral("进入 Shutdown"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0007, RequestPriority::Control, CommandType::EnableOperation, baseContext(nodeId, QStringLiteral("enableStep"), QStringLiteral("进入 Switch On"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x000F, RequestPriority::Control, CommandType::EnableOperation, baseContext(nodeId, QStringLiteral("enableStep"), QStringLiteral("进入 Enable Operation"))));
    if (!positionMode) {
        submit(CanOpenProtocol::makeVelocityRequest(
            m_busConfig.busId,
            nodeId,
            0,
            RequestPriority::Critical,
            baseContext(nodeId, QStringLiteral("velocityCommand"), QStringLiteral("使能后再次清零速度目标"))));
    }
    submit(CanOpenProtocol::makeNmtRequest(
        m_busConfig.busId, nodeId, 0x01, CommandType::StartNmt, RequestPriority::Control, baseContext(nodeId, QStringLiteral("nmt"), QStringLiteral("启动 NMT"))));
    refreshNode(nodeId, RequestPriority::Control);
    scheduleRealtimeSnapshot(nodeId, 350, RequestPriority::Control);
}

void MotorService::disableMotor(quint32 nodeId)
{
    submit(CanOpenProtocol::makeVelocityRequest(
        m_busConfig.busId, nodeId, 0, RequestPriority::Critical, baseContext(nodeId, QStringLiteral("velocityCommand"), QStringLiteral("失能前清零速度目标"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0006, RequestPriority::Critical, CommandType::DisableOperation, baseContext(nodeId, QStringLiteral("disable"), QStringLiteral("disable"))));
    refreshNode(nodeId, RequestPriority::Critical);
    scheduleRealtimeSnapshot(nodeId, 150, RequestPriority::Control);
}

void MotorService::quickStopMotor(quint32 nodeId)
{
    submit(CanOpenProtocol::makeVelocityRequest(
        m_busConfig.busId, nodeId, 0, RequestPriority::Critical, baseContext(nodeId, QStringLiteral("velocityCommand"), QStringLiteral("急停前清零速度目标"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0002, RequestPriority::Critical, CommandType::QuickStop, baseContext(nodeId, QStringLiteral("quickStop"), QStringLiteral("quick stop"))));
    refreshNode(nodeId, RequestPriority::Critical);
    scheduleRealtimeSnapshot(nodeId, 150, RequestPriority::Control);
}

void MotorService::faultResetMotor(quint32 nodeId)
{
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0080, RequestPriority::Critical, CommandType::FaultReset, baseContext(nodeId, QStringLiteral("faultReset"), QStringLiteral("fault reset set"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x0000, RequestPriority::Critical, CommandType::FaultReset, baseContext(nodeId, QStringLiteral("faultReset"), QStringLiteral("fault reset clear"))));
    MotorState state = stateForNode(nodeId);
    state.collisionTriggered = false;
    updateState(state);
    refreshNode(nodeId, RequestPriority::Critical);
    scheduleRealtimeSnapshot(nodeId, 180, RequestPriority::Control);
}

void MotorService::setVelocity(quint32 nodeId, qint32 rpm)
{
    const MotorConfig config = configForNode(nodeId);
    const qint32 actualRpm = config.directionInverted ? -rpm : rpm;
    log(QStringLiteral("节点 %1 下发速度命令 %2 RPM").arg(formatCanId(nodeId)).arg(rpm));
    submit(CanOpenProtocol::makeVelocityRequest(
        m_busConfig.busId, nodeId, actualRpm, RequestPriority::Control, baseContext(nodeId, QStringLiteral("velocityCommand"), QStringLiteral("设置速度"))));
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdStatusWord, 0x00, RequestPriority::Control, baseContext(nodeId, QStringLiteral("statusWord"), QStringLiteral("读取状态字")), 150));
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdVelocityActual, 0x00, RequestPriority::Control, baseContext(nodeId, QStringLiteral("velocity"), QStringLiteral("读取实际速度")), 150));
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdActualTorque, 0x00, RequestPriority::Control, baseContext(nodeId, QStringLiteral("torque"), QStringLiteral("读取实际转矩")), 150));
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdPhaseCurrent, CanOpenProtocol::OdPhaseCurrentSubIndex, RequestPriority::Control, baseContext(nodeId, QStringLiteral("current"), QStringLiteral("读取相电流")), 150));
    scheduleRealtimeSnapshot(nodeId, 120, RequestPriority::Control);
}

void MotorService::movePosition(quint32 nodeId, double degrees, bool absolute, quint32 profileVelocity, quint32 acceleration, quint32 deceleration)
{
    const MotorConfig config = configForNode(nodeId);
    const double actualDegrees = config.directionInverted ? -degrees : degrees;

    submit(CanOpenProtocol::makeProfileVelocityRequest(
        m_busConfig.busId, nodeId, profileVelocity, RequestPriority::Control, baseContext(nodeId, QStringLiteral("profileVelocity"), QStringLiteral("profile velocity"))));
    submit(CanOpenProtocol::makeProfileAccelerationRequest(
        m_busConfig.busId, nodeId, acceleration, RequestPriority::Control, baseContext(nodeId, QStringLiteral("profileAcceleration"), QStringLiteral("profile acceleration"))));
    submit(CanOpenProtocol::makeProfileDecelerationRequest(
        m_busConfig.busId, nodeId, deceleration, RequestPriority::Control, baseContext(nodeId, QStringLiteral("profileDeceleration"), QStringLiteral("profile deceleration"))));
    submit(CanOpenProtocol::makePositionRequest(
        m_busConfig.busId, nodeId, actualDegrees, RequestPriority::Control, baseContext(nodeId, QStringLiteral("positionCommand"), absolute ? QStringLiteral("absolute position") : QStringLiteral("relative position"))));
    submit(CanOpenProtocol::makeStartPositionRequest(
        m_busConfig.busId, nodeId, !absolute, RequestPriority::Control, baseContext(nodeId, QStringLiteral("positionStart"), QStringLiteral("start position"))));
    submit(CanOpenProtocol::makeControlWordRequest(
        m_busConfig.busId, nodeId, 0x000F, RequestPriority::Control, CommandType::SetPosition, baseContext(nodeId, QStringLiteral("positionLatchReset"), QStringLiteral("position latch reset"))));
}

void MotorService::enableAll(bool positionMode)
{
    for (const MotorConfig &config : m_motorConfigs) {
        enableMotor(config.nodeId, positionMode);
    }
}

void MotorService::disableAll()
{
    for (const MotorConfig &config : m_motorConfigs) {
        disableMotor(config.nodeId);
    }
}

void MotorService::quickStopAll()
{
    for (const MotorConfig &config : m_motorConfigs) {
        quickStopMotor(config.nodeId);
    }
}

void MotorService::faultResetAll()
{
    for (const MotorConfig &config : m_motorConfigs) {
        faultResetMotor(config.nodeId);
    }
}

void MotorService::handleBusResult(const BusResult &result)
{
    const quint32 nodeId = result.context.value(QStringLiteral("nodeId")).toUInt();
    if (!m_states.contains(nodeId)) {
        return;
    }

    const QString kind = result.context.value(QStringLiteral("kind")).toString();
    const QString action = result.context.value(QStringLiteral("action")).toString();
    MotorState state = stateForNode(nodeId);

    if (!result.success) {
        if (kind == QLatin1String("statusWord")) {
            state.online = false;
            state.enabled = false;
            state.actualVelocityRpm = 0;
            state.torquePermille = 0;
            state.currentAmp = 0.0;
        }
        if (!action.isEmpty()) {
            log(QStringLiteral("%1 %2 failed: %3").arg(state.name, action, result.errorMessage));
        }
        updateState(state);
        return;
    }

    state.online = true;
    state.lastUpdate = QDateTime::currentDateTime();

    if (kind == QLatin1String("statusWord")) {
        quint16 value = 0;
        if (CanOpenProtocol::decodeUInt16(result.rawResponse, value)) {
            state.statusWord = value;
            state.enabled = CanOpenProtocol::isOperationEnabled(value);
            log(QStringLiteral("%1 状态字=0x%2 使能=%3")
                .arg(state.name)
                .arg(value, 4, 16, QLatin1Char('0'))
                .arg(state.enabled ? QStringLiteral("是") : QStringLiteral("否")));
            if (!CanOpenProtocol::isFault(value) && state.faultCode == 0) {
                state.faultText = QStringLiteral("无故障");
            }
        }
    } else if (kind == QLatin1String("mode")) {
        qint8 value = 0;
        if (CanOpenProtocol::decodeInt8(result.rawResponse, value)) {
            state.modeText = CanOpenProtocol::operationModeToText(value);
        }
    } else if (kind == QLatin1String("velocity")) {
        qint32 value = 0;
        if (CanOpenProtocol::decodeInt32(result.rawResponse, value)) {
            qint32 rpm = CanOpenProtocol::encoderUnitsToRpm(value);
            if (configForNode(nodeId).directionInverted) {
                rpm = -rpm;
            }
            state.actualVelocityRpm = rpm;
        }
    } else if (kind == QLatin1String("position")) {
        qint32 value = 0;
        if (CanOpenProtocol::decodeInt32(result.rawResponse, value)) {
            double degrees = CanOpenProtocol::pulsesToDegrees(value);
            if (configForNode(nodeId).directionInverted) {
                degrees = -degrees;
            }
            state.actualPositionDeg = degrees;
        }
    } else if (kind == QLatin1String("torque")) {
        qint16 value = 0;
        if (CanOpenProtocol::decodeInt16(result.rawResponse, value)) {
            state.torquePermille = value;
        }
    } else if (kind == QLatin1String("current")) {
        quint16 value = 0;
        if (CanOpenProtocol::decodeUInt16(result.rawResponse, value)) {
            state.currentAmp = static_cast<double>(value) / 100.0;
        }
    } else if (kind == QLatin1String("fault")) {
        quint16 value = 0;
        if (CanOpenProtocol::decodeUInt16(result.rawResponse, value)) {
            state.faultCode = value;
            state.faultText = CanOpenProtocol::faultCodeToText(value);
            if (value == 0) {
                state.collisionTriggered = false;
            }
        }
    } else if (kind == QLatin1String("positionDeviation")) {
        qint32 value = 0;
        if (CanOpenProtocol::decodeInt32(result.rawResponse, value)) {
            state.collisionTriggered = qAbs(value) > 100000;
        }
    }

    if (!action.isEmpty() && kind != QLatin1String("statusWord")) {
        log(QStringLiteral("%1 %2 ok").arg(state.name, action));
    }

    updateState(state);
}

void MotorService::handleBusConnectionChanged(const QString &busId, bool connected, const QString &message)
{
    if (busId != m_busConfig.busId) {
        return;
    }

    m_busConnected = connected;
    if (!connected) {
        m_tableModel->setAllOffline();
    }

    emit busConnectionChanged(connected, message);
}

QList<BusRequest> MotorService::buildFastPollRequests(quint32 nodeId) const
{
    const QString prefix = QStringLiteral("%1-fast").arg(nodeTaskPrefix(nodeId));
    const MotorState state = stateForNode(nodeId);
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();

    if (!state.online) {
        if (!shouldProbeOfflineNode(nodeId, nowMs, 1000)) {
            return {};
        }
        return {
            CanOpenProtocol::makeReadSdoRequest(
                m_busConfig.busId,
                nodeId,
                CanOpenProtocol::OdStatusWord,
                0x00,
                RequestPriority::Monitoring,
                baseContext(nodeId, QStringLiteral("statusWord")),
                60,
                prefix + QStringLiteral("-probe-status"))
        };
    }

    if (nodeId == m_focusedNodeId) {
        return {};
    }

    return {
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdStatusWord, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("statusWord")), 80, prefix + QStringLiteral("-status")),
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdVelocityActual, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("velocity")), 80, prefix + QStringLiteral("-velocity")),
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdActualTorque, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("torque")), 80, prefix + QStringLiteral("-torque")),
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdPhaseCurrent, CanOpenProtocol::OdPhaseCurrentSubIndex, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("current")), 80, prefix + QStringLiteral("-current"))
    };
}

QList<BusRequest> MotorService::buildSlowPollRequests(quint32 nodeId) const
{
    const QString prefix = QStringLiteral("%1-slow").arg(nodeTaskPrefix(nodeId));
    if (!stateForNode(nodeId).online) {
        return {};
    }
    return {
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdModeDisplay, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("mode")), 100, prefix + QStringLiteral("-mode")),
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdPositionActual, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("position")), 100, prefix + QStringLiteral("-position")),
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdErrorCode, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("fault")), 100, prefix + QStringLiteral("-fault")),
        CanOpenProtocol::makeReadSdoRequest(m_busConfig.busId, nodeId, CanOpenProtocol::OdPositionDeviation, 0x00, RequestPriority::Monitoring, baseContext(nodeId, QStringLiteral("positionDeviation")), 100, prefix + QStringLiteral("-deviation"))
    };
}

QList<BusRequest> MotorService::buildFocusedPollRequests() const
{
    if (m_focusedNodeId == 0) {
        return {};
    }

    const MotorState state = stateForNode(m_focusedNodeId);
    if (!state.online) {
        return {};
    }

    const QString prefix = QStringLiteral("focus-%1").arg(nodeTaskPrefix(m_focusedNodeId));
    const quint32 phase = m_focusPollPhase++ % 4;

    switch (phase) {
    case 0:
        return {
            CanOpenProtocol::makeReadSdoRequest(
                m_busConfig.busId,
                m_focusedNodeId,
                CanOpenProtocol::OdActualTorque,
                0x00,
                RequestPriority::Control,
                baseContext(m_focusedNodeId, QStringLiteral("torque")),
                70,
                prefix + QStringLiteral("-torque-a"))
        };
    case 1:
        return {
            CanOpenProtocol::makeReadSdoRequest(
                m_busConfig.busId,
                m_focusedNodeId,
                CanOpenProtocol::OdVelocityActual,
                0x00,
                RequestPriority::Control,
                baseContext(m_focusedNodeId, QStringLiteral("velocity")),
                70,
                prefix + QStringLiteral("-velocity"))
        };
    case 2:
        return {
            CanOpenProtocol::makeReadSdoRequest(
                m_busConfig.busId,
                m_focusedNodeId,
                CanOpenProtocol::OdActualTorque,
                0x00,
                RequestPriority::Control,
                baseContext(m_focusedNodeId, QStringLiteral("torque")),
                70,
                prefix + QStringLiteral("-torque-b"))
        };
    default:
        return {
            CanOpenProtocol::makeReadSdoRequest(
                m_busConfig.busId,
                m_focusedNodeId,
                CanOpenProtocol::OdStatusWord,
                0x00,
                RequestPriority::Control,
                baseContext(m_focusedNodeId, QStringLiteral("statusWord")),
                70,
                prefix + QStringLiteral("-status"))
        };
    }
}

bool MotorService::shouldProbeOfflineNode(quint32 nodeId, qint64 nowMs, int minIntervalMs) const
{
    const qint64 lastProbeMs = m_lastOfflineProbeMs.value(nodeId, 0);
    if (lastProbeMs > 0 && nowMs - lastProbeMs < minIntervalMs) {
        return false;
    }

    m_lastOfflineProbeMs.insert(nodeId, nowMs);
    return true;
}

void MotorService::scheduleRealtimeSnapshot(quint32 nodeId, int delayMs, RequestPriority priority)
{
    QTimer::singleShot(delayMs, this, [this, nodeId, priority]() {
        if (!m_busConnected || !m_states.contains(nodeId)) {
            return;
        }

        refreshNode(nodeId, priority);
        submit(CanOpenProtocol::makeReadSdoRequest(
            m_busConfig.busId, nodeId, CanOpenProtocol::OdVelocityActual, 0x00, priority, baseContext(nodeId, QStringLiteral("velocity"), QStringLiteral("补读实际速度")), 120));
        submit(CanOpenProtocol::makeReadSdoRequest(
            m_busConfig.busId, nodeId, CanOpenProtocol::OdActualTorque, 0x00, priority, baseContext(nodeId, QStringLiteral("torque"), QStringLiteral("补读实际转矩")), 120));
        submit(CanOpenProtocol::makeReadSdoRequest(
            m_busConfig.busId, nodeId, CanOpenProtocol::OdPhaseCurrent, CanOpenProtocol::OdPhaseCurrentSubIndex, priority, baseContext(nodeId, QStringLiteral("current"), QStringLiteral("补读相电流")), 120));
    });
}

QVariantMap MotorService::baseContext(quint32 nodeId, const QString &kind, const QString &action) const
{
    QVariantMap context;
    context.insert(QStringLiteral("nodeId"), nodeId);
    context.insert(QStringLiteral("kind"), kind);
    if (!action.isEmpty()) {
        context.insert(QStringLiteral("action"), action);
    }
    return context;
}

MotorConfig MotorService::configForNode(quint32 nodeId) const
{
    for (const MotorConfig &config : m_motorConfigs) {
        if (config.nodeId == nodeId) {
            return config;
        }
    }
    return {};
}

MotorState MotorService::stateForNode(quint32 nodeId) const
{
    return m_states.value(nodeId);
}

void MotorService::updateState(const MotorState &state)
{
    m_states.insert(state.nodeId, state);
    m_tableModel->updateState(state);
    emit motorStateChanged(state);
}

void MotorService::log(const QString &message)
{
    emit logMessage(QStringLiteral("[MotorService] %1").arg(message));
}

void MotorService::submit(const BusRequest &request)
{
    m_communicationManager->submitRequest(request);
}

void MotorService::refreshNode(quint32 nodeId, RequestPriority priority)
{
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdStatusWord, 0x00, priority, baseContext(nodeId, QStringLiteral("statusWord"))));
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdModeDisplay, 0x00, priority, baseContext(nodeId, QStringLiteral("mode"))));
    submit(CanOpenProtocol::makeReadSdoRequest(
        m_busConfig.busId, nodeId, CanOpenProtocol::OdErrorCode, 0x00, priority, baseContext(nodeId, QStringLiteral("fault"))));
}

void MotorService::resetMonitoringTasks()
{
    for (const MotorConfig &config : m_motorConfigs) {
        m_communicationManager->unregisterPollingTask(nodeTaskPrefix(config.nodeId));
    }
    m_communicationManager->unregisterPollingTask(QStringLiteral("focus-telemetry"));
}
