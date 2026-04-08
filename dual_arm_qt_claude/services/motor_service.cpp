/**
 * MotorService 实现
 */
#include "motor_service.h"
#include <QTimer>
#include <QDebug>

namespace dar {

MotorService::MotorService(CommunicationManager *commMgr, QObject *parent)
    : QObject(parent)
    , commMgr_(commMgr)
    , protocol_(new CanopenMotorProtocol(this))
{
    connect(commMgr_, &CommunicationManager::requestCompleted,
            this, &MotorService::onRequestCompleted);
    connect(commMgr_, &CommunicationManager::frameReceived,
            this, &MotorService::onFrameReceived);
}

MotorService::~MotorService()
{
}

// ==================== 节点管理 ====================

void MotorService::addMotor(const QString &busId, uint32_t nodeId)
{
    if (motors_.contains(nodeId)) return;

    MotorEntry entry;
    entry.busId = busId;
    entry.state.nodeId = nodeId;
    entry.state.online = false;
    entry.state.enabled = false;
    entry.state.lastUpdateTime = QDateTime::currentDateTime();

    motors_[nodeId] = entry;

    log(LogEntry::Info, QStringLiteral("添加电机节点: 0x%1 -> 总线 %2")
        .arg(nodeId, 3, 16, QChar('0')).arg(busId));

    emit motorStateChanged(nodeId, entry.state);
}

void MotorService::removeMotor(uint32_t nodeId)
{
    if (!motors_.contains(nodeId)) return;

    stopMonitoring(nodeId);
    motors_.remove(nodeId);

    log(LogEntry::Info, QStringLiteral("移除电机节点: 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

QList<uint32_t> MotorService::motorNodeIds() const
{
    return motors_.keys();
}

MotorState MotorService::motorState(uint32_t nodeId) const
{
    auto it = motors_.find(nodeId);
    if (it != motors_.end()) return it->state;
    return MotorState();
}

QMap<uint32_t, MotorState> MotorService::allMotorStates() const
{
    QMap<uint32_t, MotorState> result;
    for (auto it = motors_.begin(); it != motors_.end(); ++it) {
        result[it.key()] = it->state;
    }
    return result;
}

QString MotorService::busIdForMotor(uint32_t nodeId) const
{
    auto it = motors_.find(nodeId);
    if (it != motors_.end()) return it->busId;
    return QString();
}

// ==================== 单节点控制 ====================

void MotorService::startCommunication(uint32_t nodeId)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    if (!commMgr_->isBusConnected(busId)) {
        commMgr_->connectBus(busId);
    }

    // 尝试读取状态字来确认设备在线
    CommRequest req = protocol_->makeReadStatusWordRequest(busId, nodeId);
    pendingRequests_[req.requestId] = {nodeId, "startComm_probe"};
    commMgr_->submitRequest(req);

    log(LogEntry::Info, QStringLiteral("开始通信: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::stopCommunication(uint32_t nodeId)
{
    stopMonitoring(nodeId);

    auto &motor = motors_[nodeId];
    motor.state.online = false;
    motor.state.enabled = false;
    motor.state.lastUpdateTime = QDateTime::currentDateTime();

    emit motorStateChanged(nodeId, motor.state);
    emit motorOnlineChanged(nodeId, false);

    log(LogEntry::Info, QStringLiteral("停止通信: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::enableMotor(uint32_t nodeId, OperationMode mode)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    log(LogEntry::Info, QStringLiteral("使能电机: 节点 0x%1, 模式=%2")
        .arg(nodeId, 3, 16, QChar('0'))
        .arg(mode == OperationMode::Velocity ? "速度" : "位置"));

    // 启动使能状态机
    sendEnableSequence(nodeId, mode, 0);
}

void MotorService::sendEnableSequence(uint32_t nodeId, OperationMode mode, int step)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req;

    switch (step) {
    case 0: {
        // Step 0: 设置运行模式
        uint8_t modeVal = static_cast<uint8_t>(mode);
        req = protocol_->makeWriteSDO(busId, nodeId,
                                       CanopenMotorProtocol::OD_OPERATION_MODE, 0x00,
                                       &modeVal, 1, Priority::Control, false);
        break;
    }
    case 1: {
        // Step 1: Shutdown (0x0006)
        uint8_t data[2] = {0x06, 0x00};
        req = protocol_->makeWriteSDO(busId, nodeId,
                                       CanopenMotorProtocol::OD_CONTROL_WORD, 0x00,
                                       data, 2, Priority::Control, false);
        break;
    }
    case 2: {
        // Step 2: Switch On (0x0007)
        uint8_t data[2] = {0x07, 0x00};
        req = protocol_->makeWriteSDO(busId, nodeId,
                                       CanopenMotorProtocol::OD_CONTROL_WORD, 0x00,
                                       data, 2, Priority::Control, false);
        break;
    }
    case 3: {
        // Step 3: Enable Operation (0x000F)
        uint8_t data[2] = {0x0F, 0x00};
        req = protocol_->makeWriteSDO(busId, nodeId,
                                       CanopenMotorProtocol::OD_CONTROL_WORD, 0x00,
                                       data, 2, Priority::Control, false);
        break;
    }
    case 4: {
        // Step 4: NMT Start
        req = protocol_->makeNmtStartRequest(busId);
        break;
    }
    case 5: {
        // 使能完成
        auto &motor = motors_[nodeId];
        motor.state.enabled = true;
        motor.state.mode = mode;
        motor.state.lastUpdateTime = QDateTime::currentDateTime();

        emit motorStateChanged(nodeId, motor.state);
        emit motorEnabled(nodeId);
        enableStates_.remove(nodeId);

        log(LogEntry::Info, QStringLiteral("✅ 电机已使能: 节点 0x%1")
            .arg(nodeId, 3, 16, QChar('0')));
        return;
    }
    default:
        return;
    }

    EnableState es;
    es.mode = mode;
    es.step = step;
    es.lastRequestId = req.requestId;
    enableStates_[nodeId] = es;

    pendingRequests_[req.requestId] = {nodeId, QStringLiteral("enable_step%1").arg(step)};
    commMgr_->submitRequest(req);

    // 步骤之间加延时，使用 QTimer::singleShot 自动推进
    QTimer::singleShot(100, this, [this, nodeId, mode, step]() {
        if (enableStates_.contains(nodeId) && enableStates_[nodeId].step == step) {
            sendEnableSequence(nodeId, mode, step + 1);
        }
    });
}

void MotorService::disableMotor(uint32_t nodeId)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req = protocol_->makeDisableRequest(busId, nodeId);
    pendingRequests_[req.requestId] = {nodeId, "disable"};
    commMgr_->submitRequest(req);

    auto &motor = motors_[nodeId];
    motor.state.enabled = false;
    motor.state.lastUpdateTime = QDateTime::currentDateTime();

    emit motorStateChanged(nodeId, motor.state);
    emit motorDisabled(nodeId);

    log(LogEntry::Info, QStringLiteral("失能电机: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::quickStopMotor(uint32_t nodeId)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req = protocol_->makeQuickStopRequest(busId, nodeId);
    pendingRequests_[req.requestId] = {nodeId, "quickStop"};
    commMgr_->submitRequest(req);

    log(LogEntry::Warning, QStringLiteral("⚠️ 急停: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::faultResetMotor(uint32_t nodeId)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req = protocol_->makeFaultResetRequest(busId, nodeId);
    pendingRequests_[req.requestId] = {nodeId, "faultReset"};
    commMgr_->submitRequest(req);

    // 复位后清除故障位
    QTimer::singleShot(100, this, [this, nodeId]() {
        auto busId = busIdForMotor(nodeId);
        if (busId.isEmpty()) return;
        uint8_t data[2] = {0x00, 0x00};
        CommRequest req2 = protocol_->makeWriteSDO(busId, nodeId,
            CanopenMotorProtocol::OD_CONTROL_WORD, 0x00, data, 2,
            Priority::Critical, false);
        commMgr_->submitRequest(req2);

        auto &motor = motors_[nodeId];
        motor.state.faultCode = 0;
        motor.state.faultText.clear();
        motor.state.enabled = false;
        motor.state.lastUpdateTime = QDateTime::currentDateTime();
        emit motorStateChanged(nodeId, motor.state);
    });

    log(LogEntry::Info, QStringLiteral("故障复位: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::setVelocity(uint32_t nodeId, int32_t rpm)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req = protocol_->makeSetVelocityRequest(busId, nodeId, rpm);
    pendingRequests_[req.requestId] = {nodeId, "setVelocity"};
    commMgr_->submitRequest(req);
}

void MotorService::setPosition(uint32_t nodeId, double degrees, bool absolute)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req = protocol_->makeSetPositionRequest(busId, nodeId, degrees, absolute);
    pendingRequests_[req.requestId] = {nodeId, "setPosition"};
    commMgr_->submitRequest(req);

    // 自动开始运动
    QTimer::singleShot(50, this, [this, nodeId, absolute]() {
        startPositionMove(nodeId, !absolute);
    });
}

void MotorService::startPositionMove(uint32_t nodeId, bool relative)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;

    CommRequest req = protocol_->makeStartPositionMoveRequest(busId, nodeId, relative);
    commMgr_->submitRequest(req);

    // 复位 bit4
    QTimer::singleShot(50, this, [this, nodeId]() {
        auto busId = busIdForMotor(nodeId);
        if (busId.isEmpty()) return;
        uint8_t data[2] = {0x0F, 0x00};
        CommRequest req2 = protocol_->makeWriteSDO(busId, nodeId,
            CanopenMotorProtocol::OD_CONTROL_WORD, 0x00, data, 2,
            Priority::Control, false);
        commMgr_->submitRequest(req2);
    });
}

void MotorService::setProfileVelocity(uint32_t nodeId, uint32_t rpm)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;
    commMgr_->submitRequest(protocol_->makeSetProfileVelocityRequest(busId, nodeId, rpm));
}

void MotorService::setProfileAcceleration(uint32_t nodeId, uint32_t acc)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;
    commMgr_->submitRequest(protocol_->makeSetProfileAccelerationRequest(busId, nodeId, acc));
}

void MotorService::setProfileDeceleration(uint32_t nodeId, uint32_t dec)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;
    commMgr_->submitRequest(protocol_->makeSetProfileDecelerationRequest(busId, nodeId, dec));
}

void MotorService::setMaxTorque(uint32_t nodeId, uint16_t permille)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;
    commMgr_->submitRequest(protocol_->makeSetMaxTorqueRequest(busId, nodeId, permille));
}

void MotorService::releaseBrake(uint32_t nodeId)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;
    commMgr_->submitRequest(protocol_->makeReleaseBrakeRequest(busId, nodeId));
    log(LogEntry::Warning, QStringLiteral("🔓 松闸: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::lockBrake(uint32_t nodeId)
{
    auto busId = busIdForMotor(nodeId);
    if (busId.isEmpty()) return;
    commMgr_->submitRequest(protocol_->makeLockBrakeRequest(busId, nodeId));
    log(LogEntry::Info, QStringLiteral("🔒 锁闸: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

// ==================== 批量操作 ====================

void MotorService::enableAllMotors(OperationMode mode)
{
    for (auto nodeId : motors_.keys()) {
        enableMotor(nodeId, mode);
    }
}

void MotorService::disableAllMotors()
{
    for (auto nodeId : motors_.keys()) {
        disableMotor(nodeId);
    }
}

void MotorService::quickStopAllMotors()
{
    for (auto nodeId : motors_.keys()) {
        quickStopMotor(nodeId);
    }
}

void MotorService::faultResetAllMotors()
{
    for (auto nodeId : motors_.keys()) {
        faultResetMotor(nodeId);
    }
}

// ==================== 监控 ====================

void MotorService::startMonitoring(uint32_t nodeId)
{
    auto it = motors_.find(nodeId);
    if (it == motors_.end()) return;

    auto &motor = *it;
    if (motor.monitoring) return;

    motor.monitoring = true;

    // 注册轮询任务
    PollTask torqueTask;
    torqueTask.taskId = QStringLiteral("torque_%1").arg(nodeId, 3, 16, QChar('0'));
    torqueTask.nodeId = nodeId;
    torqueTask.paramName = "torque";
    torqueTask.intervalMs = 100;
    commMgr_->registerPollTask(motor.busId, torqueTask);

    PollTask statusTask;
    statusTask.taskId = QStringLiteral("status_%1").arg(nodeId, 3, 16, QChar('0'));
    statusTask.nodeId = nodeId;
    statusTask.paramName = "statusWord";
    statusTask.intervalMs = 200;
    commMgr_->registerPollTask(motor.busId, statusTask);

    log(LogEntry::Info, QStringLiteral("开始监控: 节点 0x%1")
        .arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::stopMonitoring(uint32_t nodeId)
{
    auto it = motors_.find(nodeId);
    if (it == motors_.end()) return;

    auto &motor = *it;
    motor.monitoring = false;

    commMgr_->unregisterPollTask(motor.busId,
        QStringLiteral("torque_%1").arg(nodeId, 3, 16, QChar('0')));
    commMgr_->unregisterPollTask(motor.busId,
        QStringLiteral("status_%1").arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::scanOnlineNodes()
{
    log(LogEntry::Info, QStringLiteral("开始扫描在线节点..."));

    // 对 0x601 ~ 0x610 发送状态字读取探测
    for (uint32_t nodeId = 0x601; nodeId <= 0x610; ++nodeId) {
        // 找到一个已注册的 busId
        QString busId;
        for (auto it = motors_.begin(); it != motors_.end(); ++it) {
            busId = it->busId;
            break;
        }
        if (busId.isEmpty()) {
            // 没有已注册的电机，使用第一个总线
            auto ids = commMgr_->busIds();
            if (!ids.isEmpty()) busId = ids.first();
        }
        if (busId.isEmpty()) break;

        CommRequest req = protocol_->makeReadStatusWordRequest(busId, nodeId);
        req.timeoutMs = 100;
        pendingRequests_[req.requestId] = {nodeId, "scan_probe"};
        commMgr_->submitRequest(req);
    }
}

// ==================== 响应处理 ====================

void MotorService::onRequestCompleted(const dar::CommResult &result)
{
    auto it = pendingRequests_.find(result.requestId);
    if (it == pendingRequests_.end()) return;

    auto pending = it.value();
    pendingRequests_.erase(it);

    uint32_t nodeId = pending.nodeId;
    auto motorIt = motors_.find(nodeId);

    if (pending.commandType == "startComm_probe" ||
        pending.commandType == "scan_probe") {
        if (result.success && motorIt != motors_.end()) {
            motorIt->state.online = true;
            motorIt->state.lastUpdateTime = QDateTime::currentDateTime();

            // 解析状态字
            uint8_t data[2];
            if (CanopenMotorProtocol::parseReadSDOResponse(result.responseFrame, data, 2)) {
                uint16_t sw = static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8);
                motorIt->state.statusWord = sw;
                motorIt->state.enabled = CanopenMotorProtocol::isEnabled(sw);
                if (CanopenMotorProtocol::isInFault(sw)) {
                    motorIt->state.faultCode = 1;
                    motorIt->state.faultText = CanopenMotorProtocol::decodeStatusWord(sw);
                }
            }

            emit motorStateChanged(nodeId, motorIt->state);
            emit motorOnlineChanged(nodeId, true);
        }
    }

    // 处理轮询响应
    if (pending.commandType.startsWith("poll_") && result.success && motorIt != motors_.end()) {
        uint8_t data[4];
        if (pending.commandType.contains("torque")) {
            if (CanopenMotorProtocol::parseReadSDOResponse(result.responseFrame, data, 2)) {
                motorIt->state.torque = static_cast<int16_t>(data[0] | (data[1] << 8));
                motorIt->state.lastUpdateTime = QDateTime::currentDateTime();
                emit motorStateChanged(nodeId, motorIt->state);
            }
        } else if (pending.commandType.contains("statusWord")) {
            if (CanopenMotorProtocol::parseReadSDOResponse(result.responseFrame, data, 2)) {
                uint16_t sw = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
                motorIt->state.statusWord = sw;
                bool wasEnabled = motorIt->state.enabled;
                motorIt->state.enabled = CanopenMotorProtocol::isEnabled(sw);

                bool wasFault = (motorIt->state.faultCode != 0);
                if (CanopenMotorProtocol::isInFault(sw)) {
                    motorIt->state.faultCode = 1;
                    motorIt->state.faultText = QStringLiteral("故障");
                    if (!wasFault) {
                        emit motorFaultOccurred(nodeId, 1, motorIt->state.faultText);
                    }
                } else {
                    motorIt->state.faultCode = 0;
                    motorIt->state.faultText.clear();
                }

                if (wasEnabled != motorIt->state.enabled) {
                    if (motorIt->state.enabled) emit motorEnabled(nodeId);
                    else emit motorDisabled(nodeId);
                }

                motorIt->state.lastUpdateTime = QDateTime::currentDateTime();
                emit motorStateChanged(nodeId, motorIt->state);
            }
        }
    }
}

void MotorService::onFrameReceived(const QString &busId, const dar::CanFrame &frame)
{
    Q_UNUSED(busId);
    // 可以用来处理 Emergency 帧或 PDO 帧
    // Emergency: CAN ID = 0x80 + node_id_offset
    // 暂时留空，后续扩展
    Q_UNUSED(frame);
}

void MotorService::log(LogEntry::Level level, const QString &msg)
{
    LogEntry entry;
    entry.level = level;
    entry.timestamp = QDateTime::currentDateTime();
    entry.source = QStringLiteral("MotorService");
    entry.message = msg;
    emit logMessage(entry);
}

}  // namespace dar
