#include "services/motor_service.h"
#include "protocols/canopen_motor_protocol.h"
#include <QDateTime>
#include <cmath>

namespace dac {

MotorService::MotorService(CommunicationManager *comm, QObject *parent)
    : QObject(parent), comm_(comm)
{}

// ==================== 节点管理 ====================

void MotorService::addNode(uint32_t nodeId, const QString &name, bool directionInverted)
{
    NodeInfo info;
    info.nodeId = nodeId;
    info.name = name;
    info.directionInverted = directionInverted;
    info.state.nodeId = nodeId;
    nodes_[nodeId] = info;
    emit logMessage(LogLevel::Info, QStringLiteral("添加电机节点: 0x%1 (%2)")
                        .arg(nodeId, 3, 16, QChar('0')).arg(name));
}

void MotorService::removeNode(uint32_t nodeId)
{
    stopMonitoring(nodeId);
    nodes_.remove(nodeId);
}

QList<uint32_t> MotorService::nodeIds() const
{
    return nodes_.keys();
}

QString MotorService::nodeName(uint32_t nodeId) const
{
    auto it = nodes_.find(nodeId);
    return (it != nodes_.end()) ? it->name : QString();
}

MotorState MotorService::motorState(uint32_t nodeId) const
{
    auto it = nodes_.find(nodeId);
    return (it != nodes_.end()) ? it->state : MotorState();
}

// ==================== 辅助 ====================

void MotorService::sendCommand(uint32_t nodeId, const CommRequest &req,
                               std::function<void(const CommResult &)> callback)
{
    if (!comm_->isBusOpen()) {
        emit logMessage(LogLevel::Error, QStringLiteral("总线未打开，无法发送命令"));
        return;
    }
    comm_->submitRequest(req, callback);
}

// ==================== 控制指令 ====================

void MotorService::startCommunication(uint32_t nodeId, OperationMode mode)
{
    enableMotor(nodeId, mode);
}

void MotorService::stopCommunication(uint32_t nodeId)
{
    disableMotor(nodeId);
}

void MotorService::enableMotor(uint32_t nodeId, OperationMode mode)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;

    emit logMessage(LogLevel::Info, QStringLiteral("使能电机 0x%1 [%2]")
                        .arg(nodeId, 3, 16, QChar('0')).arg(operationModeText(mode)));

    // 设置运行模式
    sendCommand(nodeId, CanopenMotorProtocol::setOperationMode(nodeId, mode));

    // CiA 402 状态机: Shutdown -> SwitchOn -> Enable
    QTimer::singleShot(50, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::sendControlWord(nodeId, 0x0006));
    });
    QTimer::singleShot(150, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::sendControlWord(nodeId, 0x0007));
    });
    QTimer::singleShot(250, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::sendControlWord(nodeId, 0x000F));
    });
    QTimer::singleShot(350, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::nmtStart(nodeId));
    });
    QTimer::singleShot(400, this, [this, nodeId, mode]() {
        auto it2 = nodes_.find(nodeId);
        if (it2 != nodes_.end()) {
            it2->state.enabled = true;
            it2->state.online = true;
            it2->state.mode = mode;
            it2->state.lastUpdateTime = QDateTime::currentDateTime();
            emit motorEnabled(nodeId, true);
            emit motorStateChanged(nodeId, it2->state);
            emit logMessage(LogLevel::Info, QStringLiteral("电机 0x%1 使能完成")
                                .arg(nodeId, 3, 16, QChar('0')));
        }
    });
}

void MotorService::disableMotor(uint32_t nodeId)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;

    emit logMessage(LogLevel::Info, QStringLiteral("失能电机 0x%1").arg(nodeId, 3, 16, QChar('0')));

    if (it->state.mode == OperationMode::Velocity) {
        sendCommand(nodeId, CanopenMotorProtocol::setTargetVelocity(nodeId, 0, it->directionInverted));
    }

    QTimer::singleShot(100, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::sendControlWord(nodeId, 0x0006));
    });
    QTimer::singleShot(200, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::nmtPreOperational(nodeId));
    });
    QTimer::singleShot(250, this, [this, nodeId]() {
        auto it2 = nodes_.find(nodeId);
        if (it2 != nodes_.end()) {
            it2->state.enabled = false;
            it2->state.lastUpdateTime = QDateTime::currentDateTime();
            emit motorDisabled(nodeId);
            emit motorStateChanged(nodeId, it2->state);
        }
    });
}

void MotorService::faultReset(uint32_t nodeId)
{
    emit logMessage(LogLevel::Info, QStringLiteral("故障复位 0x%1").arg(nodeId, 3, 16, QChar('0')));
    sendCommand(nodeId, CanopenMotorProtocol::sendControlWord(nodeId, 0x0080));
    QTimer::singleShot(100, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::sendControlWord(nodeId, 0x0000));
    });
    QTimer::singleShot(200, this, [this, nodeId]() {
        auto it = nodes_.find(nodeId);
        if (it != nodes_.end()) {
            it->state.enabled = false;
            it->state.faultCode = 0;
            it->state.faultText.clear();
            it->state.collisionTriggered = false;
            it->state.lastUpdateTime = QDateTime::currentDateTime();
            emit motorStateChanged(nodeId, it->state);
        }
    });
}

void MotorService::emergencyStop(uint32_t nodeId)
{
    CommRequest req = CanopenMotorProtocol::sendControlWord(nodeId, 0x0002);
    req.priority = Priority::Critical;
    sendCommand(nodeId, req);
    emit logMessage(LogLevel::Critical, QStringLiteral("急停! 电机 0x%1").arg(nodeId, 3, 16, QChar('0')));

    auto it = nodes_.find(nodeId);
    if (it != nodes_.end()) {
        it->state.enabled = false;
        it->state.lastUpdateTime = QDateTime::currentDateTime();
        emit motorStateChanged(nodeId, it->state);
    }
}

void MotorService::setVelocity(uint32_t nodeId, int32_t rpm)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;
    sendCommand(nodeId, CanopenMotorProtocol::setTargetVelocity(nodeId, rpm, it->directionInverted));
}

void MotorService::setPosition(uint32_t nodeId, double degrees, bool absolute)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;
    sendCommand(nodeId, CanopenMotorProtocol::setTargetPosition(nodeId, degrees, it->directionInverted));
    QTimer::singleShot(30, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::startPositionMove(nodeId, !absolute));
    });
    QTimer::singleShot(80, this, [=]() {
        sendCommand(nodeId, CanopenMotorProtocol::clearPositionNewSetpoint(nodeId));
    });
}

void MotorService::setProfileVelocity(uint32_t nodeId, uint32_t rpm)
{
    sendCommand(nodeId, CanopenMotorProtocol::setProfileVelocity(nodeId, rpm));
}

void MotorService::setMaxTorqueLimit(uint32_t nodeId, uint16_t permille)
{
    sendCommand(nodeId, CanopenMotorProtocol::setMaxTorqueLimit(nodeId, permille));
}

void MotorService::releaseBrake(uint32_t nodeId)
{
    sendCommand(nodeId, CanopenMotorProtocol::releaseBrake(nodeId));
    emit logMessage(LogLevel::Warning, QStringLiteral("松闸 0x%1 - 电机将失去保持力!").arg(nodeId, 3, 16, QChar('0')));
}

void MotorService::lockBrake(uint32_t nodeId)
{
    sendCommand(nodeId, CanopenMotorProtocol::lockBrake(nodeId));
    emit logMessage(LogLevel::Info, QStringLiteral("锁闸 0x%1").arg(nodeId, 3, 16, QChar('0')));
}

// ==================== 清除运动指令 ====================

void MotorService::clearMotionCommand(uint32_t nodeId)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;

    switch (it->state.mode) {
    case OperationMode::Velocity:
    case OperationMode::CyclicSyncVelocity:
        sendCommand(nodeId, CanopenMotorProtocol::setTargetVelocity(nodeId, 0, it->directionInverted));
        break;
    case OperationMode::ProfilePosition:
    case OperationMode::CyclicSyncPosition:
        sendCommand(nodeId, CanopenMotorProtocol::setTargetVelocity(nodeId, 0, it->directionInverted));
        break;
    default:
        break;
    }
    emit logMessage(LogLevel::Info, QStringLiteral("已清除电机 0x%1 运动指令")
                        .arg(nodeId, 3, 16, QChar('0')));
}

// ==================== 碰撞保护 ====================

void MotorService::enableCollisionProtection(uint32_t nodeId, const CollisionConfig &cfg)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;
    it->collisionCfg = cfg;
    it->state.collisionProtectionOn = true;
    it->state.collisionTriggered = false;
    it->consecutiveHits = 0;

    sendCommand(nodeId, CanopenMotorProtocol::setMaxTorqueLimit(nodeId, cfg.torqueLimitPermille));

    emit logMessage(LogLevel::Info,
                    QStringLiteral("碰撞保护启用: 0x%1, 限制=%2‰, 触发=%3‰")
                        .arg(nodeId, 3, 16, QChar('0'))
                        .arg(cfg.torqueLimitPermille)
                        .arg(cfg.triggerTorquePermille));
    emit motorStateChanged(nodeId, it->state);
}

void MotorService::disableCollisionProtection(uint32_t nodeId)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;
    it->state.collisionProtectionOn = false;
    it->consecutiveHits = 0;
    emit motorStateChanged(nodeId, it->state);
}

// ==================== 批量操作 ====================

void MotorService::enableAll(OperationMode mode)
{
    for (auto &node : nodes_)
        enableMotor(node.nodeId, mode);
}

void MotorService::disableAll()
{
    for (auto &node : nodes_)
        disableMotor(node.nodeId);
}

void MotorService::emergencyStopAll()
{
    for (auto &node : nodes_)
        emergencyStop(node.nodeId);
}

void MotorService::faultResetAll()
{
    for (auto &node : nodes_)
        faultReset(node.nodeId);
}

// ==================== 轮询 ====================

void MotorService::startMonitoring(uint32_t nodeId, int intervalMs)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;
    if (it->torquePollTaskId >= 0) return; // 已在监控

    CommRequest tmpl = CanopenMotorProtocol::readActualTorque(nodeId);
    int taskId = comm_->registerPollTask(tmpl, intervalMs,
                                          [this, nodeId](const CommResult &r) {
                                              onTorqueResult(nodeId, r);
                                          });
    it->torquePollTaskId = taskId;
    emit logMessage(LogLevel::Info, QStringLiteral("开始监控 0x%1 (每%2ms)")
                        .arg(nodeId, 3, 16, QChar('0')).arg(intervalMs));
}

void MotorService::stopMonitoring(uint32_t nodeId)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;
    if (it->torquePollTaskId >= 0) {
        comm_->unregisterPollTask(it->torquePollTaskId);
        it->torquePollTaskId = -1;
    }
}

void MotorService::stopAllMonitoring()
{
    for (auto &node : nodes_)
        stopMonitoring(node.nodeId);
}

void MotorService::onTorqueResult(uint32_t nodeId, const CommResult &result)
{
    auto it = nodes_.find(nodeId);
    if (it == nodes_.end()) return;

    if (!result.success) return;

    int16_t torque = CanopenMotorProtocol::parseTorquePermille(result);
    it->state.torquePermille = torque;
    it->state.online = true;
    it->state.lastUpdateTime = QDateTime::currentDateTime();

    emit torqueUpdated(nodeId, torque);

    // 碰撞检测
    if (it->state.collisionProtectionOn && it->state.enabled) {
        if (std::abs(static_cast<int>(torque)) >= it->collisionCfg.triggerTorquePermille) {
            it->consecutiveHits++;
        } else {
            it->consecutiveHits = 0;
        }
        if (it->consecutiveHits >= it->collisionCfg.consecutiveSamples) {
            it->state.collisionTriggered = true;
            it->state.collisionProtectionOn = false;
            it->consecutiveHits = 0;

            emit logMessage(LogLevel::Critical,
                            QStringLiteral("碰撞触发! 电机 0x%1, 扭矩=%2‰")
                                .arg(nodeId, 3, 16, QChar('0')).arg(torque));

            clearMotionCommand(nodeId);

            if (it->collisionCfg.useQuickStop) {
                emergencyStop(nodeId);
            } else {
                disableMotor(nodeId);
            }

            emit collisionDetected(nodeId, torque);
        }
    }

    emit motorStateChanged(nodeId, it->state);
}

} // namespace dac
