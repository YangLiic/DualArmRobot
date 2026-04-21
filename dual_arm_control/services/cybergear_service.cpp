#include "services/cybergear_service.h"
#include "protocols/cybergear_protocol.h"
#include <cmath>
#include <cstring>
#include <QStringList>

namespace dac {

namespace {

QString formatPayload(const uint8_t *data, uint8_t len)
{
    QStringList parts;
    for (uint8_t i = 0; i < len; ++i) {
        parts << QStringLiteral("%1").arg(data[i], 2, 16, QChar('0'));
    }
    return parts.join(' ');
}

float clampAbs(float value, float limit)
{
    const float maxAbs = std::max(0.0f, limit);
    return std::clamp(value, -maxAbs, maxAbs);
}

} // namespace

CyberGearService::CyberGearService(CommunicationManager *comm, QObject *parent)
    : QObject(parent), comm_(comm)
{
    qRegisterMetaType<dac::CyberGearStatus>("dac::CyberGearStatus");

    connect(comm_, &CommunicationManager::rawFrameReceived, this, [this](const CanFrame &frame) {
        if (!frame.isExtended) return;
        uint8_t motorCan = (frame.canId >> 8) & 0xFF;
        auto it = nodes_.find(motorCan);
        if (it == nodes_.end()) return;
        uint8_t targetId = frame.canId & 0xFF;
        if (targetId != it->masterId) return;
        CyberGearProtocol::parseFeedback(frame.canId, frame.data, it->status);
    });
}

void CyberGearService::addNode(uint8_t motorId, const QString &name, uint8_t masterId)
{
    NodeInfo n;
    n.motorId = motorId;
    n.masterId = masterId;
    n.name = name;
    nodes_[motorId] = n;
    emit logMessage(LogLevel::Info, QStringLiteral("CyberGear 节点已添加: %1 (motor=0x%2, master=0x%3)")
                        .arg(name)
                        .arg(motorId, 2, 16, QChar('0'))
                        .arg(masterId, 2, 16, QChar('0')));
}

void CyberGearService::removeNode(uint8_t motorId)
{
    stopMonitoring(motorId);
    nodes_.remove(motorId);
}

QList<uint8_t> CyberGearService::motorIds() const { return nodes_.keys(); }
QString CyberGearService::nodeName(uint8_t motorId) const {
    auto it = nodes_.find(motorId); return it != nodes_.end() ? it->name : QString();
}
CyberGearStatus CyberGearService::motorStatus(uint8_t motorId) const {
    auto it = nodes_.find(motorId); return it != nodes_.end() ? it->status : CyberGearStatus();
}

void CyberGearService::sendCmd(uint8_t motorId, const CommRequest &req,
                                std::function<void(const CommResult&)> cb)
{
    if (!comm_) {
        emit logMessage(LogLevel::Error, QStringLiteral("CyberGear: comm_ 为空，无法发送指令"));
        return;
    }
    emit logMessage(LogLevel::Info, QStringLiteral("CyberGear (0x%1): 发送指令 canId=0x%2, len=%3, resp=%4, data=[%5]")
                        .arg(motorId, 2, 16, QChar('0'))
                        .arg(req.canId, 8, 16, QChar('0'))
                        .arg(req.payloadLen)
                        .arg(req.expectResponse)
                        .arg(formatPayload(req.payload, req.payloadLen)));
    comm_->submitRequest(req, [this, motorId, cb](const CommResult &r) {
        onFeedback(motorId, r);
        if (cb) cb(r);
    });
}

void CyberGearService::onFeedback(uint8_t motorId, const CommResult &result)
{
    auto it = nodes_.find(motorId);
    if (it == nodes_.end()) return;
    if (result.success) {
        it->status.online = true;
        it->status.lastUpdateTime = QDateTime::currentDateTime();
    }
    if (it->status.hasFault) emit cgFaultDetected(motorId);
    emit cgStateChanged(motorId, it->status);
}

void CyberGearService::applyCachedSpeedLimit(uint8_t motorId)
{
    auto it = nodes_.find(motorId);
    if (it == nodes_.end()) return;

    sendCmd(motorId, CyberGearProtocol::setSpeedLimit(motorId, it->masterId, it->configuredSpeedLimitRadS));
}

void CyberGearService::applyCachedLimits(uint8_t motorId)
{
    auto it = nodes_.find(motorId);
    if (it == nodes_.end()) return;

    applyCachedSpeedLimit(motorId);
    sendCmd(motorId, CyberGearProtocol::setCurrentLimit(motorId, it->masterId, it->configuredCurrentLimitA));
}

// ===== 控制 =====
void CyberGearService::enable(uint8_t motorId) {
    auto it = nodes_.find(motorId);
    if (it == nodes_.end()) {
        emit logMessage(LogLevel::Error, QStringLiteral("CyberGear enable: 未找到节点 0x%1").arg(motorId, 2, 16, QChar('0')));
        return;
    }
    emit logMessage(LogLevel::Info, QStringLiteral("CyberGear (0x%1): 使能电机").arg(motorId, 2, 16, QChar('0')));
    sendCmd(motorId, CyberGearProtocol::enable(motorId, it->masterId), [this, motorId](const CommResult &r) {
        auto it2 = nodes_.find(motorId);
        if (it2 != nodes_.end()) it2->status.enabled = true;
        emit cgCommandResult(motorId, QStringLiteral("使能"), r.success, r.success ? QStringLiteral("已使能") : r.errorMessage);
    });
}

void CyberGearService::stop(uint8_t motorId) {
    auto it = nodes_.find(motorId);
    if (it == nodes_.end()) {
        emit logMessage(LogLevel::Error, QStringLiteral("CyberGear stop: 未找到节点 0x%1").arg(motorId, 2, 16, QChar('0')));
        return;
    }
    it->lastAppliedSpeedRadS = 0.0f;
    emit logMessage(LogLevel::Info, QStringLiteral("CyberGear (0x%1): 停止电机").arg(motorId, 2, 16, QChar('0')));
    sendCmd(motorId, CyberGearProtocol::stop(motorId, it->masterId), [this, motorId](const CommResult &r) {
        auto it2 = nodes_.find(motorId);
        if (it2 != nodes_.end()) it2->status.enabled = false;
        emit cgCommandResult(motorId, QStringLiteral("停止"), true, QStringLiteral("已停止"));
    });
}

void CyberGearService::setRunMode(uint8_t motorId, CgRunMode mode) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    it->status.runMode = mode;
    sendCmd(motorId, CyberGearProtocol::setRunMode(motorId, it->masterId, mode));
}

void CyberGearService::enableSpeedMode(uint8_t motorId) {
    stop(motorId);
    QTimer::singleShot(120, this, [this, motorId]() {
        setRunMode(motorId, CgRunMode::Speed);
        QTimer::singleShot(60, this, [this, motorId]() {
            enable(motorId);
            QTimer::singleShot(60, this, [this, motorId]() {
                applyCachedLimits(motorId);
            });
        });
    });
}

void CyberGearService::enablePositionMode(uint8_t motorId) {
    stop(motorId);
    QTimer::singleShot(120, this, [this, motorId]() {
        setRunMode(motorId, CgRunMode::Position);
        QTimer::singleShot(60, this, [this, motorId]() {
            enable(motorId);
            QTimer::singleShot(60, this, [this, motorId]() {
                applyCachedLimits(motorId);
            });
        });
    });
}

void CyberGearService::setSpeedRadS(uint8_t motorId, float radS) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;

    const float clampedRadS = clampAbs(radS, it->configuredSpeedLimitRadS);
    if (!qFuzzyCompare(std::abs(radS) + 1.0f, std::abs(clampedRadS) + 1.0f)) {
        emit logMessage(LogLevel::Warning,
                        QStringLiteral("CyberGear (0x%1): 速度指令 %.3f rad/s 已按限制 %.3f rad/s 限幅为 %.3f rad/s")
                            .arg(motorId, 2, 16, QChar('0'))
                            .arg(radS, 0, 'f', 3)
                            .arg(it->configuredSpeedLimitRadS, 0, 'f', 3)
                            .arg(clampedRadS, 0, 'f', 3));
    }

    it->lastAppliedSpeedRadS = clampedRadS;
    applyCachedSpeedLimit(motorId);
    sendCmd(motorId, CyberGearProtocol::setSpeedRadS(motorId, it->masterId, clampedRadS));
}

void CyberGearService::setSpeedRPM(uint8_t motorId, float rpm) {
    setSpeedRadS(motorId, rpm * 2.0f * (float)M_PI / 60.0f);
}

void CyberGearService::setPositionRad(uint8_t motorId, float rad) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    applyCachedSpeedLimit(motorId);
    sendCmd(motorId, CyberGearProtocol::setPositionRad(motorId, it->masterId, rad));
}

void CyberGearService::setPositionDeg(uint8_t motorId, float deg) {
    setPositionRad(motorId, deg * (float)M_PI / 180.0f);
}

void CyberGearService::setCurrentLimit(uint8_t motorId, float amps) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    it->configuredCurrentLimitA = std::clamp(amps, 0.0f, 27.0f);
    sendCmd(motorId, CyberGearProtocol::setCurrentLimit(motorId, it->masterId, it->configuredCurrentLimitA));
}

void CyberGearService::setSpeedLimit(uint8_t motorId, float radS) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    it->configuredSpeedLimitRadS = std::clamp(radS, 0.0f, 30.0f);
    sendCmd(motorId, CyberGearProtocol::setSpeedLimit(motorId, it->masterId, it->configuredSpeedLimitRadS));

    const float clampedSpeed = clampAbs(it->lastAppliedSpeedRadS, it->configuredSpeedLimitRadS);
    if (it->status.runMode == CgRunMode::Speed &&
        it->status.enabled &&
        !qFuzzyCompare(std::abs(it->lastAppliedSpeedRadS) + 1.0f, std::abs(clampedSpeed) + 1.0f)) {
        it->lastAppliedSpeedRadS = clampedSpeed;
        sendCmd(motorId, CyberGearProtocol::setSpeedRadS(motorId, it->masterId, clampedSpeed));
        emit logMessage(LogLevel::Info,
                        QStringLiteral("CyberGear (0x%1): 新速度限制 %.3f rad/s 已立即重发当前限幅速度 %.3f rad/s")
                            .arg(motorId, 2, 16, QChar('0'))
                            .arg(it->configuredSpeedLimitRadS, 0, 'f', 3)
                            .arg(clampedSpeed, 0, 'f', 3));
    }
}

void CyberGearService::setMechZero(uint8_t motorId) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    sendCmd(motorId, CyberGearProtocol::setMechZero(motorId, it->masterId), [this, motorId](const CommResult &r) {
        emit cgCommandResult(motorId, QStringLiteral("设零位"), r.success,
                              r.success ? QStringLiteral("已设置") : r.errorMessage);
    });
}

void CyberGearService::goToZero(uint8_t motorId) {
    enablePositionMode(motorId);
    QTimer::singleShot(300, this, [this, motorId]() {
        setPositionRad(motorId, 0.0f);
    });
}

// ===== 轮询 =====
void CyberGearService::startMonitoring(uint8_t motorId, int intervalMs) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    if (it->pollTaskId >= 0) comm_->unregisterPollTask(it->pollTaskId);
    auto req = CyberGearProtocol::requestFeedback(motorId, it->masterId);
    it->pollTaskId = comm_->registerPollTask(req, intervalMs,
        [this, motorId](const CommResult &r) { onFeedback(motorId, r); });
}

void CyberGearService::stopMonitoring(uint8_t motorId) {
    auto it = nodes_.find(motorId); if (it == nodes_.end()) return;
    if (it->pollTaskId >= 0) { comm_->unregisterPollTask(it->pollTaskId); it->pollTaskId = -1; }
}

void CyberGearService::stopAllMonitoring() {
    for (auto &n : nodes_) {
        if (n.pollTaskId >= 0) { comm_->unregisterPollTask(n.pollTaskId); n.pollTaskId = -1; }
    }
}

void CyberGearService::stopAll() {
    for (auto &n : nodes_) stop(n.motorId);
}

} // namespace dac
