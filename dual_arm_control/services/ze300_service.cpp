#include "services/ze300_service.h"
#include "protocols/ze300_protocol.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace dac {

Ze300Service::Ze300Service(CommunicationManager *comm, QObject *parent)
    : QObject(parent), comm_(comm)
{
    qRegisterMetaType<dac::Ze300Status>("dac::Ze300Status");

    // 监听原始帧，用于捕获 ZE300 的主动上报
    connect(comm_, &CommunicationManager::rawFrameReceived, this, [this](const CanFrame &frame) {
        // 检查是否是某个已注册 ZE300 节点的响应帧
        for (auto &node : nodes_) {
            if (frame.canId == (node.devAddr & 0xFF) && frame.dlc >= 1) {
                // 被动更新状态（非请求触发的帧也会更新状态缓存）
                const Ze300Status previousStatus = node.status;
                CommResult fakeResult;
                fakeResult.success = true;
                fakeResult.dlc = frame.dlc;
                std::memcpy(fakeResult.data, frame.data, 8);
                Ze300Protocol::parseResponse(fakeResult, node.status);
                emitStateUpdate(node.devAddr, previousStatus);
            }
        }
    });
}

void Ze300Service::addNode(uint16_t devAddr, const QString &name, bool useHostAddr)
{
    NodeInfo info;
    info.devAddr = devAddr;
    info.name = name;
    info.useHostAddr = useHostAddr;
    nodes_[devAddr] = info;
    emit logMessage(LogLevel::Info, QStringLiteral("ZE300 节点已添加: %1 (0x%2)")
                        .arg(name).arg(devAddr, 2, 16, QChar('0')));
}

void Ze300Service::removeNode(uint16_t devAddr)
{
    stopMonitoring(devAddr);
    nodes_.remove(devAddr);
}

QList<uint16_t> Ze300Service::nodeAddrs() const { return nodes_.keys(); }

QString Ze300Service::nodeName(uint16_t devAddr) const
{
    auto it = nodes_.find(devAddr);
    return (it != nodes_.end()) ? it->name : QString();
}

Ze300Status Ze300Service::motorStatus(uint16_t devAddr) const
{
    auto it = nodes_.find(devAddr);
    return (it != nodes_.end()) ? it->status : Ze300Status();
}

void Ze300Service::sendCommand(uint16_t devAddr, const CommRequest &req,
                               std::function<void(const CommResult&)> callback)
{
    if (!comm_) return;
    comm_->submitRequest(req, [this, devAddr, callback](const CommResult &result) {
        onStatusResult(devAddr, result);
        if (callback) callback(result);
    });
}

void Ze300Service::onStatusResult(uint16_t devAddr, const CommResult &result)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;

    const Ze300Status previousStatus = it->status;
    Ze300Protocol::parseResponse(result, it->status);
    emitStateUpdate(devAddr, previousStatus);
}

void Ze300Service::emitStateUpdate(uint16_t devAddr, const Ze300Status &previousStatus)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;

    // 只在故障新出现或故障码变化时上报，避免轮询时重复弹窗。
    if (it->status.fault &&
        (!previousStatus.fault || previousStatus.faultCode != it->status.faultCode)) {
        emit ze300FaultDetected(devAddr, it->status.faultCode);
    }

    emit ze300StateChanged(devAddr, it->status);
}

void Ze300Service::registerMonitorTask(NodeInfo &node, const CommRequest &req, int intervalMs)
{
    if (!comm_) return;
    const uint16_t devAddr = node.devAddr;
    const int taskId = comm_->registerPollTask(req, intervalMs,
        [this, devAddr](const CommResult &result) {
            onStatusResult(devAddr, result);
        });
    node.pollTaskIds.push_back(taskId);
}

void Ze300Service::requestMonitorSnapshot(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;

    sendCommand(devAddr, Ze300Protocol::readQuickStatus(devAddr, it->useHostAddr));
    sendCommand(devAddr, Ze300Protocol::readSpeed(devAddr, it->useHostAddr));
    sendCommand(devAddr, Ze300Protocol::readPosition(devAddr, it->useHostAddr));
    sendCommand(devAddr, Ze300Protocol::readStatus(devAddr, it->useHostAddr));
    sendCommand(devAddr, Ze300Protocol::readBrakeState(devAddr, it->useHostAddr));
}

// ============ 控制指令 ============

void Ze300Service::setSpeedRpm(uint16_t devAddr, float rpm)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::setSpeedRpm(devAddr, it->useHostAddr, rpm);
    sendCommand(devAddr, req, [this, devAddr, rpm](const CommResult &r) {
        emit ze300CommandResult(devAddr, QStringLiteral("速度控制"),
                                r.success, r.success ? QStringLiteral("设定 %1 rpm").arg(rpm, 0, 'f', 2)
                                                     : r.errorMessage);
    });
}

void Ze300Service::setAbsolutePositionDeg(uint16_t devAddr, float deg)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    int32_t count = (int32_t)std::round(deg / DEG_PER_COUNT);
    auto req = Ze300Protocol::setAbsolutePositionCount(devAddr, it->useHostAddr, count);
    sendCommand(devAddr, req, [this, devAddr, deg](const CommResult &r) {
        emit ze300CommandResult(devAddr, QStringLiteral("绝对位置"),
                                r.success, r.success ? QStringLiteral("%1°").arg(deg, 0, 'f', 2)
                                                     : r.errorMessage);
    });
}

void Ze300Service::setRelativePositionDeg(uint16_t devAddr, float deg)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    int32_t count = (int32_t)std::round(deg / DEG_PER_COUNT);
    auto req = Ze300Protocol::setRelativePositionCount(devAddr, it->useHostAddr, count);
    sendCommand(devAddr, req, [this, devAddr, deg](const CommResult &r) {
        emit ze300CommandResult(devAddr, QStringLiteral("相对位置"),
                                r.success, r.success ? QStringLiteral("%1°").arg(deg, 0, 'f', 2)
                                                     : r.errorMessage);
    });
}

void Ze300Service::setTorqueCurrentA(uint16_t devAddr, float currentA)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::setTorqueCurrentA(devAddr, it->useHostAddr, currentA);
    sendCommand(devAddr, req);
}

void Ze300Service::goOriginShortest(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::goOriginShortest(devAddr, it->useHostAddr);
    sendCommand(devAddr, req, [this, devAddr](const CommResult &r) {
        emit ze300CommandResult(devAddr, QStringLiteral("回原点"), r.success,
                                r.success ? QStringLiteral("已执行") : r.errorMessage);
    });
}

void Ze300Service::setZero(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::setZero(devAddr, it->useHostAddr);
    sendCommand(devAddr, req, [this, devAddr](const CommResult &r) {
        emit ze300CommandResult(devAddr, QStringLiteral("设置零点"), r.success,
                                r.success ? QStringLiteral("已设置") : r.errorMessage);
    });
}

void Ze300Service::freeOutput(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::freeOutput(devAddr, it->useHostAddr);
    sendCommand(devAddr, req, [this, devAddr](const CommResult &r) {
        emit ze300CommandResult(devAddr, QStringLiteral("自由态"), r.success,
                                r.success ? QStringLiteral("已自由态") : r.errorMessage);
    });
}

void Ze300Service::clearFault(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::clearFault(devAddr, it->useHostAddr);
    sendCommand(devAddr, req, [this, devAddr](const CommResult &r) {
        QString msg;
        if (r.success && r.dlc >= 2) {
            msg = (r.data[1] == 0) ? QStringLiteral("故障已清除")
                                   : QStringLiteral("剩余故障码: 0x%1").arg(r.data[1], 2, 16, QChar('0'));
        } else {
            msg = r.errorMessage;
        }
        emit ze300CommandResult(devAddr, QStringLiteral("清故障"), r.success, msg);
    });
}

void Ze300Service::reboot(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::reboot(devAddr, it->useHostAddr);
    sendCommand(devAddr, req);
    emit logMessage(LogLevel::Warning, QStringLiteral("ZE300 (0x%1) 已发送重启命令")
                        .arg(devAddr, 2, 16, QChar('0')));
}

// ============ 抱闸 ============

void Ze300Service::setBrakeClosed(uint16_t devAddr, bool closed)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::setBrakeClosed(devAddr, it->useHostAddr, closed);
    sendCommand(devAddr, req, [this, devAddr, closed](const CommResult &r) {
        if (r.success && r.dlc >= 2) {
            bool state = (r.data[1] == 0x01);
            emit ze300BrakeStateRead(devAddr, state);
        }
        emit ze300CommandResult(devAddr, QStringLiteral("抱闸"),
                                r.success,
                                r.success ? (closed ? QStringLiteral("闭合") : QStringLiteral("断开"))
                                          : r.errorMessage);
    });
}

void Ze300Service::readBrakeState(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::readBrakeState(devAddr, it->useHostAddr);
    sendCommand(devAddr, req, [this, devAddr](const CommResult &r) {
        if (r.success && r.dlc >= 2) {
            bool state = (r.data[1] == 0x01);
            emit ze300BrakeStateRead(devAddr, state);
        }
    });
}

// ============ 参数 ============

void Ze300Service::setPositionMaxSpeedRpm(uint16_t devAddr, float rpm)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    sendCommand(devAddr, Ze300Protocol::setPositionMaxSpeedRpm(devAddr, it->useHostAddr, rpm));
}

void Ze300Service::setMaxCurrentA(uint16_t devAddr, float currentA)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    sendCommand(devAddr, Ze300Protocol::setMaxCurrentA(devAddr, it->useHostAddr, currentA));
}

void Ze300Service::setSpeedAccelerationRpmPerSec(uint16_t devAddr, float accRpmS)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    sendCommand(devAddr, Ze300Protocol::setSpeedAccelerationRpmPerSec(devAddr, it->useHostAddr, accRpmS));
}

// ============ MIT ============

void Ze300Service::sendMitControl(uint16_t devAddr, float posRad, float velRadS,
                                  float kp, float kd, float torqueNm)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;
    auto req = Ze300Protocol::sendMitControl(devAddr, it->useHostAddr,
                                             posRad, velRadS, kp, kd, torqueNm);
    sendCommand(devAddr, req);
}

// ============ 轮询 ============

void Ze300Service::startMonitoring(uint16_t devAddr, int intervalMs)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;

    stopMonitoring(devAddr);

    const int fastInterval = std::max(60, intervalMs);
    const int stateInterval = std::max(250, fastInterval * 3);
    const int brakeInterval = std::max(500, fastInterval * 5);

    registerMonitorTask(*it, Ze300Protocol::readQuickStatus(devAddr, it->useHostAddr), fastInterval);
    registerMonitorTask(*it, Ze300Protocol::readSpeed(devAddr, it->useHostAddr), fastInterval);
    registerMonitorTask(*it, Ze300Protocol::readPosition(devAddr, it->useHostAddr), fastInterval);
    registerMonitorTask(*it, Ze300Protocol::readStatus(devAddr, it->useHostAddr), stateInterval);
    registerMonitorTask(*it, Ze300Protocol::readBrakeState(devAddr, it->useHostAddr), brakeInterval);

    // 连接后立即拉一轮完整状态，避免界面要等到第一次轮询或动作后才显示。
    requestMonitorSnapshot(devAddr);

    emit logMessage(LogLevel::Info,
                    QStringLiteral("ZE300 (0x%1) 开始监控: 快速/转速/位置 %2ms, 状态 %3ms, 抱闸 %4ms")
                        .arg(devAddr, 2, 16, QChar('0'))
                        .arg(fastInterval)
                        .arg(stateInterval)
                        .arg(brakeInterval));
}

void Ze300Service::stopMonitoring(uint16_t devAddr)
{
    auto it = nodes_.find(devAddr);
    if (it == nodes_.end()) return;

    for (int taskId : it->pollTaskIds) {
        comm_->unregisterPollTask(taskId);
    }
    it->pollTaskIds.clear();
}

void Ze300Service::stopAllMonitoring()
{
    for (auto &node : nodes_) {
        for (int taskId : node.pollTaskIds) {
            comm_->unregisterPollTask(taskId);
        }
        node.pollTaskIds.clear();
    }
}

// ============ 批量 ============

void Ze300Service::freeOutputAll()
{
    for (auto &node : nodes_) {
        freeOutput(node.devAddr);
    }
}

} // namespace dac
