/**
 * CommunicationManager 实现
 */
#include "communication_manager.h"
#include <QDebug>

namespace dar {

CommunicationManager::CommunicationManager(QObject *parent)
    : QObject(parent)
{
}

CommunicationManager::~CommunicationManager()
{
    disconnectAll();

    for (auto &entry : buses_) {
        if (entry.thread) {
            entry.thread->quit();
            entry.thread->wait(3000);
            delete entry.thread;
        }
        // worker 被 moveToThread, 通过 deleteLater 清理
    }
    buses_.clear();
}

bool CommunicationManager::addBus(const BusConfig &config)
{
    if (buses_.contains(config.busId)) {
        log(LogEntry::Warning, QStringLiteral("总线已存在: %1").arg(config.busId));
        return false;
    }

    BusEntry entry;
    entry.config = config;

    // 创建 worker 和专属线程
    entry.worker = new CanBusWorker(config);
    entry.thread = new QThread(this);

    entry.worker->moveToThread(entry.thread);

    // 转发信号
    connect(entry.worker, &CanBusWorker::connected, this, [this, busId = config.busId]() {
        emit busConnected(busId);
    });
    connect(entry.worker, &CanBusWorker::disconnected, this, [this, busId = config.busId]() {
        emit busDisconnected(busId);
    });
    connect(entry.worker, &CanBusWorker::requestCompleted, this, &CommunicationManager::requestCompleted);
    connect(entry.worker, &CanBusWorker::frameReceived, this, [this, busId = config.busId](const CanFrame &f) {
        emit frameReceived(busId, f);
    });
    connect(entry.worker, &CanBusWorker::logMessage, this, &CommunicationManager::logMessage);
    connect(entry.worker, &CanBusWorker::busError, this, [this, busId = config.busId](const QString &msg) {
        emit busError(busId, msg);
    });

    // 线程退出时清理 worker
    connect(entry.thread, &QThread::finished, entry.worker, &QObject::deleteLater);

    entry.thread->start();

    buses_[config.busId] = entry;
    log(LogEntry::Info, QStringLiteral("添加总线: %1 -> %2").arg(config.busId, config.devicePath));
    return true;
}

void CommunicationManager::removeBus(const QString &busId)
{
    if (!buses_.contains(busId)) return;

    disconnectBus(busId);

    auto &entry = buses_[busId];
    if (entry.thread) {
        entry.thread->quit();
        entry.thread->wait(3000);
        delete entry.thread;
    }

    buses_.remove(busId);
    log(LogEntry::Info, QStringLiteral("移除总线: %1").arg(busId));
}

void CommunicationManager::connectBus(const QString &busId)
{
    if (!buses_.contains(busId)) return;

    auto &entry = buses_[busId];
    QMetaObject::invokeMethod(entry.worker, "connectBus", Qt::QueuedConnection);
    // 连接后开始处理
    QMetaObject::invokeMethod(entry.worker, "startProcessing", Qt::QueuedConnection);
}

void CommunicationManager::disconnectBus(const QString &busId)
{
    if (!buses_.contains(busId)) return;

    auto &entry = buses_[busId];
    QMetaObject::invokeMethod(entry.worker, "stopProcessing", Qt::QueuedConnection);
    QMetaObject::invokeMethod(entry.worker, "disconnectBus", Qt::QueuedConnection);
}

void CommunicationManager::connectAll()
{
    for (auto it = buses_.begin(); it != buses_.end(); ++it) {
        connectBus(it.key());
    }
}

void CommunicationManager::disconnectAll()
{
    for (auto it = buses_.begin(); it != buses_.end(); ++it) {
        disconnectBus(it.key());
    }
}

void CommunicationManager::submitRequest(const CommRequest &request)
{
    if (!buses_.contains(request.busId)) {
        CommResult result;
        result.requestId = request.requestId;
        result.success = false;
        result.errorMessage = QStringLiteral("总线不存在: %1").arg(request.busId);
        emit requestCompleted(result);
        return;
    }

    auto &entry = buses_[request.busId];
    QMetaObject::invokeMethod(entry.worker, "submitRequest",
                              Qt::QueuedConnection,
                              Q_ARG(dar::CommRequest, request));
}

void CommunicationManager::registerPollTask(const QString &busId, const PollTask &task)
{
    if (!buses_.contains(busId)) return;

    auto &entry = buses_[busId];
    QMetaObject::invokeMethod(entry.worker, "registerPollTask",
                              Qt::QueuedConnection,
                              Q_ARG(dar::PollTask, task));
}

void CommunicationManager::unregisterPollTask(const QString &busId, const QString &taskId)
{
    if (!buses_.contains(busId)) return;

    auto &entry = buses_[busId];
    QMetaObject::invokeMethod(entry.worker, "unregisterPollTask",
                              Qt::QueuedConnection,
                              Q_ARG(QString, taskId));
}

bool CommunicationManager::isBusConnected(const QString &busId) const
{
    auto it = buses_.find(busId);
    if (it == buses_.end()) return false;
    return it->worker && it->worker->isConnected();
}

QStringList CommunicationManager::busIds() const
{
    return buses_.keys();
}

CanBusWorker *CommunicationManager::worker(const QString &busId) const
{
    auto it = buses_.find(busId);
    if (it == buses_.end()) return nullptr;
    return it->worker;
}

void CommunicationManager::log(LogEntry::Level level, const QString &msg)
{
    LogEntry entry;
    entry.level = level;
    entry.timestamp = QDateTime::currentDateTime();
    entry.source = QStringLiteral("CommManager");
    entry.message = msg;
    emit logMessage(entry);
}

}  // namespace dar
