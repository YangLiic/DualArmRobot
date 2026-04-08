#include "communication/CommunicationManager.h"

#include <QMetaObject>

CommunicationManager::CommunicationManager(QObject *parent)
    : QObject(parent)
{
}

CommunicationManager::~CommunicationManager()
{
    const QStringList taskIds = m_pollTasks.keys();
    for (const QString &taskId : taskIds) {
        unregisterPollingTask(taskId);
    }

    const QStringList busIds = m_workers.keys();
    for (const QString &busId : busIds) {
        destroyBus(busId);
    }
}

void CommunicationManager::configureCanBus(const BusConfig &config)
{
    if (m_workers.contains(config.busId)) {
        const WorkerHandle existing = m_workers.value(config.busId);
        if (existing.config.devicePath == config.devicePath && existing.config.baudRate == config.baudRate) {
            return;
        }
        destroyBus(config.busId);
    }

    WorkerHandle handle;
    handle.config = config;
    handle.thread = new QThread(this);
    handle.worker = new CanBusWorker(config);
    handle.worker->moveToThread(handle.thread);

    connect(handle.worker, &CanBusWorker::requestCompleted, this, &CommunicationManager::requestCompleted);
    connect(handle.worker, &CanBusWorker::busConnectionChanged, this, &CommunicationManager::busConnectionChanged);
    connect(handle.worker, &CanBusWorker::logMessage, this, &CommunicationManager::logMessage);

    handle.thread->start();
    m_workers.insert(config.busId, handle);
}

void CommunicationManager::connectBus(const QString &busId)
{
    if (!m_workers.contains(busId)) {
        emit logMessage(QStringLiteral("[%1] No worker configured").arg(busId));
        return;
    }

    const WorkerHandle &handle = m_workers.value(busId);
    QMetaObject::invokeMethod(handle.worker, "connectBus", Qt::QueuedConnection);
}

void CommunicationManager::disconnectBus(const QString &busId)
{
    if (!m_workers.contains(busId)) {
        return;
    }

    const WorkerHandle &handle = m_workers.value(busId);
    QMetaObject::invokeMethod(handle.worker, "disconnectBus", Qt::QueuedConnection);
}

void CommunicationManager::submitRequest(const BusRequest &request)
{
    if (!m_workers.contains(request.busId)) {
        BusResult result;
        result.requestId = request.requestId;
        result.busId = request.busId;
        result.deviceId = request.deviceId;
        result.commandType = request.commandType;
        result.context = request.context;
        result.rawRequest = request.payload;
        result.success = false;
        result.errorMessage = QStringLiteral("Bus %1 is not configured").arg(request.busId);
        emit requestCompleted(result);
        return;
    }

    const WorkerHandle &handle = m_workers.value(request.busId);
    QMetaObject::invokeMethod(
        handle.worker,
        "submitRequest",
        Qt::QueuedConnection,
        Q_ARG(BusRequest, request));
}

void CommunicationManager::registerPollingTask(
    const QString &taskId,
    int intervalMs,
    const std::function<QList<BusRequest>()> &builder)
{
    unregisterPollingTask(taskId);

    PollTask task;
    task.taskId = taskId;
    task.timer = new QTimer(this);
    task.timer->setInterval(intervalMs);
    task.builder = builder;

    connect(task.timer, &QTimer::timeout, this, [this, taskId]() {
        if (!m_pollTasks.contains(taskId)) {
            return;
        }

        const QList<BusRequest> requests = m_pollTasks.value(taskId).builder();
        for (const BusRequest &request : requests) {
            submitRequest(request);
        }
    });

    task.timer->start();
    m_pollTasks.insert(taskId, task);
}

void CommunicationManager::unregisterPollingTask(const QString &taskIdPrefix)
{
    const QStringList taskIds = m_pollTasks.keys();
    for (const QString &taskId : taskIds) {
        if (!taskId.startsWith(taskIdPrefix)) {
            continue;
        }

        PollTask task = m_pollTasks.take(taskId);
        if (task.timer) {
            task.timer->stop();
            delete task.timer;
        }
    }
}

void CommunicationManager::destroyBus(const QString &busId)
{
    if (!m_workers.contains(busId)) {
        return;
    }

    const WorkerHandle handle = m_workers.take(busId);
    if (handle.worker) {
        QMetaObject::invokeMethod(handle.worker, "disconnectBus", Qt::BlockingQueuedConnection);
    }
    if (handle.thread) {
        handle.thread->quit();
        handle.thread->wait();
    }
    if (handle.worker) {
        delete handle.worker;
    }
    if (handle.thread) {
        delete handle.thread;
    }
}
