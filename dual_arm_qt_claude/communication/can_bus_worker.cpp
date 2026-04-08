/**
 * CanBusWorker 实现
 */
#include "can_bus_worker.h"
#include <QDebug>
#include <QElapsedTimer>

namespace dar {

CanBusWorker::CanBusWorker(const BusConfig &config, QObject *parent)
    : QObject(parent)
    , config_(config)
{
    frameIO_ = new FrameIO(this);
}

CanBusWorker::~CanBusWorker()
{
    stopProcessing();
    disconnectBus();
}

void CanBusWorker::connectBus()
{
    log(LogEntry::Info, QStringLiteral("正在连接总线: %1 (%2)")
        .arg(config_.busId, config_.devicePath));

    if (!frameIO_->open(config_.devicePath, config_.baudRate)) {
        log(LogEntry::Error, QStringLiteral("打开串口失败: %1").arg(config_.devicePath));
        emit busError(QStringLiteral("打开串口失败: %1").arg(config_.devicePath));
        return;
    }

    // 启动后台接收线程
    receiveThread_ = new QThread(this);
    frameIO_->moveToThread(receiveThread_);

    connect(receiveThread_, &QThread::started, frameIO_, &FrameIO::startReceiveLoop);
    connect(frameIO_, &FrameIO::frameReceived, this, [this](const CanFrame &frame) {
        emit frameReceived(frame);
    }, Qt::QueuedConnection);

    receiveThread_->start();

    connected_ = true;
    log(LogEntry::Info, QStringLiteral("总线已连接: %1").arg(config_.busId));
    emit connected();
}

void CanBusWorker::disconnectBus()
{
    if (!connected_) return;

    stopProcessing();

    frameIO_->stopReceive();
    if (receiveThread_) {
        receiveThread_->quit();
        receiveThread_->wait(3000);
        delete receiveThread_;
        receiveThread_ = nullptr;
    }

    frameIO_->close();
    connected_ = false;

    log(LogEntry::Info, QStringLiteral("总线已断开: %1").arg(config_.busId));
    emit disconnected();
}

void CanBusWorker::submitRequest(const dar::CommRequest &request)
{
    QMutexLocker lock(&requestMutex_);

    int queueIdx = static_cast<int>(request.priority);
    if (queueIdx < 0 || queueIdx > 2) queueIdx = 2;

    CommRequest req = request;
    req.requestId = nextRequestId_++;

    requestQueues_[queueIdx].enqueue(req);

    // Critical 请求立即处理
    if (request.priority == Priority::Critical) {
        QMetaObject::invokeMethod(this, "processNextRequest", Qt::QueuedConnection);
    }
}

void CanBusWorker::registerPollTask(const dar::PollTask &task)
{
    QMutexLocker lock(&requestMutex_);
    pollTasks_[task.taskId] = task;
    log(LogEntry::Debug, QStringLiteral("注册轮询: %1 (node=0x%2, %3ms)")
        .arg(task.taskId)
        .arg(task.nodeId, 3, 16, QChar('0'))
        .arg(task.intervalMs));
}

void CanBusWorker::unregisterPollTask(const QString &taskId)
{
    QMutexLocker lock(&requestMutex_);
    pollTasks_.remove(taskId);
    log(LogEntry::Debug, QStringLiteral("注销轮询: %1").arg(taskId));
}

void CanBusWorker::startProcessing()
{
    processing_ = true;

    // 主处理定时器：10ms 轮询请求队列
    processTimer_ = new QTimer(this);
    processTimer_->setInterval(10);
    connect(processTimer_, &QTimer::timeout, this, &CanBusWorker::processNextRequest);
    processTimer_->start();

    // 轮询任务定时器：50ms 检查一次
    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(50);
    connect(pollTimer_, &QTimer::timeout, this, &CanBusWorker::processPollTasks);
    pollTimer_->start();

    log(LogEntry::Info, QStringLiteral("调度器已启动"));
}

void CanBusWorker::stopProcessing()
{
    processing_ = false;

    if (processTimer_) {
        processTimer_->stop();
        delete processTimer_;
        processTimer_ = nullptr;
    }
    if (pollTimer_) {
        pollTimer_->stop();
        delete pollTimer_;
        pollTimer_ = nullptr;
    }
}

void CanBusWorker::processNextRequest()
{
    if (!connected_ || !processing_) return;

    CommRequest req;
    bool found = false;

    {
        QMutexLocker lock(&requestMutex_);
        // 按优先级出队
        for (int i = 0; i < 3; ++i) {
            if (!requestQueues_[i].isEmpty()) {
                req = requestQueues_[i].dequeue();
                found = true;
                break;
            }
        }
    }

    if (!found) return;

    CommResult result = executeRequest(req);
    emit requestCompleted(result);
}

void CanBusWorker::processPollTasks()
{
    if (!connected_ || !processing_) return;

    // 收集当前需要执行的轮询任务
    QMutexLocker lock(&requestMutex_);

    static QMap<QString, qint64> lastPollTime;
    qint64 now = QDateTime::currentMSecsSinceEpoch();

    for (auto it = pollTasks_.begin(); it != pollTasks_.end(); ++it) {
        const PollTask &task = it.value();
        if (!task.active) continue;

        qint64 lastTime = lastPollTime.value(task.taskId, 0);
        if (now - lastTime >= task.intervalMs) {
            lastPollTime[task.taskId] = now;

            // 转为请求，插到 Monitoring 队列
            CommRequest req;
            req.busId = config_.busId;
            req.deviceId = task.nodeId;
            req.commandType = QStringLiteral("poll_%1").arg(task.paramName);
            req.priority = task.priority;
            req.timeoutMs = 100;

            requestQueues_[static_cast<int>(Priority::Monitoring)].enqueue(req);
        }
    }
}

CommResult CanBusWorker::executeRequest(const CommRequest &req)
{
    CommResult result;
    result.requestId = req.requestId;

    QElapsedTimer timer;
    timer.start();

    if (!connected_ || !frameIO_->isOpen()) {
        result.success = false;
        result.errorMessage = QStringLiteral("总线未连接");
        return result;
    }

    // 从 payload 中提取 CAN ID 和数据
    if (req.payload.size() < 9) {
        // payload 格式: [canId(4)] [data(8)] [dlc(1)]
        // 对于简单命令，可能直接在 protocol 层编码好了
        result.success = false;
        result.errorMessage = QStringLiteral("Payload 格式错误");
        return result;
    }

    const uint8_t *raw = reinterpret_cast<const uint8_t *>(req.payload.constData());
    uint32_t canId = (static_cast<uint32_t>(raw[0]) << 24) |
                     (static_cast<uint32_t>(raw[1]) << 16) |
                     (static_cast<uint32_t>(raw[2]) << 8)  |
                     static_cast<uint32_t>(raw[3]);
    uint8_t dlc = raw[12];

    uint64_t lastSeq = frameIO_->currentSequence();

    if (!frameIO_->sendFrame(canId, &raw[4], dlc)) {
        result.success = false;
        result.errorMessage = QStringLiteral("发送失败");
        result.latencyMs = timer.elapsed();
        return result;
    }

    // 如果不需要响应（如 NMT），直接返回成功
    if (!req.responseMatcher) {
        result.success = true;
        result.latencyMs = timer.elapsed();
        return result;
    }

    // 等待响应
    int remainingMs = req.timeoutMs;
    while (remainingMs > 0) {
        CanFrame rxFrame;
        if (frameIO_->waitForFrame(lastSeq, rxFrame, remainingMs)) {
            lastSeq = rxFrame.sequence;

            if (req.responseMatcher(rxFrame)) {
                result.success = true;
                result.responseFrame = rxFrame;
                result.latencyMs = timer.elapsed();
                return result;
            }
        }

        remainingMs = req.timeoutMs - static_cast<int>(timer.elapsed());
    }

    // 超时
    result.success = false;
    result.errorMessage = QStringLiteral("响应超时 (%1ms)").arg(req.timeoutMs);
    result.latencyMs = timer.elapsed();

    // 重试
    if (req.retryCount < req.maxRetries) {
        CommRequest retryReq = req;
        retryReq.retryCount++;
        submitRequest(retryReq);
        log(LogEntry::Warning, QStringLiteral("请求超时，重试 %1/%2: %3")
            .arg(retryReq.retryCount).arg(retryReq.maxRetries).arg(req.commandType));
    }

    return result;
}

void CanBusWorker::log(LogEntry::Level level, const QString &msg)
{
    LogEntry entry;
    entry.level = level;
    entry.timestamp = QDateTime::currentDateTime();
    entry.source = QStringLiteral("BusWorker[%1]").arg(config_.busId);
    entry.message = msg;
    emit logMessage(entry);
}

}  // namespace dar
