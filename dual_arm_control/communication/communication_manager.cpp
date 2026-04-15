#include "communication/communication_manager.h"
#include <QElapsedTimer>
#include <QDateTime>

namespace dac {

CommunicationManager::CommunicationManager(QObject *parent)
    : QObject(parent)
{
    qRegisterMetaType<dac::CommRequest>("dac::CommRequest");
    qRegisterMetaType<dac::CommResult>("dac::CommResult");
    qRegisterMetaType<dac::CanFrame>("dac::CanFrame");
    qRegisterMetaType<dac::LogLevel>("dac::LogLevel");

    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(20);
    connect(pollTimer_, &QTimer::timeout, this, &CommunicationManager::onPollTimerTick);
}

CommunicationManager::~CommunicationManager()
{
    closeBus();
}

bool CommunicationManager::openBus(const BusConfig &cfg)
{
    if (busOpen_) closeBus();

    worker_ = new CanBusWorker();
    worker_->configure(cfg);
    workerThread_ = new QThread(this);
    worker_->moveToThread(workerThread_);

    connect(workerThread_, &QThread::started, worker_, &CanBusWorker::start);
    connect(worker_, &CanBusWorker::busOpened, this, &CommunicationManager::onBusOpened);
    connect(worker_, &CanBusWorker::requestCompleted, this, &CommunicationManager::onRequestCompleted);
    connect(worker_, &CanBusWorker::frameReceived, this, &CommunicationManager::onRawFrame);
    connect(worker_, &CanBusWorker::logMessage, this, &CommunicationManager::logMessage);
    connect(workerThread_, &QThread::finished, worker_, &QObject::deleteLater);

    workerThread_->start();
    return true;
}

void CommunicationManager::closeBus()
{
    pollTimer_->stop();
    if (worker_ && workerThread_ && workerThread_->isRunning()) {
        // 同步调用 stop()，确保 worker 停止定时器和串口
        QMetaObject::invokeMethod(worker_, "stop", Qt::BlockingQueuedConnection);
    }
    if (workerThread_) {
        workerThread_->quit();
        if (!workerThread_->wait(3000)) {
            workerThread_->terminate();
            workerThread_->wait();
        }
        delete workerThread_;
        workerThread_ = nullptr;
    }
    worker_ = nullptr;  // already deleted by QThread::finished → deleteLater
    busOpen_ = false;
    emit busStateChanged(false, QStringLiteral("总线已关闭"));
}

bool CommunicationManager::isBusOpen() const
{
    return busOpen_;
}

void CommunicationManager::submitRequest(const CommRequest &req,
                                          std::function<void(const CommResult &)> callback)
{
    if (!worker_ || !busOpen_) {
        if (callback) {
            CommResult fail;
            fail.requestId = req.requestId;
            fail.success = false;
            fail.errorMessage = QStringLiteral("总线未打开");
            callback(fail);
        }
        return;
    }
    CommRequest r = req;
    {
        QMutexLocker lk(&callbackMutex_);
        r.requestId = nextReqId_++;
        if (callback)
            pendingCallbacks_[r.requestId] = callback;
    }
    QMetaObject::invokeMethod(worker_, "submitRequest", Qt::QueuedConnection,
                              Q_ARG(dac::CommRequest, r));
}

int CommunicationManager::registerPollTask(const CommRequest &templateReq, int intervalMs,
                                            std::function<void(const CommResult &)> callback)
{
    PollTask task;
    task.id = nextPollId_++;
    task.intervalMs = intervalMs;
    task.active = true;
    task.templateReq = templateReq;
    task.callback = callback;
    pollTasks_[task.id] = task;
    pollLastFired_[task.id] = 0;

    if (!pollTimer_->isActive())
        pollTimer_->start();
    return task.id;
}

void CommunicationManager::unregisterPollTask(int taskId)
{
    pollTasks_.remove(taskId);
    pollLastFired_.remove(taskId);
    if (pollTasks_.isEmpty())
        pollTimer_->stop();
}

void CommunicationManager::setAllPollActive(bool active)
{
    for (auto &task : pollTasks_)
        task.active = active;
    if (active && !pollTimer_->isActive())
        pollTimer_->start();
}

void CommunicationManager::onBusOpened(bool ok, const QString &msg)
{
    busOpen_ = ok;
    emit busStateChanged(ok, msg);
    emit logMessage(ok ? LogLevel::Info : LogLevel::Error, msg);
}

void CommunicationManager::onRequestCompleted(const dac::CommResult &result)
{
    std::function<void(const CommResult&)> cb;
    {
        QMutexLocker lk(&callbackMutex_);
        auto it = pendingCallbacks_.find(result.requestId);
        if (it != pendingCallbacks_.end()) {
            cb = *it;
            pendingCallbacks_.erase(it);
        }
    }
    if (cb) cb(result);
}

void CommunicationManager::onRawFrame(const dac::CanFrame &frame)
{
    emit rawFrameReceived(frame);
}

void CommunicationManager::onPollTimerTick()
{
    if (!busOpen_) return;

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    for (auto &task : pollTasks_) {
        if (!task.active) continue;
        qint64 last = pollLastFired_.value(task.id, 0);
        if (now - last >= task.intervalMs) {
            pollLastFired_[task.id] = now;
            submitRequest(task.templateReq, task.callback);
        }
    }
}

} // namespace dac
