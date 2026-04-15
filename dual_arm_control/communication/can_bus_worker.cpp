#include "communication/can_bus_worker.h"
#include <QThread>
#include <QElapsedTimer>
#include <cstring>

namespace dac {

CanBusWorker::CanBusWorker(QObject *parent)
    : QObject(parent)
{}

CanBusWorker::~CanBusWorker()
{
    stop();
}

void CanBusWorker::configure(const BusConfig &cfg)
{
    config_ = cfg;
}

void CanBusWorker::start()
{
    io_ = new FrameIO(this);
    if (!io_->openPort(config_.devicePath, config_.baudRate)) {
        emit busOpened(false, QStringLiteral("串口打开失败: %1").arg(config_.devicePath));
        return;
    }
    running_ = true;
    emit busOpened(true, QStringLiteral("总线已连接: %1").arg(config_.devicePath));
    emit logMessage(LogLevel::Info, QStringLiteral("CAN 总线已打开: %1 @ %2 baud")
                        .arg(config_.devicePath).arg(config_.baudRate));

    tickTimer_ = new QTimer(this);
    tickTimer_->setInterval(2);
    connect(tickTimer_, &QTimer::timeout, this, &CanBusWorker::onTick);
    tickTimer_->start();
}

void CanBusWorker::stop()
{
    running_ = false;
    if (tickTimer_) {
        tickTimer_->stop();
        tickTimer_->deleteLater();
        tickTimer_ = nullptr;
    }
    if (io_) {
        io_->closePort();
        io_->deleteLater();
        io_ = nullptr;
    }
    emit busClosed();
}

void CanBusWorker::submitRequest(const dac::CommRequest &req)
{
    QMutexLocker lk(&queueMutex_);
    CommRequest r = req;
    if (r.requestId == 0)
        r.requestId = nextReqId_++;

    if (r.priority == Priority::Critical) {
        requestQueue_.prepend(r);
    } else if (r.priority == Priority::Control) {
        int insertPos = 0;
        for (int i = 0; i < requestQueue_.size(); ++i) {
            if (requestQueue_[i].priority == Priority::Critical) {
                insertPos = i + 1;
            } else {
                break;
            }
        }
        requestQueue_.insert(insertPos, r);
    } else {
        requestQueue_.enqueue(r);
    }
}

void CanBusWorker::onTick()
{
    if (!running_) return;

    if (io_ && io_->isOpen()) {
        io_->processIncoming();
    }

    processNextRequest();
}

void CanBusWorker::processNextRequest()
{
    CommRequest req;
    {
        QMutexLocker lk(&queueMutex_);
        if (requestQueue_.isEmpty()) return;
        req = requestQueue_.dequeue();
    }

    CommResult result = executeRequest(req);
    emit requestCompleted(result);
}

CommResult CanBusWorker::executeRequest(const CommRequest &req)
{
    CommResult result;
    result.requestId = req.requestId;

    QElapsedTimer timer;
    timer.start();

    CanFrame txFrame;
    txFrame.canId = req.canId;
    txFrame.dlc = req.payloadLen;
    std::memcpy(txFrame.data, req.payload, 8);

    if (!io_ || !io_->sendFrame(txFrame)) {
        result.success = false;
        result.errorMessage = QStringLiteral("发送失败");
        result.latencyMs = static_cast<int>(timer.elapsed());
        return result;
    }

    if (!req.expectResponse) {
        QThread::msleep(10);
        result.success = true;
        result.latencyMs = static_cast<int>(timer.elapsed());
        return result;
    }

    auto deadline = timer.elapsed() + req.timeoutMs;
    while (timer.elapsed() < deadline) {
        if (io_) io_->processIncoming();

        CanFrame rx;
        int remaining = static_cast<int>(deadline - timer.elapsed());
        if (remaining <= 0) break;

        if (io_->waitForFrame(rx, qMin(remaining, 50))) {
            emit frameReceived(rx);

            if (matchResponse(rx, req)) {
                result.success = true;
                result.dlc = rx.dlc;
                std::memcpy(result.data, rx.data, 8);
                result.latencyMs = static_cast<int>(timer.elapsed());

                if (rx.data[0] == 0x80) {
                    result.success = false;
                    result.errorMessage = QStringLiteral("SDO 被驱动器拒绝");
                }
                return result;
            }
        }
    }

    result.success = false;
    result.errorMessage = QStringLiteral("响应超时 (%1ms)").arg(req.timeoutMs);
    result.latencyMs = static_cast<int>(timer.elapsed());
    return result;
}

bool CanBusWorker::matchResponse(const CanFrame &frame, const CommRequest &req) const
{
    if (frame.canId != req.responseCanId) return false;
    if (frame.dlc < 4) return false;
    if (frame.data[1] != (req.sdoIndex & 0xFF)) return false;
    if (frame.data[2] != ((req.sdoIndex >> 8) & 0xFF)) return false;
    if (frame.data[3] != req.sdoSubindex) return false;
    return true;
}

} // namespace dac
