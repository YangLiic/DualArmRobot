#include "communication/CanBusWorker.h"

#include "communication/SerialCanFrameCodec.h"

#include <QMetaObject>
#include <QTimer>

namespace {

QString requestSummary(const BusRequest &request)
{
    return QStringLiteral("%1 %2 %3")
        .arg(priorityToString(request.priority))
        .arg(commandTypeToString(request.commandType))
        .arg(formatCanId(request.deviceId));
}

} // namespace

CanBusWorker::CanBusWorker(const BusConfig &config, QObject *parent)
    : AbstractBusWorker(parent)
    , m_config(config)
{
}

CanBusWorker::~CanBusWorker()
{
    disconnectBus();
}

void CanBusWorker::connectBus()
{
    if (m_connected) {
        emit busConnectionChanged(m_config.busId, true, QStringLiteral("Bus already connected"));
        return;
    }

    if (!ensurePort()) {
        emit busConnectionChanged(m_config.busId, false, QStringLiteral("Failed to create serial port"));
        return;
    }

    m_port->setPortName(m_config.devicePath);
    m_port->setBaudRate(m_config.baudRate);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_port->open(QIODevice::ReadWrite)) {
        const QString error = QStringLiteral("Open %1 failed: %2")
            .arg(m_config.devicePath, m_port->errorString());
        log(error);
        emit busConnectionChanged(m_config.busId, false, error);
        return;
    }

    m_port->clear(QSerialPort::AllDirections);
    m_connected = true;
    log(QStringLiteral("Connected to %1 @ %2 baud").arg(m_config.devicePath).arg(m_config.baudRate));
    emit busConnectionChanged(m_config.busId, true, QStringLiteral("Connected"));
    QMetaObject::invokeMethod(this, &CanBusWorker::processNextRequest, Qt::QueuedConnection);
}

void CanBusWorker::disconnectBus()
{
    if (m_port && m_port->isOpen()) {
        m_port->clear(QSerialPort::AllDirections);
        m_port->close();
    }

    const bool wasConnected = m_connected;
    m_connected = false;
    m_busy = false;
    if (m_timeoutTimer) {
        m_timeoutTimer->stop();
    }
    m_rxBuffer.clear();
    failQueuedRequests(QStringLiteral("Bus disconnected"));

    if (wasConnected) {
        log(QStringLiteral("Disconnected from %1").arg(m_config.devicePath));
        emit busConnectionChanged(m_config.busId, false, QStringLiteral("Disconnected"));
    }
}

void CanBusWorker::submitRequest(const BusRequest &request)
{
    if (!m_connected) {
        BusResult result;
        result.requestId = request.requestId;
        result.busId = request.busId;
        result.deviceId = request.deviceId;
        result.commandType = request.commandType;
        result.context = request.context;
        result.rawRequest = request.payload;
        result.success = false;
        result.errorMessage = QStringLiteral("Bus is not connected");
        emit requestCompleted(result);
        return;
    }

    if (!request.dedupKey.isEmpty() && m_pendingDedupKeys.contains(request.dedupKey)) {
        return;
    }

    if (!request.dedupKey.isEmpty()) {
        m_pendingDedupKeys.insert(request.dedupKey);
    }

    insertRequestByPriority(request);
    QMetaObject::invokeMethod(this, &CanBusWorker::processNextRequest, Qt::QueuedConnection);
}

void CanBusWorker::onReadyRead()
{
    if (!m_port) {
        return;
    }

    m_rxBuffer.append(m_port->readAll());
    const QList<CanFrame> frames = SerialCanFrameCodec::parse(m_rxBuffer, m_sequenceCounter);

    for (const CanFrame &frame : frames) {
        if (!m_busy) {
            continue;
        }

        if (!frameMatches(frame, m_activeRequest.matcher)) {
            continue;
        }

        const quint8 specifier = frame.data.isEmpty() ? 0 : static_cast<quint8>(frame.data.at(0));
        if (specifier == 0x80) {
            finishRequest(false, QStringLiteral("SDO abort frame from %1").arg(formatCanId(frame.canId)), frame.data);
        } else {
            finishRequest(true, QString(), frame.data);
        }
        break;
    }
}

void CanBusWorker::onRequestTimeout()
{
    if (!m_busy) {
        return;
    }

    if (m_retryCount < m_activeRequest.maxRetries) {
        ++m_retryCount;
        BusRequest retryRequest = m_activeRequest;
        retryRequest.context.insert(QStringLiteral("_retryCount"), m_retryCount);
        log(QStringLiteral("Retry %1/%2 for %3")
                .arg(m_retryCount)
                .arg(m_activeRequest.maxRetries)
                .arg(requestSummary(m_activeRequest)));
        m_busy = false;
        m_activeRequest = {};
        insertRequestByPriority(retryRequest);
        QMetaObject::invokeMethod(this, &CanBusWorker::processNextRequest, Qt::QueuedConnection);
        return;
    }

    finishRequest(false, QStringLiteral("Request timeout"));
}

void CanBusWorker::processNextRequest()
{
    if (!m_connected || m_busy || m_queue.isEmpty()) {
        return;
    }

    m_activeRequest = m_queue.takeFirst();
    m_retryCount = m_activeRequest.context.value(QStringLiteral("_retryCount"), 0).toInt();
    m_busy = true;
    m_latencyTimer.restart();

    if (!m_port || !m_port->isOpen()) {
        finishRequest(false, QStringLiteral("Serial port unavailable"));
        return;
    }

    const QByteArray wireFrame = SerialCanFrameCodec::encode(m_activeRequest.deviceId, m_activeRequest.payload);
    const qint64 written = m_port->write(wireFrame);
    if (written != wireFrame.size() || !m_port->waitForBytesWritten(m_config.timeoutMs)) {
        finishRequest(false, QStringLiteral("Write failed: %1").arg(m_port->errorString()));
        return;
    }

    log(QStringLiteral("TX %1").arg(requestSummary(m_activeRequest)));

    if (!m_activeRequest.matcher.enabled) {
        const int postDelayMs = m_activeRequest.context.value(QStringLiteral("postDelayMs"), 0).toInt();
        if (postDelayMs > 0) {
            QTimer::singleShot(postDelayMs, this, [this]() {
                finishRequest(true, QString());
            });
        } else {
            finishRequest(true, QString());
        }
        return;
    }

    if (!m_timeoutTimer) {
        m_timeoutTimer = new QTimer(this);
        m_timeoutTimer->setSingleShot(true);
        connect(m_timeoutTimer, &QTimer::timeout, this, &CanBusWorker::onRequestTimeout);
    }
    m_timeoutTimer->start(m_activeRequest.timeoutMs);
}

bool CanBusWorker::ensurePort()
{
    if (m_port) {
        return true;
    }

    m_port = new QSerialPort(this);
    connect(m_port, &QSerialPort::readyRead, this, &CanBusWorker::onReadyRead);
    return true;
}

bool CanBusWorker::frameMatches(const CanFrame &frame, const ResponseMatcher &matcher) const
{
    if (!matcher.enabled) {
        return false;
    }

    if (matcher.expectedCanId >= 0 && frame.canId != static_cast<quint32>(matcher.expectedCanId)) {
        return false;
    }

    if (frame.data.size() < matcher.minimumPayloadLength) {
        return false;
    }

    if (matcher.expectedIndex >= 0) {
        if (frame.data.size() < 4) {
            return false;
        }
        const int index = static_cast<quint8>(frame.data.at(1))
            | (static_cast<quint8>(frame.data.at(2)) << 8);
        if (index != matcher.expectedIndex) {
            return false;
        }
    }

    if (matcher.expectedSubIndex >= 0) {
        if (frame.data.size() < 4) {
            return false;
        }
        if (static_cast<quint8>(frame.data.at(3)) != matcher.expectedSubIndex) {
            return false;
        }
    }

    if (!matcher.acceptedSpecifiers.isEmpty()) {
        const quint8 specifier = frame.data.isEmpty() ? 0 : static_cast<quint8>(frame.data.at(0));
        if (specifier == 0x80 && matcher.acceptAbortFrame) {
            return true;
        }
        if (!matcher.acceptedSpecifiers.contains(static_cast<char>(specifier))) {
            return false;
        }
    }

    return true;
}

void CanBusWorker::finishRequest(bool success, const QString &errorMessage, const QByteArray &response)
{
    const BusRequest completedRequest = m_activeRequest;

    BusResult result;
    result.requestId = completedRequest.requestId;
    result.busId = completedRequest.busId;
    result.deviceId = completedRequest.deviceId;
    result.commandType = completedRequest.commandType;
    result.context = completedRequest.context;
    result.rawRequest = completedRequest.payload;
    result.rawResponse = response;
    result.success = success;
    result.errorMessage = errorMessage;
    result.latencyMs = m_latencyTimer.isValid() ? m_latencyTimer.elapsed() : 0;

    if (m_timeoutTimer) {
        m_timeoutTimer->stop();
    }
    m_busy = false;
    m_activeRequest = {};

    if (!completedRequest.dedupKey.isEmpty()) {
        m_pendingDedupKeys.remove(completedRequest.dedupKey);
    }

    if (!success) {
        log(QStringLiteral("FAIL %1: %2").arg(requestSummary(completedRequest), errorMessage));
    }

    emit requestCompleted(result);
    QMetaObject::invokeMethod(this, &CanBusWorker::processNextRequest, Qt::QueuedConnection);
}

void CanBusWorker::failQueuedRequests(const QString &reason)
{
    if (m_busy) {
        BusRequest active = m_activeRequest;
        m_busy = false;
        m_activeRequest = {};
        if (!active.dedupKey.isEmpty()) {
            m_pendingDedupKeys.remove(active.dedupKey);
        }

        BusResult result;
        result.requestId = active.requestId;
        result.busId = active.busId;
        result.deviceId = active.deviceId;
        result.commandType = active.commandType;
        result.context = active.context;
        result.rawRequest = active.payload;
        result.success = false;
        result.errorMessage = reason;
        emit requestCompleted(result);
    }

    while (!m_queue.isEmpty()) {
        const BusRequest request = m_queue.takeFirst();
        if (!request.dedupKey.isEmpty()) {
            m_pendingDedupKeys.remove(request.dedupKey);
        }

        BusResult result;
        result.requestId = request.requestId;
        result.busId = request.busId;
        result.deviceId = request.deviceId;
        result.commandType = request.commandType;
        result.context = request.context;
        result.rawRequest = request.payload;
        result.success = false;
        result.errorMessage = reason;
        emit requestCompleted(result);
    }
}

void CanBusWorker::log(const QString &message)
{
    emit logMessage(QStringLiteral("[%1] %2").arg(m_config.busId, message));
}

void CanBusWorker::insertRequestByPriority(const BusRequest &request)
{
    const int requestRank = priorityRank(request.priority);
    int insertIndex = m_queue.size();
    for (int i = 0; i < m_queue.size(); ++i) {
        if (requestRank > priorityRank(m_queue.at(i).priority)) {
            insertIndex = i;
            break;
        }
    }
    m_queue.insert(insertIndex, request);
}

int CanBusWorker::priorityRank(RequestPriority priority) const
{
    switch (priority) {
    case RequestPriority::Critical:
        return 3;
    case RequestPriority::Control:
        return 2;
    case RequestPriority::Monitoring:
    default:
        return 1;
    }
}
