#include "services/gripper_worker.h"
#include <QDateTime>
#include <QElapsedTimer>
#include <QIODevice>
#include <QSerialPort>
#include <QStringList>
#include <QTcpSocket>
#include <QTimer>
#include <QtGlobal>
#include <algorithm>

namespace {

constexpr quint16 kTcpAngleSetReg = 1486;
constexpr quint16 kTcpForceSetReg = 1498;
constexpr quint16 kTcpSpeedSetReg = 1522;
constexpr quint16 kTcpAngleActReg = 1546;

constexpr quint16 kSerialAngleSetReg = 1486;
constexpr quint16 kSerialForceSetReg = 1498;
constexpr quint16 kSerialSpeedSetReg = 1522;
constexpr quint16 kSerialAngleActReg = 1546;

QByteArray readExactFromSocket(QTcpSocket *socket, int bytesNeeded, int timeoutMs)
{
    QByteArray data;
    QElapsedTimer timer;
    timer.start();
    while (data.size() < bytesNeeded && timer.elapsed() < timeoutMs) {
        const qint64 remaining = bytesNeeded - data.size();
        const QByteArray chunk = socket->read(remaining);
        if (!chunk.isEmpty()) {
            data += chunk;
            continue;
        }

        const int waitMs = std::max(1, timeoutMs - static_cast<int>(timer.elapsed()));
        if (!socket->waitForReadyRead(waitMs)) {
            break;
        }
    }
    return data;
}

QString formatAngles(const QVector<int> &angles)
{
    QStringList parts;
    parts.reserve(angles.size());
    for (int angle : angles) {
        parts << QString::number(angle);
    }
    return parts.join(QStringLiteral(", "));
}

} // namespace

namespace dac {

GripperWorker::GripperWorker(QObject *parent)
    : QObject(parent)
{
    state_.side = side_;
}

GripperWorker::~GripperWorker()
{
    disconnectDevice();
}

void GripperWorker::configure(dac::ArmSide side, const dac::GripperEndpointConfig &cfg)
{
    side_ = side;
    cfg_ = cfg;
    state_.side = side;
    state_.transport = cfg.transport;

    if (pollTimer_) {
        pollTimer_->setInterval(std::max(20, cfg_.pollIntervalMs));
    }
}

void GripperWorker::connectDevice()
{
    disconnectDevice();

    if (!openCurrentTransport()) {
        updateConnectionState(false, QStringLiteral("%1连接失败").arg(gripperTransportText(cfg_.transport)));
        return;
    }

    if (!initializeDevice()) {
        closeCurrentTransport();
        updateConnectionState(false, QStringLiteral("%1初始化失败").arg(gripperTransportText(cfg_.transport)));
        return;
    }

    if (!pollTimer_) {
        pollTimer_ = new QTimer(this);
        connect(pollTimer_, &QTimer::timeout, this, &GripperWorker::refreshState);
    }
    pollTimer_->setInterval(std::max(20, cfg_.pollIntervalMs));
    pollTimer_->start();

    updateConnectionState(true,
                          QStringLiteral("%1已连接 (%2)")
                              .arg(armSideText(side_))
                              .arg(gripperTransportText(cfg_.transport)));
    refreshState();
}

void GripperWorker::disconnectDevice()
{
    if (pollTimer_) {
        pollTimer_->stop();
    }

    const bool wasConnected = state_.connected;
    closeCurrentTransport();
    state_.connected = false;
    state_.online = false;

    if (wasConnected) {
        state_.detailText = QStringLiteral("已断开");
        emitState();
        emit connectionChanged(side_, false, QStringLiteral("%1已断开").arg(armSideText(side_)));
        emit logMessage(LogLevel::Info,
                        QStringLiteral("%1夹爪已断开").arg(armSideText(side_)));
    }
}

void GripperWorker::setTarget(double normalized)
{
    if (!state_.connected) {
        emit logMessage(LogLevel::Error,
                        QStringLiteral("%1夹爪未连接，无法下发目标")
                            .arg(armSideText(side_)));
        return;
    }

    const double clamped = clamp01(normalized);
    state_.targetNormalized = clamped;
    bool ok = false;

    if (cfg_.transport == GripperTransport::TCP) {
        ok = writeTcpPosition(clamped);
    } else {
        ok = writeRtuPosition(clamped);
    }

    if (!ok) {
        emit logMessage(LogLevel::Error,
                        QStringLiteral("%1夹爪目标下发失败")
                            .arg(armSideText(side_)));
        state_.online = false;
        emitState();
        return;
    }

    emit logMessage(LogLevel::Info,
                    QStringLiteral("%1夹爪目标已下发: %2")
                        .arg(armSideText(side_))
                        .arg(clamped, 0, 'f', 3));
    emitState();
    QTimer::singleShot(80, this, [this]() { refreshState(); });
}

void GripperWorker::refreshState()
{
    if (!state_.connected) {
        return;
    }

    const bool ok = (cfg_.transport == GripperTransport::TCP)
        ? readTcpState()
        : readRtuState();

    if (!ok) {
        state_.online = false;
        state_.lastUpdateTime = QDateTime::currentDateTime();
        emitState();
    }
}

bool GripperWorker::openCurrentTransport()
{
    return cfg_.transport == GripperTransport::TCP ? openTcp() : openRtu();
}

bool GripperWorker::openTcp()
{
    if (!tcpSocket_) {
        tcpSocket_ = new QTcpSocket(this);
    }

    transactionId_ = 1;
    tcpSocket_->abort();
    tcpSocket_->connectToHost(cfg_.ip, static_cast<quint16>(cfg_.tcpPort));
    if (!tcpSocket_->waitForConnected(1200)) {
        emit logMessage(LogLevel::Error,
                        QStringLiteral("%1夹爪 TCP 连接失败: %2:%3")
                            .arg(armSideText(side_))
                            .arg(cfg_.ip)
                            .arg(cfg_.tcpPort));
        return false;
    }
    return true;
}

bool GripperWorker::openRtu()
{
    if (!serialPort_) {
        serialPort_ = new QSerialPort(this);
    }

    if (serialPort_->isOpen()) {
        serialPort_->close();
    }

    serialPort_->setPortName(cfg_.serialPort);
    serialPort_->setBaudRate(cfg_.serialBaudRate);
    serialPort_->setDataBits(QSerialPort::Data8);
    serialPort_->setParity(QSerialPort::NoParity);
    serialPort_->setStopBits(QSerialPort::OneStop);
    serialPort_->setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort_->open(QIODevice::ReadWrite)) {
        emit logMessage(LogLevel::Error,
                        QStringLiteral("%1夹爪串口打开失败: %2")
                            .arg(armSideText(side_))
                            .arg(cfg_.serialPort));
        return false;
    }
    serialPort_->clear();
    return true;
}

void GripperWorker::closeCurrentTransport()
{
    if (tcpSocket_) {
        tcpSocket_->abort();
    }
    if (serialPort_ && serialPort_->isOpen()) {
        serialPort_->close();
    }
}

bool GripperWorker::initializeDevice()
{
    return cfg_.transport == GripperTransport::TCP ? initTcpDevice() : initRtuDevice();
}

bool GripperWorker::initTcpDevice()
{
    const QVector<quint16> speedValues(6, static_cast<quint16>(std::clamp(cfg_.speed, 10, 1000)));
    const QVector<quint16> forceValues(6, static_cast<quint16>(std::clamp(cfg_.force, 100, 1000)));
    return writeMultipleRegistersTcp(kTcpSpeedSetReg, speedValues)
        && writeMultipleRegistersTcp(kTcpForceSetReg, forceValues);
}

bool GripperWorker::initRtuDevice()
{
    const QVector<quint16> speedValues(6, static_cast<quint16>(std::clamp(cfg_.speed, 10, 1000)));
    const QVector<quint16> forceValues(6, static_cast<quint16>(std::clamp(cfg_.force, 100, 1000)));
    return writeSerialRegisters(kSerialSpeedSetReg, speedValues)
        && writeSerialRegisters(kSerialForceSetReg, forceValues);
}

bool GripperWorker::writeTcpPosition(double normalized)
{
    return writeMultipleRegistersTcp(kTcpAngleSetReg, interpolateTcpPose(normalized));
}

bool GripperWorker::writeRtuPosition(double normalized)
{
    return writeSerialRegisters(kSerialAngleSetReg, interpolateSerialPose(normalized));
}

bool GripperWorker::readTcpState()
{
    const QVector<quint16> registers = readHoldingRegistersTcp(kTcpAngleActReg, 6);
    if (registers.size() != 6) {
        return false;
    }

    QVector<int> angles;
    angles.reserve(registers.size());
    for (quint16 reg : registers) {
        angles.push_back(static_cast<int>(reg));
    }

    state_.transport = GripperTransport::TCP;
    state_.online = true;
    state_.fingerAngles = angles;
    state_.actualNormalized = normalizedFromTcpAngles(angles);
    state_.detailText = QStringLiteral("angles=[%1]").arg(formatAngles(angles));
    state_.lastUpdateTime = QDateTime::currentDateTime();
    emitState();
    return true;
}

bool GripperWorker::readRtuState()
{
    const QVector<quint16> registers = readSerialRegisters(kSerialAngleActReg, 6);
    if (registers.size() != 6) {
        return false;
    }

    QVector<int> angles;
    angles.reserve(registers.size());
    for (quint16 reg : registers) {
        angles.push_back(static_cast<int>(reg));
    }

    state_.transport = GripperTransport::RTU;
    state_.online = true;
    state_.fingerAngles = angles;
    state_.actualNormalized = normalizedFromSerialAngles(angles);
    state_.statusCode = 0;
    state_.detailText = QStringLiteral("angles=[%1]").arg(formatAngles(angles));
    state_.lastUpdateTime = QDateTime::currentDateTime();
    emitState();
    return true;
}

QByteArray GripperWorker::transactTcp(quint8 func, const QByteArray &payload, int timeoutMs)
{
    if (!tcpSocket_ || tcpSocket_->state() != QAbstractSocket::ConnectedState) {
        return {};
    }

    while (tcpSocket_->bytesAvailable() > 0) {
        tcpSocket_->readAll();
    }

    const quint16 txId = transactionId_++;
    QByteArray request;
    request.reserve(7 + 1 + payload.size());
    request.append(static_cast<char>((txId >> 8) & 0xFF));
    request.append(static_cast<char>(txId & 0xFF));
    request.append('\0');
    request.append('\0');
    const quint16 length = static_cast<quint16>(2 + payload.size());
    request.append(static_cast<char>((length >> 8) & 0xFF));
    request.append(static_cast<char>(length & 0xFF));
    request.append(static_cast<char>(cfg_.slaveId & 0xFF));
    request.append(static_cast<char>(func));
    request.append(payload);

    if (tcpSocket_->write(request) != request.size()) {
        return {};
    }
    if (!tcpSocket_->waitForBytesWritten(timeoutMs)) {
        return {};
    }

    const QByteArray header = readExactFromSocket(tcpSocket_, 7, timeoutMs);
    if (header.size() != 7) {
        return {};
    }

    const quint16 respLength = (static_cast<quint8>(header[4]) << 8) | static_cast<quint8>(header[5]);
    if (respLength < 2) {
        return {};
    }

    const QByteArray body = readExactFromSocket(tcpSocket_, respLength - 1, timeoutMs);
    if (body.size() != respLength - 1) {
        return {};
    }
    return body;
}

QByteArray GripperWorker::transactRtu(const QByteArray &frame, int timeoutMs)
{
    if (!serialPort_ || !serialPort_->isOpen()) {
        return {};
    }

    serialPort_->clear(QSerialPort::Input);
    if (serialPort_->write(frame) != frame.size()) {
        return {};
    }
    if (!serialPort_->waitForBytesWritten(timeoutMs)) {
        return {};
    }

    QByteArray reply;
    QElapsedTimer timer;
    timer.start();
    bool sawAny = false;

    while (timer.elapsed() < timeoutMs) {
        const int waitMs = std::max(1, timeoutMs - static_cast<int>(timer.elapsed()));
        if (serialPort_->waitForReadyRead(std::min(waitMs, 20))) {
            sawAny = true;
            reply += serialPort_->readAll();
            while (serialPort_->waitForReadyRead(5)) {
                reply += serialPort_->readAll();
            }
            if (reply.size() >= 5) {
                break;
            }
        } else if (sawAny) {
            break;
        }
    }

    return reply;
}

bool GripperWorker::writeSerialRegisters(quint16 startReg, const QVector<quint16> &values)
{
    if (!serialPort_ || !serialPort_->isOpen() || values.isEmpty()) {
        return false;
    }

    const QByteArray frame = buildSerialWriteFrame(startReg, values);
    const QByteArray reply = transactRtu(frame, 120);
    Q_UNUSED(reply);
    // Keep write semantics aligned with demo_485.py: the device may return
    // a short/non-standard ack, but the demo treats write completion as success
    // once bytes are sent and any immediate response is drained.
    return true;
}

bool GripperWorker::writeMultipleRegistersTcp(quint16 startReg, const QVector<quint16> &values)
{
    if (values.isEmpty()) {
        return false;
    }

    QByteArray payload;
    payload.append(static_cast<char>((startReg >> 8) & 0xFF));
    payload.append(static_cast<char>(startReg & 0xFF));
    payload.append(static_cast<char>((values.size() >> 8) & 0xFF));
    payload.append(static_cast<char>(values.size() & 0xFF));
    payload.append(static_cast<char>(values.size() * 2));
    for (quint16 value : values) {
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        payload.append(static_cast<char>(value & 0xFF));
    }

    const QByteArray reply = transactTcp(0x10, payload, 400);
    return reply.size() >= 5 && static_cast<quint8>(reply[0]) == 0x10;
}

QVector<quint16> GripperWorker::readHoldingRegistersTcp(quint16 startReg, quint16 count)
{
    QByteArray payload;
    payload.append(static_cast<char>((startReg >> 8) & 0xFF));
    payload.append(static_cast<char>(startReg & 0xFF));
    payload.append(static_cast<char>((count >> 8) & 0xFF));
    payload.append(static_cast<char>(count & 0xFF));

    const QByteArray reply = transactTcp(0x03, payload, 400);
    QVector<quint16> values;
    if (reply.size() < 2 || static_cast<quint8>(reply[0]) != 0x03) {
        return values;
    }

    const int byteCount = static_cast<quint8>(reply[1]);
    if (byteCount != count * 2 || reply.size() < 2 + byteCount) {
        return values;
    }

    values.reserve(count);
    for (int i = 0; i < count; ++i) {
        const int idx = 2 + i * 2;
        const quint16 value = (static_cast<quint8>(reply[idx]) << 8)
            | static_cast<quint8>(reply[idx + 1]);
        values.push_back(value);
    }
    return values;
}

QVector<quint16> GripperWorker::readSerialRegisters(quint16 startReg, int registerCount)
{
    QVector<quint16> values;
    if (registerCount <= 0) {
        return values;
    }

    const QByteArray reply = transactRtu(buildSerialReadFrame(startReg, static_cast<quint8>(registerCount * 2)), 150);
    QByteArray payload;
    if (!parseSerialReply(reply, 0x11, &payload)) {
        return values;
    }

    if (payload.size() < registerCount * 2) {
        return values;
    }

    values.reserve(registerCount);
    for (int i = 0; i < registerCount; ++i) {
        const int offset = i * 2;
        const quint16 value = static_cast<quint8>(payload[offset])
            | (static_cast<quint16>(static_cast<quint8>(payload[offset + 1])) << 8);
        values.push_back(value);
    }
    return values;
}

QByteArray GripperWorker::buildSerialWriteFrame(quint16 startReg, const QVector<quint16> &values) const
{
    QByteArray frame;
    frame.reserve(8 + values.size() * 2);
    frame.append(char(0xEB));
    frame.append(char(0x90));
    frame.append(static_cast<char>(cfg_.slaveId & 0xFF));
    frame.append(static_cast<char>(values.size() * 2 + 3));
    frame.append(char(0x12));
    frame.append(static_cast<char>(startReg & 0xFF));
    frame.append(static_cast<char>((startReg >> 8) & 0xFF));
    for (quint16 value : values) {
        frame.append(static_cast<char>(value & 0xFF));
        frame.append(static_cast<char>((value >> 8) & 0xFF));
    }
    frame.append(static_cast<char>(checksum(frame)));
    return frame;
}

QByteArray GripperWorker::buildSerialReadFrame(quint16 startReg, quint8 byteCount) const
{
    QByteArray frame;
    frame.reserve(9);
    frame.append(char(0xEB));
    frame.append(char(0x90));
    frame.append(static_cast<char>(cfg_.slaveId & 0xFF));
    frame.append(char(0x04));
    frame.append(char(0x11));
    frame.append(static_cast<char>(startReg & 0xFF));
    frame.append(static_cast<char>((startReg >> 8) & 0xFF));
    frame.append(static_cast<char>(byteCount));
    frame.append(static_cast<char>(checksum(frame)));
    return frame;
}

bool GripperWorker::parseSerialReply(const QByteArray &reply, quint8 expectedCmd, QByteArray *payloadOut) const
{
    if (payloadOut) {
        payloadOut->clear();
    }
    if (reply.size() < 8) {
        return false;
    }
    if (static_cast<quint8>(reply[0]) != 0xEB || static_cast<quint8>(reply[1]) != 0x90) {
        return false;
    }
    if (static_cast<quint8>(reply[2]) != static_cast<quint8>(cfg_.slaveId & 0xFF)) {
        return false;
    }
    if (static_cast<quint8>(reply[4]) != expectedCmd) {
        return false;
    }

    const int payloadLen = static_cast<quint8>(reply[3]) - 3;
    if (payloadLen < 0 || reply.size() < 7 + payloadLen + 1) {
        return false;
    }

    QByteArray frameWithoutChecksum = reply.left(7 + payloadLen);
    if (checksum(frameWithoutChecksum) != static_cast<quint8>(reply[7 + payloadLen])) {
        return false;
    }

    if (payloadOut) {
        *payloadOut = reply.mid(7, payloadLen);
    }
    return true;
}

void GripperWorker::updateConnectionState(bool connected, const QString &message)
{
    state_.connected = connected;
    state_.online = connected;
    state_.transport = cfg_.transport;
    state_.detailText = message;
    state_.lastUpdateTime = QDateTime::currentDateTime();
    emitState();
    emit connectionChanged(side_, connected, message);
    emit logMessage(connected ? LogLevel::Info : LogLevel::Error, message);
}

void GripperWorker::emitState()
{
    emit stateChanged(state_);
}

quint8 GripperWorker::checksum(const QByteArray &data)
{
    quint32 sum = 0;
    for (int i = 2; i < data.size(); ++i) {
        sum += static_cast<quint8>(data[i]);
    }
    return static_cast<quint8>(sum & 0xFF);
}

QVector<quint16> GripperWorker::interpolateTcpPose(double normalized)
{
    constexpr int kStartPose[6] = {1000, 1000, 1000, 1000, 1000, 0};
    constexpr int kEndPose[6] = {0, 0, 0, 0, 0, 0};

    const double p = clamp01(normalized);
    QVector<quint16> values;
    values.reserve(6);
    for (int i = 0; i < 6; ++i) {
        const double raw = kStartPose[i] + (kEndPose[i] - kStartPose[i]) * p;
        values.push_back(static_cast<quint16>(std::clamp(qRound(raw), 0, 1000)));
    }
    return values;
}

QVector<quint16> GripperWorker::interpolateSerialPose(double normalized)
{
    const quint16 value = static_cast<quint16>(std::clamp(qRound(clamp01(normalized) * 1000.0), 0, 1000));
    return QVector<quint16>(6, value);
}

double GripperWorker::normalizedFromTcpAngles(const QVector<int> &angles)
{
    if (angles.size() < 5) {
        return 0.0;
    }

    double sum = 0.0;
    for (int i = 0; i < 5; ++i) {
        sum += std::clamp(angles[i], 0, 1000);
    }
    const double opened = sum / 5.0;
    return clamp01(1.0 - opened / 1000.0);
}

double GripperWorker::normalizedFromSerialAngles(const QVector<int> &angles)
{
    if (angles.isEmpty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (int angle : angles) {
        sum += std::clamp(angle, 0, 1000);
    }
    return clamp01(sum / static_cast<double>(angles.size()) / 1000.0);
}

double GripperWorker::clamp01(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

} // namespace dac
