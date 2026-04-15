#include "communication/frame_io.h"
#include <QSerialPortInfo>
#include <cstring>

namespace dac {

FrameIO::FrameIO(QObject *parent)
    : QObject(parent)
{}

FrameIO::~FrameIO()
{
    closePort();
}

bool FrameIO::openPort(const QString &portName, int baudRate)
{
    closePort();
    serial_ = new QSerialPort(this);
    serial_->setPortName(portName);
    serial_->setBaudRate(baudRate);
    serial_->setDataBits(QSerialPort::Data8);
    serial_->setParity(QSerialPort::NoParity);
    serial_->setStopBits(QSerialPort::OneStop);
    serial_->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial_->open(QIODevice::ReadWrite)) {
        emit errorOccurred(QStringLiteral("打开串口失败: %1 - %2")
                               .arg(portName, serial_->errorString()));
        delete serial_;
        serial_ = nullptr;
        return false;
    }
    serial_->clear();
    return true;
}

void FrameIO::closePort()
{
    if (serial_) {
        if (serial_->isOpen()) serial_->close();
        delete serial_;
        serial_ = nullptr;
    }
    rxBuffer_.clear();
    QMutexLocker lk(&mutex_);
    frameQueue_.clear();
}

bool FrameIO::isOpen() const
{
    return serial_ && serial_->isOpen();
}

bool FrameIO::sendFrame(const CanFrame &frame)
{
    if (!isOpen()) return false;

    uint8_t buf[FRAME_LEN];
    buf[0] = FRAME_HEAD;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = frame.dlc;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = (frame.canId >> 8) & 0xFF;
    buf[7] = frame.canId & 0xFF;
    std::memcpy(&buf[8], frame.data, 8);
    buf[16] = FRAME_TAIL;

    qint64 written = serial_->write(reinterpret_cast<const char*>(buf), FRAME_LEN);
    serial_->flush();
    return (written == FRAME_LEN);
}

void FrameIO::processIncoming()
{
    if (!isOpen()) return;
    if (!serial_->waitForReadyRead(5)) return;

    rxBuffer_.append(serial_->readAll());
    while (parseBuffer()) { /* keep parsing */ }
}

bool FrameIO::parseBuffer()
{
    while (rxBuffer_.size() >= FRAME_LEN) {
        int idx = rxBuffer_.indexOf(static_cast<char>(FRAME_HEAD));
        if (idx < 0) {
            rxBuffer_.clear();
            return false;
        }
        if (idx > 0) {
            rxBuffer_.remove(0, idx);
        }
        if (rxBuffer_.size() < FRAME_LEN) return false;

        auto raw = reinterpret_cast<const uint8_t*>(rxBuffer_.constData());
        if (raw[16] != FRAME_TAIL) {
            rxBuffer_.remove(0, 1);
            continue;
        }

        CanFrame f;
        f.dlc = raw[3];
        f.canId = (static_cast<uint32_t>(raw[6]) << 8) | raw[7];
        std::memcpy(f.data, &raw[8], 8);

        {
            QMutexLocker lk(&mutex_);
            ++sequence_;
            f.sequence = sequence_;
            frameQueue_.push_back(f);
            if (frameQueue_.size() > 512)
                frameQueue_.pop_front();
        }
        frameCond_.wakeAll();
        emit frameReceived(f);

        rxBuffer_.remove(0, FRAME_LEN);
    }
    return false;
}

bool FrameIO::waitForFrame(CanFrame &out, int timeoutMs)
{
    QMutexLocker lk(&mutex_);
    if (!frameQueue_.empty()) {
        out = frameQueue_.front();
        frameQueue_.pop_front();
        return true;
    }
    if (!frameCond_.wait(&mutex_, timeoutMs))
        return false;
    if (frameQueue_.empty()) return false;
    out = frameQueue_.front();
    frameQueue_.pop_front();
    return true;
}

uint64_t FrameIO::currentSequence() const
{
    QMutexLocker lk(&mutex_);
    return sequence_;
}

} // namespace dac
