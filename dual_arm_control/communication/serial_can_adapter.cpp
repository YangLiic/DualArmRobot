#include "communication/serial_can_adapter.h"
#include <cstring>

namespace dac {
SerialCanAdapter::SerialCanAdapter() = default;
SerialCanAdapter::~SerialCanAdapter() { close(); }
bool SerialCanAdapter::open(const BusConfig &cfg) {
    close();
    serial_ = new QSerialPort();
    serial_->setPortName(cfg.devicePath);
    serial_->setBaudRate(cfg.baudRate);
    if (!serial_->open(QIODevice::ReadWrite)) { delete serial_; serial_ = nullptr; return false; }
    return true;
}
void SerialCanAdapter::close() {
    if (serial_) { serial_->close(); delete serial_; serial_ = nullptr; }
    QMutexLocker lk(&mutex_); frameQueue_.clear();
}
bool SerialCanAdapter::isOpen() const { return serial_ && serial_->isOpen(); }
bool SerialCanAdapter::sendFrame(const CanFrame &frame) {
    if (!isOpen()) return false;
    uint8_t buf[17] = {0xAA, 0, 0, frame.dlc, 0, 0, (uint8_t)(frame.canId >> 8), (uint8_t)frame.canId};
    std::memcpy(&buf[8], frame.data, 8); buf[16] = 0x7A;
    return serial_->write((const char*)buf, 17) == 17;
}
void SerialCanAdapter::processIncoming() {
    if (!isOpen() || !serial_->waitForReadyRead(5)) return;
    rxBuffer_.append(serial_->readAll());
    while (rxBuffer_.size() >= 17) {
        int idx = rxBuffer_.indexOf((char)0xAA);
        if (idx < 0) { rxBuffer_.clear(); break; }
        if (idx > 0) rxBuffer_.remove(0, idx);
        if (rxBuffer_.size() < 17) break;
        if ((uint8_t)rxBuffer_[16] != 0x7A) { rxBuffer_.remove(0, 1); continue; }
        CanFrame f; f.dlc = rxBuffer_[3]; f.canId = ((uint8_t)rxBuffer_[6] << 8) | (uint8_t)rxBuffer_[7];
        std::memcpy(f.data, rxBuffer_.constData() + 8, 8);
        { QMutexLocker lk(&mutex_); f.sequence = ++sequence_; frameQueue_.push_back(f); }
        frameCond_.wakeAll(); rxBuffer_.remove(0, 17);
    }
}
bool SerialCanAdapter::waitForFrame(CanFrame &out, int timeoutMs) {
    QMutexLocker lk(&mutex_);
    if (frameQueue_.empty() && !frameCond_.wait(&mutex_, timeoutMs)) return false;
    if (frameQueue_.empty()) return false;
    out = frameQueue_.front(); frameQueue_.pop_front(); return true;
}
uint64_t SerialCanAdapter::currentSequence() const { QMutexLocker lk(&mutex_); return sequence_; }
} // namespace dac
