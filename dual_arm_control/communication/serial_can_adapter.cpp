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
    if (frame.isExtended) {
        // 扩展帧: AA [Type|0x20] [DLC] [00 00] [ID3 ID2 ID1 ID0] [Data×8] 7A
        uint8_t buf[17];
        buf[0] = 0xAA;
        buf[1] = 0x20;  // bit5=1 表示扩展帧
        buf[2] = frame.dlc;
        buf[3] = 0x00; buf[4] = 0x00;
        buf[5] = (frame.canId >> 24) & 0xFF;
        buf[6] = (frame.canId >> 16) & 0xFF;
        buf[7] = (frame.canId >> 8)  & 0xFF;
        buf[8] = frame.canId & 0xFF;
        std::memcpy(&buf[9], frame.data, 8);
        // 注意: 17字节 = 1+1+1+2+4+8 = 17, 但没有尾字节位置
        // 实际格式: AA Type DLC 00 00 ID3 ID2 ID1 ID0 D0..D7 7A = 18 bytes
        uint8_t fullBuf[18];
        std::memcpy(fullBuf, buf, 9);
        std::memcpy(&fullBuf[9], frame.data, 8);
        fullBuf[17] = 0x7A;
        return serial_->write((const char*)fullBuf, 18) == 18;
    } else {
        // 标准帧: AA 00 DLC 00 00 IDH IDL D0..D7 7A = 17 bytes
        uint8_t buf[17] = {0xAA, 0, 0, frame.dlc, 0, 0, (uint8_t)(frame.canId >> 8), (uint8_t)frame.canId};
        std::memcpy(&buf[8], frame.data, 8); buf[16] = 0x7A;
        return serial_->write((const char*)buf, 17) == 17;
    }
}
void SerialCanAdapter::processIncoming() {
    if (!isOpen() || !serial_->waitForReadyRead(5)) return;
    rxBuffer_.append(serial_->readAll());
    while (rxBuffer_.size() >= 17) {
        int idx = rxBuffer_.indexOf((char)0xAA);
        if (idx < 0) { rxBuffer_.clear(); break; }
        if (idx > 0) rxBuffer_.remove(0, idx);
        if (rxBuffer_.size() < 17) break;

        uint8_t typeByte = (uint8_t)rxBuffer_[1];
        bool isExt = (typeByte & 0x20) != 0;

        if (isExt) {
            // 扩展帧: 18 bytes
            if (rxBuffer_.size() < 18) break;
            if ((uint8_t)rxBuffer_[17] != 0x7A) { rxBuffer_.remove(0, 1); continue; }
            CanFrame f;
            f.dlc = (uint8_t)rxBuffer_[2];
            f.isExtended = true;
            f.canId = ((uint32_t)(uint8_t)rxBuffer_[5] << 24) |
                      ((uint32_t)(uint8_t)rxBuffer_[6] << 16) |
                      ((uint32_t)(uint8_t)rxBuffer_[7] << 8)  |
                      (uint32_t)(uint8_t)rxBuffer_[8];
            std::memcpy(f.data, rxBuffer_.constData() + 9, 8);
            { QMutexLocker lk(&mutex_); f.sequence = ++sequence_; frameQueue_.push_back(f); }
            frameCond_.wakeAll(); rxBuffer_.remove(0, 18);
        } else {
            // 标准帧: 17 bytes
            if ((uint8_t)rxBuffer_[16] != 0x7A) { rxBuffer_.remove(0, 1); continue; }
            CanFrame f; f.dlc = rxBuffer_[3]; f.canId = ((uint8_t)rxBuffer_[6] << 8) | (uint8_t)rxBuffer_[7];
            f.isExtended = false;
            std::memcpy(f.data, rxBuffer_.constData() + 8, 8);
            { QMutexLocker lk(&mutex_); f.sequence = ++sequence_; frameQueue_.push_back(f); }
            frameCond_.wakeAll(); rxBuffer_.remove(0, 17);
        }
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

