#pragma once
#include "communication/ican_adapter.h"
#include <QSerialPort>
#include <QByteArray>
#include <deque>
#include <QMutex>
#include <QWaitCondition>

namespace dac {

class SerialCanAdapter : public ICanAdapter
{
public:
    SerialCanAdapter();
    ~SerialCanAdapter() override;
    bool open(const BusConfig &cfg) override;
    void close() override;
    bool isOpen() const override;
    bool sendFrame(const CanFrame &frame) override;
    void processIncoming() override;
    bool waitForFrame(CanFrame &out, int timeoutMs) override;
    uint64_t currentSequence() const override;
    QString adapterName() const override { return QStringLiteral("Serial (CH340)"); }
private:
    bool parseBuffer();
    QSerialPort *serial_ = nullptr;
    QByteArray   rxBuffer_;
    mutable QMutex  mutex_;
    QWaitCondition   frameCond_;
    std::deque<CanFrame> frameQueue_;
    uint64_t  sequence_ = 0;
};

} // namespace dac
