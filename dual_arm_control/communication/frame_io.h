#pragma once
#include "models/common_types.h"
#include <QObject>
#include <QMutex>
#include <QWaitCondition>
#include <QSerialPort>
#include <deque>

namespace dac {

/*
 * FrameIO: 最底层串口帧收发。
 * 负责 USB-CAN 适配器 17 字节帧协议的编解码。
 * 帧格式: 0xAA [res][res][dlc][00][00][id_hi][id_lo][8 data bytes] 0x7A
 *
 * 此类只在 CanBusWorker 线程内使用，不跨线程。
 */
class FrameIO : public QObject
{
    Q_OBJECT
public:
    explicit FrameIO(QObject *parent = nullptr);
    ~FrameIO();

    bool openPort(const QString &portName, int baudRate);
    void closePort();
    bool isOpen() const;

    bool sendFrame(const CanFrame &frame);

    // 从串口读数据，解析出帧后发信号
    void processIncoming();

    // 阻塞等待一帧（供同步调用场景）
    bool waitForFrame(CanFrame &out, int timeoutMs);

    // 获取当前帧序号
    uint64_t currentSequence() const;

signals:
    void frameReceived(const dac::CanFrame &frame);
    void errorOccurred(const QString &msg);

private:
    bool parseBuffer();

    QSerialPort *serial_ = nullptr;
    QByteArray   rxBuffer_;
    mutable QMutex  mutex_;
    QWaitCondition   frameCond_;
    std::deque<CanFrame> frameQueue_;
    uint64_t  sequence_ = 0;

    static constexpr uint8_t FRAME_HEAD = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0x7A;
    static constexpr int     FRAME_LEN  = 17;
};

} // namespace dac
