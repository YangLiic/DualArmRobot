/**
 * FrameIO - USB-CAN 适配器底层帧收发
 *
 * 职责：
 * - 打开/关闭串口
 * - 发送 CAN 帧（AA...7A 格式）
 * - 接收并解析 CAN 帧
 * - 线程安全的帧队列
 *
 * 本类只负责物理链路的 IO，不参与任何协议语义。
 */
#pragma once

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QQueue>
#include <QString>
#include <atomic>
#include "../models/common_types.h"

namespace dar {

class FrameIO : public QObject
{
    Q_OBJECT

public:
    explicit FrameIO(QObject *parent = nullptr);
    ~FrameIO();

    /**
     * @brief 打开串口
     * @param portName 串口路径，如 "/dev/ttyUSB0"
     * @param baudRate 波特率
     * @return true 成功
     */
    bool open(const QString &portName, int baudRate = 9600);

    /**
     * @brief 关闭串口
     */
    void close();

    /**
     * @brief 是否已打开
     */
    bool isOpen() const;

    /**
     * @brief 发送 CAN 帧
     * @return true 成功
     */
    bool sendFrame(uint32_t canId, const uint8_t *data, uint8_t dlc);

    /**
     * @brief 启动接收循环（在 worker 线程调用）
     * 此函数会阻塞，直到 stopReceive() 被调用
     */
    void startReceiveLoop();

    /**
     * @brief 停止接收循环
     */
    void stopReceive();

    /**
     * @brief 等待下一帧
     * @param lastSeq 上一次处理的帧序号
     * @param frame 输出参数
     * @param timeoutMs 超时毫秒
     * @return true 收到
     */
    bool waitForFrame(uint64_t lastSeq, CanFrame &frame, int timeoutMs);

    /**
     * @brief 获取当前帧序号
     */
    uint64_t currentSequence() const;

signals:
    void frameReceived(const dar::CanFrame &frame);
    void errorOccurred(const QString &message);
    void portOpened();
    void portClosed();

private:
    int openSerial(const QString &portName, int baudRate);
    int parseFrame(const uint8_t *buffer, size_t len);

    int serialFd_ = -1;
    std::atomic<bool> receiving_{false};
    QString portName_;
    int baudRate_ = 9600;

    mutable QMutex frameMutex_;
    QWaitCondition frameCv_;
    QQueue<CanFrame> frameQueue_;
    uint64_t frameSequence_ = 0;
};

}  // namespace dar
