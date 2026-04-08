/**
 * CanBusWorker - CAN 总线工作线程
 *
 * 职责：
 * - 独占一条物理 USB-CAN 总线
 * - 在专属 QThread 中运行
 * - 管理请求队列（带优先级）
 * - 串行处理请求→响应
 * - 管理轮询任务
 */
#pragma once

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QTimer>
#include <QQueue>
#include <QMap>
#include <atomic>
#include "../models/common_types.h"
#include "frame_io.h"

namespace dar {

class CanBusWorker : public QObject
{
    Q_OBJECT

public:
    explicit CanBusWorker(const BusConfig &config, QObject *parent = nullptr);
    ~CanBusWorker();

    const BusConfig &config() const { return config_; }
    bool isConnected() const { return connected_; }

public slots:
    /**
     * @brief 打开总线连接（在 worker 线程中调用）
     */
    void connectBus();

    /**
     * @brief 关闭总线连接
     */
    void disconnectBus();

    /**
     * @brief 提交通信请求
     * 请求会按优先级排队，在 worker 线程中串行执行
     */
    void submitRequest(const dar::CommRequest &request);

    /**
     * @brief 注册轮询任务
     */
    void registerPollTask(const dar::PollTask &task);

    /**
     * @brief 注销轮询任务
     */
    void unregisterPollTask(const QString &taskId);

    /**
     * @brief 启动调度循环（在 worker 线程中调用）
     */
    void startProcessing();

    /**
     * @brief 停止调度循环
     */
    void stopProcessing();

signals:
    void connected();
    void disconnected();
    void requestCompleted(const dar::CommResult &result);
    void frameReceived(const dar::CanFrame &frame);
    void logMessage(const dar::LogEntry &entry);
    void busError(const QString &errorMessage);

private:
    void processNextRequest();
    void processPollTasks();
    CommResult executeRequest(const CommRequest &req);
    void log(LogEntry::Level level, const QString &msg);

    BusConfig config_;
    FrameIO *frameIO_ = nullptr;
    QThread *receiveThread_ = nullptr;

    QMutex requestMutex_;
    // 3 优先级队列：Critical, Control, Monitoring
    QQueue<CommRequest> requestQueues_[3];
    QMap<QString, PollTask> pollTasks_;

    QTimer *pollTimer_ = nullptr;
    QTimer *processTimer_ = nullptr;

    std::atomic<bool> connected_{false};
    std::atomic<bool> processing_{false};
    uint64_t nextRequestId_ = 1;
};

}  // namespace dar
