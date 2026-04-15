#pragma once
#include "models/common_types.h"
#include "communication/can_bus_worker.h"
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QMap>
#include <functional>

namespace dac {

/*
 * CommunicationManager: 通信调度层顶层管理器。
 *
 * 职责：
 * - 管理所有 BusWorker 生命周期与线程
 * - 统一调度请求分发
 * - 管理周期轮询任务
 * - 信号中转（worker → 业务层）
 */

struct PollTask {
    int      id = 0;
    int      intervalMs = 100;
    bool     active = false;
    CommRequest templateReq;
    std::function<void(const CommResult&)> callback;
};

class CommunicationManager : public QObject
{
    Q_OBJECT
public:
    explicit CommunicationManager(QObject *parent = nullptr);
    ~CommunicationManager();

    // 总线管理
    bool openBus(const BusConfig &cfg);
    void closeBus();
    bool isBusOpen() const;

    // 提交请求
    void submitRequest(const CommRequest &req,
                       std::function<void(const CommResult&)> callback = nullptr);

    // 轮询管理
    int  registerPollTask(const CommRequest &templateReq, int intervalMs,
                          std::function<void(const CommResult&)> callback);
    void unregisterPollTask(int taskId);
    void setAllPollActive(bool active);

signals:
    void busStateChanged(bool open, const QString &msg);
    void rawFrameReceived(const dac::CanFrame &frame);
    void logMessage(dac::LogLevel level, const QString &msg);

private slots:
    void onBusOpened(bool ok, const QString &msg);
    void onRequestCompleted(const dac::CommResult &result);
    void onRawFrame(const dac::CanFrame &frame);
    void onPollTimerTick();

private:
    CanBusWorker *worker_ = nullptr;
    QThread      *workerThread_ = nullptr;
    bool          busOpen_ = false;

    // 请求回调映射
    QMutex callbackMutex_;
    QMap<uint64_t, std::function<void(const CommResult&)>> pendingCallbacks_;
    uint64_t nextReqId_ = 1;

    // 轮询管理
    QTimer *pollTimer_ = nullptr;
    QMap<int, PollTask> pollTasks_;
    QMap<int, qint64>   pollLastFired_;
    int nextPollId_ = 1;
};

} // namespace dac
