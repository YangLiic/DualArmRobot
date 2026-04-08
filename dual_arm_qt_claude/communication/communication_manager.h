/**
 * CommunicationManager - 通信管理器
 *
 * 职责：
 * - 管理所有 BusWorker 的生命周期
 * - 将请求分发到正确的总线
 * - 提供统一的通信入口
 * - 管理 QThread 与 BusWorker 的绑定
 */
#pragma once

#include <QObject>
#include <QMap>
#include <QThread>
#include "../models/common_types.h"
#include "can_bus_worker.h"

namespace dar {

class CommunicationManager : public QObject
{
    Q_OBJECT

public:
    explicit CommunicationManager(QObject *parent = nullptr);
    ~CommunicationManager();

    /**
     * @brief 添加总线配置并创建 worker
     */
    bool addBus(const BusConfig &config);

    /**
     * @brief 移除总线
     */
    void removeBus(const QString &busId);

    /**
     * @brief 连接指定总线
     */
    void connectBus(const QString &busId);

    /**
     * @brief 断开指定总线
     */
    void disconnectBus(const QString &busId);

    /**
     * @brief 连接所有总线
     */
    void connectAll();

    /**
     * @brief 断开所有总线
     */
    void disconnectAll();

    /**
     * @brief 提交通信请求到指定总线
     */
    void submitRequest(const CommRequest &request);

    /**
     * @brief 注册轮询任务
     */
    void registerPollTask(const QString &busId, const PollTask &task);

    /**
     * @brief 注销轮询任务
     */
    void unregisterPollTask(const QString &busId, const QString &taskId);

    /**
     * @brief 获取总线是否已连接
     */
    bool isBusConnected(const QString &busId) const;

    /**
     * @brief 获取所有总线 ID
     */
    QStringList busIds() const;

    /**
     * @brief 直接获取 FrameIO （供 protocol 层使用）
     */
    CanBusWorker *worker(const QString &busId) const;

signals:
    void busConnected(const QString &busId);
    void busDisconnected(const QString &busId);
    void requestCompleted(const dar::CommResult &result);
    void frameReceived(const QString &busId, const dar::CanFrame &frame);
    void logMessage(const dar::LogEntry &entry);
    void busError(const QString &busId, const QString &errorMessage);

private:
    struct BusEntry {
        BusConfig config;
        CanBusWorker *worker = nullptr;
        QThread *thread = nullptr;
    };

    QMap<QString, BusEntry> buses_;
    void log(LogEntry::Level level, const QString &msg);
};

}  // namespace dar
