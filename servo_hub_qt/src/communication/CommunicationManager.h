#pragma once

#include "communication/CanBusWorker.h"

#include <QHash>
#include <QThread>
#include <QTimer>
#include <functional>

class CommunicationManager : public QObject
{
    Q_OBJECT

public:
    explicit CommunicationManager(QObject *parent = nullptr);
    ~CommunicationManager() override;

    void configureCanBus(const BusConfig &config);
    void connectBus(const QString &busId);
    void disconnectBus(const QString &busId);
    void submitRequest(const BusRequest &request);

    void registerPollingTask(
        const QString &taskId,
        int intervalMs,
        const std::function<QList<BusRequest>()> &builder);
    void unregisterPollingTask(const QString &taskIdPrefix);

signals:
    void requestCompleted(const BusResult &result);
    void busConnectionChanged(const QString &busId, bool connected, const QString &message);
    void logMessage(const QString &message);

private:
    struct WorkerHandle {
        BusConfig config;
        QThread *thread = nullptr;
        CanBusWorker *worker = nullptr;
    };

    struct PollTask {
        QString taskId;
        QTimer *timer = nullptr;
        std::function<QList<BusRequest>()> builder;
    };

    void destroyBus(const QString &busId);

    QHash<QString, WorkerHandle> m_workers;
    QHash<QString, PollTask> m_pollTasks;
};

