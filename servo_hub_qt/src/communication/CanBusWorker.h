#pragma once

#include "communication/AbstractBusWorker.h"

#include <QElapsedTimer>
#include <QList>
#include <QSerialPort>
#include <QSet>
#include <QTimer>

class CanBusWorker : public AbstractBusWorker
{
    Q_OBJECT

public:
    explicit CanBusWorker(const BusConfig &config, QObject *parent = nullptr);
    ~CanBusWorker() override;

public slots:
    void connectBus() override;
    void disconnectBus() override;
    void submitRequest(const BusRequest &request) override;

private slots:
    void onReadyRead();
    void onRequestTimeout();
    void processNextRequest();

private:
    bool ensurePort();
    bool frameMatches(const CanFrame &frame, const ResponseMatcher &matcher) const;
    void finishRequest(bool success, const QString &errorMessage, const QByteArray &response = {});
    void failQueuedRequests(const QString &reason);
    void log(const QString &message);
    void insertRequestByPriority(const BusRequest &request);
    int priorityRank(RequestPriority priority) const;

    BusConfig m_config;
    QSerialPort *m_port = nullptr;
    QByteArray m_rxBuffer;
    QList<BusRequest> m_queue;
    QSet<QString> m_pendingDedupKeys;
    QTimer *m_timeoutTimer = nullptr;
    QElapsedTimer m_latencyTimer;
    quint64 m_sequenceCounter = 0;

    bool m_connected = false;
    bool m_busy = false;
    int m_retryCount = 0;
    BusRequest m_activeRequest;
};
