#pragma once

#include "models/Types.h"

#include <QObject>

class AbstractBusWorker : public QObject
{
    Q_OBJECT

public:
    explicit AbstractBusWorker(QObject *parent = nullptr)
        : QObject(parent)
    {
    }

    ~AbstractBusWorker() override = default;

signals:
    void requestCompleted(const BusResult &result);
    void busConnectionChanged(const QString &busId, bool connected, const QString &message);
    void logMessage(const QString &message);

public slots:
    virtual void connectBus() = 0;
    virtual void disconnectBus() = 0;
    virtual void submitRequest(const BusRequest &request) = 0;
};

