#pragma once
#include "models/common_types.h"
#include "communication/ican_adapter.h"
#include <QObject>
#include <QMutex>
#include <QQueue>
#include <QTimer>
#include <memory>

namespace dac {

/*
 * CanBusWorker: 单条 CAN 总线的独占管理者。
 * 运行在专属 QThread 中，用 QTimer 驱动事件循环而非阻塞死循环，
 * 以便 QThread 的事件循环能正常处理 signal/slot 和 invokeMethod。
 * 按 Priority 插入排队（Critical 插队到最前）。
 *
 * 通过 ICanAdapter 接口适配不同硬件后端（串口 CH340 / VCI CANalyst-II）。
 */
class CanBusWorker : public QObject
{
    Q_OBJECT
public:
    explicit CanBusWorker(QObject *parent = nullptr);
    ~CanBusWorker();

    void configure(const BusConfig &cfg);

public slots:
    void start();
    void stop();
    void submitRequest(const dac::CommRequest &req);

signals:
    void busOpened(bool ok, const QString &msg);
    void busClosed();
    void requestCompleted(const dac::CommResult &result);
    void frameReceived(const dac::CanFrame &frame);
    void logMessage(dac::LogLevel level, const QString &msg);

private slots:
    void onTick();

private:
    void processNextRequest();
    CommResult executeRequest(const CommRequest &req);
    bool matchResponse(const CanFrame &frame, const CommRequest &req) const;
    std::unique_ptr<ICanAdapter> createAdapter(AdapterType type);

    BusConfig config_;
    std::unique_ptr<ICanAdapter> adapter_;
    QTimer   *tickTimer_ = nullptr;
    bool      running_ = false;

    QMutex    queueMutex_;
    QQueue<CommRequest> requestQueue_;
    uint64_t nextReqId_ = 1;
};

} // namespace dac
