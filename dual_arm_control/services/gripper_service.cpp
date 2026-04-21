#include "services/gripper_service.h"
#include "services/gripper_worker.h"
#include <algorithm>
#include <QMetaObject>
#include <QThread>

namespace dac {

GripperService::GripperService(QObject *parent)
    : QObject(parent)
{
    qRegisterMetaType<dac::ArmSide>("dac::ArmSide");
    qRegisterMetaType<dac::GripperEndpointConfig>("dac::GripperEndpointConfig");
    qRegisterMetaType<dac::GripperState>("dac::GripperState");

    left_.side = ArmSide::Left;
    left_.state.side = ArmSide::Left;
    left_.config.transport = GripperTransport::TCP;
    left_.config.ip = QStringLiteral("192.168.123.211");
    left_.config.tcpPort = 6000;
    left_.config.slaveId = 1;

    right_.side = ArmSide::Right;
    right_.state.side = ArmSide::Right;
    right_.config.transport = GripperTransport::TCP;
    right_.config.ip = QStringLiteral("192.168.123.212");
    right_.config.tcpPort = 6000;
    right_.config.slaveId = 1;

    setupEndpoint(left_);
    setupEndpoint(right_);
}

GripperService::~GripperService()
{
    shutdownEndpoint(left_);
    shutdownEndpoint(right_);
}

void GripperService::connectEndpoint(ArmSide side, const GripperEndpointConfig &cfg)
{
    EndpointInfo &info = endpoint(side);
    info.config = cfg;
    QMetaObject::invokeMethod(info.worker, "configure", Qt::QueuedConnection,
                              Q_ARG(dac::ArmSide, side),
                              Q_ARG(dac::GripperEndpointConfig, cfg));
    QMetaObject::invokeMethod(info.worker, "connectDevice", Qt::QueuedConnection);
}

void GripperService::disconnectEndpoint(ArmSide side)
{
    EndpointInfo &info = endpoint(side);
    QMetaObject::invokeMethod(info.worker, "disconnectDevice", Qt::QueuedConnection);
}

void GripperService::setTargetPosition(ArmSide side, double normalized)
{
    EndpointInfo &info = endpoint(side);
    info.state.targetNormalized = std::clamp(normalized, 0.0, 1.0);
    QMetaObject::invokeMethod(info.worker, "setTarget", Qt::QueuedConnection,
                              Q_ARG(double, normalized));
}

GripperState GripperService::gripperState(ArmSide side) const
{
    return endpoint(side).state;
}

GripperEndpointConfig GripperService::endpointConfig(ArmSide side) const
{
    return endpoint(side).config;
}

GripperService::EndpointInfo &GripperService::endpoint(ArmSide side)
{
    return side == ArmSide::Left ? left_ : right_;
}

const GripperService::EndpointInfo &GripperService::endpoint(ArmSide side) const
{
    return side == ArmSide::Left ? left_ : right_;
}

void GripperService::setupEndpoint(EndpointInfo &info)
{
    EndpointInfo *endpointInfo = &info;
    info.thread = new QThread(this);
    info.worker = new GripperWorker;
    info.worker->moveToThread(info.thread);

    connect(info.thread, &QThread::finished, info.worker, &QObject::deleteLater);
    connect(info.worker, &GripperWorker::stateChanged, this, [this, endpointInfo](const dac::GripperState &state) {
        endpointInfo->state = state;
        emit gripperStateChanged(state);
    });
    connect(info.worker, &GripperWorker::connectionChanged, this, &GripperService::connectionChanged);
    connect(info.worker, &GripperWorker::logMessage, this, &GripperService::logMessage);

    info.thread->start();
    QMetaObject::invokeMethod(info.worker, "configure", Qt::QueuedConnection,
                              Q_ARG(dac::ArmSide, info.side),
                              Q_ARG(dac::GripperEndpointConfig, info.config));
}

void GripperService::shutdownEndpoint(EndpointInfo &info)
{
    if (!info.worker || !info.thread) {
        return;
    }

    if (info.thread->isRunning()) {
        QMetaObject::invokeMethod(info.worker, "disconnectDevice", Qt::BlockingQueuedConnection);
        info.thread->quit();
        info.thread->wait();
    }
    info.thread = nullptr;
    info.worker = nullptr;
}

} // namespace dac
