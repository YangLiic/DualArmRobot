#pragma once
#include "models/common_types.h"
#include <QObject>

class QThread;

namespace dac {

class GripperWorker;

class GripperService : public QObject
{
    Q_OBJECT
public:
    explicit GripperService(QObject *parent = nullptr);
    ~GripperService();

    void connectEndpoint(ArmSide side, const GripperEndpointConfig &cfg);
    void disconnectEndpoint(ArmSide side);
    void setTargetPosition(ArmSide side, double normalized);

    GripperState gripperState(ArmSide side) const;
    GripperEndpointConfig endpointConfig(ArmSide side) const;

signals:
    void gripperStateChanged(const dac::GripperState &state);
    void connectionChanged(dac::ArmSide side, bool connected, const QString &message);
    void logMessage(dac::LogLevel level, const QString &message);

private:
    struct EndpointInfo {
        ArmSide side = ArmSide::Left;
        GripperEndpointConfig config;
        GripperState state;
        QThread *thread = nullptr;
        GripperWorker *worker = nullptr;
    };

    EndpointInfo &endpoint(ArmSide side);
    const EndpointInfo &endpoint(ArmSide side) const;
    void setupEndpoint(EndpointInfo &info);
    void shutdownEndpoint(EndpointInfo &info);

    EndpointInfo left_;
    EndpointInfo right_;
};

} // namespace dac
