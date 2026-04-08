#include "models/Types.h"

QString formatCanId(quint32 canId)
{
    return QStringLiteral("0x%1").arg(canId, 3, 16, QLatin1Char('0')).toUpper();
}

QString commandTypeToString(CommandType commandType)
{
    switch (commandType) {
    case CommandType::ConnectBus:
        return QStringLiteral("ConnectBus");
    case CommandType::DisconnectBus:
        return QStringLiteral("DisconnectBus");
    case CommandType::ReadSdo:
        return QStringLiteral("ReadSdo");
    case CommandType::WriteSdo:
        return QStringLiteral("WriteSdo");
    case CommandType::EnableOperation:
        return QStringLiteral("EnableOperation");
    case CommandType::DisableOperation:
        return QStringLiteral("DisableOperation");
    case CommandType::QuickStop:
        return QStringLiteral("QuickStop");
    case CommandType::FaultReset:
        return QStringLiteral("FaultReset");
    case CommandType::SetVelocity:
        return QStringLiteral("SetVelocity");
    case CommandType::SetPosition:
        return QStringLiteral("SetPosition");
    case CommandType::StartNmt:
        return QStringLiteral("StartNmt");
    case CommandType::StopNmt:
        return QStringLiteral("StopNmt");
    case CommandType::ScanNode:
        return QStringLiteral("ScanNode");
    case CommandType::Unknown:
    default:
        return QStringLiteral("Unknown");
    }
}

QString priorityToString(RequestPriority priority)
{
    switch (priority) {
    case RequestPriority::Critical:
        return QStringLiteral("Critical");
    case RequestPriority::Control:
        return QStringLiteral("Control");
    case RequestPriority::Monitoring:
    default:
        return QStringLiteral("Monitoring");
    }
}

void registerCommonMetaTypes()
{
    qRegisterMetaType<CanFrame>("CanFrame");
    qRegisterMetaType<ResponseMatcher>("ResponseMatcher");
    qRegisterMetaType<BusConfig>("BusConfig");
    qRegisterMetaType<BusRequest>("BusRequest");
    qRegisterMetaType<BusResult>("BusResult");
    qRegisterMetaType<MotorConfig>("MotorConfig");
    qRegisterMetaType<MotorState>("MotorState");
}

