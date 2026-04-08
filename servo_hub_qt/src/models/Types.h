#pragma once

#include <QByteArray>
#include <QDateTime>
#include <QMetaType>
#include <QString>
#include <QVariantMap>

enum class BusType {
    CanUsbSerial
};

enum class RequestPriority {
    Monitoring = 0,
    Control = 1,
    Critical = 2
};

enum class CommandType {
    Unknown,
    ConnectBus,
    DisconnectBus,
    ReadSdo,
    WriteSdo,
    EnableOperation,
    DisableOperation,
    QuickStop,
    FaultReset,
    SetVelocity,
    SetPosition,
    StartNmt,
    StopNmt,
    ScanNode
};

struct CanFrame {
    quint32 canId = 0;
    QByteArray data;
    quint64 sequence = 0;
};

struct ResponseMatcher {
    bool enabled = false;
    int expectedCanId = -1;
    int expectedIndex = -1;
    int expectedSubIndex = -1;
    QByteArray acceptedSpecifiers;
    int minimumPayloadLength = 0;
    bool acceptAbortFrame = true;
};

struct BusConfig {
    QString busId;
    BusType busType = BusType::CanUsbSerial;
    QString devicePath;
    int baudRate = 9600;
    int timeoutMs = 150;
    bool enabled = true;
};

struct BusRequest {
    QString requestId;
    QString busId;
    quint32 deviceId = 0;
    CommandType commandType = CommandType::Unknown;
    RequestPriority priority = RequestPriority::Monitoring;
    QByteArray payload;
    int timeoutMs = 200;
    int maxRetries = 0;
    ResponseMatcher matcher;
    QVariantMap context;
    QString dedupKey;
    qint64 createdAtMs = 0;
};

struct BusResult {
    QString requestId;
    QString busId;
    quint32 deviceId = 0;
    CommandType commandType = CommandType::Unknown;
    bool success = false;
    int errorCode = 0;
    QString errorMessage;
    QByteArray rawRequest;
    QByteArray rawResponse;
    qint64 latencyMs = 0;
    QVariantMap context;
};

struct MotorConfig {
    quint32 nodeId = 0;
    QString name;
    bool directionInverted = false;
    qint16 maxTorquePermille = 1200;
    qint16 collisionThresholdPermille = 500;
};

struct MotorState {
    quint32 nodeId = 0;
    QString name;
    bool online = false;
    bool enabled = false;
    QString modeText = "未知";
    qint32 actualVelocityRpm = 0;
    double actualPositionDeg = 0.0;
    qint16 torquePermille = 0;
    qint16 maxTorquePermille = 1200;
    qint16 collisionThresholdPermille = 500;
    double currentAmp = 0.0;
    quint16 statusWord = 0;
    quint16 faultCode = 0;
    QString faultText;
    bool collisionTriggered = false;
    QDateTime lastUpdate;
};

QString formatCanId(quint32 canId);
QString commandTypeToString(CommandType commandType);
QString priorityToString(RequestPriority priority);
void registerCommonMetaTypes();

Q_DECLARE_METATYPE(CanFrame)
Q_DECLARE_METATYPE(ResponseMatcher)
Q_DECLARE_METATYPE(BusConfig)
Q_DECLARE_METATYPE(BusRequest)
Q_DECLARE_METATYPE(BusResult)
Q_DECLARE_METATYPE(MotorConfig)
Q_DECLARE_METATYPE(MotorState)
