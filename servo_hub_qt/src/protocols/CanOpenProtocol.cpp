#include "protocols/CanOpenProtocol.h"

#include <QDateTime>
#include <QUuid>

namespace {

QString createRequestId()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces);
}

QByteArray littleEndianInt16(qint16 value)
{
    QByteArray bytes(2, Qt::Uninitialized);
    bytes[0] = static_cast<char>(value & 0xFF);
    bytes[1] = static_cast<char>((value >> 8) & 0xFF);
    return bytes;
}

QByteArray littleEndianUInt16(quint16 value)
{
    QByteArray bytes(2, Qt::Uninitialized);
    bytes[0] = static_cast<char>(value & 0xFF);
    bytes[1] = static_cast<char>((value >> 8) & 0xFF);
    return bytes;
}

QByteArray littleEndianInt32(qint32 value)
{
    QByteArray bytes(4, Qt::Uninitialized);
    bytes[0] = static_cast<char>(value & 0xFF);
    bytes[1] = static_cast<char>((value >> 8) & 0xFF);
    bytes[2] = static_cast<char>((value >> 16) & 0xFF);
    bytes[3] = static_cast<char>((value >> 24) & 0xFF);
    return bytes;
}

BusRequest baseRequest(
    const QString &busId,
    quint32 nodeId,
    CommandType commandType,
    RequestPriority priority,
    const QVariantMap &context)
{
    BusRequest request;
    request.requestId = createRequestId();
    request.busId = busId;
    request.deviceId = nodeId;
    request.commandType = commandType;
    request.priority = priority;
    request.context = context;
    request.createdAtMs = QDateTime::currentMSecsSinceEpoch();
    return request;
}

} // namespace

quint32 CanOpenProtocol::responseCanId(quint32 requestCanId)
{
    return requestCanId >= 0x80 ? requestCanId - 0x80 : requestCanId;
}

qint32 CanOpenProtocol::rpmToEncoderUnits(qint32 rpm)
{
    return static_cast<qint32>((static_cast<qint64>(rpm) * EncoderResolution) / 60);
}

qint32 CanOpenProtocol::encoderUnitsToRpm(qint32 value)
{
    return static_cast<qint32>((static_cast<qint64>(value) * 60) / EncoderResolution);
}

qint32 CanOpenProtocol::degreesToPulses(double degrees)
{
    return static_cast<qint32>(degrees * PulsesPerDegree);
}

double CanOpenProtocol::pulsesToDegrees(qint32 pulses)
{
    return static_cast<double>(pulses) / PulsesPerDegree;
}

BusRequest CanOpenProtocol::makeReadSdoRequest(
    const QString &busId,
    quint32 nodeId,
    quint16 index,
    quint8 subIndex,
    RequestPriority priority,
    const QVariantMap &context,
    int timeoutMs,
    const QString &dedupKey)
{
    BusRequest request = baseRequest(busId, nodeId, CommandType::ReadSdo, priority, context);
    request.timeoutMs = timeoutMs;
    request.dedupKey = dedupKey;

    request.payload = QByteArray(8, '\0');
    request.payload[0] = 0x40;
    request.payload[1] = static_cast<char>(index & 0xFF);
    request.payload[2] = static_cast<char>((index >> 8) & 0xFF);
    request.payload[3] = static_cast<char>(subIndex);

    request.matcher.enabled = true;
    request.matcher.expectedCanId = static_cast<int>(responseCanId(nodeId));
    request.matcher.expectedIndex = index;
    request.matcher.expectedSubIndex = subIndex;
    request.matcher.acceptedSpecifiers = QByteArray::fromRawData("\x4F\x4B\x43", 3);
    request.matcher.minimumPayloadLength = 4;
    return request;
}

BusRequest CanOpenProtocol::makeWriteSdoRequest(
    const QString &busId,
    quint32 nodeId,
    quint16 index,
    quint8 subIndex,
    const QByteArray &littleEndianValue,
    RequestPriority priority,
    CommandType commandType,
    const QVariantMap &context,
    int timeoutMs,
    bool waitForResponse,
    int postDelayMs)
{
    BusRequest request = baseRequest(busId, nodeId, commandType, priority, context);
    request.timeoutMs = timeoutMs;
    if (postDelayMs > 0) {
        request.context.insert(QStringLiteral("postDelayMs"), postDelayMs);
    }

    request.payload = QByteArray(8, '\0');
    switch (littleEndianValue.size()) {
    case 1:
        request.payload[0] = 0x2F;
        break;
    case 2:
        request.payload[0] = 0x2B;
        break;
    case 4:
        request.payload[0] = 0x23;
        break;
    default:
        request.payload[0] = 0x23;
        break;
    }

    request.payload[1] = static_cast<char>(index & 0xFF);
    request.payload[2] = static_cast<char>((index >> 8) & 0xFF);
    request.payload[3] = static_cast<char>(subIndex);
    for (int i = 0; i < littleEndianValue.size() && i < 4; ++i) {
        request.payload[4 + i] = littleEndianValue.at(i);
    }

    request.matcher.enabled = waitForResponse;
    if (waitForResponse) {
        request.matcher.expectedCanId = static_cast<int>(responseCanId(nodeId));
        request.matcher.expectedIndex = index;
        request.matcher.expectedSubIndex = subIndex;
        request.matcher.acceptedSpecifiers = QByteArray(1, static_cast<char>(0x60));
        request.matcher.minimumPayloadLength = 4;
    }
    return request;
}

BusRequest CanOpenProtocol::makeOperationModeRequest(
    const QString &busId,
    quint32 nodeId,
    qint8 mode,
    RequestPriority priority,
    const QVariantMap &context,
    bool waitForResponse,
    int postDelayMs)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdOperationMode,
        0x00,
        QByteArray(1, static_cast<char>(mode)),
        priority,
        CommandType::WriteSdo,
        context,
        220,
        waitForResponse,
        postDelayMs);
}

BusRequest CanOpenProtocol::makeControlWordRequest(
    const QString &busId,
    quint32 nodeId,
    quint16 controlWord,
    RequestPriority priority,
    CommandType commandType,
    const QVariantMap &context,
    bool waitForResponse,
    int postDelayMs)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdControlWord,
        0x00,
        littleEndianUInt16(controlWord),
        priority,
        commandType,
        context,
        220,
        waitForResponse,
        postDelayMs);
}

BusRequest CanOpenProtocol::makeVelocityRequest(
    const QString &busId,
    quint32 nodeId,
    qint32 rpm,
    RequestPriority priority,
    const QVariantMap &context,
    bool waitForResponse,
    int postDelayMs)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdTargetVelocity,
        0x00,
        littleEndianInt32(rpmToEncoderUnits(rpm)),
        priority,
        CommandType::SetVelocity,
        context,
        220,
        waitForResponse,
        postDelayMs);
}

BusRequest CanOpenProtocol::makeProfileVelocityRequest(
    const QString &busId,
    quint32 nodeId,
    quint32 rpm,
    RequestPriority priority,
    const QVariantMap &context,
    bool waitForResponse,
    int postDelayMs)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdProfileVelocity,
        0x00,
        littleEndianInt32(rpmToEncoderUnits(static_cast<qint32>(rpm))),
        priority,
        CommandType::WriteSdo,
        context);
}

BusRequest CanOpenProtocol::makeProfileAccelerationRequest(
    const QString &busId,
    quint32 nodeId,
    quint32 rpmPerSecond,
    RequestPriority priority,
    const QVariantMap &context)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdProfileAcceleration,
        0x00,
        littleEndianInt32(rpmToEncoderUnits(static_cast<qint32>(rpmPerSecond))),
        priority,
        CommandType::WriteSdo,
        context);
}

BusRequest CanOpenProtocol::makeProfileDecelerationRequest(
    const QString &busId,
    quint32 nodeId,
    quint32 rpmPerSecond,
    RequestPriority priority,
    const QVariantMap &context)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdProfileDeceleration,
        0x00,
        littleEndianInt32(rpmToEncoderUnits(static_cast<qint32>(rpmPerSecond))),
        priority,
        CommandType::WriteSdo,
        context);
}

BusRequest CanOpenProtocol::makePositionRequest(
    const QString &busId,
    quint32 nodeId,
    double degrees,
    RequestPriority priority,
    const QVariantMap &context,
    bool waitForResponse,
    int postDelayMs)
{
    return makeWriteSdoRequest(
        busId,
        nodeId,
        OdTargetPosition,
        0x00,
        littleEndianInt32(degreesToPulses(degrees)),
        priority,
        CommandType::SetPosition,
        context,
        220,
        waitForResponse,
        postDelayMs);
}

BusRequest CanOpenProtocol::makeStartPositionRequest(
    const QString &busId,
    quint32 nodeId,
    bool relative,
    RequestPriority priority,
    const QVariantMap &context,
    bool waitForResponse,
    int postDelayMs)
{
    const quint16 controlWord = relative ? 0x007F : 0x003F;
    return makeControlWordRequest(busId, nodeId, controlWord, priority, CommandType::SetPosition, context, waitForResponse, postDelayMs);
}

BusRequest CanOpenProtocol::makeNmtRequest(
    const QString &busId,
    quint32 nodeId,
    quint8 commandSpecifier,
    CommandType commandType,
    RequestPriority priority,
    const QVariantMap &context)
{
    BusRequest request = baseRequest(busId, 0x000, commandType, priority, context);
    request.timeoutMs = 100;
    request.payload = QByteArray(2, '\0');
    request.payload[0] = static_cast<char>(commandSpecifier);
    request.payload[1] = static_cast<char>(nodeId & 0x7F);
    request.matcher.enabled = false;
    return request;
}

bool CanOpenProtocol::decodeInt16(const QByteArray &sdoResponse, qint16 &value)
{
    if (sdoResponse.size() < 6) {
        return false;
    }
    value = static_cast<qint16>(
        static_cast<quint8>(sdoResponse.at(4))
        | (static_cast<quint8>(sdoResponse.at(5)) << 8));
    return true;
}

bool CanOpenProtocol::decodeInt8(const QByteArray &sdoResponse, qint8 &value)
{
    if (sdoResponse.size() < 5) {
        return false;
    }
    value = static_cast<qint8>(sdoResponse.at(4));
    return true;
}

bool CanOpenProtocol::decodeUInt16(const QByteArray &sdoResponse, quint16 &value)
{
    if (sdoResponse.size() < 6) {
        return false;
    }
    value = static_cast<quint16>(
        static_cast<quint8>(sdoResponse.at(4))
        | (static_cast<quint8>(sdoResponse.at(5)) << 8));
    return true;
}

bool CanOpenProtocol::decodeInt32(const QByteArray &sdoResponse, qint32 &value)
{
    if (sdoResponse.size() < 8) {
        return false;
    }
    value = static_cast<qint32>(
        static_cast<quint8>(sdoResponse.at(4))
        | (static_cast<quint8>(sdoResponse.at(5)) << 8)
        | (static_cast<quint8>(sdoResponse.at(6)) << 16)
        | (static_cast<quint8>(sdoResponse.at(7)) << 24));
    return true;
}

bool CanOpenProtocol::decodeUInt32(const QByteArray &sdoResponse, quint32 &value)
{
    if (sdoResponse.size() < 8) {
        return false;
    }
    value = static_cast<quint32>(
        static_cast<quint8>(sdoResponse.at(4))
        | (static_cast<quint8>(sdoResponse.at(5)) << 8)
        | (static_cast<quint8>(sdoResponse.at(6)) << 16)
        | (static_cast<quint8>(sdoResponse.at(7)) << 24));
    return true;
}

QString CanOpenProtocol::operationModeToText(qint8 mode)
{
    switch (mode) {
    case 1:
        return QStringLiteral("位置模式");
    case 3:
        return QStringLiteral("速度模式");
    case 6:
        return QStringLiteral("回零模式");
    case 8:
        return QStringLiteral("循环同步位置");
    case 9:
        return QStringLiteral("循环同步速度");
    default:
        return QStringLiteral("未知");
    }
}

bool CanOpenProtocol::isOperationEnabled(quint16 statusWord)
{
    return (statusWord & 0x006F) == 0x0027;
}

bool CanOpenProtocol::isFault(quint16 statusWord)
{
    return (statusWord & 0x0008) != 0;
}

QString CanOpenProtocol::faultCodeToText(quint16 faultCode)
{
    if (faultCode == 0) {
        return QStringLiteral("无故障");
    }

    switch (faultCode) {
    case 0x2310:
        return QStringLiteral("过流");
    case 0x3210:
        return QStringLiteral("过压");
    case 0x3220:
        return QStringLiteral("欠压");
    case 0x4210:
        return QStringLiteral("过温");
    case 0x8611:
        return QStringLiteral("跟随误差");
    default:
        return QStringLiteral("驱动器故障");
    }
}
