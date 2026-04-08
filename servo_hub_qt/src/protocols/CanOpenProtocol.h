#pragma once

#include "models/Types.h"

#include <QByteArray>

class CanOpenProtocol
{
public:
    static constexpr qint32 EncoderResolution = 8388608;
    static constexpr double PulsesPerDegree = static_cast<double>(EncoderResolution) / 360.0;

    static constexpr quint16 OdControlWord = 0x6040;
    static constexpr quint16 OdStatusWord = 0x6041;
    static constexpr quint16 OdErrorCode = 0x603F;
    static constexpr quint16 OdOperationMode = 0x6060;
    static constexpr quint16 OdModeDisplay = 0x6061;
    static constexpr quint16 OdPositionActual = 0x6064;
    static constexpr quint16 OdVelocityActual = 0x606C;
    static constexpr quint16 OdTargetPosition = 0x607A;
    static constexpr quint16 OdPositionDeviation = 0x60F4;
    static constexpr quint16 OdActualTorque = 0x6077;
    static constexpr quint16 OdTargetVelocity = 0x60FF;
    static constexpr quint16 OdProfileVelocity = 0x6081;
    static constexpr quint16 OdProfileAcceleration = 0x6083;
    static constexpr quint16 OdProfileDeceleration = 0x6084;
    static constexpr quint16 OdPhaseCurrent = 0x200B;
    static constexpr quint8 OdPhaseCurrentSubIndex = 0x19;

    static quint32 responseCanId(quint32 requestCanId);
    static qint32 rpmToEncoderUnits(qint32 rpm);
    static qint32 encoderUnitsToRpm(qint32 value);
    static qint32 degreesToPulses(double degrees);
    static double pulsesToDegrees(qint32 pulses);

    static BusRequest makeReadSdoRequest(
        const QString &busId,
        quint32 nodeId,
        quint16 index,
        quint8 subIndex,
        RequestPriority priority,
        const QVariantMap &context = {},
        int timeoutMs = 160,
        const QString &dedupKey = {});

    static BusRequest makeWriteSdoRequest(
        const QString &busId,
        quint32 nodeId,
        quint16 index,
        quint8 subIndex,
        const QByteArray &littleEndianValue,
        RequestPriority priority,
        CommandType commandType,
        const QVariantMap &context = {},
        int timeoutMs = 220,
        bool waitForResponse = true,
        int postDelayMs = 0);

    static BusRequest makeOperationModeRequest(
        const QString &busId,
        quint32 nodeId,
        qint8 mode,
        RequestPriority priority,
        const QVariantMap &context = {},
        bool waitForResponse = false,
        int postDelayMs = 50);

    static BusRequest makeControlWordRequest(
        const QString &busId,
        quint32 nodeId,
        quint16 controlWord,
        RequestPriority priority,
        CommandType commandType,
        const QVariantMap &context = {},
        bool waitForResponse = false,
        int postDelayMs = 100);

    static BusRequest makeVelocityRequest(
        const QString &busId,
        quint32 nodeId,
        qint32 rpm,
        RequestPriority priority,
        const QVariantMap &context = {},
        bool waitForResponse = false,
        int postDelayMs = 10);

    static BusRequest makeProfileVelocityRequest(
        const QString &busId,
        quint32 nodeId,
        quint32 rpm,
        RequestPriority priority,
        const QVariantMap &context = {},
        bool waitForResponse = true,
        int postDelayMs = 0);

    static BusRequest makeProfileAccelerationRequest(
        const QString &busId,
        quint32 nodeId,
        quint32 rpmPerSecond,
        RequestPriority priority,
        const QVariantMap &context = {});

    static BusRequest makeProfileDecelerationRequest(
        const QString &busId,
        quint32 nodeId,
        quint32 rpmPerSecond,
        RequestPriority priority,
        const QVariantMap &context = {});

    static BusRequest makePositionRequest(
        const QString &busId,
        quint32 nodeId,
        double degrees,
        RequestPriority priority,
        const QVariantMap &context = {},
        bool waitForResponse = true,
        int postDelayMs = 0);

    static BusRequest makeStartPositionRequest(
        const QString &busId,
        quint32 nodeId,
        bool relative,
        RequestPriority priority,
        const QVariantMap &context = {},
        bool waitForResponse = false,
        int postDelayMs = 50);

    static BusRequest makeNmtRequest(
        const QString &busId,
        quint32 nodeId,
        quint8 commandSpecifier,
        CommandType commandType,
        RequestPriority priority,
        const QVariantMap &context = {});

    static bool decodeInt16(const QByteArray &sdoResponse, qint16 &value);
    static bool decodeInt8(const QByteArray &sdoResponse, qint8 &value);
    static bool decodeUInt16(const QByteArray &sdoResponse, quint16 &value);
    static bool decodeInt32(const QByteArray &sdoResponse, qint32 &value);
    static bool decodeUInt32(const QByteArray &sdoResponse, quint32 &value);

    static QString operationModeToText(qint8 mode);
    static bool isOperationEnabled(quint16 statusWord);
    static bool isFault(quint16 statusWord);
    static QString faultCodeToText(quint16 faultCode);
};
