#pragma once
#include "models/common_types.h"
#include <QObject>
#include <QVector>
#include <QByteArray>

class QSerialPort;
class QTcpSocket;
class QTimer;

namespace dac {

class GripperWorker : public QObject
{
    Q_OBJECT
public:
    explicit GripperWorker(QObject *parent = nullptr);
    ~GripperWorker();

public slots:
    void configure(dac::ArmSide side, const dac::GripperEndpointConfig &cfg);
    void connectDevice();
    void disconnectDevice();
    void setTarget(double normalized);
    void refreshState();

signals:
    void stateChanged(const dac::GripperState &state);
    void connectionChanged(dac::ArmSide side, bool connected, const QString &message);
    void logMessage(dac::LogLevel level, const QString &message);

private:
    bool openCurrentTransport();
    bool openTcp();
    bool openRtu();
    void closeCurrentTransport();
    bool initializeDevice();
    bool initTcpDevice();
    bool initRtuDevice();
    bool writeTcpPosition(double normalized);
    bool writeRtuPosition(double normalized);
    bool readTcpState();
    bool readRtuState();
    QByteArray transactTcp(quint8 func, const QByteArray &payload, int timeoutMs);
    QByteArray transactRtu(const QByteArray &frame, int timeoutMs);
    bool writeMultipleRegistersTcp(quint16 startReg, const QVector<quint16> &values);
    QVector<quint16> readHoldingRegistersTcp(quint16 startReg, quint16 count);
    bool writeSerialRegisters(quint16 startReg, const QVector<quint16> &values);
    QVector<quint16> readSerialRegisters(quint16 startReg, int registerCount);
    QByteArray buildSerialWriteFrame(quint16 startReg, const QVector<quint16> &values) const;
    QByteArray buildSerialReadFrame(quint16 startReg, quint8 byteCount) const;
    bool parseSerialReply(const QByteArray &reply, quint8 expectedCmd, QByteArray *payloadOut) const;
    void updateConnectionState(bool connected, const QString &message);
    void emitState();

    static quint8 checksum(const QByteArray &data);
    static QVector<quint16> interpolateTcpPose(double normalized);
    static QVector<quint16> interpolateSerialPose(double normalized);
    static double normalizedFromTcpAngles(const QVector<int> &angles);
    static double normalizedFromSerialAngles(const QVector<int> &angles);
    static double clamp01(double value);

    dac::ArmSide side_ = dac::ArmSide::Left;
    dac::GripperEndpointConfig cfg_;
    dac::GripperState state_;
    QSerialPort *serialPort_ = nullptr;
    QTcpSocket  *tcpSocket_ = nullptr;
    QTimer      *pollTimer_ = nullptr;
    quint16 transactionId_ = 1;
};

} // namespace dac
