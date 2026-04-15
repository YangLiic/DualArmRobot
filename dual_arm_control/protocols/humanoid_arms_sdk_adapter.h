#pragma once
#include "models/common_types.h"
#include <QObject>

namespace dac {

class HumanoidArmsSdkAdapter : public QObject
{
    Q_OBJECT
public:
    explicit HumanoidArmsSdkAdapter(QObject *parent = nullptr);

public slots:
    void initializeSdk(int deviceIndex, int canIndex);
    void shutdownSdk();
    void refreshState(dac::ArmSide side);
    void refreshAllStates();
    void refreshRealtimeStates();
    void moveToJointPositions(dac::ArmSide side, const QVector<double> &jointPositionsRad, double velocity);
    void brakeArm(dac::ArmSide side);
    void clearArmErrors(dac::ArmSide side);

signals:
    void initializationChanged(bool initialized, const QString &message);
    void armStateUpdated(const dac::ArmState &state);
    void logMessage(dac::LogLevel level, const QString &message);

private:
    dac::ArmState readArmState(dac::ArmSide side, bool includeErrors, bool *ok) const;

    int  deviceIndex_ = 0;
    int  canIndex_ = 0;
    bool initialized_ = false;
};

} // namespace dac
