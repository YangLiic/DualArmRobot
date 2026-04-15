#pragma once
#include "models/common_types.h"
#include <QObject>

class QThread;
class QTimer;

namespace dac {

class HumanoidArmsSdkAdapter;

class ArmService : public QObject
{
    Q_OBJECT
public:
    explicit ArmService(QObject *parent = nullptr);
    ~ArmService();

    bool isInitialized() const;
    ArmState armState(ArmSide side) const;

    void initializeSdk(int deviceIndex, int canIndex);
    void shutdownSdk();
    void refreshAllStates();
    void refreshRealtimeStates();
    void moveJointTargets(ArmSide side, const QVector<double> &jointTargetsDeg, double velocity);
    void brakeArm(ArmSide side);
    void clearErrors(ArmSide side);

signals:
    void initializationChanged(bool initialized, const QString &message);
    void armStateChanged(const dac::ArmState &state);
    void logMessage(dac::LogLevel level, const QString &message);

private:
    void storeState(const dac::ArmState &state);

    QThread *workerThread_ = nullptr;
    HumanoidArmsSdkAdapter *adapter_ = nullptr;
    QTimer *pollTimer_ = nullptr;

    bool initialized_ = false;
    ArmState leftArmState_;
    ArmState rightArmState_;
};

} // namespace dac
