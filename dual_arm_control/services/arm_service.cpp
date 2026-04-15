#include "services/arm_service.h"
#include "protocols/humanoid_arms_sdk_adapter.h"
#include <QMetaObject>
#include <QThread>
#include <QTimer>
#include <cmath>

namespace {

constexpr double kDegToRad = M_PI / 180.0;

}

namespace dac {

ArmService::ArmService(QObject *parent)
    : QObject(parent)
{
    qRegisterMetaType<dac::ArmSide>("dac::ArmSide");
    qRegisterMetaType<dac::ArmState>("dac::ArmState");
    qRegisterMetaType<QVector<double>>("QVector<double>");

    leftArmState_.side = ArmSide::Left;
    rightArmState_.side = ArmSide::Right;

    workerThread_ = new QThread(this);
    adapter_ = new HumanoidArmsSdkAdapter;
    adapter_->moveToThread(workerThread_);

    connect(workerThread_, &QThread::finished, adapter_, &QObject::deleteLater);
    connect(adapter_, &HumanoidArmsSdkAdapter::armStateUpdated, this, [this](const dac::ArmState &state) {
        storeState(state);
        emit armStateChanged(state);
    });
    connect(adapter_, &HumanoidArmsSdkAdapter::initializationChanged, this, [this](bool initialized, const QString &message) {
        initialized_ = initialized;
        if (initialized_) {
            pollTimer_->start();
            refreshAllStates();
        } else {
            pollTimer_->stop();
        }
        emit initializationChanged(initialized, message);
    });
    connect(adapter_, &HumanoidArmsSdkAdapter::logMessage, this, &ArmService::logMessage);

    pollTimer_ = new QTimer(this);
    pollTimer_->setInterval(800);
    connect(pollTimer_, &QTimer::timeout, this, &ArmService::refreshRealtimeStates);

    workerThread_->start();
}

ArmService::~ArmService()
{
    pollTimer_->stop();

    if (adapter_ && workerThread_->isRunning()) {
        QMetaObject::invokeMethod(adapter_, "shutdownSdk", Qt::BlockingQueuedConnection);
        workerThread_->quit();
        workerThread_->wait();
    }
}

bool ArmService::isInitialized() const
{
    return initialized_;
}

ArmState ArmService::armState(ArmSide side) const
{
    return side == ArmSide::Left ? leftArmState_ : rightArmState_;
}

void ArmService::initializeSdk(int deviceIndex, int canIndex)
{
    QMetaObject::invokeMethod(adapter_, "initializeSdk", Qt::QueuedConnection,
                              Q_ARG(int, deviceIndex),
                              Q_ARG(int, canIndex));
}

void ArmService::shutdownSdk()
{
    QMetaObject::invokeMethod(adapter_, "shutdownSdk", Qt::QueuedConnection);
}

void ArmService::refreshAllStates()
{
    QMetaObject::invokeMethod(adapter_, "refreshAllStates", Qt::QueuedConnection);
}

void ArmService::refreshRealtimeStates()
{
    QMetaObject::invokeMethod(adapter_, "refreshRealtimeStates", Qt::QueuedConnection);
}

void ArmService::moveJointTargets(ArmSide side, const QVector<double> &jointTargetsDeg, double velocity)
{
    QVector<double> jointTargetsRad;
    jointTargetsRad.reserve(jointTargetsDeg.size());
    for (double degrees : jointTargetsDeg) {
        jointTargetsRad.push_back(degrees * kDegToRad);
    }

    QMetaObject::invokeMethod(adapter_, "moveToJointPositions", Qt::QueuedConnection,
                              Q_ARG(dac::ArmSide, side),
                              Q_ARG(QVector<double>, jointTargetsRad),
                              Q_ARG(double, velocity));
}

void ArmService::brakeArm(ArmSide side)
{
    QMetaObject::invokeMethod(adapter_, "brakeArm", Qt::QueuedConnection,
                              Q_ARG(dac::ArmSide, side));
}

void ArmService::clearErrors(ArmSide side)
{
    QMetaObject::invokeMethod(adapter_, "clearArmErrors", Qt::QueuedConnection,
                              Q_ARG(dac::ArmSide, side));
}

void ArmService::storeState(const dac::ArmState &state)
{
    ArmState merged = state;
    ArmState &target = (state.side == ArmSide::Left) ? leftArmState_ : rightArmState_;

    if (merged.errorCodes.isEmpty()) {
        merged.errorCodes = target.errorCodes;
    }
    if (merged.errorSummary.isEmpty()) {
        merged.errorSummary = target.errorSummary;
    }

    target = merged;
}

} // namespace dac
