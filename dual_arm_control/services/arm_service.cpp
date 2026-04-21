#include "services/arm_service.h"
#include "protocols/humanoid_arms_sdk_adapter.h"
#include <QMetaObject>
#include <QStringList>
#include <QThread>
#include <QTimer>
#include <cmath>
#include <algorithm>

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kSdkVelocityMin = 0.05;
constexpr double kSdkVelocityMax = 0.40;

QString formatJointValues(const QVector<double> &values, int precision, const QString &unitSuffix)
{
    QStringList parts;
    parts.reserve(values.size());
    for (int i = 0; i < values.size(); ++i) {
        parts << QStringLiteral("J%1=%2%3")
                     .arg(i + 1)
                     .arg(values[i], 0, 'f', precision)
                     .arg(unitSuffix);
    }
    return parts.join(QStringLiteral(", "));
}

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

    // 左臂默认软件限位（单位：度）
    leftJointLimits_.minDegrees = QVector<double>{0.0, 0.0, -90.0, -90.0, -90.0, -90.0, -90.0};
    leftJointLimits_.maxDegrees = QVector<double>{90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
    // 右臂默认软件限位（单位：度）
    rightJointLimits_.minDegrees = QVector<double>{180.0, -180.0, -270.0, -90.0, -90.0, -90.0, -90.0};
    rightJointLimits_.maxDegrees = QVector<double>{270.0,  -90.0,  -90.0,  90.0,  90.0,  90.0,  90.0};

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
        if (!initialized_) {
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

ArmJointLimits ArmService::jointLimits(ArmSide side) const
{
    return side == ArmSide::Left ? leftJointLimits_ : rightJointLimits_;
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
    if (jointTargetsDeg.size() != kArmJointCount) {
        emit logMessage(LogLevel::Error,
                        QStringLiteral("%1关节目标数量错误，期望 %2 个，实际 %3 个")
                            .arg(armSideText(side))
                            .arg(kArmJointCount)
                            .arg(jointTargetsDeg.size()));
        return;
    }

    const ArmJointLimits &limits = (side == ArmSide::Left) ? leftJointLimits_ : rightJointLimits_;
    QVector<double> clampedTargetsDeg;
    clampedTargetsDeg.reserve(jointTargetsDeg.size());
    QStringList clampMessages;
    for (int i = 0; i < jointTargetsDeg.size(); ++i) {
        const double requestedDeg = jointTargetsDeg[i];
        if (!std::isfinite(requestedDeg)) {
            emit logMessage(LogLevel::Error,
                            QStringLiteral("%1关节目标无效: J%2=%3")
                                .arg(armSideText(side))
                                .arg(i + 1)
                                .arg(requestedDeg));
            return;
        }

        const double minDeg = limits.minDegrees.value(i, kDefaultArmJointMinDeg);
        const double maxDeg = limits.maxDegrees.value(i, kDefaultArmJointMaxDeg);
        const double clampedDeg = std::clamp(requestedDeg, minDeg, maxDeg);
        if (std::abs(requestedDeg - clampedDeg) > 1e-9) {
            clampMessages << QStringLiteral("J%1 %2°→%3° [%4°, %5°]")
                                 .arg(i + 1)
                                 .arg(requestedDeg, 0, 'f', 1)
                                 .arg(clampedDeg, 0, 'f', 1)
                                 .arg(minDeg, 0, 'f', 1)
                                 .arg(maxDeg, 0, 'f', 1);
        }
        clampedTargetsDeg.push_back(clampedDeg);
    }

    if (!clampMessages.isEmpty()) {
        emit logMessage(LogLevel::Warning,
                        QStringLiteral("%1软限位已生效: %2")
                            .arg(armSideText(side))
                            .arg(clampMessages.join(QStringLiteral(" | "))));
    }

    QVector<double> jointTargetsRad;
    jointTargetsRad.reserve(clampedTargetsDeg.size());
    for (double degrees : clampedTargetsDeg) {
        jointTargetsRad.push_back(degrees * kDegToRad);
    }

    double safeVelocity = velocity;
    if (!std::isfinite(safeVelocity)) {
        safeVelocity = kSdkVelocityMin;
    }
    safeVelocity = std::clamp(safeVelocity, kSdkVelocityMin, kSdkVelocityMax);
    if (std::abs(safeVelocity - velocity) > 1e-9) {
        emit logMessage(
            LogLevel::Warning,
            QStringLiteral("%1速度参数越界，已钳制到 %2 m/s（输入=%3）")
                .arg(armSideText(side))
                .arg(safeVelocity, 0, 'f', 2)
                .arg(velocity, 0, 'f', 2));
    }

    emit logMessage(
        LogLevel::Info,
        QStringLiteral("%1关节目标下发: deg={%2} | rad={%3} | vel=%4 m/s")
            .arg(armSideText(side))
            .arg(formatJointValues(clampedTargetsDeg, 1, QStringLiteral("°")))
            .arg(formatJointValues(jointTargetsRad, 4, QStringLiteral("rad")))
            .arg(safeVelocity, 0, 'f', 2));

    QMetaObject::invokeMethod(adapter_, "moveToJointPositions", Qt::QueuedConnection,
                              Q_ARG(dac::ArmSide, side),
                              Q_ARG(QVector<double>, jointTargetsRad),
                              Q_ARG(double, safeVelocity));
}

void ArmService::setJointLimits(ArmSide side,
                                const QVector<double> &minLimitsDeg,
                                const QVector<double> &maxLimitsDeg)
{
    ArmJointLimits &target = (side == ArmSide::Left) ? leftJointLimits_ : rightJointLimits_;
    const ArmJointLimits normalized = normalizeJointLimits(minLimitsDeg, maxLimitsDeg);
    if (target.minDegrees == normalized.minDegrees && target.maxDegrees == normalized.maxDegrees) {
        return;
    }

    target = normalized;
    emit jointLimitsChanged(side, target.minDegrees, target.maxDegrees);
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

ArmJointLimits ArmService::normalizeJointLimits(const QVector<double> &minLimitsDeg,
                                                const QVector<double> &maxLimitsDeg)
{
    ArmJointLimits normalized;
    for (int i = 0; i < kArmJointCount; ++i) {
        double minDeg = minLimitsDeg.value(i, normalized.minDegrees[i]);
        double maxDeg = maxLimitsDeg.value(i, normalized.maxDegrees[i]);

        if (!std::isfinite(minDeg)) {
            minDeg = normalized.minDegrees[i];
        }
        if (!std::isfinite(maxDeg)) {
            maxDeg = normalized.maxDegrees[i];
        }
        if (minDeg > maxDeg) {
            std::swap(minDeg, maxDeg);
        }

        normalized.minDegrees[i] = minDeg;
        normalized.maxDegrees[i] = maxDeg;
    }
    return normalized;
}

} // namespace dac
