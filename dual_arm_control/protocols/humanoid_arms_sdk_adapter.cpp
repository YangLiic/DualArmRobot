#include "protocols/humanoid_arms_sdk_adapter.h"
#include <QDateTime>
#include <QStringList>
#include <QtGlobal>
#include <algorithm>
#include <cmath>

#include "Ti5BASIC.h"

namespace {

constexpr int kArmJointCount = 7;
constexpr int kArmPoseCount = 6;

::ArmSide toSdkArmSide(dac::ArmSide side)
{
    return side == dac::ArmSide::Left ? LEFT_ARM : RIGHT_ARM;
}

QVector<double> matrixToVector(const MatrixXd &matrix, int expectedCount)
{
    QVector<double> values;
    values.reserve(expectedCount);

    const int count = std::min(expectedCount, static_cast<int>(matrix.size()));
    for (int i = 0; i < count; ++i) {
        values.push_back(matrix(i));
    }
    while (values.size() < expectedCount) {
        values.push_back(0.0);
    }
    return values;
}

QVector<int32_t> errorArrayToVector(const int32_t *errors, int count)
{
    QVector<int32_t> values;
    values.reserve(count);
    for (int i = 0; i < count; ++i) {
        values.push_back(errors[i]);
    }
    return values;
}

QString summarizeErrors(const QVector<int32_t> &errors)
{
    QStringList parts;
    for (int i = 0; i < errors.size(); ++i) {
        if (errors[i] == 0) {
            continue;
        }
        parts << QStringLiteral("J%1=0x%2")
                     .arg(i + 1)
                     .arg(static_cast<qulonglong>(static_cast<uint32_t>(errors[i])), 8, 16, QChar('0'));
    }
    return parts.isEmpty() ? QStringLiteral("无错误") : parts.join(QStringLiteral(" | "));
}

} // namespace

namespace dac {

HumanoidArmsSdkAdapter::HumanoidArmsSdkAdapter(QObject *parent)
    : QObject(parent)
{
}

void HumanoidArmsSdkAdapter::initializeSdk(int deviceIndex, int canIndex)
{
    if (initialized_) {
        emit initializationChanged(true, QStringLiteral("机械臂 SDK 已连接"));
        return;
    }

    deviceIndex_ = deviceIndex;
    canIndex_ = canIndex;

    can_init();
    initialized_ = true;

    emit logMessage(LogLevel::Info,
                    QStringLiteral("机械臂 SDK 初始化完成，deviceIndex=%1, canIndex=%2")
                        .arg(deviceIndex_)
                        .arg(canIndex_));
    emit initializationChanged(true, QStringLiteral("机械臂 SDK 已连接"));

    refreshAllStates();
}

void HumanoidArmsSdkAdapter::shutdownSdk()
{
    if (!initialized_) {
        emit initializationChanged(false, QStringLiteral("机械臂 SDK 未连接"));
        return;
    }

    Exit();
    initialized_ = false;

    emit logMessage(LogLevel::Info, QStringLiteral("机械臂 SDK 已断开"));
    emit initializationChanged(false, QStringLiteral("机械臂 SDK 已断开"));
}

void HumanoidArmsSdkAdapter::refreshState(dac::ArmSide side)
{
    bool ok = false;
    const ArmState state = readArmState(side, true, &ok);
    emit armStateUpdated(state);

    if (!ok) {
        emit logMessage(LogLevel::Warning,
                        QStringLiteral("%1状态刷新失败: %2")
                            .arg(armSideText(side))
                            .arg(state.errorSummary));
    }
}

void HumanoidArmsSdkAdapter::refreshAllStates()
{
    refreshState(ArmSide::Left);
    refreshState(ArmSide::Right);
}

void HumanoidArmsSdkAdapter::refreshRealtimeStates()
{
    bool ok = false;
    emit armStateUpdated(readArmState(ArmSide::Left, false, &ok));
    emit armStateUpdated(readArmState(ArmSide::Right, false, &ok));
}

void HumanoidArmsSdkAdapter::moveToJointPositions(dac::ArmSide side,
                                                  const QVector<double> &jointPositionsRad,
                                                  double velocity)
{
    if (!initialized_) {
        emit logMessage(LogLevel::Error, QStringLiteral("机械臂 SDK 未初始化，无法发送关节位置命令"));
        return;
    }

    if (jointPositionsRad.size() != kArmJointCount) {
        emit logMessage(LogLevel::Error, QStringLiteral("关节目标数量错误，期望 7 个关节"));
        return;
    }

    Matrix<double, 1, 7> qd;
    for (int i = 0; i < kArmJointCount; ++i) {
        qd(i) = jointPositionsRad[i];
    }

    const int rc = moveJ_ToJoint(toSdkArmSide(side), deviceIndex_, canIndex_, qd, velocity);
    const QString msg = QStringLiteral("%1关节位置命令已发送，速度=%2 m/s，返回码=%3")
                            .arg(armSideText(side))
                            .arg(velocity, 0, 'f', 2)
                            .arg(rc);
    emit logMessage(rc == 0 ? LogLevel::Info : LogLevel::Warning, msg);

    refreshState(side);
}

void HumanoidArmsSdkAdapter::brakeArm(dac::ArmSide side)
{
    if (!initialized_) {
        emit logMessage(LogLevel::Error, QStringLiteral("机械臂 SDK 未初始化，无法执行刹车"));
        return;
    }

    const bool ok = ::brake(toSdkArmSide(side), deviceIndex_, canIndex_);
    emit logMessage(ok ? LogLevel::Warning : LogLevel::Error,
                    ok ? QStringLiteral("%1刹车已触发").arg(armSideText(side))
                       : QStringLiteral("%1刹车触发失败").arg(armSideText(side)));

    refreshState(side);
}

void HumanoidArmsSdkAdapter::clearArmErrors(dac::ArmSide side)
{
    if (!initialized_) {
        emit logMessage(LogLevel::Error, QStringLiteral("机械臂 SDK 未初始化，无法清错"));
        return;
    }

    clear_elc_error(toSdkArmSide(side), deviceIndex_, canIndex_);
    emit logMessage(LogLevel::Info, QStringLiteral("%1清错命令已发送").arg(armSideText(side)));

    refreshState(side);
}

ArmState HumanoidArmsSdkAdapter::readArmState(dac::ArmSide side, bool includeErrors, bool *ok) const
{
    ArmState state;
    state.side = side;
    state.initialized = initialized_;
    state.lastUpdateTime = QDateTime::currentDateTime();

    if (!initialized_) {
        state.online = false;
        state.errorSummary = QStringLiteral("机械臂 SDK 未初始化");
        if (ok) {
            *ok = false;
        }
        return state;
    }

    int32_t errors[kArmJointCount] = {};
    const MatrixXd joint = get_joint(toSdkArmSide(side), deviceIndex_, canIndex_);
    const MatrixXd pose = get_Pos(toSdkArmSide(side), deviceIndex_, canIndex_);

    state.online = true;
    state.jointPositionsRad = matrixToVector(joint, kArmJointCount);
    state.tcpPose = matrixToVector(pose, kArmPoseCount);
    if (includeErrors) {
        get_mechanicalarm_status(toSdkArmSide(side), deviceIndex_, canIndex_, errors);
        state.errorCodes = errorArrayToVector(errors, kArmJointCount);
        state.errorSummary = summarizeErrors(state.errorCodes);
    }

    if (ok) {
        *ok = true;
    }
    return state;
}

} // namespace dac
