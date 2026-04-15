#include "ui/humanoid_arms_panel.h"
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace {

constexpr int kArmJointCount = 7;
constexpr int kSliderScale = 10;
constexpr int kSliderMin = -180 * kSliderScale;
constexpr int kSliderMax = 180 * kSliderScale;
constexpr double kRadToDeg = 180.0 / M_PI;

const char *kJointNames[kArmJointCount] = {
    "J1 肩俯仰",
    "J2 肩翻滚",
    "J3 肩偏航",
    "J4 肘偏航",
    "J5 腕俯仰",
    "J6 腕偏航",
    "J7 腕翻滚"
};

QString formatDegrees(double value)
{
    return QStringLiteral("%1°").arg(value, 0, 'f', 1);
}

} // namespace

namespace dac {

HumanoidArmsPanel::HumanoidArmsPanel(QWidget *parent)
    : QWidget(parent)
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    buildConnectionGroup(mainLayout);
    buildArmGroup(mainLayout, ArmSide::Left);
    buildArmGroup(mainLayout, ArmSide::Right);
    mainLayout->addStretch();

    setConnectionState(false, QStringLiteral("机械臂 SDK 未连接"));
}

void HumanoidArmsPanel::setConnectionState(bool connected, const QString &message)
{
    connected_ = connected;

    connectBtn_->setEnabled(!connected_);
    disconnectBtn_->setEnabled(connected_);
    refreshBtn_->setEnabled(connected_);
    velocitySpin_->setEnabled(connected_);

    sdkStateLabel_->setText(QStringLiteral("状态: %1").arg(message));
    sdkStateLabel_->setStyleSheet(connected_
        ? "QLabel { color: #a6e3a1; font-weight: bold; padding: 2px 0; }"
        : "QLabel { color: #f9e2af; font-weight: bold; padding: 2px 0; }");

    auto resetArm = [connected_ = connected_](ArmWidgets &widgets) {
        widgets.syncBtn->setEnabled(connected_ && widgets.stateReceived);
        widgets.clearErrorsBtn->setEnabled(connected_);
        widgets.brakeBtn->setEnabled(connected_);
        for (auto *slider : widgets.sliders) {
            slider->setEnabled(connected_);
        }
    };

    if (!connected_) {
        leftArm_.stateReceived = false;
        leftArm_.targetsInitialized = false;
        rightArm_.stateReceived = false;
        rightArm_.targetsInitialized = false;
    }

    resetArm(leftArm_);
    resetArm(rightArm_);
}

void HumanoidArmsPanel::updateArmState(const dac::ArmState &state)
{
    ArmWidgets &widgets = widgetsForSide(state.side);
    widgets.stateReceived = state.online;

    widgets.actualDegrees.clear();
    widgets.actualDegrees.reserve(state.jointPositionsRad.size());
    for (double radians : state.jointPositionsRad) {
        widgets.actualDegrees.push_back(radians * kRadToDeg);
    }

    for (int i = 0; i < widgets.actualLabels.size(); ++i) {
        const double value = i < widgets.actualDegrees.size() ? widgets.actualDegrees[i] : 0.0;
        widgets.actualLabels[i]->setText(QStringLiteral("当前 %1").arg(formatDegrees(value)));
    }

    if (widgets.stateReceived && !widgets.targetsInitialized) {
        syncTargetsFromActual(state.side);
    }

    updateArmStateText(state.side);

    widgets.poseLabel->setText(QStringLiteral("末端位姿: %1").arg(poseText(state.tcpPose)));
    widgets.errorLabel->setText(QStringLiteral("错误状态: %1").arg(errorText(state)));
    widgets.lastUpdateLabel->setText(QStringLiteral("更新时间: %1")
                                         .arg(state.lastUpdateTime.isValid()
                                                  ? state.lastUpdateTime.toString(QStringLiteral("HH:mm:ss"))
                                                  : QStringLiteral("-")));

    widgets.syncBtn->setEnabled(connected_ && widgets.stateReceived);
}

void HumanoidArmsPanel::buildConnectionGroup(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("HumanoidArms 机械臂"));
    auto *vbox = new QVBoxLayout(group);

    auto *line = new QHBoxLayout;
    deviceIndexSpin_ = new QSpinBox;
    deviceIndexSpin_->setRange(0, 8);
    deviceIndexSpin_->setValue(0);
    line->addWidget(new QLabel(QStringLiteral("deviceIndex:")));
    line->addWidget(deviceIndexSpin_);

    canIndexSpin_ = new QSpinBox;
    canIndexSpin_->setRange(0, 1);
    canIndexSpin_->setValue(0);
    line->addWidget(new QLabel(QStringLiteral("canIndex:")));
    line->addWidget(canIndexSpin_);

    velocitySpin_ = new QDoubleSpinBox;
    velocitySpin_->setRange(0.05, 0.40);
    velocitySpin_->setSingleStep(0.01);
    velocitySpin_->setDecimals(2);
    velocitySpin_->setValue(0.20);
    velocitySpin_->setSuffix(QStringLiteral(" m/s"));
    line->addWidget(new QLabel(QStringLiteral("速度:")));
    line->addWidget(velocitySpin_);
    line->addStretch();
    vbox->addLayout(line);

    auto *btnLine = new QHBoxLayout;
    connectBtn_ = new QPushButton(QStringLiteral("连接 SDK"));
    disconnectBtn_ = new QPushButton(QStringLiteral("断开 SDK"));
    refreshBtn_ = new QPushButton(QStringLiteral("刷新状态"));
    btnLine->addWidget(connectBtn_);
    btnLine->addWidget(disconnectBtn_);
    btnLine->addWidget(refreshBtn_);
    btnLine->addStretch();
    vbox->addLayout(btnLine);

    sdkStateLabel_ = new QLabel;
    vbox->addWidget(sdkStateLabel_);

    auto *hint = new QLabel(QStringLiteral("关节滑块采用角度显示，内部会自动换算为 SDK 所需的弧度；拖动释放后按整臂 7 关节目标发送。"));
    hint->setWordWrap(true);
    hint->setStyleSheet("QLabel { color: #bac2de; }");
    vbox->addWidget(hint);

    layout->addWidget(group);

    connect(connectBtn_, &QPushButton::clicked, this, [this]() {
        emit connectRequested(deviceIndexSpin_->value(), canIndexSpin_->value());
    });
    connect(disconnectBtn_, &QPushButton::clicked, this, &HumanoidArmsPanel::disconnectRequested);
    connect(refreshBtn_, &QPushButton::clicked, this, &HumanoidArmsPanel::refreshRequested);
}

void HumanoidArmsPanel::buildArmGroup(QVBoxLayout *layout, dac::ArmSide side)
{
    ArmWidgets &widgets = widgetsForSide(side);
    widgets.targetDegrees.fill(0.0, kArmJointCount);
    widgets.actualDegrees.fill(0.0, kArmJointCount);

    auto *group = new QGroupBox(QStringLiteral("%1关节控制").arg(armSideText(side)));
    auto *vbox = new QVBoxLayout(group);

    widgets.stateLabel = new QLabel(QStringLiteral("状态: 等待状态刷新"));
    widgets.stateLabel->setStyleSheet("QLabel { color: #cdd6f4; }");
    vbox->addWidget(widgets.stateLabel);

    for (int i = 0; i < kArmJointCount; ++i) {
        auto *row = new QHBoxLayout;

        auto *nameLabel = new QLabel(QString::fromUtf8(kJointNames[i]));
        nameLabel->setMinimumWidth(86);
        row->addWidget(nameLabel);

        auto *slider = new QSlider(Qt::Horizontal);
        slider->setRange(kSliderMin, kSliderMax);
        slider->setSingleStep(1);
        slider->setPageStep(50);
        slider->setToolTip(QStringLiteral("当前暂按 ±180° 配置滑块范围，可后续按真实关节限制再细化"));
        row->addWidget(slider, 1);

        auto *targetLabel = new QLabel(QStringLiteral("目标 0.0°"));
        targetLabel->setMinimumWidth(82);
        row->addWidget(targetLabel);

        auto *actualLabel = new QLabel(QStringLiteral("当前 0.0°"));
        actualLabel->setMinimumWidth(82);
        row->addWidget(actualLabel);

        widgets.sliders.push_back(slider);
        widgets.targetLabels.push_back(targetLabel);
        widgets.actualLabels.push_back(actualLabel);

        connect(slider, &QSlider::valueChanged, this, [this, side, i](int value) {
            ArmWidgets &arm = widgetsForSide(side);
            arm.targetDegrees[i] = value / static_cast<double>(kSliderScale);
            arm.targetLabels[i]->setText(QStringLiteral("目标 %1").arg(formatDegrees(arm.targetDegrees[i])));
        });
        connect(slider, &QSlider::sliderReleased, this, [this, side]() {
            emitJointTargets(side);
        });

        vbox->addLayout(row);
    }

    auto *btnRow = new QHBoxLayout;
    widgets.syncBtn = new QPushButton(QStringLiteral("同步当前姿态"));
    widgets.clearErrorsBtn = new QPushButton(QStringLiteral("清错"));
    widgets.brakeBtn = new QPushButton(QStringLiteral("刹车"));
    btnRow->addWidget(widgets.syncBtn);
    btnRow->addWidget(widgets.clearErrorsBtn);
    btnRow->addWidget(widgets.brakeBtn);
    btnRow->addStretch();
    vbox->addLayout(btnRow);

    widgets.poseLabel = new QLabel(QStringLiteral("末端位姿: -"));
    widgets.poseLabel->setWordWrap(true);
    vbox->addWidget(widgets.poseLabel);

    widgets.errorLabel = new QLabel(QStringLiteral("错误状态: -"));
    widgets.errorLabel->setWordWrap(true);
    vbox->addWidget(widgets.errorLabel);

    widgets.lastUpdateLabel = new QLabel(QStringLiteral("更新时间: -"));
    widgets.lastUpdateLabel->setStyleSheet("QLabel { color: #7f849c; }");
    vbox->addWidget(widgets.lastUpdateLabel);

    layout->addWidget(group);

    connect(widgets.syncBtn, &QPushButton::clicked, this, [this, side]() {
        syncTargetsFromActual(side);
    });
    connect(widgets.clearErrorsBtn, &QPushButton::clicked, this, [this, side]() {
        emit clearErrorsRequested(side);
    });
    connect(widgets.brakeBtn, &QPushButton::clicked, this, [this, side]() {
        emit brakeRequested(side);
    });
}

HumanoidArmsPanel::ArmWidgets &HumanoidArmsPanel::widgetsForSide(dac::ArmSide side)
{
    return side == ArmSide::Left ? leftArm_ : rightArm_;
}

const HumanoidArmsPanel::ArmWidgets &HumanoidArmsPanel::widgetsForSide(dac::ArmSide side) const
{
    return side == ArmSide::Left ? leftArm_ : rightArm_;
}

void HumanoidArmsPanel::syncTargetsFromActual(dac::ArmSide side)
{
    ArmWidgets &widgets = widgetsForSide(side);
    if (!widgets.stateReceived || widgets.actualDegrees.size() != kArmJointCount) {
        return;
    }

    widgets.targetDegrees = widgets.actualDegrees;
    widgets.targetsInitialized = true;

    for (int i = 0; i < widgets.sliders.size(); ++i) {
        const int sliderValue = std::clamp(static_cast<int>(std::lround(widgets.targetDegrees[i] * kSliderScale)),
                                           kSliderMin, kSliderMax);
        const QSignalBlocker blocker(widgets.sliders[i]);
        widgets.sliders[i]->setValue(sliderValue);
        widgets.targetLabels[i]->setText(QStringLiteral("目标 %1").arg(formatDegrees(widgets.targetDegrees[i])));
    }

    updateArmStateText(side);
}

void HumanoidArmsPanel::updateArmStateText(dac::ArmSide side)
{
    const ArmWidgets &widgets = widgetsForSide(side);
    QString text = QStringLiteral("%1 | %2 | 目标姿态%3")
                       .arg(armSideText(side))
                       .arg(widgets.stateReceived ? QStringLiteral("在线") : QStringLiteral("离线"))
                       .arg(widgets.targetsInitialized ? QStringLiteral("已同步") : QStringLiteral("未同步"));
    widgets.stateLabel->setText(text);
    widgets.stateLabel->setStyleSheet(widgets.stateReceived
        ? "QLabel { color: #a6e3a1; font-weight: bold; }"
        : "QLabel { color: #f9e2af; font-weight: bold; }");
}

void HumanoidArmsPanel::emitJointTargets(dac::ArmSide side)
{
    ArmWidgets &widgets = widgetsForSide(side);
    if (!connected_ || !widgets.stateReceived) {
        return;
    }

    if (!widgets.targetsInitialized) {
        syncTargetsFromActual(side);
        if (!widgets.targetsInitialized) {
            return;
        }
    }

    emit jointTargetsRequested(side, widgets.targetDegrees, velocitySpin_->value());
}

QString HumanoidArmsPanel::poseText(const QVector<double> &pose)
{
    if (pose.size() < 6) {
        return QStringLiteral("-");
    }

    return QStringLiteral("x=%1 m, y=%2 m, z=%3 m, rx=%4°, ry=%5°, rz=%6°")
        .arg(pose[0], 0, 'f', 3)
        .arg(pose[1], 0, 'f', 3)
        .arg(pose[2], 0, 'f', 3)
        .arg(pose[3] * kRadToDeg, 0, 'f', 1)
        .arg(pose[4] * kRadToDeg, 0, 'f', 1)
        .arg(pose[5] * kRadToDeg, 0, 'f', 1);
}

QString HumanoidArmsPanel::errorText(const dac::ArmState &state)
{
    if (!state.initialized) {
        return QStringLiteral("机械臂 SDK 未初始化");
    }
    if (!state.online) {
        return QStringLiteral("机械臂离线");
    }
    return state.errorSummary.isEmpty() ? QStringLiteral("无错误") : state.errorSummary;
}

} // namespace dac
