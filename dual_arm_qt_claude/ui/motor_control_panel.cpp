/**
 * MotorControlPanel 实现
 */
#include "motor_control_panel.h"
#include "../services/motor_service.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QMessageBox>

namespace dar {

MotorControlPanel::MotorControlPanel(MotorService *motorService, QWidget *parent)
    : QWidget(parent)
    , motorService_(motorService)
{
    setupUi();
}

void MotorControlPanel::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(12);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // ========== 节点选择 ==========
    auto *selectGroup = new QGroupBox(QStringLiteral("🎯 选择电机"), this);
    auto *selectLayout = new QHBoxLayout(selectGroup);

    motorCombo_ = new QComboBox();
    motorCombo_->setMinimumWidth(150);
    connect(motorCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MotorControlPanel::onMotorSelected);

    stateLabel_ = new QLabel(QStringLiteral("状态: --"));
    stateLabel_->setStyleSheet("font-weight: bold; font-size: 13px;");
    modeLabel_ = new QLabel(QStringLiteral("模式: --"));
    modeLabel_->setStyleSheet("font-size: 13px;");

    selectLayout->addWidget(new QLabel(QStringLiteral("电机:")));
    selectLayout->addWidget(motorCombo_);
    selectLayout->addSpacing(20);
    selectLayout->addWidget(stateLabel_);
    selectLayout->addSpacing(10);
    selectLayout->addWidget(modeLabel_);
    selectLayout->addStretch();
    mainLayout->addWidget(selectGroup);

    // ========== 基础控制 ==========
    auto *ctrlGroup = new QGroupBox(QStringLiteral("🕹️  基础控制"), this);
    auto *ctrlLayout = new QVBoxLayout(ctrlGroup);

    // 模式选择
    auto *modeRow = new QHBoxLayout();
    velocityModeRadio_ = new QRadioButton(QStringLiteral("速度模式"));
    positionModeRadio_ = new QRadioButton(QStringLiteral("位置模式"));
    velocityModeRadio_->setChecked(true);
    modeRow->addWidget(new QLabel(QStringLiteral("使能模式:")));
    modeRow->addWidget(velocityModeRadio_);
    modeRow->addWidget(positionModeRadio_);
    modeRow->addStretch();
    ctrlLayout->addLayout(modeRow);

    // 控制按钮
    auto *btnRow = new QHBoxLayout();
    enableBtn_ = new QPushButton(QStringLiteral("✅ 使能"));
    enableBtn_->setObjectName("enableBtn");
    enableBtn_->setMinimumHeight(45);
    disableBtn_ = new QPushButton(QStringLiteral("⏸️  失能"));
    disableBtn_->setObjectName("disableBtn");
    disableBtn_->setMinimumHeight(45);
    quickStopBtn_ = new QPushButton(QStringLiteral("🛑 急停"));
    quickStopBtn_->setObjectName("quickStopBtn");
    quickStopBtn_->setMinimumHeight(45);
    faultResetBtn_ = new QPushButton(QStringLiteral("🔄 故障复位"));
    faultResetBtn_->setObjectName("faultResetBtn");
    faultResetBtn_->setMinimumHeight(45);

    connect(enableBtn_, &QPushButton::clicked, this, &MotorControlPanel::onEnableClicked);
    connect(disableBtn_, &QPushButton::clicked, this, &MotorControlPanel::onDisableClicked);
    connect(quickStopBtn_, &QPushButton::clicked, this, &MotorControlPanel::onQuickStopClicked);
    connect(faultResetBtn_, &QPushButton::clicked, this, &MotorControlPanel::onFaultResetClicked);

    btnRow->addWidget(enableBtn_);
    btnRow->addWidget(disableBtn_);
    btnRow->addWidget(quickStopBtn_);
    btnRow->addWidget(faultResetBtn_);
    ctrlLayout->addLayout(btnRow);

    // 抱闸
    auto *brakeRow = new QHBoxLayout();
    releaseBrakeBtn_ = new QPushButton(QStringLiteral("🔓 松闸"));
    lockBrakeBtn_ = new QPushButton(QStringLiteral("🔒 锁闸"));
    releaseBrakeBtn_->setMinimumHeight(35);
    lockBrakeBtn_->setMinimumHeight(35);
    connect(releaseBrakeBtn_, &QPushButton::clicked, this, &MotorControlPanel::onReleaseBrakeClicked);
    connect(lockBrakeBtn_, &QPushButton::clicked, this, &MotorControlPanel::onLockBrakeClicked);
    brakeRow->addWidget(releaseBrakeBtn_);
    brakeRow->addWidget(lockBrakeBtn_);
    brakeRow->addStretch();
    ctrlLayout->addLayout(brakeRow);

    mainLayout->addWidget(ctrlGroup);

    // ========== 速度控制 ==========
    auto *velGroup = new QGroupBox(QStringLiteral("🚀 速度控制"), this);
    auto *velLayout = new QVBoxLayout(velGroup);

    auto *velRow = new QHBoxLayout();
    velocitySlider_ = new QSlider(Qt::Horizontal);
    velocitySlider_->setRange(-500, 500);
    velocitySlider_->setValue(0);
    velocitySlider_->setTickPosition(QSlider::TicksBelow);
    velocitySlider_->setTickInterval(100);

    velocitySpin_ = new QSpinBox();
    velocitySpin_->setRange(-3000, 3000);
    velocitySpin_->setValue(0);
    velocitySpin_->setSuffix(" RPM");
    velocitySpin_->setMinimumWidth(120);

    connect(velocitySlider_, &QSlider::valueChanged, this, &MotorControlPanel::onVelocitySliderChanged);

    velRow->addWidget(new QLabel(QStringLiteral("速度:")));
    velRow->addWidget(velocitySlider_, 1);
    velRow->addWidget(velocitySpin_);
    velLayout->addLayout(velRow);

    auto *velBtnRow = new QHBoxLayout();
    setVelocityBtn_ = new QPushButton(QStringLiteral("▶  设置速度"));
    setVelocityBtn_->setObjectName("setVelocityBtn");
    setVelocityBtn_->setMinimumHeight(38);
    stopVelocityBtn_ = new QPushButton(QStringLiteral("⏹  速度归零"));
    stopVelocityBtn_->setMinimumHeight(38);

    connect(setVelocityBtn_, &QPushButton::clicked, this, &MotorControlPanel::onSetVelocity);
    connect(stopVelocityBtn_, &QPushButton::clicked, this, &MotorControlPanel::onStopVelocity);

    velBtnRow->addWidget(setVelocityBtn_);
    velBtnRow->addWidget(stopVelocityBtn_);
    velBtnRow->addStretch();
    velLayout->addLayout(velBtnRow);

    mainLayout->addWidget(velGroup);

    // ========== 位置控制 ==========
    auto *posGroup = new QGroupBox(QStringLiteral("📍 位置控制"), this);
    auto *posLayout = new QVBoxLayout(posGroup);

    auto *posRow = new QHBoxLayout();
    positionSpin_ = new QDoubleSpinBox();
    positionSpin_->setRange(-36000, 36000);
    positionSpin_->setDecimals(2);
    positionSpin_->setSuffix(" °");
    positionSpin_->setMinimumWidth(150);

    relativeRadio_ = new QRadioButton(QStringLiteral("相对"));
    absoluteRadio_ = new QRadioButton(QStringLiteral("绝对"));
    relativeRadio_->setChecked(true);

    setPositionBtn_ = new QPushButton(QStringLiteral("▶  运动到目标"));
    setPositionBtn_->setObjectName("setPositionBtn");
    setPositionBtn_->setMinimumHeight(38);
    connect(setPositionBtn_, &QPushButton::clicked, this, &MotorControlPanel::onSetPosition);

    posRow->addWidget(new QLabel(QStringLiteral("角度:")));
    posRow->addWidget(positionSpin_);
    posRow->addWidget(relativeRadio_);
    posRow->addWidget(absoluteRadio_);
    posRow->addWidget(setPositionBtn_);
    posRow->addStretch();
    posLayout->addLayout(posRow);

    // 运动参数
    auto *paramRow = new QHBoxLayout();
    profileVelSpin_ = new QSpinBox();
    profileVelSpin_->setRange(1, 3000);
    profileVelSpin_->setValue(60);
    profileVelSpin_->setSuffix(" RPM");
    profileAccSpin_ = new QSpinBox();
    profileAccSpin_->setRange(1, 10000);
    profileAccSpin_->setValue(500);
    profileAccSpin_->setSuffix(" RPM/s");
    profileDecSpin_ = new QSpinBox();
    profileDecSpin_->setRange(1, 10000);
    profileDecSpin_->setValue(500);
    profileDecSpin_->setSuffix(" RPM/s");

    paramRow->addWidget(new QLabel(QStringLiteral("速度:")));
    paramRow->addWidget(profileVelSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("加速:")));
    paramRow->addWidget(profileAccSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("减速:")));
    paramRow->addWidget(profileDecSpin_);
    paramRow->addStretch();
    posLayout->addLayout(paramRow);

    mainLayout->addWidget(posGroup);

    // ========== 批量操作 ==========
    auto *batchGroup = new QGroupBox(QStringLiteral("📦 批量操作"), this);
    auto *batchLayout = new QHBoxLayout(batchGroup);

    enableAllBtn_ = new QPushButton(QStringLiteral("✅ 全部使能"));
    enableAllBtn_->setMinimumHeight(38);
    disableAllBtn_ = new QPushButton(QStringLiteral("⏸️  全部失能"));
    disableAllBtn_->setMinimumHeight(38);
    quickStopAllBtn_ = new QPushButton(QStringLiteral("🛑 全部急停"));
    quickStopAllBtn_->setObjectName("quickStopAllBtn");
    quickStopAllBtn_->setMinimumHeight(38);
    faultResetAllBtn_ = new QPushButton(QStringLiteral("🔄 全部复位"));
    faultResetAllBtn_->setMinimumHeight(38);

    connect(enableAllBtn_, &QPushButton::clicked, this, &MotorControlPanel::onEnableAllClicked);
    connect(disableAllBtn_, &QPushButton::clicked, this, &MotorControlPanel::onDisableAllClicked);
    connect(quickStopAllBtn_, &QPushButton::clicked, this, &MotorControlPanel::onQuickStopAllClicked);
    connect(faultResetAllBtn_, &QPushButton::clicked, this, &MotorControlPanel::onFaultResetAllClicked);

    batchLayout->addWidget(enableAllBtn_);
    batchLayout->addWidget(disableAllBtn_);
    batchLayout->addWidget(quickStopAllBtn_);
    batchLayout->addWidget(faultResetAllBtn_);
    mainLayout->addWidget(batchGroup);

    mainLayout->addStretch();
}

void MotorControlPanel::refreshMotorList()
{
    motorCombo_->clear();
    auto nodeIds = motorService_->motorNodeIds();
    for (auto nodeId : nodeIds) {
        motorCombo_->addItem(
            QStringLiteral("电机 0x%1").arg(nodeId, 3, 16, QChar('0')),
            QVariant(nodeId));
    }
}

uint32_t MotorControlPanel::currentNodeId() const
{
    if (motorCombo_->count() == 0) return 0;
    return motorCombo_->currentData().toUInt();
}

void MotorControlPanel::onMotorSelected(int index)
{
    Q_UNUSED(index);
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;

    auto state = motorService_->motorState(nodeId);
    onMotorStateChanged(nodeId, state);
}

void MotorControlPanel::onMotorStateChanged(uint32_t nodeId, const dar::MotorState &state)
{
    if (nodeId != currentNodeId()) return;

    stateLabel_->setText(QStringLiteral("状态: %1").arg(state.stateText()));
    modeLabel_->setText(QStringLiteral("模式: %1").arg(state.modeText()));

    // 根据状态调整颜色
    if (state.faultCode != 0) {
        stateLabel_->setStyleSheet("font-weight: bold; font-size: 13px; color: #ef4444;");
    } else if (state.enabled) {
        stateLabel_->setStyleSheet("font-weight: bold; font-size: 13px; color: #22c55e;");
    } else if (state.online) {
        stateLabel_->setStyleSheet("font-weight: bold; font-size: 13px; color: #3b82f6;");
    } else {
        stateLabel_->setStyleSheet("font-weight: bold; font-size: 13px; color: #888;");
    }
}

void MotorControlPanel::onEnableClicked()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;

    OperationMode mode = velocityModeRadio_->isChecked()
                             ? OperationMode::Velocity
                             : OperationMode::ProfilePosition;
    motorService_->enableMotor(nodeId, mode);
}

void MotorControlPanel::onDisableClicked()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;
    motorService_->disableMotor(nodeId);
}

void MotorControlPanel::onQuickStopClicked()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;
    motorService_->quickStopMotor(nodeId);
}

void MotorControlPanel::onFaultResetClicked()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;
    motorService_->faultResetMotor(nodeId);
}

void MotorControlPanel::onSetVelocity()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;
    motorService_->setVelocity(nodeId, velocitySpin_->value());
}

void MotorControlPanel::onVelocitySliderChanged(int value)
{
    velocitySpin_->setValue(value);
}

void MotorControlPanel::onStopVelocity()
{
    velocitySpin_->setValue(0);
    velocitySlider_->setValue(0);
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;
    motorService_->setVelocity(nodeId, 0);
}

void MotorControlPanel::onSetPosition()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;

    // 先设置运动参数
    motorService_->setProfileVelocity(nodeId, static_cast<uint32_t>(profileVelSpin_->value()));
    motorService_->setProfileAcceleration(nodeId, static_cast<uint32_t>(profileAccSpin_->value()));
    motorService_->setProfileDeceleration(nodeId, static_cast<uint32_t>(profileDecSpin_->value()));

    // 设置位置
    motorService_->setPosition(nodeId, positionSpin_->value(), absoluteRadio_->isChecked());
}

void MotorControlPanel::onEnableAllClicked()
{
    OperationMode mode = velocityModeRadio_->isChecked()
                             ? OperationMode::Velocity
                             : OperationMode::ProfilePosition;
    motorService_->enableAllMotors(mode);
}

void MotorControlPanel::onDisableAllClicked()
{
    motorService_->disableAllMotors();
}

void MotorControlPanel::onQuickStopAllClicked()
{
    motorService_->quickStopAllMotors();
}

void MotorControlPanel::onFaultResetAllClicked()
{
    motorService_->faultResetAllMotors();
}

void MotorControlPanel::onReleaseBrakeClicked()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;

    auto ret = QMessageBox::warning(this, QStringLiteral("警告"),
        QStringLiteral("松闸后电机将失去保持力！\n垂直安装时负载会下落！\n\n确定松闸？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret == QMessageBox::Yes) {
        motorService_->releaseBrake(nodeId);
    }
}

void MotorControlPanel::onLockBrakeClicked()
{
    uint32_t nodeId = currentNodeId();
    if (nodeId == 0) return;
    motorService_->lockBrake(nodeId);
}

}  // namespace dar
