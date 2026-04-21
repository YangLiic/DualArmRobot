#include "ui/motor_control_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

namespace dac {

MotorControlPanel::MotorControlPanel(QWidget *parent) : QWidget(parent)
{
    auto *vbox = new QVBoxLayout(this);
    vbox->setContentsMargins(0, 0, 0, 0);

    buildNodeSelector(vbox);
    buildModeAndEnable(vbox);
    buildVelocityControl(vbox);
    buildPositionControl(vbox);
    buildBrakeControl(vbox);
    buildBatchButtons(vbox);

    stateLabel_ = new QLabel(QStringLiteral("状态: -"));
    stateLabel_->setStyleSheet(
        "QLabel { background: #fffaf4; color: #314152; border: 1px solid #ddd2c4; "
        "border-radius: 8px; padding: 6px 10px; }");
    vbox->addWidget(stateLabel_);
    vbox->addStretch();
}

uint32_t MotorControlPanel::currentNodeId() const
{
    return nodeCombo_->currentData().toUInt();
}

void MotorControlPanel::buildNodeSelector(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("电机选择"));
    auto *h = new QHBoxLayout;
    nodeCombo_ = new QComboBox;
    nodeCombo_->setMinimumWidth(160);
    h->addWidget(new QLabel(QStringLiteral("节点:")));
    h->addWidget(nodeCombo_);
    h->addStretch();
    group->setLayout(h);
    layout->addWidget(group);
}

void MotorControlPanel::buildModeAndEnable(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("使能控制"));
    auto *h = new QHBoxLayout;

    modeCombo_ = new QComboBox;
    modeCombo_->addItem(QStringLiteral("速度模式"), static_cast<int>(OperationMode::Velocity));
    modeCombo_->addItem(QStringLiteral("位置模式"), static_cast<int>(OperationMode::ProfilePosition));
    h->addWidget(new QLabel(QStringLiteral("模式:")));
    h->addWidget(modeCombo_);

    enableBtn_ = new QPushButton(QStringLiteral("使能"));
    enableBtn_->setStyleSheet(
        "QPushButton { background: #a6e3a1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 6px 14px; }"
        "QPushButton:hover { background: #94e2d5; }");
    h->addWidget(enableBtn_);

    disableBtn_ = new QPushButton(QStringLiteral("失能"));
    disableBtn_->setStyleSheet(
        "QPushButton { background: #fab387; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 6px 14px; }"
        "QPushButton:hover { background: #f9e2af; }");
    h->addWidget(disableBtn_);

    estopBtn_ = new QPushButton(QStringLiteral("急停"));
    estopBtn_->setStyleSheet(
        "QPushButton { background: #f38ba8; color: #1e1e2e; font-weight: bold; font-size: 13px; border-radius: 4px; padding: 6px 18px; }"
        "QPushButton:hover { background: #eb6f92; }");
    h->addWidget(estopBtn_);

    faultResetBtn_ = new QPushButton(QStringLiteral("故障复位"));
    faultResetBtn_->setStyleSheet(
        "QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 6px 14px; }"
        "QPushButton:hover { background: #74c7ec; }");
    h->addWidget(faultResetBtn_);

    group->setLayout(h);
    layout->addWidget(group);

    connect(enableBtn_, &QPushButton::clicked, this, [this]() {
        auto mode = static_cast<OperationMode>(modeCombo_->currentData().toInt());
        emit enableRequested(currentNodeId(), mode);
    });
    connect(disableBtn_, &QPushButton::clicked, this, [this]() {
        emit disableRequested(currentNodeId());
    });
    connect(estopBtn_, &QPushButton::clicked, this, [this]() {
        emit emergencyStopRequested(currentNodeId());
    });
    connect(faultResetBtn_, &QPushButton::clicked, this, [this]() {
        emit faultResetRequested(currentNodeId());
    });
}

void MotorControlPanel::buildVelocityControl(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("速度控制"));
    auto *h = new QHBoxLayout;
    velocitySpin_ = new QSpinBox;
    velocitySpin_->setRange(-3000, 3000);
    velocitySpin_->setSuffix(" RPM");
    velocitySpin_->setValue(100);
    h->addWidget(new QLabel(QStringLiteral("速度:")));
    h->addWidget(velocitySpin_);

    velGoBtn_ = new QPushButton(QStringLiteral("执行"));
    velGoBtn_->setStyleSheet(
        "QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(velGoBtn_);

    velStopBtn_ = new QPushButton(QStringLiteral("停止"));
    velStopBtn_->setStyleSheet(
        "QPushButton { background: #f9e2af; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(velStopBtn_);

    group->setLayout(h);
    layout->addWidget(group);

    connect(velGoBtn_, &QPushButton::clicked, this, [this]() {
        emit velocityRequested(currentNodeId(), velocitySpin_->value());
    });
    connect(velStopBtn_, &QPushButton::clicked, this, [this]() {
        emit velocityRequested(currentNodeId(), 0);
    });
}

void MotorControlPanel::buildPositionControl(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("位置控制"));
    auto *h = new QHBoxLayout;

    positionSpin_ = new QDoubleSpinBox;
    positionSpin_->setRange(-36000, 36000);
    positionSpin_->setDecimals(1);
    positionSpin_->setSuffix(QStringLiteral("°"));
    positionSpin_->setValue(90.0);
    h->addWidget(new QLabel(QStringLiteral("角度:")));
    h->addWidget(positionSpin_);

    profileVelSpin_ = new QSpinBox;
    profileVelSpin_->setRange(1, 3000);
    profileVelSpin_->setSuffix(" RPM");
    profileVelSpin_->setValue(30);
    h->addWidget(new QLabel(QStringLiteral("限速:")));
    h->addWidget(profileVelSpin_);

    posTypeCombo_ = new QComboBox;
    posTypeCombo_->addItem(QStringLiteral("相对"), false);
    posTypeCombo_->addItem(QStringLiteral("绝对"), true);
    h->addWidget(posTypeCombo_);

    posGoBtn_ = new QPushButton(QStringLiteral("执行"));
    posGoBtn_->setStyleSheet(
        "QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(posGoBtn_);

    group->setLayout(h);
    layout->addWidget(group);

    connect(posGoBtn_, &QPushButton::clicked, this, [this]() {
        uint32_t nid = currentNodeId();
        emit profileVelocityRequested(nid, static_cast<uint32_t>(profileVelSpin_->value()));
        bool absolute = posTypeCombo_->currentData().toBool();
        emit positionRequested(nid, positionSpin_->value(), absolute);
    });
}

void MotorControlPanel::buildBrakeControl(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("抱闸控制"));
    auto *h = new QHBoxLayout;

    releaseBrakeBtn_ = new QPushButton(QStringLiteral("松闸"));
    releaseBrakeBtn_->setStyleSheet(
        "QPushButton { background: #f9e2af; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(releaseBrakeBtn_);

    lockBrakeBtn_ = new QPushButton(QStringLiteral("锁闸"));
    lockBrakeBtn_->setStyleSheet(
        "QPushButton { background: #a6e3a1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(lockBrakeBtn_);
    h->addStretch();

    group->setLayout(h);
    layout->addWidget(group);

    connect(releaseBrakeBtn_, &QPushButton::clicked, this, [this]() {
        emit releaseBrakeRequested(currentNodeId());
    });
    connect(lockBrakeBtn_, &QPushButton::clicked, this, [this]() {
        emit lockBrakeRequested(currentNodeId());
    });
}

void MotorControlPanel::buildBatchButtons(QVBoxLayout *layout)
{
    auto *group = new QGroupBox(QStringLiteral("批量操作"));
    auto *h = new QHBoxLayout;

    auto *btnEnableAll = new QPushButton(QStringLiteral("全部使能"));
    btnEnableAll->setStyleSheet(
        "QPushButton { background: #a6e3a1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(btnEnableAll);

    auto *btnDisableAll = new QPushButton(QStringLiteral("全部失能"));
    btnDisableAll->setStyleSheet(
        "QPushButton { background: #fab387; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(btnDisableAll);

    auto *btnEstopAll = new QPushButton(QStringLiteral("全部急停"));
    btnEstopAll->setStyleSheet(
        "QPushButton { background: #f38ba8; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(btnEstopAll);

    auto *btnResetAll = new QPushButton(QStringLiteral("全部复位"));
    btnResetAll->setStyleSheet(
        "QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    h->addWidget(btnResetAll);

    group->setLayout(h);
    layout->addWidget(group);

    connect(btnEnableAll, &QPushButton::clicked, this, [this]() {
        auto mode = static_cast<OperationMode>(modeCombo_->currentData().toInt());
        emit enableAllRequested(mode);
    });
    connect(btnDisableAll, &QPushButton::clicked, this, &MotorControlPanel::disableAllRequested);
    connect(btnEstopAll, &QPushButton::clicked, this, &MotorControlPanel::emergencyStopAllRequested);
    connect(btnResetAll, &QPushButton::clicked, this, &MotorControlPanel::faultResetAllRequested);
}

void MotorControlPanel::setNodeList(const QList<uint32_t> &nodeIds, const QMap<uint32_t, QString> &names)
{
    nodeNames_ = names;
    nodeCombo_->clear();
    for (uint32_t id : nodeIds) {
        QString label = QStringLiteral("0x%1").arg(id, 3, 16, QChar('0'));
        if (names.contains(id))
            label += " (" + names[id] + ")";
        nodeCombo_->addItem(label, id);
    }
}

void MotorControlPanel::updateMotorState(uint32_t nodeId, const MotorState &state)
{
    if (currentNodeId() != nodeId) return;

    QString stateStr = QStringLiteral("节点: 0x%1 | %2 | %3 | 模式: %4 | 扭矩: %5‰ | 碰撞保护: %6 %7")
        .arg(nodeId, 3, 16, QChar('0'))
        .arg(state.online ? QStringLiteral("在线") : QStringLiteral("离线"))
        .arg(state.enabled ? QStringLiteral("已使能") : QStringLiteral("未使能"))
        .arg(operationModeText(state.mode))
        .arg(state.torquePermille)
        .arg(state.collisionProtectionOn ? QStringLiteral("开启") : QStringLiteral("关闭"))
        .arg(state.collisionTriggered ? QStringLiteral("⚠ 已触发!") : QString());
    stateLabel_->setText(stateStr);

    if (state.collisionTriggered) {
        stateLabel_->setStyleSheet(
            "QLabel { background: #f7e4e0; color: #bf6655; font-weight: bold; "
            "border: 1px solid #e2bdb5; border-radius: 8px; padding: 6px 10px; }");
    } else if (state.enabled) {
        stateLabel_->setStyleSheet(
            "QLabel { background: #edf4ec; color: #688a74; font-weight: 600; "
            "border: 1px solid #c8d8c8; border-radius: 8px; padding: 6px 10px; }");
    } else {
        stateLabel_->setStyleSheet(
            "QLabel { background: #fffaf4; color: #314152; border: 1px solid #ddd2c4; "
            "border-radius: 8px; padding: 6px 10px; }");
    }
}

} // namespace dac
