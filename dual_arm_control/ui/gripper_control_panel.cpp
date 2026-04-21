#include "ui/gripper_control_panel.h"
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSerialPortInfo>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStringList>
#include <QVBoxLayout>
#include <algorithm>

namespace dac {

GripperControlPanel::GripperControlPanel(QWidget *parent)
    : QWidget(parent)
{
    leftUi_.side = ArmSide::Left;
    rightUi_.side = ArmSide::Right;
    setupUi();
    refreshSerialPorts();
}

void GripperControlPanel::setGripperState(const dac::GripperState &state)
{
    EndpointUi &ui = endpointUi(state.side);

    ui.actualLabel->setText(QStringLiteral("%1").arg(state.actualNormalized, 0, 'f', 3));
    ui.targetLabel->setText(QStringLiteral("%1").arg(state.targetNormalized, 0, 'f', 3));
    ui.detailLabel->setText(state.detailText.isEmpty() ? QStringLiteral("--") : state.detailText);
    ui.updatedLabel->setText(state.lastUpdateTime.isValid()
        ? state.lastUpdateTime.toString(QStringLiteral("HH:mm:ss"))
        : QStringLiteral("--"));
}

void GripperControlPanel::setConnectionState(dac::ArmSide side, bool connected, const QString &message)
{
    EndpointUi &ui = endpointUi(side);

    ui.connectBtn->setEnabled(!connected);
    ui.disconnectBtn->setEnabled(connected);
    ui.sendBtn->setEnabled(connected);

    const QList<QWidget *> configWidgets = {
        ui.transportCombo, ui.transportStack, ui.deviceIdSpin, ui.speedSpin, ui.forceSpin
    };
    for (QWidget *widget : configWidgets) {
        widget->setEnabled(!connected);
    }

    ui.statusLabel->setText(message);
    ui.statusLabel->setStyleSheet(connected
        ? "QLabel { background: #edf4ec; color: #688a74; font-weight: bold; border: 1px solid #c8d8c8; border-radius: 9px; padding: 4px 10px; }"
        : "QLabel { background: #f8ebe6; color: #b56755; font-weight: bold; border: 1px solid #e7c8be; border-radius: 9px; padding: 4px 10px; }");
}

void GripperControlPanel::refreshSerialPorts()
{
    const auto ports = QSerialPortInfo::availablePorts();
    const auto refreshOne = [&](EndpointUi &ui) {
        const QString currentText = ui.serialPortCombo->currentText();
        ui.serialPortCombo->clear();
        for (const auto &info : ports) {
            ui.serialPortCombo->addItem(info.portName());
        }
        if (ui.serialPortCombo->count() == 0) {
            ui.serialPortCombo->addItem(QStringLiteral("/dev/ttyUSB0"));
        }
        const int index = ui.serialPortCombo->findText(currentText);
        if (index >= 0) {
            ui.serialPortCombo->setCurrentIndex(index);
        } else if (!currentText.isEmpty()) {
            ui.serialPortCombo->setEditText(currentText);
        }
    };

    refreshOne(leftUi_);
    refreshOne(rightUi_);
}

void GripperControlPanel::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(12);

    auto *topRow = new QHBoxLayout;
    auto *note = new QLabel(QStringLiteral("每个夹爪独立连接，控制量统一为 0~1。485 串口按 demo_485.py 的自定义协议映射，TCP 按你提供的 6 指姿态插值映射。"));
    note->setWordWrap(true);
    auto *refreshBtn = new QPushButton(QStringLiteral("刷新串口"));
    refreshBtn->setStyleSheet("QPushButton { background: #f9e2af; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    connect(refreshBtn, &QPushButton::clicked, this, &GripperControlPanel::refreshSerialPorts);
    topRow->addWidget(note, 1);
    topRow->addWidget(refreshBtn);
    mainLayout->addLayout(topRow);

    auto *cardsLayout = new QHBoxLayout;

    GripperEndpointConfig leftDefaults;
    leftDefaults.transport = GripperTransport::TCP;
    leftDefaults.ip = QStringLiteral("192.168.123.211");
    leftDefaults.tcpPort = 6000;
    leftDefaults.serialPort = QStringLiteral("/dev/ttyUSB0");
    leftDefaults.serialBaudRate = 115200;
    leftDefaults.slaveId = 1;
    leftDefaults.speed = 500;
    leftDefaults.force = 500;

    GripperEndpointConfig rightDefaults = leftDefaults;
    rightDefaults.ip = QStringLiteral("192.168.123.212");

    cardsLayout->addWidget(buildEndpointCard(QStringLiteral("左夹爪"), leftUi_, leftDefaults), 1);
    cardsLayout->addWidget(buildEndpointCard(QStringLiteral("右夹爪"), rightUi_, rightDefaults), 1);
    mainLayout->addLayout(cardsLayout);
}

QWidget *GripperControlPanel::buildEndpointCard(const QString &title, EndpointUi &ui, const GripperEndpointConfig &defaults)
{
    EndpointUi *endpoint = &ui;
    auto *group = new QGroupBox(title);
    auto *layout = new QVBoxLayout(group);

    auto *topRow = new QHBoxLayout;
    ui.statusLabel = new QLabel(QStringLiteral("未连接"));
    ui.statusLabel->setStyleSheet(
        "QLabel { background: #f6ead8; color: #9f7645; font-weight: bold; border: 1px solid #e3cfb1; border-radius: 9px; padding: 4px 10px; }");
    topRow->addWidget(ui.statusLabel);
    topRow->addStretch();

    ui.connectBtn = new QPushButton(QStringLiteral("连接"));
    ui.connectBtn->setStyleSheet("QPushButton { background: #a6e3a1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    connect(ui.connectBtn, &QPushButton::clicked, this, [this, endpoint]() {
        emit connectRequested(endpoint->side, configFromUi(*endpoint));
    });
    topRow->addWidget(ui.connectBtn);

    ui.disconnectBtn = new QPushButton(QStringLiteral("断开"));
    ui.disconnectBtn->setEnabled(false);
    ui.disconnectBtn->setStyleSheet("QPushButton { background: #f38ba8; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    connect(ui.disconnectBtn, &QPushButton::clicked, this, [this, endpoint]() {
        emit disconnectRequested(endpoint->side);
    });
    topRow->addWidget(ui.disconnectBtn);
    layout->addLayout(topRow);

    auto *configGroup = new QGroupBox(QStringLiteral("连接参数"));
    auto *configLayout = new QGridLayout(configGroup);
    int row = 0;

    configLayout->addWidget(new QLabel(QStringLiteral("传输方式:")), row, 0);
    ui.transportCombo = new QComboBox;
    ui.transportCombo->addItem(QStringLiteral("Modbus TCP"), static_cast<int>(GripperTransport::TCP));
    ui.transportCombo->addItem(QStringLiteral("485 串口"), static_cast<int>(GripperTransport::RTU));
    configLayout->addWidget(ui.transportCombo, row, 1, 1, 3);
    row++;

    ui.transportStack = new QStackedWidget;

    auto *tcpPage = new QWidget;
    auto *tcpLayout = new QGridLayout(tcpPage);
    tcpLayout->addWidget(new QLabel(QStringLiteral("IP:")), 0, 0);
    ui.ipEdit = new QLineEdit(defaults.ip);
    tcpLayout->addWidget(ui.ipEdit, 0, 1);
    tcpLayout->addWidget(new QLabel(QStringLiteral("端口:")), 0, 2);
    ui.tcpPortSpin = new QSpinBox;
    ui.tcpPortSpin->setRange(1, 65535);
    ui.tcpPortSpin->setValue(defaults.tcpPort);
    tcpLayout->addWidget(ui.tcpPortSpin, 0, 3);
    ui.transportStack->addWidget(tcpPage);

    auto *rtuPage = new QWidget;
    auto *rtuLayout = new QGridLayout(rtuPage);
    rtuLayout->addWidget(new QLabel(QStringLiteral("串口:")), 0, 0);
    ui.serialPortCombo = new QComboBox;
    ui.serialPortCombo->setEditable(true);
    ui.serialPortCombo->setMinimumWidth(150);
    ui.serialPortCombo->setEditText(defaults.serialPort);
    rtuLayout->addWidget(ui.serialPortCombo, 0, 1);
    rtuLayout->addWidget(new QLabel(QStringLiteral("波特率:")), 0, 2);
    ui.baudSpin = new QSpinBox;
    ui.baudSpin->setRange(1200, 3000000);
    ui.baudSpin->setValue(defaults.serialBaudRate);
    rtuLayout->addWidget(ui.baudSpin, 0, 3);
    ui.transportStack->addWidget(rtuPage);

    configLayout->addWidget(ui.transportStack, row, 0, 1, 4);
    row++;

    configLayout->addWidget(new QLabel(QStringLiteral("设备ID:")), row, 0);
    ui.deviceIdSpin = new QSpinBox;
    ui.deviceIdSpin->setRange(1, 247);
    ui.deviceIdSpin->setValue(defaults.slaveId);
    configLayout->addWidget(ui.deviceIdSpin, row, 1);

    configLayout->addWidget(new QLabel(QStringLiteral("速度:")), row, 2);
    ui.speedSpin = new QSpinBox;
    ui.speedSpin->setRange(10, 1000);
    ui.speedSpin->setValue(defaults.speed);
    configLayout->addWidget(ui.speedSpin, row, 3);
    row++;

    configLayout->addWidget(new QLabel(QStringLiteral("力度:")), row, 0);
    ui.forceSpin = new QSpinBox;
    ui.forceSpin->setRange(100, 1000);
    ui.forceSpin->setValue(defaults.force);
    configLayout->addWidget(ui.forceSpin, row, 1);
    layout->addWidget(configGroup);

    auto *controlGroup = new QGroupBox(QStringLiteral("夹爪控制"));
    auto *controlLayout = new QVBoxLayout(controlGroup);

    auto *valueRow = new QHBoxLayout;
    valueRow->addWidget(new QLabel(QStringLiteral("目标值:")));
    ui.commandSpin = new QDoubleSpinBox;
    ui.commandSpin->setRange(0.0, 1.0);
    ui.commandSpin->setSingleStep(0.01);
    ui.commandSpin->setDecimals(3);
    valueRow->addWidget(ui.commandSpin);
    ui.sendBtn = new QPushButton(QStringLiteral("执行"));
    ui.sendBtn->setEnabled(false);
    ui.sendBtn->setStyleSheet("QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 16px; }");
    connect(ui.sendBtn, &QPushButton::clicked, this, [this, endpoint]() {
        emit targetRequested(endpoint->side, endpoint->commandSpin->value());
    });
    valueRow->addWidget(ui.sendBtn);
    controlLayout->addLayout(valueRow);

    ui.commandSlider = new QSlider(Qt::Horizontal);
    ui.commandSlider->setRange(0, 1000);
    controlLayout->addWidget(ui.commandSlider);

    connect(ui.commandSlider, &QSlider::valueChanged, this, [endpoint](int value) {
        const QSignalBlocker blocker(endpoint->commandSpin);
        endpoint->commandSpin->setValue(value / 1000.0);
    });
    connect(ui.commandSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [endpoint](double value) {
        const QSignalBlocker blocker(endpoint->commandSlider);
        endpoint->commandSlider->setValue(qRound(value * 1000.0));
    });
    layout->addWidget(controlGroup);

    auto *stateGroup = new QGroupBox(QStringLiteral("状态反馈"));
    auto *stateLayout = new QGridLayout(stateGroup);
    int stateRow = 0;
    auto addStateRow = [&](const QString &label, QLabel *&valueLabel) {
        stateLayout->addWidget(new QLabel(label), stateRow, 0);
        valueLabel = new QLabel(QStringLiteral("--"));
        valueLabel->setWordWrap(true);
        stateLayout->addWidget(valueLabel, stateRow, 1);
        stateRow++;
    };

    addStateRow(QStringLiteral("实际值:"), ui.actualLabel);
    addStateRow(QStringLiteral("目标缓存:"), ui.targetLabel);
    addStateRow(QStringLiteral("详细信息:"), ui.detailLabel);
    addStateRow(QStringLiteral("最近更新:"), ui.updatedLabel);
    layout->addWidget(stateGroup);

    connect(ui.transportCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [endpoint, this](int) {
        applyTransportUi(*endpoint);
    });

    ui.transportCombo->setCurrentIndex(static_cast<int>(defaults.transport));
    applyTransportUi(ui);
    setCommandValue(ui, 0.0);

    return group;
}

GripperEndpointConfig GripperControlPanel::configFromUi(const EndpointUi &ui) const
{
    GripperEndpointConfig cfg;
    cfg.transport = static_cast<GripperTransport>(ui.transportCombo->currentData().toInt());
    cfg.ip = ui.ipEdit->text().trimmed();
    cfg.tcpPort = ui.tcpPortSpin->value();
    cfg.serialPort = ui.serialPortCombo->currentText().trimmed();
    cfg.serialBaudRate = ui.baudSpin->value();
    cfg.slaveId = ui.deviceIdSpin->value();
    cfg.speed = ui.speedSpin->value();
    cfg.force = ui.forceSpin->value();
    cfg.pollIntervalMs = 100;
    return cfg;
}

GripperControlPanel::EndpointUi &GripperControlPanel::endpointUi(dac::ArmSide side)
{
    return side == ArmSide::Left ? leftUi_ : rightUi_;
}

void GripperControlPanel::applyTransportUi(EndpointUi &ui)
{
    const auto transport = static_cast<GripperTransport>(ui.transportCombo->currentData().toInt());
    ui.transportStack->setCurrentIndex(transport == GripperTransport::TCP ? 0 : 1);
}

void GripperControlPanel::setCommandValue(EndpointUi &ui, double normalized)
{
    const double clamped = std::clamp(normalized, 0.0, 1.0);
    {
        const QSignalBlocker blocker(ui.commandSpin);
        ui.commandSpin->setValue(clamped);
    }
    {
        const QSignalBlocker blocker(ui.commandSlider);
        ui.commandSlider->setValue(qRound(clamped * 1000.0));
    }
}

} // namespace dac
