#include "ui/connection_panel.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QSerialPortInfo>

namespace dac {

ConnectionPanel::ConnectionPanel(QWidget *parent) : QWidget(parent)
{
    auto *group = new QGroupBox(QStringLiteral("总线连接"));
    auto *grid = new QHBoxLayout;

    portCombo_ = new QComboBox;
    portCombo_->setMinimumWidth(140);
    grid->addWidget(new QLabel(QStringLiteral("串口:")));
    grid->addWidget(portCombo_);

    baudCombo_ = new QComboBox;
    baudCombo_->addItems({"9600", "115200", "230400", "460800", "921600", "2000000"});
    baudCombo_->setCurrentText("9600");
    grid->addWidget(new QLabel(QStringLiteral("波特率:")));
    grid->addWidget(baudCombo_);

    refreshBtn_ = new QPushButton(QStringLiteral("刷新"));
    refreshBtn_->setFixedWidth(60);
    grid->addWidget(refreshBtn_);

    connectBtn_ = new QPushButton(QStringLiteral("连接"));
    connectBtn_->setStyleSheet(
        "QPushButton { background: #a6e3a1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 16px; }"
        "QPushButton:hover { background: #94e2d5; }");
    grid->addWidget(connectBtn_);

    disconnectBtn_ = new QPushButton(QStringLiteral("断开"));
    disconnectBtn_->setEnabled(false);
    disconnectBtn_->setStyleSheet(
        "QPushButton { background: #f38ba8; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 16px; }"
        "QPushButton:hover { background: #eba0ac; }"
        "QPushButton:disabled { background: #585b70; color: #7f849c; }");
    grid->addWidget(disconnectBtn_);

    statusLabel_ = new QLabel(QStringLiteral("未连接"));
    statusLabel_->setStyleSheet("QLabel { color: #f9e2af; font-weight: bold; }");
    grid->addWidget(statusLabel_);
    grid->addStretch();

    group->setLayout(grid);
    auto *main = new QVBoxLayout(this);
    main->setContentsMargins(0, 0, 0, 0);
    main->addWidget(group);

    connect(refreshBtn_, &QPushButton::clicked, this, &ConnectionPanel::refreshPorts);
    connect(connectBtn_, &QPushButton::clicked, this, [this]() {
        BusConfig cfg;
        cfg.devicePath = portCombo_->currentText();
        cfg.baudRate = baudCombo_->currentText().toInt();
        emit connectRequested(cfg);
    });
    connect(disconnectBtn_, &QPushButton::clicked, this, &ConnectionPanel::disconnectRequested);

    refreshPorts();
}

void ConnectionPanel::refreshPorts()
{
    portCombo_->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const auto &info : ports) {
        portCombo_->addItem(info.portName());
    }
    if (portCombo_->count() == 0)
        portCombo_->addItem(QStringLiteral("/dev/ttyUSB0"));
}

void ConnectionPanel::setBusState(bool open, const QString &msg)
{
    connected_ = open;
    connectBtn_->setEnabled(!open);
    disconnectBtn_->setEnabled(open);
    portCombo_->setEnabled(!open);
    baudCombo_->setEnabled(!open);
    statusLabel_->setText(open ? QStringLiteral("已连接") : QStringLiteral("未连接"));
    statusLabel_->setStyleSheet(open
        ? "QLabel { color: #a6e3a1; font-weight: bold; }"
        : "QLabel { color: #f9e2af; font-weight: bold; }");
    Q_UNUSED(msg);
}

} // namespace dac
