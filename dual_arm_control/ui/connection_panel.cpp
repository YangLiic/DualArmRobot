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

    // 适配器类型选择
    adapterCombo_ = new QComboBox;
    adapterCombo_->addItem(QStringLiteral("串口 (CH340)"), static_cast<int>(AdapterType::Serial));
    adapterCombo_->addItem(QStringLiteral("VCI (CANalyst-II)"), static_cast<int>(AdapterType::VCI));
    grid->addWidget(new QLabel(QStringLiteral("适配器:")));
    grid->addWidget(adapterCombo_);

    // ===== Serial 参数面板 =====
    serialParamsWidget_ = new QWidget;
    auto *serialLayout = new QHBoxLayout(serialParamsWidget_);
    serialLayout->setContentsMargins(0, 0, 0, 0);

    portCombo_ = new QComboBox;
    portCombo_->setMinimumWidth(140);
    serialLayout->addWidget(new QLabel(QStringLiteral("串口:")));
    serialLayout->addWidget(portCombo_);

    baudCombo_ = new QComboBox;
    baudCombo_->addItems({"9600", "115200", "230400", "460800", "921600", "2000000"});
    baudCombo_->setCurrentText("9600");
    serialLayout->addWidget(new QLabel(QStringLiteral("波特率:")));
    serialLayout->addWidget(baudCombo_);

    // ===== VCI 参数面板 =====
    vciParamsWidget_ = new QWidget;
    auto *vciLayout = new QHBoxLayout(vciParamsWidget_);
    vciLayout->setContentsMargins(0, 0, 0, 0);

    vciDeviceCombo_ = new QComboBox;
    vciDeviceCombo_->addItems({"0", "1", "2", "3"});
    vciDeviceCombo_->setCurrentText("0");
    vciLayout->addWidget(new QLabel(QStringLiteral("设备号:")));
    vciLayout->addWidget(vciDeviceCombo_);

    vciChannelCombo_ = new QComboBox;
    vciChannelCombo_->addItems({"0", "1"});
    vciChannelCombo_->setCurrentText("0");
    vciLayout->addWidget(new QLabel(QStringLiteral("通道:")));
    vciLayout->addWidget(vciChannelCombo_);

    vciBitrateCombo_ = new QComboBox;
    vciBitrateCombo_->addItem(QStringLiteral("1000 kbps"), 1000);
    vciBitrateCombo_->addItem(QStringLiteral("500 kbps"), 500);
    vciBitrateCombo_->addItem(QStringLiteral("250 kbps"), 250);
    vciBitrateCombo_->addItem(QStringLiteral("125 kbps"), 125);
    vciBitrateCombo_->setCurrentIndex(0);
    vciLayout->addWidget(new QLabel(QStringLiteral("CAN波特率:")));
    vciLayout->addWidget(vciBitrateCombo_);

    // ===== Stacked Widget =====
    paramsStack_ = new QStackedWidget;
    paramsStack_->addWidget(serialParamsWidget_);
    paramsStack_->addWidget(vciParamsWidget_);
    grid->addWidget(paramsStack_);

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
    statusLabel_->setStyleSheet(
        "QLabel { background: #f6ead8; color: #9f7645; font-weight: bold; "
        "border: 1px solid #e3cfb1; border-radius: 9px; padding: 4px 10px; }");
    grid->addWidget(statusLabel_);
    grid->addStretch();

    group->setLayout(grid);
    auto *main = new QVBoxLayout(this);
    main->setContentsMargins(0, 0, 0, 0);
    main->addWidget(group);

    // 信号连接
    connect(adapterCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ConnectionPanel::onAdapterTypeChanged);
    connect(refreshBtn_, &QPushButton::clicked, this, &ConnectionPanel::refreshPorts);
    connect(connectBtn_, &QPushButton::clicked, this, [this]() {
        BusConfig cfg;
        auto type = static_cast<AdapterType>(adapterCombo_->currentData().toInt());
        cfg.adapterType = type;

        if (type == AdapterType::Serial) {
            cfg.devicePath = portCombo_->currentText();
            cfg.baudRate = baudCombo_->currentText().toInt();
        } else {
            cfg.devicePath = vciDeviceCombo_->currentText();
            cfg.canChannel = vciChannelCombo_->currentText().toInt();
            cfg.canBitrate = vciBitrateCombo_->currentData().toInt();
        }
        emit connectRequested(cfg);
    });
    connect(disconnectBtn_, &QPushButton::clicked, this, &ConnectionPanel::disconnectRequested);

    refreshPorts();
}

void ConnectionPanel::onAdapterTypeChanged(int index)
{
    paramsStack_->setCurrentIndex(index);
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
    adapterCombo_->setEnabled(!open);
    portCombo_->setEnabled(!open);
    baudCombo_->setEnabled(!open);
    vciDeviceCombo_->setEnabled(!open);
    vciChannelCombo_->setEnabled(!open);
    vciBitrateCombo_->setEnabled(!open);
    statusLabel_->setText(open ? QStringLiteral("已连接") : QStringLiteral("未连接"));
    statusLabel_->setStyleSheet(open
        ? "QLabel { background: #edf4ec; color: #688a74; font-weight: bold; border: 1px solid #c8d8c8; border-radius: 9px; padding: 4px 10px; }"
        : "QLabel { background: #f6ead8; color: #9f7645; font-weight: bold; border: 1px solid #e3cfb1; border-radius: 9px; padding: 4px 10px; }");
    Q_UNUSED(msg);
}

} // namespace dac
