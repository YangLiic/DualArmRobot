/**
 * ConnectionPanel 实现
 */
#include "connection_panel.h"
#include "../communication/communication_manager.h"
#include "../services/motor_service.h"
#include "../models/common_types.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QDir>
#include <QMessageBox>

namespace dar {

ConnectionPanel::ConnectionPanel(CommunicationManager *commMgr,
                                 MotorService *motorService,
                                 QWidget *parent)
    : QWidget(parent)
    , commMgr_(commMgr)
    , motorService_(motorService)
{
    setupUi();
}

void ConnectionPanel::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(16);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // ========== 总线连接 ==========
    auto *connGroup = new QGroupBox(QStringLiteral("🔌 总线连接"), this);
    connGroup->setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; }");
    auto *connLayout = new QVBoxLayout(connGroup);

    auto *formLayout = new QFormLayout();

    // 串口选择
    auto *portRow = new QHBoxLayout();
    portCombo_ = new QComboBox();
    portCombo_->setMinimumWidth(200);
    refreshPortsBtn_ = new QPushButton(QStringLiteral("🔄 刷新"));
    refreshPortsBtn_->setFixedWidth(80);
    connect(refreshPortsBtn_, &QPushButton::clicked, this, &ConnectionPanel::onRefreshPorts);
    portRow->addWidget(portCombo_);
    portRow->addWidget(refreshPortsBtn_);
    formLayout->addRow(QStringLiteral("串口:"), portRow);

    // 波特率
    baudRateCombo_ = new QComboBox();
    baudRateCombo_->addItems({"9600", "115200", "230400", "460800", "921600", "2000000"});
    baudRateCombo_->setCurrentText("9600");
    formLayout->addRow(QStringLiteral("波特率:"), baudRateCombo_);

    // 总线 ID
    busIdEdit_ = new QLineEdit("can0");
    formLayout->addRow(QStringLiteral("总线 ID:"), busIdEdit_);

    connLayout->addLayout(formLayout);

    // 连接按钮
    auto *btnRow = new QHBoxLayout();
    connectBtn_ = new QPushButton(QStringLiteral("▶  连接"));
    connectBtn_->setObjectName("connectBtn");
    connectBtn_->setMinimumHeight(40);
    disconnectBtn_ = new QPushButton(QStringLiteral("⏹  断开"));
    disconnectBtn_->setObjectName("disconnectBtn");
    disconnectBtn_->setMinimumHeight(40);
    disconnectBtn_->setEnabled(false);

    connect(connectBtn_, &QPushButton::clicked, this, &ConnectionPanel::onConnectClicked);
    connect(disconnectBtn_, &QPushButton::clicked, this, &ConnectionPanel::onDisconnectClicked);

    btnRow->addWidget(connectBtn_);
    btnRow->addWidget(disconnectBtn_);
    connLayout->addLayout(btnRow);

    connectionStatusLabel_ = new QLabel(QStringLiteral("● 未连接"));
    connectionStatusLabel_->setStyleSheet("color: #888; font-size: 13px;");
    connLayout->addWidget(connectionStatusLabel_);

    mainLayout->addWidget(connGroup);

    // ========== 节点管理 ==========
    auto *nodeGroup = new QGroupBox(QStringLiteral("⚙️  电机节点管理"), this);
    nodeGroup->setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; }");
    auto *nodeLayout = new QVBoxLayout(nodeGroup);

    auto *addRow = new QHBoxLayout();
    auto *nodeLabel = new QLabel(QStringLiteral("节点 ID (hex):"));
    nodeIdSpin_ = new QSpinBox();
    nodeIdSpin_->setPrefix("0x");
    nodeIdSpin_->setDisplayIntegerBase(16);
    nodeIdSpin_->setRange(0x601, 0x67F);
    nodeIdSpin_->setValue(0x601);
    nodeIdSpin_->setMinimumWidth(100);

    addMotorBtn_ = new QPushButton(QStringLiteral("➕ 添加"));
    removeMotorBtn_ = new QPushButton(QStringLiteral("➖ 移除"));
    scanBtn_ = new QPushButton(QStringLiteral("🔍 扫描在线"));

    connect(addMotorBtn_, &QPushButton::clicked, this, &ConnectionPanel::onAddMotorClicked);
    connect(removeMotorBtn_, &QPushButton::clicked, this, &ConnectionPanel::onRemoveMotorClicked);
    connect(scanBtn_, &QPushButton::clicked, this, &ConnectionPanel::onScanClicked);

    addRow->addWidget(nodeLabel);
    addRow->addWidget(nodeIdSpin_);
    addRow->addWidget(addMotorBtn_);
    addRow->addWidget(removeMotorBtn_);
    addRow->addWidget(scanBtn_);
    addRow->addStretch();
    nodeLayout->addLayout(addRow);

    // 节点列表
    motorTable_ = new QTableWidget(0, 4);
    motorTable_->setHorizontalHeaderLabels({
        QStringLiteral("节点 ID"),
        QStringLiteral("总线"),
        QStringLiteral("状态"),
        QStringLiteral("在线")
    });
    motorTable_->horizontalHeader()->setStretchLastSection(true);
    motorTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    motorTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    motorTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    motorTable_->setAlternatingRowColors(true);
    nodeLayout->addWidget(motorTable_);

    mainLayout->addWidget(nodeGroup);
    mainLayout->addStretch();

    // 初始刷新串口
    onRefreshPorts();
}

void ConnectionPanel::onRefreshPorts()
{
    portCombo_->clear();

    QDir devDir("/dev");
    QStringList filters;
    filters << "ttyUSB*" << "ttyACM*";
    QStringList ports = devDir.entryList(filters, QDir::System, QDir::Name);

    for (const QString &port : ports) {
        portCombo_->addItem("/dev/" + port);
    }

    if (portCombo_->count() == 0) {
        portCombo_->addItem(QStringLiteral("(未检测到串口)"));
    }
}

void ConnectionPanel::onConnectClicked()
{
    QString port = portCombo_->currentText();
    if (port.startsWith("(")) {
        QMessageBox::warning(this, QStringLiteral("错误"), QStringLiteral("请选择有效串口"));
        return;
    }

    BusConfig config;
    config.busId = busIdEdit_->text();
    config.devicePath = port;
    config.baudRate = baudRateCombo_->currentText().toInt();
    config.busType = BusType::UsbCan;
    config.protocol = ProtocolType::CanopenMotor;

    if (!commMgr_->addBus(config)) {
        // 总线可能已存在，尝试直接连接
    }
    commMgr_->connectBus(config.busId);

    connectBtn_->setEnabled(false);
    disconnectBtn_->setEnabled(true);
    connectionStatusLabel_->setText(QStringLiteral("● 正在连接..."));
    connectionStatusLabel_->setStyleSheet("color: #f59e0b; font-size: 13px;");
}

void ConnectionPanel::onDisconnectClicked()
{
    QString busId = busIdEdit_->text();
    commMgr_->disconnectBus(busId);

    connectBtn_->setEnabled(true);
    disconnectBtn_->setEnabled(false);
    connectionStatusLabel_->setText(QStringLiteral("● 已断开"));
    connectionStatusLabel_->setStyleSheet("color: #888; font-size: 13px;");
}

void ConnectionPanel::onAddMotorClicked()
{
    uint32_t nodeId = static_cast<uint32_t>(nodeIdSpin_->value());
    QString busId = busIdEdit_->text();

    motorService_->addMotor(busId, nodeId);
    updateMotorTable();
}

void ConnectionPanel::onRemoveMotorClicked()
{
    int row = motorTable_->currentRow();
    if (row < 0) return;

    bool ok;
    uint32_t nodeId = motorTable_->item(row, 0)->text().toUInt(&ok, 16);
    if (ok) {
        motorService_->removeMotor(nodeId);
        updateMotorTable();
    }
}

void ConnectionPanel::onScanClicked()
{
    motorService_->scanOnlineNodes();
}

void ConnectionPanel::updateMotorTable()
{
    auto states = motorService_->allMotorStates();
    motorTable_->setRowCount(states.size());

    int row = 0;
    for (auto it = states.begin(); it != states.end(); ++it, ++row) {
        const auto &state = it.value();

        motorTable_->setItem(row, 0, new QTableWidgetItem(
            QStringLiteral("0x%1").arg(state.nodeId, 3, 16, QChar('0'))));

        // 查找busId
        auto motorIt = motorService_->allMotorStates().find(state.nodeId);
        motorTable_->setItem(row, 1, new QTableWidgetItem(
            busIdEdit_->text()));

        motorTable_->setItem(row, 2, new QTableWidgetItem(state.stateText()));

        auto *onlineItem = new QTableWidgetItem(state.online ? "✅" : "❌");
        onlineItem->setTextAlignment(Qt::AlignCenter);
        motorTable_->setItem(row, 3, onlineItem);
    }
}

}  // namespace dar
