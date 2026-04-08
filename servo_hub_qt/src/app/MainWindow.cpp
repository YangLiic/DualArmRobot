#include "app/MainWindow.h"

#include <QAbstractItemView>
#include <QDateTime>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QItemSelectionModel>
#include <QRegularExpression>
#include <QSerialPortInfo>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(MotorService *motorService, QWidget *parent)
    : QMainWindow(parent)
    , m_motorService(motorService)
{
    buildUi();
    wireSignals();
    refreshPorts();
    applyBusAndNodes();
    setConnectionState(false, QStringLiteral("未连接"));
}

void MainWindow::refreshPorts()
{
    const QString currentText = m_portCombo->currentText();
    m_portCombo->clear();

    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : ports) {
        m_portCombo->addItem(info.systemLocation());
    }

    if (m_portCombo->count() == 0) {
        m_portCombo->addItems({
            QStringLiteral("/dev/ttyUSB0"),
            QStringLiteral("/dev/ttyUSB1"),
            QStringLiteral("/dev/ttyACM0")
        });
    }

    const int index = m_portCombo->findText(currentText);
    if (index >= 0) {
        m_portCombo->setCurrentIndex(index);
    }
}

void MainWindow::applyBusAndNodes()
{
    QList<MotorConfig> configs = parseNodeConfigs();
    if (configs.isEmpty()) {
        configs = {
            {0x601, QStringLiteral("升降电机"), false},
            {0x602, QStringLiteral("旋转电机"), true}
        };
    }

    BusConfig config;
    config.busId = QStringLiteral("main-can");
    config.devicePath = m_portCombo->currentText().trimmed();
    config.baudRate = m_baudCombo->currentText().toInt();
    config.timeoutMs = 150;

    m_motorService->setBusConfig(config);
    m_motorService->setMotorConfigs(configs);
}

void MainWindow::onBusConnectionChanged(bool connected, const QString &message)
{
    setConnectionState(connected, message);
    if (connected) {
        if (m_monitoringButton->text() == QLatin1String("启动监控")) {
            m_motorService->startMonitoring();
            m_monitoringButton->setText(QStringLiteral("停止监控"));
            appendLog(QStringLiteral("[UI] 已自动启动监控"));
        }
    } else if (m_monitoringButton->text() == QLatin1String("停止监控")) {
        m_monitoringButton->setText(QStringLiteral("启动监控"));
    }
}

void MainWindow::appendLog(const QString &message)
{
    const QString line = QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), message);
    m_logView->appendPlainText(line);
}

void MainWindow::reloadNodeSelector(const QList<quint32> &nodeIds)
{
    m_nodeSelector->clear();
    for (quint32 nodeId : nodeIds) {
        m_nodeSelector->addItem(formatCanId(nodeId), nodeId);
    }
    m_motorService->setFocusedNode(currentNodeId());
    refreshTorquePanelForCurrentNode();
}

void MainWindow::syncSelectedNodeFromTable()
{
    const QModelIndex currentIndex = m_motorTable->currentIndex();
    if (!currentIndex.isValid()) {
        return;
    }

    const quint32 nodeId = m_motorService->tableModel()->nodeIdAtRow(currentIndex.row());
    const int comboIndex = m_nodeSelector->findData(nodeId);
    if (comboIndex >= 0) {
        m_nodeSelector->setCurrentIndex(comboIndex);
    }
    m_motorService->setFocusedNode(nodeId);
    refreshTorquePanelForCurrentNode();
}

void MainWindow::updateTorquePanel(const MotorState &state)
{
    if (state.nodeId != currentNodeId()) {
        return;
    }

    m_torqueTitleLabel->setText(QStringLiteral("节点 %1 扭矩监控").arg(formatCanId(state.nodeId)));
    m_torqueBarWidget->setThresholdPermille(state.collisionThresholdPermille);
    m_torqueBarWidget->setTorquePermille(state.torquePermille);
    m_torqueValueLabel->setText(QStringLiteral("实时转矩：%1 ‰").arg(state.torquePermille));
    m_maxTorqueLabel->setText(QStringLiteral("最大转矩限制：%1 ‰").arg(state.maxTorquePermille));
    m_torqueThresholdLabel->setText(QStringLiteral("碰撞检测阈值：±%1 ‰").arg(state.collisionThresholdPermille));
}

void MainWindow::refreshTorquePanelForCurrentNode()
{
    const quint32 nodeId = currentNodeId();
    const MotorConfig config = configForNode(nodeId);
    const MotorState state = m_motorService->motorState(nodeId);
    m_torqueTitleLabel->setText(QStringLiteral("节点 %1 扭矩监控").arg(formatCanId(nodeId)));
    const qint16 collisionThreshold = state.collisionThresholdPermille > 0
        ? state.collisionThresholdPermille
        : (config.collisionThresholdPermille > 0 ? config.collisionThresholdPermille : 500);
    const qint16 maxTorque = state.maxTorquePermille > 0
        ? state.maxTorquePermille
        : (config.maxTorquePermille > 0 ? config.maxTorquePermille : 1200);
    m_torqueBarWidget->setThresholdPermille(collisionThreshold);
    m_torqueBarWidget->setTorquePermille(state.torquePermille);
    m_torqueValueLabel->setText(QStringLiteral("实时转矩：%1 ‰").arg(state.torquePermille));
    m_maxTorqueLabel->setText(QStringLiteral("最大转矩限制：%1 ‰").arg(maxTorque));
    m_torqueThresholdLabel->setText(QStringLiteral("碰撞检测阈值：±%1 ‰").arg(collisionThreshold));
}

QList<MotorConfig> MainWindow::parseNodeConfigs() const
{
    QList<MotorConfig> configs;
    const QStringList tokens = m_nodeLineEdit->text().split(QRegularExpression(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);

    int index = 0;
    for (const QString &token : tokens) {
        bool ok = false;
        quint32 nodeId = token.startsWith(QStringLiteral("0x"), Qt::CaseInsensitive)
            ? token.toUInt(&ok, 16)
            : token.toUInt(&ok, 10);
        if (!ok) {
            continue;
        }

        MotorConfig config;
        config.nodeId = nodeId;
        if (nodeId == 0x601) {
            config.name = QStringLiteral("升降电机");
            config.directionInverted = false;
            config.maxTorquePermille = 1200;
            config.collisionThresholdPermille = 500;
        } else if (nodeId == 0x602) {
            config.name = QStringLiteral("旋转电机");
            config.directionInverted = true;
            config.maxTorquePermille = 1200;
            config.collisionThresholdPermille = 200;
        } else {
            config.name = QStringLiteral("电机 %1").arg(++index);
            config.maxTorquePermille = 1200;
            config.collisionThresholdPermille = 500;
        }
        configs.append(config);
    }
    return configs;
}

quint32 MainWindow::currentNodeId() const
{
    return m_nodeSelector->currentData().toUInt();
}

void MainWindow::buildUi()
{
    setWindowTitle(QStringLiteral("双臂机器人伺服上位机"));
    resize(1500, 880);

    auto *central = new QWidget(this);
    setCentralWidget(central);

    central->setStyleSheet(QStringLiteral(
        "QWidget { background: #f4efe6; color: #1f2937; font-size: 13px; }"
        "QGroupBox { border: 1px solid #d6c8b5; border-radius: 12px; margin-top: 12px; padding-top: 14px; background: #fbf8f3; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #0f766e; font-weight: 600; }"
        "QPushButton, QToolButton, QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {"
        " border: 1px solid #cbbca7; border-radius: 8px; min-height: 34px; padding: 4px 10px; background: #fffdf8; }"
        "QPushButton:hover, QToolButton:hover { background: #e8f3f1; }"
        "QHeaderView::section { background: #ded6ca; padding: 8px; border: none; border-right: 1px solid #cbbca7; }"
        "QTableView { gridline-color: #e5ddd1; selection-background-color: #d5ebe8; selection-color: #1f2937; alternate-background-color: #f8f4ee; }"
        "QPlainTextEdit { border: 1px solid #d6c8b5; border-radius: 12px; background: #fffdf8; }"
        "QLabel#busStatusLabel { padding: 6px 12px; border-radius: 999px; background: #e8d7c2; font-weight: 600; }"));

    auto *rootLayout = new QVBoxLayout(central);
    rootLayout->setContentsMargins(16, 16, 16, 16);
    rootLayout->setSpacing(12);

    auto *topBar = new QGroupBox(QStringLiteral("总线配置"), central);
    auto *topLayout = new QHBoxLayout(topBar);

    m_portCombo = new QComboBox(topBar);
    m_baudCombo = new QComboBox(topBar);
    m_baudCombo->addItems({QStringLiteral("9600"), QStringLiteral("115200"), QStringLiteral("460800"), QStringLiteral("921600")});
    m_nodeLineEdit = new QLineEdit(QStringLiteral("0x601, 0x602"), topBar);
    m_busStatusLabel = new QLabel(topBar);
    m_busStatusLabel->setObjectName(QStringLiteral("busStatusLabel"));
    m_refreshPortsButton = new QToolButton(topBar);
    m_refreshPortsButton->setText(QStringLiteral("刷新"));
    m_connectButton = new QPushButton(QStringLiteral("连接"), topBar);
    m_disconnectButton = new QPushButton(QStringLiteral("断开"), topBar);
    m_scanButton = new QPushButton(QStringLiteral("扫描已配置节点"), topBar);
    m_monitoringButton = new QPushButton(QStringLiteral("启动监控"), topBar);

    topLayout->addWidget(new QLabel(QStringLiteral("串口"), topBar));
    topLayout->addWidget(m_portCombo, 2);
    topLayout->addWidget(m_refreshPortsButton);
    topLayout->addWidget(new QLabel(QStringLiteral("波特率"), topBar));
    topLayout->addWidget(m_baudCombo);
    topLayout->addWidget(new QLabel(QStringLiteral("节点"), topBar));
    topLayout->addWidget(m_nodeLineEdit, 2);
    topLayout->addWidget(m_connectButton);
    topLayout->addWidget(m_disconnectButton);
    topLayout->addWidget(m_scanButton);
    topLayout->addWidget(m_monitoringButton);
    topLayout->addWidget(m_busStatusLabel);

    auto *splitter = new QSplitter(Qt::Horizontal, central);

    auto *leftPanel = new QWidget(splitter);
    auto *leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(12);

    auto *statusGroup = new QGroupBox(QStringLiteral("电机总览"), leftPanel);
    auto *statusLayout = new QVBoxLayout(statusGroup);
    m_motorTable = new QTableView(statusGroup);
    m_motorTable->setModel(m_motorService->tableModel());
    m_motorTable->setAlternatingRowColors(true);
    m_motorTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_motorTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_motorTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_motorTable->verticalHeader()->setVisible(false);
    statusLayout->addWidget(m_motorTable);
    leftLayout->addWidget(statusGroup, 1);

    auto *rightPanel = new QWidget(splitter);
    auto *rightLayout = new QVBoxLayout(rightPanel);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(12);

    auto *torqueGroup = new QGroupBox(QStringLiteral("实时扭矩可视化"), rightPanel);
    auto *torqueLayout = new QVBoxLayout(torqueGroup);
    m_torqueTitleLabel = new QLabel(QStringLiteral("节点 0x601 扭矩监控"), torqueGroup);
    m_torqueBarWidget = new TorqueBarWidget(torqueGroup);
    m_torqueValueLabel = new QLabel(QStringLiteral("实时转矩：0 ‰"), torqueGroup);
    m_maxTorqueLabel = new QLabel(QStringLiteral("最大转矩限制：1200 ‰"), torqueGroup);
    m_torqueThresholdLabel = new QLabel(QStringLiteral("碰撞检测阈值：±500 ‰"), torqueGroup);
    torqueLayout->addWidget(m_torqueTitleLabel);
    torqueLayout->addWidget(m_torqueBarWidget);
    torqueLayout->addWidget(m_torqueValueLabel);
    torqueLayout->addWidget(m_maxTorqueLabel);
    torqueLayout->addWidget(m_torqueThresholdLabel);

    auto *controlGroup = new QGroupBox(QStringLiteral("单电机控制"), rightPanel);
    auto *controlLayout = new QVBoxLayout(controlGroup);
    auto *controlForm = new QFormLayout();

    m_nodeSelector = new QComboBox(controlGroup);
    m_modeSelector = new QComboBox(controlGroup);
    m_modeSelector->addItems({QStringLiteral("速度模式"), QStringLiteral("位置模式")});
    m_velocitySpin = new QSpinBox(controlGroup);
    m_velocitySpin->setRange(-500, 500);
    m_velocitySpin->setValue(50);
    m_velocitySpin->setSuffix(QStringLiteral(" RPM"));
    m_positionSpin = new QDoubleSpinBox(controlGroup);
    m_positionSpin->setRange(-360.0, 360.0);
    m_positionSpin->setDecimals(2);
    m_positionSpin->setValue(30.0);
    m_positionSpin->setSuffix(QStringLiteral(" deg"));
    m_profileVelocitySpin = new QSpinBox(controlGroup);
    m_profileVelocitySpin->setRange(1, 600);
    m_profileVelocitySpin->setValue(60);
    m_profileVelocitySpin->setSuffix(QStringLiteral(" RPM"));
    m_profileAccelerationSpin = new QSpinBox(controlGroup);
    m_profileAccelerationSpin->setRange(1, 2000);
    m_profileAccelerationSpin->setValue(200);
    m_profileAccelerationSpin->setSuffix(QStringLiteral(" RPM/s"));
    m_profileDecelerationSpin = new QSpinBox(controlGroup);
    m_profileDecelerationSpin->setRange(1, 2000);
    m_profileDecelerationSpin->setValue(200);
    m_profileDecelerationSpin->setSuffix(QStringLiteral(" RPM/s"));

    controlForm->addRow(QStringLiteral("节点"), m_nodeSelector);
    controlForm->addRow(QStringLiteral("使能模式"), m_modeSelector);
    controlForm->addRow(QStringLiteral("目标速度"), m_velocitySpin);
    controlForm->addRow(QStringLiteral("目标位置"), m_positionSpin);
    controlForm->addRow(QStringLiteral("位置速度"), m_profileVelocitySpin);
    controlForm->addRow(QStringLiteral("加速度"), m_profileAccelerationSpin);
    controlForm->addRow(QStringLiteral("减速度"), m_profileDecelerationSpin);
    controlLayout->addLayout(controlForm);

    auto *buttonRow1 = new QHBoxLayout();
    auto *enableButton = new QPushButton(QStringLiteral("使能"), controlGroup);
    auto *disableButton = new QPushButton(QStringLiteral("失能"), controlGroup);
    auto *quickStopButton = new QPushButton(QStringLiteral("急停"), controlGroup);
    auto *faultResetButton = new QPushButton(QStringLiteral("故障复位"), controlGroup);
    buttonRow1->addWidget(enableButton);
    buttonRow1->addWidget(disableButton);
    buttonRow1->addWidget(quickStopButton);
    buttonRow1->addWidget(faultResetButton);
    controlLayout->addLayout(buttonRow1);

    auto *buttonRow2 = new QHBoxLayout();
    auto *setVelocityButton = new QPushButton(QStringLiteral("设置速度"), controlGroup);
    auto *moveRelativeButton = new QPushButton(QStringLiteral("相对运动"), controlGroup);
    auto *moveAbsoluteButton = new QPushButton(QStringLiteral("绝对运动"), controlGroup);
    buttonRow2->addWidget(setVelocityButton);
    buttonRow2->addWidget(moveRelativeButton);
    buttonRow2->addWidget(moveAbsoluteButton);
    controlLayout->addLayout(buttonRow2);

    auto *batchGroup = new QGroupBox(QStringLiteral("批量安全操作"), rightPanel);
    auto *batchLayout = new QHBoxLayout(batchGroup);
    auto *enableAllButton = new QPushButton(QStringLiteral("全部使能"), batchGroup);
    auto *disableAllButton = new QPushButton(QStringLiteral("全部失能"), batchGroup);
    auto *quickStopAllButton = new QPushButton(QStringLiteral("全部急停"), batchGroup);
    auto *faultResetAllButton = new QPushButton(QStringLiteral("全部复位"), batchGroup);
    batchLayout->addWidget(enableAllButton);
    batchLayout->addWidget(disableAllButton);
    batchLayout->addWidget(quickStopAllButton);
    batchLayout->addWidget(faultResetAllButton);

    auto *logGroup = new QGroupBox(QStringLiteral("事件日志"), rightPanel);
    auto *logLayout = new QVBoxLayout(logGroup);
    m_logView = new QPlainTextEdit(logGroup);
    m_logView->setReadOnly(true);
    logLayout->addWidget(m_logView);

    rightLayout->addWidget(torqueGroup);
    rightLayout->addWidget(controlGroup);
    rightLayout->addWidget(batchGroup);
    rightLayout->addWidget(logGroup, 1);

    splitter->addWidget(leftPanel);
    splitter->addWidget(rightPanel);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);

    rootLayout->addWidget(topBar);
    rootLayout->addWidget(splitter, 1);

    connect(m_refreshPortsButton, &QToolButton::clicked, this, &MainWindow::refreshPorts);
    connect(m_connectButton, &QPushButton::clicked, this, [this]() {
        applyBusAndNodes();
        m_motorService->connectBus();
    });
    connect(m_disconnectButton, &QPushButton::clicked, m_motorService, &MotorService::disconnectBus);
    connect(m_scanButton, &QPushButton::clicked, this, [this]() {
        applyBusAndNodes();
        m_motorService->scanConfiguredNodes();
    });
    connect(m_monitoringButton, &QPushButton::clicked, this, [this]() {
        if (m_monitoringButton->text() == QLatin1String("启动监控")) {
            m_motorService->startMonitoring();
            m_monitoringButton->setText(QStringLiteral("停止监控"));
        } else {
            m_motorService->stopMonitoring();
            m_monitoringButton->setText(QStringLiteral("启动监控"));
        }
    });

    connect(enableButton, &QPushButton::clicked, this, [this]() {
        m_motorService->enableMotor(currentNodeId(), m_modeSelector->currentIndex() == 1);
    });
    connect(disableButton, &QPushButton::clicked, this, [this]() {
        m_motorService->disableMotor(currentNodeId());
    });
    connect(quickStopButton, &QPushButton::clicked, this, [this]() {
        m_motorService->quickStopMotor(currentNodeId());
    });
    connect(faultResetButton, &QPushButton::clicked, this, [this]() {
        m_motorService->faultResetMotor(currentNodeId());
    });
    connect(setVelocityButton, &QPushButton::clicked, this, [this]() {
        m_motorService->setVelocity(currentNodeId(), m_velocitySpin->value());
    });
    connect(moveRelativeButton, &QPushButton::clicked, this, [this]() {
        m_motorService->movePosition(
            currentNodeId(),
            m_positionSpin->value(),
            false,
            static_cast<quint32>(m_profileVelocitySpin->value()),
            static_cast<quint32>(m_profileAccelerationSpin->value()),
            static_cast<quint32>(m_profileDecelerationSpin->value()));
    });
    connect(moveAbsoluteButton, &QPushButton::clicked, this, [this]() {
        m_motorService->movePosition(
            currentNodeId(),
            m_positionSpin->value(),
            true,
            static_cast<quint32>(m_profileVelocitySpin->value()),
            static_cast<quint32>(m_profileAccelerationSpin->value()),
            static_cast<quint32>(m_profileDecelerationSpin->value()));
    });
    connect(enableAllButton, &QPushButton::clicked, this, [this]() {
        m_motorService->enableAll(m_modeSelector->currentIndex() == 1);
    });
    connect(disableAllButton, &QPushButton::clicked, m_motorService, &MotorService::disableAll);
    connect(quickStopAllButton, &QPushButton::clicked, m_motorService, &MotorService::quickStopAll);
    connect(faultResetAllButton, &QPushButton::clicked, m_motorService, &MotorService::faultResetAll);
    connect(m_nodeSelector, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
        m_motorService->setFocusedNode(currentNodeId());
        refreshTorquePanelForCurrentNode();
    });
}

void MainWindow::wireSignals()
{
    connect(m_motorService, &MotorService::logMessage, this, &MainWindow::appendLog);
    connect(m_motorService, &MotorService::busConnectionChanged, this, &MainWindow::onBusConnectionChanged);
    connect(m_motorService, &MotorService::configuredNodesChanged, this, &MainWindow::reloadNodeSelector);
    connect(m_motorService, &MotorService::motorStateChanged, this, &MainWindow::updateTorquePanel);
    connect(m_motorTable->selectionModel(), &QItemSelectionModel::currentRowChanged, this, [this]() {
        syncSelectedNodeFromTable();
    });
}

void MainWindow::setConnectionState(bool connected, const QString &message)
{
    m_busStatusLabel->setText(connected ? QStringLiteral("已连接") : QStringLiteral("未连接"));
    m_busStatusLabel->setToolTip(message);
    m_busStatusLabel->setStyleSheet(connected
            ? QStringLiteral("QLabel#busStatusLabel { padding: 6px 12px; border-radius: 999px; background: #c9ebe6; color: #115e59; font-weight: 700; }")
            : QStringLiteral("QLabel#busStatusLabel { padding: 6px 12px; border-radius: 999px; background: #f7d6d0; color: #991b1b; font-weight: 700; }"));
}

MotorConfig MainWindow::configForNode(quint32 nodeId) const
{
    const QList<MotorConfig> configs = parseNodeConfigs();
    for (const MotorConfig &config : configs) {
        if (config.nodeId == nodeId) {
            return config;
        }
    }
    return {};
}
