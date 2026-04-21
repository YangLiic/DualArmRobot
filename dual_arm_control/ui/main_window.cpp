#include "ui/main_window.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QFrame>
#include <QScrollArea>
#include <QApplication>
#include <QMessageBox>
#include <QAbstractSpinBox>
#include <QComboBox>
#include <QEvent>
#include <QWheelEvent>
#include <QLabel>

// 全局事件过滤器：禁止 SpinBox / ComboBox 的滚轮误触
class WheelGuard : public QObject {
public:
    using QObject::QObject;
protected:
    bool eventFilter(QObject *obj, QEvent *ev) override {
        if (ev->type() == QEvent::Wheel) {
            if (qobject_cast<QAbstractSpinBox*>(obj) || qobject_cast<QComboBox*>(obj)) {
                ev->ignore();
                return true;   // 吞掉滚轮事件
            }
        }
        return QObject::eventFilter(obj, ev);
    }
};

namespace dac {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    commManager_ = new CommunicationManager(this);
    motorService_ = new MotorService(commManager_, this);
    armService_ = new ArmService(this);
    ze300Service_ = new Ze300Service(commManager_, this);
    cyberGearService_ = new CyberGearService(commManager_, this);
    gripperService_ = new GripperService(this);

    setupUi();
    setupConnections();
    initMotorNodes();
    initZe300Nodes();
    initCyberGearNodes();
    applyGlobalStyle();

    setWindowTitle(QStringLiteral("双臂机器人电机控制系统 - DualArmControl"));
    resize(1280, 800);

    // 安装全局滚轮过滤器，防止 SpinBox/ComboBox 被滚轮误触
    auto *wheelGuard = new WheelGuard(this);
    qApp->installEventFilter(wheelGuard);
}

MainWindow::~MainWindow()
{
    motorService_->stopAllMonitoring();
    ze300Service_->stopAllMonitoring();
    cyberGearService_->stopAllMonitoring();
    commManager_->closeBus();
}

void MainWindow::setupUi()
{
    auto *central = new QWidget;
    auto *mainLayout = new QVBoxLayout(central);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // ==================== 顶部：连接面板 ====================
    connectionPanel_ = new ConnectionPanel;
    mainLayout->addWidget(connectionPanel_);

    // ==================== 中部：设备控制选项卡 ====================
    auto *deviceTabs = new QTabWidget;
    deviceTabs->setTabPosition(QTabWidget::North);
    deviceTabs->setDocumentMode(true);

    // 辅助函数：将面板包在 ScrollArea 中
    auto wrapInScroll = [](QWidget *panel) -> QScrollArea* {
        auto *scroll = new QScrollArea;
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        auto *container = new QWidget;
        auto *layout = new QVBoxLayout(container);
        layout->setContentsMargins(12, 12, 12, 12);
        layout->addWidget(panel);
        layout->addStretch();
        scroll->setWidget(container);
        return scroll;
    };

    // Tab 1: ino Motor
    motorControlPanel_ = new MotorControlPanel;
    deviceTabs->addTab(wrapInScroll(motorControlPanel_), QStringLiteral("⚙ ino Motor"));

    // Tab 2: ZE300
    ze300ControlPanel_ = new Ze300ControlPanel;
    deviceTabs->addTab(wrapInScroll(ze300ControlPanel_), QStringLiteral("🔧 ZE300"));

    // Tab 3: Humanoid Arms
    humanoidArmsPanel_ = new HumanoidArmsPanel;
    deviceTabs->addTab(wrapInScroll(humanoidArmsPanel_), QStringLiteral("🦾 机械臂"));

    // Tab 4: CyberGear
    cyberGearPanel_ = new CyberGearControlPanel;
    deviceTabs->addTab(wrapInScroll(cyberGearPanel_), QStringLiteral("🤖 CyberGear"));

    // Tab 5: Inspire Gripper
    gripperControlPanel_ = new GripperControlPanel;
    deviceTabs->addTab(wrapInScroll(gripperControlPanel_), QStringLiteral("🤏 Inspire夹爪"));

    // ==================== 底部：监控 + 日志 ====================
    tabWidget_ = new QTabWidget;
    tabWidget_->setTabPosition(QTabWidget::North);
    tabWidget_->setDocumentMode(true);

    statusMonitorPanel_ = new StatusMonitorPanel;
    tabWidget_->addTab(statusMonitorPanel_, QStringLiteral("📊 实时监控"));

    logWidget_ = new LogWidget;
    tabWidget_->addTab(logWidget_, QStringLiteral("📋 系统日志"));

    // ==================== 垂直分割器 ====================
    auto *splitter = new QSplitter(Qt::Vertical);
    splitter->addWidget(deviceTabs);
    splitter->addWidget(tabWidget_);
    splitter->setStretchFactor(0, 3);   // 设备面板占 60%
    splitter->setStretchFactor(1, 2);   // 监控日志占 40%
    splitter->setChildrenCollapsible(false);

    mainLayout->addWidget(splitter, 1);
    setCentralWidget(central);
}

void MainWindow::setupConnections()
{
    // ====== 连接面板 → 通信管理器 ======
    connect(connectionPanel_, &ConnectionPanel::connectRequested, this, [this](const BusConfig &cfg) {
        if (armService_->isInitialized()) {
            logWidget_->appendLog(
                LogLevel::Warning,
                QStringLiteral("机械臂 SDK 已连接，已阻止总线连接，避免 libcontrolcan 资源争用。请先断开机械臂 SDK。"));
            return;
        }
        commManager_->openBus(cfg);
    });
    connect(connectionPanel_, &ConnectionPanel::disconnectRequested, this, [this]() {
        motorService_->stopAllMonitoring();
        ze300Service_->stopAllMonitoring();
        cyberGearService_->stopAllMonitoring();
        commManager_->closeBus();
    });
    connect(commManager_, &CommunicationManager::busStateChanged, this, [this](bool open, const QString &msg) {
        connectionPanel_->setBusState(open, msg);
        if (open) {
            // 总线打开后自动开启 ino motor 扭矩监控
            for (uint32_t nid : motorService_->nodeIds()) {
                motorService_->startMonitoring(nid, 100);
            }
            // 总线打开后自动开启 ZE300 状态监控
            for (uint16_t addr : ze300Service_->nodeAddrs()) {
                ze300Service_->startMonitoring(addr, 100);
            }
            // 总线打开后自动开启 CyberGear 状态监控
            for (uint8_t mid : cyberGearService_->motorIds()) {
                cyberGearService_->startMonitoring(mid, 100);
            }
        }
    });

    // ====== 电机控制面板 → MotorService ======
    connect(motorControlPanel_, &MotorControlPanel::enableRequested, motorService_, &MotorService::enableMotor);
    connect(motorControlPanel_, &MotorControlPanel::disableRequested, motorService_, &MotorService::disableMotor);
    connect(motorControlPanel_, &MotorControlPanel::emergencyStopRequested, motorService_, &MotorService::emergencyStop);
    connect(motorControlPanel_, &MotorControlPanel::faultResetRequested, motorService_, &MotorService::faultReset);
    connect(motorControlPanel_, &MotorControlPanel::velocityRequested, motorService_, &MotorService::setVelocity);
    connect(motorControlPanel_, &MotorControlPanel::positionRequested, motorService_, &MotorService::setPosition);
    connect(motorControlPanel_, &MotorControlPanel::profileVelocityRequested, motorService_, &MotorService::setProfileVelocity);
    connect(motorControlPanel_, &MotorControlPanel::releaseBrakeRequested, motorService_, &MotorService::releaseBrake);
    connect(motorControlPanel_, &MotorControlPanel::lockBrakeRequested, motorService_, &MotorService::lockBrake);

    // 批量操作
    connect(motorControlPanel_, &MotorControlPanel::enableAllRequested, motorService_, &MotorService::enableAll);
    connect(motorControlPanel_, &MotorControlPanel::disableAllRequested, motorService_, &MotorService::disableAll);
    connect(motorControlPanel_, &MotorControlPanel::emergencyStopAllRequested, motorService_, &MotorService::emergencyStopAll);
    connect(motorControlPanel_, &MotorControlPanel::faultResetAllRequested, motorService_, &MotorService::faultResetAll);

    // ====== 使能电机后自动开启碰撞保护 ======
    connect(motorService_, &MotorService::motorEnabled, this, [this](uint32_t nodeId, bool ok) {
        if (!ok) return;
        CollisionConfig cfg;
        if (nodeId == 0x601) {
            cfg.torqueLimitPermille = 500;   // 400W: 50% 额定转矩
            cfg.triggerTorquePermille = 500;
        } else {
            cfg.torqueLimitPermille = 200;   // 750W: 20% 额定转矩
            cfg.triggerTorquePermille = 200;
        }
        cfg.consecutiveSamples = 3;
        cfg.pollIntervalMs = 20;
        cfg.useQuickStop = true;
        motorService_->enableCollisionProtection(nodeId, cfg);
    });

    // ====== MotorService → 状态面板 ======
    connect(motorService_, &MotorService::motorStateChanged, statusMonitorPanel_, &StatusMonitorPanel::updateMotorState);
    connect(motorService_, &MotorService::torqueUpdated, statusMonitorPanel_, &StatusMonitorPanel::onTorqueUpdated);
    connect(motorService_, &MotorService::collisionDetected, statusMonitorPanel_, &StatusMonitorPanel::onCollisionDetected);

    // ====== MotorService → 控制面板状态 ======
    connect(motorService_, &MotorService::motorStateChanged, motorControlPanel_, &MotorControlPanel::updateMotorState);

    // ====== 机械臂控制面板 → ArmService ======
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::connectRequested, this, [this](int deviceIndex, int canIndex) {
        if (commManager_->isBusOpen()) {
            logWidget_->appendLog(
                LogLevel::Warning,
                QStringLiteral("检测到总线已连接，连接机械臂 SDK 前已自动断开总线，避免 libcontrolcan 资源争用。"));
            motorService_->stopAllMonitoring();
            ze300Service_->stopAllMonitoring();
            cyberGearService_->stopAllMonitoring();
            commManager_->closeBus();
        }
        armService_->initializeSdk(deviceIndex, canIndex);
    });
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::disconnectRequested, armService_, &ArmService::shutdownSdk);
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::refreshRequested, armService_, &ArmService::refreshAllStates);
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::jointTargetsRequested, armService_, &ArmService::moveJointTargets);
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::jointLimitsRequested, armService_, &ArmService::setJointLimits);
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::brakeRequested, armService_, &ArmService::brakeArm);
    connect(humanoidArmsPanel_, &HumanoidArmsPanel::clearErrorsRequested, armService_, &ArmService::clearErrors);

    connect(armService_, &ArmService::initializationChanged, humanoidArmsPanel_, &HumanoidArmsPanel::setConnectionState);
    connect(armService_, &ArmService::armStateChanged, humanoidArmsPanel_, &HumanoidArmsPanel::updateArmState);
    connect(armService_, &ArmService::jointLimitsChanged, humanoidArmsPanel_, &HumanoidArmsPanel::setJointLimits);

    const ArmJointLimits leftLimits = armService_->jointLimits(ArmSide::Left);
    humanoidArmsPanel_->setJointLimits(ArmSide::Left, leftLimits.minDegrees, leftLimits.maxDegrees);
    const ArmJointLimits rightLimits = armService_->jointLimits(ArmSide::Right);
    humanoidArmsPanel_->setJointLimits(ArmSide::Right, rightLimits.minDegrees, rightLimits.maxDegrees);

    // ====== 碰撞触发弹窗（非阻塞，急停已在 MotorService 中先行完成） ======
    connect(motorService_, &MotorService::collisionDetected, this, [this](uint32_t nodeId, int16_t torque) {
        auto *msgBox = new QMessageBox(QMessageBox::Warning,
                                       QStringLiteral("碰撞保护触发"),
                                       QStringLiteral("电机 0x%1 碰撞保护触发!\n实际扭矩: %2‰ (%3% 额定)\n电机已自动急停，运动指令已清除。")
                                           .arg(nodeId, 3, 16, QChar('0'))
                                           .arg(torque)
                                           .arg(qAbs(torque) / 10.0, 0, 'f', 1),
                                       QMessageBox::Ok, this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setModal(false);
        msgBox->show();
    });

    // ====== 日志 ======
    connect(commManager_, &CommunicationManager::logMessage, logWidget_, &LogWidget::appendLog);
    connect(motorService_, &MotorService::logMessage, logWidget_, &LogWidget::appendLog);
    connect(armService_, &ArmService::logMessage, logWidget_, &LogWidget::appendLog);
    connect(ze300Service_, &Ze300Service::logMessage, logWidget_, &LogWidget::appendLog);

    // ====== ZE300 控制面板 → Ze300Service ======
    connect(ze300ControlPanel_, &Ze300ControlPanel::speedRequested, ze300Service_, &Ze300Service::setSpeedRpm);
    connect(ze300ControlPanel_, &Ze300ControlPanel::absPositionRequested, ze300Service_, &Ze300Service::setAbsolutePositionDeg);
    connect(ze300ControlPanel_, &Ze300ControlPanel::relPositionRequested, ze300Service_, &Ze300Service::setRelativePositionDeg);
    connect(ze300ControlPanel_, &Ze300ControlPanel::goOriginRequested, ze300Service_, &Ze300Service::goOriginShortest);
    connect(ze300ControlPanel_, &Ze300ControlPanel::setZeroRequested, ze300Service_, &Ze300Service::setZero);
    connect(ze300ControlPanel_, &Ze300ControlPanel::freeOutputRequested, ze300Service_, &Ze300Service::freeOutput);
    connect(ze300ControlPanel_, &Ze300ControlPanel::clearFaultRequested, ze300Service_, &Ze300Service::clearFault);
    connect(ze300ControlPanel_, &Ze300ControlPanel::rebootRequested, ze300Service_, &Ze300Service::reboot);
    connect(ze300ControlPanel_, &Ze300ControlPanel::freeOutputAllRequested, ze300Service_, &Ze300Service::freeOutputAll);

    // 抱闸
    connect(ze300ControlPanel_, &Ze300ControlPanel::brakeCloseRequested, this, [this](uint16_t addr) { ze300Service_->setBrakeClosed(addr, true); });
    connect(ze300ControlPanel_, &Ze300ControlPanel::brakeOpenRequested, this, [this](uint16_t addr) { ze300Service_->setBrakeClosed(addr, false); });
    connect(ze300ControlPanel_, &Ze300ControlPanel::brakeReadRequested, ze300Service_, &Ze300Service::readBrakeState);

    // 参数
    connect(ze300ControlPanel_, &Ze300ControlPanel::posMaxSpeedRequested, ze300Service_, &Ze300Service::setPositionMaxSpeedRpm);
    connect(ze300ControlPanel_, &Ze300ControlPanel::maxCurrentRequested, ze300Service_, &Ze300Service::setMaxCurrentA);
    connect(ze300ControlPanel_, &Ze300ControlPanel::speedAccelRequested, ze300Service_, &Ze300Service::setSpeedAccelerationRpmPerSec);

    // MIT
    connect(ze300ControlPanel_, &Ze300ControlPanel::mitControlRequested, ze300Service_, &Ze300Service::sendMitControl);

    // ====== Ze300Service → UI ======
    connect(ze300Service_, &Ze300Service::ze300StateChanged, ze300ControlPanel_, &Ze300ControlPanel::updateZe300State);

    // ZE300 故障弹窗
    connect(ze300Service_, &Ze300Service::ze300FaultDetected, this, [this](uint16_t devAddr, uint8_t faultCode) {
        auto *msgBox = new QMessageBox(QMessageBox::Warning,
                                       QStringLiteral("ZE300 故障"),
                                       QStringLiteral("ZE300 (0x%1) 检测到故障!\n故障码: 0x%2")
                                           .arg(devAddr, 2, 16, QChar('0'))
                                           .arg(faultCode, 2, 16, QChar('0')),
                                       QMessageBox::Ok, this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setModal(false);
        msgBox->show();
    });

    // ====== CyberGear 控制面板 → CyberGearService ======
    connect(cyberGearPanel_, &CyberGearControlPanel::enableRequested, cyberGearService_, &CyberGearService::enable);
    connect(cyberGearPanel_, &CyberGearControlPanel::stopRequested, cyberGearService_, &CyberGearService::stop);
    connect(cyberGearPanel_, &CyberGearControlPanel::speedModeRequested, cyberGearService_, &CyberGearService::enableSpeedMode);
    connect(cyberGearPanel_, &CyberGearControlPanel::positionModeRequested, cyberGearService_, &CyberGearService::enablePositionMode);
    connect(cyberGearPanel_, &CyberGearControlPanel::speedRequested, cyberGearService_, &CyberGearService::setSpeedRadS);
    connect(cyberGearPanel_, &CyberGearControlPanel::positionDegRequested, cyberGearService_, &CyberGearService::setPositionDeg);
    connect(cyberGearPanel_, &CyberGearControlPanel::setZeroRequested, cyberGearService_, &CyberGearService::setMechZero);
    connect(cyberGearPanel_, &CyberGearControlPanel::goToZeroRequested, cyberGearService_, &CyberGearService::goToZero);
    connect(cyberGearPanel_, &CyberGearControlPanel::currentLimitRequested, cyberGearService_, &CyberGearService::setCurrentLimit);
    connect(cyberGearPanel_, &CyberGearControlPanel::speedLimitRequested, cyberGearService_, &CyberGearService::setSpeedLimit);
    connect(cyberGearPanel_, &CyberGearControlPanel::stopAllRequested, cyberGearService_, &CyberGearService::stopAll);

    // CyberGearService → UI
    connect(cyberGearService_, &CyberGearService::cgStateChanged, cyberGearPanel_, &CyberGearControlPanel::updateCgState);
    connect(cyberGearService_, &CyberGearService::logMessage, logWidget_, &LogWidget::appendLog);

    // ====== Inspire 夹爪控制面板 → GripperService ======
    connect(gripperControlPanel_, &GripperControlPanel::connectRequested,
            gripperService_, &GripperService::connectEndpoint);
    connect(gripperControlPanel_, &GripperControlPanel::disconnectRequested,
            gripperService_, &GripperService::disconnectEndpoint);
    connect(gripperControlPanel_, &GripperControlPanel::targetRequested,
            gripperService_, &GripperService::setTargetPosition);

    // ====== GripperService → UI ======
    connect(gripperService_, &GripperService::gripperStateChanged,
            gripperControlPanel_, &GripperControlPanel::setGripperState);
    connect(gripperService_, &GripperService::connectionChanged,
            gripperControlPanel_, &GripperControlPanel::setConnectionState);
    connect(gripperService_, &GripperService::logMessage,
            logWidget_, &LogWidget::appendLog);

    // CyberGear 故障弹窗
    connect(cyberGearService_, &CyberGearService::cgFaultDetected, this, [this](uint8_t motorId) {
        auto *msgBox = new QMessageBox(QMessageBox::Warning,
                                       QStringLiteral("CyberGear 故障"),
                                       QStringLiteral("CyberGear (0x%1) 检测到故障!")
                                           .arg(motorId, 2, 16, QChar('0')),
                                       QMessageBox::Ok, this);
        msgBox->setAttribute(Qt::WA_DeleteOnClose);
        msgBox->setModal(false);
        msgBox->show();
    });
}

void MainWindow::initMotorNodes()
{
    // 预配置两个电机节点
    motorService_->addNode(0x601, QStringLiteral("400W 电机"), false);
    motorService_->addNode(0x602, QStringLiteral("750W 电机"), true);  // 0x602 方向反转

    // 更新控制面板节点列表
    QMap<uint32_t, QString> names;
    names[0x601] = QStringLiteral("400W 电机");
    names[0x602] = QStringLiteral("750W 电机");
    motorControlPanel_->setNodeList(motorService_->nodeIds(), names);

    // 状态面板：为每个电机创建紧凑条状监控卡片
    statusMonitorPanel_->addMotorGauge(0x601, QStringLiteral("400W 电机 (0x601)"), 1000, 500);
    statusMonitorPanel_->addMotorGauge(0x602, QStringLiteral("750W 电机 (0x602)"), 1000, 200);
}

void MainWindow::initZe300Nodes()
{
    // 预配置 ZE300 电机节点 (devAddr=0x01, useHostAddr=true)
    ze300Service_->addNode(0x01, QStringLiteral("ZE300 电机"), true);

    QMap<uint16_t, QString> ze300Names;
    ze300Names[0x01] = QStringLiteral("ZE300 电机");
    ze300ControlPanel_->setNodeList(ze300Service_->nodeAddrs(), ze300Names);
}

void MainWindow::initCyberGearNodes()
{
    // 预配置 CyberGear 电机节点 (motorId=0x02, masterId=0x00)
    cyberGearService_->addNode(0x02, QStringLiteral("CyberGear 电机"), CyberGearService::kDefaultMasterId);

    QMap<uint8_t, QString> cgNames;
    cgNames[0x02] = QStringLiteral("CyberGear 电机");
    cyberGearPanel_->setNodeList(cyberGearService_->motorIds(), cgNames);
}

void MainWindow::applyGlobalStyle()
{
    qApp->setStyleSheet(R"(
        QMainWindow, QWidget {
            background-color: #f4efe7;
            color: #324356;
            font-family: "Noto Sans CJK SC", "Microsoft YaHei", "PingFang SC", sans-serif;
        }
        QGroupBox {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #fffdfa, stop:1 #f7f1e8);
            border: 1px solid #d9cfc1;
            border-radius: 12px;
            margin-top: 16px;
            padding-top: 18px;
            font-weight: 700;
            color: #2f4256;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 8px;
            color: #5b7694;
        }
        QLabel {
            color: #324356;
            background: transparent;
        }
        QComboBox, QSpinBox, QDoubleSpinBox {
            background: #fffdf9;
            color: #324356;
            border: 1px solid #d7cdbf;
            border-radius: 8px;
            padding: 5px 10px;
            selection-background-color: #dce5ef;
            selection-color: #233445;
        }
        QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover {
            border-color: #b7c7d8;
        }
        QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
            border: 1px solid #8ea6bf;
        }
        QComboBox::drop-down {
            border: none;
            width: 18px;
        }
        QComboBox QAbstractItemView {
            background: #fffdfa;
            color: #324356;
            border: 1px solid #d7cdbf;
            selection-background-color: #dce5ef;
            selection-color: #233445;
        }
        QSlider::groove:horizontal {
            border: 1px solid #d7cdbf;
            height: 6px;
            background: #e9dfd3;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #6f8ca9;
            border: 1px solid #5f7f9e;
            width: 14px;
            margin: -5px 0;
            border-radius: 7px;
        }
        QSlider::sub-page:horizontal {
            background: #8fa9c1;
            border-radius: 3px;
        }
        QPushButton {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #fffdfa, stop:1 #ede2d4);
            color: #324356;
            border: 1px solid #d0c5b8;
            border-radius: 8px;
            padding: 7px 16px;
            font-weight: 600;
        }
        QPushButton:hover {
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                        stop:0 #fffdfa, stop:1 #e5d8c8);
            border-color: #b8c7d6;
        }
        QPushButton:pressed {
            background: #e7dccf;
        }
        QPushButton:disabled {
            background: #ebe4da;
            color: #96a0aa;
            border-color: #ddd4c7;
        }
        QTabWidget::pane {
            border: 1px solid #d9cfc1;
            border-radius: 14px;
            background: #fffdfa;
            top: -1px;
        }
        QTabBar::tab {
            background: #e8dfd3;
            color: #6f7d8b;
            padding: 9px 20px;
            border: 1px solid #d9cfc1;
            border-bottom: none;
            border-top-left-radius: 10px;
            border-top-right-radius: 10px;
            margin-right: 4px;
        }
        QTabBar::tab:selected {
            background: #fffdfa;
            color: #587392;
            font-weight: 700;
        }
        QTabBar::tab:hover {
            background: #f0e7dc;
        }
        QSplitter::handle {
            background: #ddd4c7;
            width: 4px;
            height: 4px;
        }
        QScrollBar:vertical {
            background: #efe7dc;
            width: 10px;
            border: none;
        }
        QScrollBar::handle:vertical {
            background: #c8bcac;
            border-radius: 5px;
            min-height: 20px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0;
        }
        QScrollArea {
            background: transparent;
            border: none;
        }
    )");
}

} // namespace dac
