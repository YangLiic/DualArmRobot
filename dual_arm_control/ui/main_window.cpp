#include "ui/main_window.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QApplication>
#include <QMessageBox>

namespace dac {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    commManager_ = new CommunicationManager(this);
    motorService_ = new MotorService(commManager_, this);

    setupUi();
    setupConnections();
    initMotorNodes();
    applyGlobalStyle();

    setWindowTitle(QStringLiteral("双臂机器人电机控制系统 - DualArmControl"));
    resize(1280, 800);
}

MainWindow::~MainWindow()
{
    motorService_->stopAllMonitoring();
    commManager_->closeBus();
}

void MainWindow::setupUi()
{
    auto *central = new QWidget;
    auto *mainLayout = new QVBoxLayout(central);

    // 顶部：连接面板
    connectionPanel_ = new ConnectionPanel;
    mainLayout->addWidget(connectionPanel_);

    // 中部：水平分割 - 左侧控制 + 右侧监控
    auto *splitter = new QSplitter(Qt::Horizontal);

    // 左侧：电机控制
    motorControlPanel_ = new MotorControlPanel;
    auto *leftScroll = new QWidget;
    auto *leftLayout = new QVBoxLayout(leftScroll);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->addWidget(motorControlPanel_);
    splitter->addWidget(leftScroll);

    // 右侧：TAB（状态监控 + 日志）
    tabWidget_ = new QTabWidget;

    statusMonitorPanel_ = new StatusMonitorPanel;
    tabWidget_->addTab(statusMonitorPanel_, QStringLiteral("实时监控"));

    logWidget_ = new LogWidget;
    tabWidget_->addTab(logWidget_, QStringLiteral("系统日志"));

    splitter->addWidget(tabWidget_);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 2);
    splitter->setSizes({380, 900});

    mainLayout->addWidget(splitter, 1);
    setCentralWidget(central);
}

void MainWindow::setupConnections()
{
    // ====== 连接面板 → 通信管理器 ======
    connect(connectionPanel_, &ConnectionPanel::connectRequested, this, [this](const BusConfig &cfg) {
        commManager_->openBus(cfg);
    });
    connect(connectionPanel_, &ConnectionPanel::disconnectRequested, this, [this]() {
        motorService_->stopAllMonitoring();
        commManager_->closeBus();
    });
    connect(commManager_, &CommunicationManager::busStateChanged, this, [this](bool open, const QString &msg) {
        connectionPanel_->setBusState(open, msg);
        if (open) {
            // 总线打开后自动开启扭矩监控
            for (uint32_t nid : motorService_->nodeIds()) {
                motorService_->startMonitoring(nid, 100);
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

    // 状态面板：为每个电机创建仪表盘
    // 400W: 满量程 1000‰ (100% 额定), 碰撞阈值 500‰ (50%)
    statusMonitorPanel_->addMotorGauge(0x601, QStringLiteral("400W 电机 (0x601)"), 1000, 500);
    // 750W: 满量程 1000‰ (100% 额定), 碰撞阈值 200‰ (20%)
    statusMonitorPanel_->addMotorGauge(0x602, QStringLiteral("750W 电机 (0x602)"), 1000, 200);
}

void MainWindow::applyGlobalStyle()
{
    qApp->setStyleSheet(R"(
        QMainWindow, QWidget {
            background-color: #1e1e2e;
            color: #cdd6f4;
        }
        QGroupBox {
            background: #181825;
            border: 1px solid #45475a;
            border-radius: 6px;
            margin-top: 14px;
            padding-top: 16px;
            font-weight: bold;
            color: #cdd6f4;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 6px;
            color: #89b4fa;
        }
        QLabel {
            color: #cdd6f4;
        }
        QComboBox {
            background: #313244;
            color: #cdd6f4;
            border: 1px solid #45475a;
            border-radius: 4px;
            padding: 4px 8px;
        }
        QComboBox::drop-down {
            border: none;
        }
        QComboBox QAbstractItemView {
            background: #313244;
            color: #cdd6f4;
            selection-background-color: #45475a;
        }
        QSpinBox, QDoubleSpinBox {
            background: #313244;
            color: #cdd6f4;
            border: 1px solid #45475a;
            border-radius: 4px;
            padding: 4px 8px;
        }
        QPushButton {
            background: #45475a;
            color: #cdd6f4;
            border: none;
            border-radius: 4px;
            padding: 6px 14px;
            font-weight: bold;
        }
        QPushButton:hover {
            background: #585b70;
        }
        QPushButton:pressed {
            background: #313244;
        }
        QPushButton:disabled {
            background: #313244;
            color: #585b70;
        }
        QTabWidget::pane {
            border: 1px solid #45475a;
            border-radius: 4px;
            background: #181825;
        }
        QTabBar::tab {
            background: #313244;
            color: #7f849c;
            padding: 8px 20px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            margin-right: 2px;
        }
        QTabBar::tab:selected {
            background: #181825;
            color: #89b4fa;
            font-weight: bold;
        }
        QTabBar::tab:hover {
            background: #45475a;
        }
        QSplitter::handle {
            background: #45475a;
            width: 2px;
        }
        QScrollBar:vertical {
            background: #181825;
            width: 8px;
            border: none;
        }
        QScrollBar::handle:vertical {
            background: #45475a;
            border-radius: 4px;
            min-height: 20px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0;
        }
    )");
}

} // namespace dac
