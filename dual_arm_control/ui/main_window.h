#pragma once
#include "models/common_types.h"
#include "communication/communication_manager.h"
#include "services/motor_service.h"
#include "services/arm_service.h"
#include "services/ze300_service.h"
#include "services/cybergear_service.h"
#include "services/gripper_service.h"
#include "ui/connection_panel.h"
#include "ui/motor_control_panel.h"
#include "ui/humanoid_arms_panel.h"
#include "ui/status_monitor_panel.h"
#include "ui/ze300_control_panel.h"
#include "ui/cybergear_control_panel.h"
#include "ui/gripper_control_panel.h"
#include "ui/log_widget.h"
#include <QMainWindow>
#include <QTabWidget>

namespace dac {

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void setupUi();
    void setupConnections();
    void initMotorNodes();
    void initZe300Nodes();
    void initCyberGearNodes();
    void applyGlobalStyle();

    // 核心对象
    CommunicationManager *commManager_;
    MotorService         *motorService_;
    ArmService           *armService_;
    Ze300Service         *ze300Service_;
    CyberGearService     *cyberGearService_;
    GripperService       *gripperService_;

    // UI 面板
    ConnectionPanel    *connectionPanel_;
    MotorControlPanel  *motorControlPanel_;
    HumanoidArmsPanel  *humanoidArmsPanel_;
    StatusMonitorPanel *statusMonitorPanel_;
    Ze300ControlPanel     *ze300ControlPanel_;
    CyberGearControlPanel *cyberGearPanel_;
    GripperControlPanel   *gripperControlPanel_;
    LogWidget          *logWidget_;
    QTabWidget         *tabWidget_;
};

} // namespace dac
