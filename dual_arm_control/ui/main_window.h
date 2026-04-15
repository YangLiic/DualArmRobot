#pragma once
#include "models/common_types.h"
#include "communication/communication_manager.h"
#include "services/motor_service.h"
#include "services/arm_service.h"
#include "ui/connection_panel.h"
#include "ui/motor_control_panel.h"
#include "ui/humanoid_arms_panel.h"
#include "ui/status_monitor_panel.h"
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
    void applyGlobalStyle();

    // 核心对象
    CommunicationManager *commManager_;
    MotorService         *motorService_;
    ArmService           *armService_;

    // UI 面板
    ConnectionPanel    *connectionPanel_;
    MotorControlPanel  *motorControlPanel_;
    HumanoidArmsPanel  *humanoidArmsPanel_;
    StatusMonitorPanel *statusMonitorPanel_;
    LogWidget          *logWidget_;
    QTabWidget         *tabWidget_;
};

} // namespace dac
