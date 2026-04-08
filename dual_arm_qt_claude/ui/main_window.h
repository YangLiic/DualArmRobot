/**
 * MainWindow - Qt 上位机主窗口
 *
 * 职责：
 * - 整合所有面板 (设备连接、电机控制、状态监控、日志)
 * - 管理 CommunicationManager 和 MotorService 的生命周期
 * - 提供全局工具栏与状态栏
 */
#pragma once

#include <QMainWindow>
#include <QTabWidget>
#include <QStatusBar>
#include <QLabel>
#include <QAction>
#include <QToolBar>
#include "../models/common_types.h"

namespace dar {

class CommunicationManager;
class MotorService;
class ConnectionPanel;
class MotorControlPanel;
class StatusMonitorPanel;
class LogWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onBusConnected(const QString &busId);
    void onBusDisconnected(const QString &busId);
    void onMotorStateChanged(uint32_t nodeId, const dar::MotorState &state);
    void onLogMessage(const dar::LogEntry &entry);
    void onEmergencyStopAll();

private:
    void setupUi();
    void setupStyle();
    void setupConnections();
    void updateStatusBar();

    // 核心服务
    CommunicationManager *commMgr_;
    MotorService *motorService_;

    // UI 组件
    QTabWidget *tabWidget_;
    ConnectionPanel *connectionPanel_;
    MotorControlPanel *motorControlPanel_;
    StatusMonitorPanel *statusMonitorPanel_;
    LogWidget *logWidget_;

    // 状态栏
    QLabel *statusLabel_;
    QLabel *busStatusLabel_;

    // 工具栏
    QAction *emergencyStopAction_;
};

}  // namespace dar
