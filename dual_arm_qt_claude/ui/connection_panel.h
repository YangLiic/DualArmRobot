/**
 * ConnectionPanel - 设备连接面板
 */
#pragma once

#include <QWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QGroupBox>
#include <QTableWidget>

namespace dar {

class CommunicationManager;
class MotorService;

class ConnectionPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ConnectionPanel(CommunicationManager *commMgr,
                             MotorService *motorService,
                             QWidget *parent = nullptr);

signals:
    void logMessage(const dar::LogEntry &entry);

private slots:
    void onConnectClicked();
    void onDisconnectClicked();
    void onAddMotorClicked();
    void onRemoveMotorClicked();
    void onScanClicked();
    void onRefreshPorts();

private:
    void setupUi();
    void updateMotorTable();

    CommunicationManager *commMgr_;
    MotorService *motorService_;

    // 连接配置
    QComboBox *portCombo_;
    QComboBox *baudRateCombo_;
    QLineEdit *busIdEdit_;
    QPushButton *connectBtn_;
    QPushButton *disconnectBtn_;
    QPushButton *refreshPortsBtn_;

    // 节点管理
    QSpinBox *nodeIdSpin_;
    QPushButton *addMotorBtn_;
    QPushButton *removeMotorBtn_;
    QPushButton *scanBtn_;
    QTableWidget *motorTable_;

    QLabel *connectionStatusLabel_;
};

}  // namespace dar
