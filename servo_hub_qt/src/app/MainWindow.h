#pragma once

#include "app/TorqueBarWidget.h"
#include "services/MotorService.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableView>
#include <QToolButton>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(MotorService *motorService, QWidget *parent = nullptr);

private slots:
    void refreshPorts();
    void applyBusAndNodes();
    void onBusConnectionChanged(bool connected, const QString &message);
    void appendLog(const QString &message);
    void reloadNodeSelector(const QList<quint32> &nodeIds);
    void syncSelectedNodeFromTable();
    void updateTorquePanel(const MotorState &state);
    void refreshTorquePanelForCurrentNode();

private:
    QList<MotorConfig> parseNodeConfigs() const;
    quint32 currentNodeId() const;
    void buildUi();
    void wireSignals();
    void setConnectionState(bool connected, const QString &message);
    MotorConfig configForNode(quint32 nodeId) const;

    MotorService *m_motorService = nullptr;

    QComboBox *m_portCombo = nullptr;
    QComboBox *m_baudCombo = nullptr;
    QLineEdit *m_nodeLineEdit = nullptr;
    QLabel *m_busStatusLabel = nullptr;
    QPushButton *m_connectButton = nullptr;
    QPushButton *m_disconnectButton = nullptr;
    QPushButton *m_scanButton = nullptr;
    QToolButton *m_refreshPortsButton = nullptr;
    QPushButton *m_monitoringButton = nullptr;

    QTableView *m_motorTable = nullptr;
    QComboBox *m_nodeSelector = nullptr;
    QComboBox *m_modeSelector = nullptr;
    QSpinBox *m_velocitySpin = nullptr;
    QDoubleSpinBox *m_positionSpin = nullptr;
    QSpinBox *m_profileVelocitySpin = nullptr;
    QSpinBox *m_profileAccelerationSpin = nullptr;
    QSpinBox *m_profileDecelerationSpin = nullptr;
    QLabel *m_torqueTitleLabel = nullptr;
    TorqueBarWidget *m_torqueBarWidget = nullptr;
    QLabel *m_torqueValueLabel = nullptr;
    QLabel *m_maxTorqueLabel = nullptr;
    QLabel *m_torqueThresholdLabel = nullptr;
    QPlainTextEdit *m_logView = nullptr;
};
