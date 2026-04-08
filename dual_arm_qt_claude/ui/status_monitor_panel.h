/**
 * StatusMonitorPanel - 实时状态监控面板
 */
#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QMap>
#include "../models/common_types.h"

namespace dar {

class MotorService;

class StatusMonitorPanel : public QWidget
{
    Q_OBJECT

public:
    explicit StatusMonitorPanel(MotorService *motorService, QWidget *parent = nullptr);

public slots:
    void onMotorStateChanged(uint32_t nodeId, const dar::MotorState &state);
    void refreshAll();

private slots:
    void onStartMonitoringClicked();
    void onStopMonitoringClicked();

private:
    void setupUi();
    int findOrCreateRow(uint32_t nodeId);

    MotorService *motorService_;
    QTableWidget *statusTable_;
    QPushButton *startMonitorBtn_;
    QPushButton *stopMonitorBtn_;
    QLabel *updateLabel_;
    QTimer *refreshTimer_;

    QMap<uint32_t, int> nodeRowMap_;
};

}  // namespace dar
