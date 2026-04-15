#pragma once
#include "models/common_types.h"
#include "ui/torque_gauge_widget.h"
#include <QWidget>
#include <QLabel>
#include <QMap>
#include <QGroupBox>
#include <QProgressBar>
#include <QHBoxLayout>

namespace dac {

/*
 * StatusMonitorPanel: 实时状态监控面板。
 *
 * 同屏显示两个电机的扭矩仪表盘 + 碰撞检测状态
 * + 各种实时数据(在线/使能/故障/扭矩/电流/碰撞)。
 */
class StatusMonitorPanel : public QWidget
{
    Q_OBJECT
public:
    explicit StatusMonitorPanel(QWidget *parent = nullptr);

    void addMotorGauge(uint32_t nodeId, const QString &label,
                       int maxRange, int threshold);

public slots:
    void updateMotorState(uint32_t nodeId, const dac::MotorState &state);
    void onTorqueUpdated(uint32_t nodeId, int16_t torquePermille);
    void onCollisionDetected(uint32_t nodeId, int16_t torquePermille);

private:
    struct MotorPanel {
        TorqueGaugeWidget *gauge = nullptr;
        QLabel *onlineLabel = nullptr;
        QLabel *enabledLabel = nullptr;
        QLabel *modeLabel = nullptr;
        QLabel *torqueLabel = nullptr;
        QLabel *collisionLabel = nullptr;
        QLabel *faultLabel = nullptr;
        QProgressBar *torqueBar = nullptr;
        int thresholdPermille = 0;
    };

    QWidget *createMotorCard(uint32_t nodeId, const QString &label,
                             int maxRange, int threshold);
    void updateCard(MotorPanel &panel, const MotorState &state);

    QMap<uint32_t, MotorPanel> panels_;
    QHBoxLayout *gaugeLayout_;
};

} // namespace dac
