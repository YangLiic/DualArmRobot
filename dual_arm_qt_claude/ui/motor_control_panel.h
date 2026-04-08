/**
 * MotorControlPanel - 电机控制面板
 *
 * 提供使能/失能/急停/速度/位置控制的交互界面
 */
#pragma once

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QGroupBox>
#include <QRadioButton>
#include "../models/common_types.h"

namespace dar {

class MotorService;

class MotorControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit MotorControlPanel(MotorService *motorService, QWidget *parent = nullptr);

    void refreshMotorList();

public slots:
    void onMotorStateChanged(uint32_t nodeId, const dar::MotorState &state);

private slots:
    void onMotorSelected(int index);
    void onEnableClicked();
    void onDisableClicked();
    void onQuickStopClicked();
    void onFaultResetClicked();
    void onSetVelocity();
    void onVelocitySliderChanged(int value);
    void onSetPosition();
    void onStopVelocity();
    void onEnableAllClicked();
    void onDisableAllClicked();
    void onQuickStopAllClicked();
    void onFaultResetAllClicked();
    void onReleaseBrakeClicked();
    void onLockBrakeClicked();

private:
    void setupUi();
    uint32_t currentNodeId() const;

    MotorService *motorService_;

    // 节点选择
    QComboBox *motorCombo_;
    QLabel *stateLabel_;
    QLabel *modeLabel_;

    // 基础控制
    QRadioButton *velocityModeRadio_;
    QRadioButton *positionModeRadio_;
    QPushButton *enableBtn_;
    QPushButton *disableBtn_;
    QPushButton *quickStopBtn_;
    QPushButton *faultResetBtn_;

    // 速度控制
    QSlider *velocitySlider_;
    QSpinBox *velocitySpin_;
    QPushButton *setVelocityBtn_;
    QPushButton *stopVelocityBtn_;

    // 位置控制
    QDoubleSpinBox *positionSpin_;
    QRadioButton *relativeRadio_;
    QRadioButton *absoluteRadio_;
    QPushButton *setPositionBtn_;
    QSpinBox *profileVelSpin_;
    QSpinBox *profileAccSpin_;
    QSpinBox *profileDecSpin_;

    // 批量
    QPushButton *enableAllBtn_;
    QPushButton *disableAllBtn_;
    QPushButton *quickStopAllBtn_;
    QPushButton *faultResetAllBtn_;

    // 抱闸
    QPushButton *releaseBrakeBtn_;
    QPushButton *lockBrakeBtn_;
};

}  // namespace dar
