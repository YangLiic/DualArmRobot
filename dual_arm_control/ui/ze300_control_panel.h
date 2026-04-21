#pragma once
#include "models/common_types.h"
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>

namespace dac {

class Ze300ControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit Ze300ControlPanel(QWidget *parent = nullptr);

    void setNodeList(const QList<uint16_t> &addrs, const QMap<uint16_t, QString> &names);
    void updateZe300State(uint16_t devAddr, const Ze300Status &status);

signals:
    // 控制
    void speedRequested(uint16_t devAddr, float rpm);
    void absPositionRequested(uint16_t devAddr, float deg);
    void relPositionRequested(uint16_t devAddr, float deg);
    void goOriginRequested(uint16_t devAddr);
    void setZeroRequested(uint16_t devAddr);
    void freeOutputRequested(uint16_t devAddr);
    void clearFaultRequested(uint16_t devAddr);
    void rebootRequested(uint16_t devAddr);
    // 抱闸
    void brakeCloseRequested(uint16_t devAddr);
    void brakeOpenRequested(uint16_t devAddr);
    void brakeReadRequested(uint16_t devAddr);
    // 参数
    void posMaxSpeedRequested(uint16_t devAddr, float rpm);
    void maxCurrentRequested(uint16_t devAddr, float currentA);
    void speedAccelRequested(uint16_t devAddr, float accRpmS);
    // MIT
    void mitControlRequested(uint16_t devAddr, float posRad, float velRadS,
                             float kp, float kd, float torqueNm);
    // 批量
    void freeOutputAllRequested();

private:
    uint16_t currentAddr() const;
    void setupUi();

    QComboBox      *nodeCombo_;

    // 状态显示
    QLabel *lblTemperature_;
    QLabel *lblQCurrent_;
    QLabel *lblSpeed_;
    QLabel *lblPosition_;
    QLabel *lblMultiTurn_;
    QLabel *lblVoltage_;
    QLabel *lblBusCurrent_;
    QLabel *lblRunMode_;
    QLabel *lblFault_;
    QLabel *lblBrake_;
    QLabel *lblOnline_;

    // 控制输入
    QDoubleSpinBox *speedInput_;
    QDoubleSpinBox *absPosInput_;
    QDoubleSpinBox *relPosInput_;

    // MIT 输入
    QDoubleSpinBox *mitPosInput_;
    QDoubleSpinBox *mitVelInput_;
    QDoubleSpinBox *mitKpInput_;
    QDoubleSpinBox *mitKdInput_;
    QDoubleSpinBox *mitTorqueInput_;

    // 参数输入
    QDoubleSpinBox *posMaxSpeedInput_;
    QDoubleSpinBox *maxCurrentInput_;
    QDoubleSpinBox *speedAccelInput_;
};

} // namespace dac
