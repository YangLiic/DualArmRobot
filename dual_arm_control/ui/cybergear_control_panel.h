#pragma once
#include "models/common_types.h"
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>

namespace dac {

class CyberGearControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit CyberGearControlPanel(QWidget *parent = nullptr);
    void setNodeList(const QList<uint8_t> &ids, const QMap<uint8_t, QString> &names);
    void updateCgState(uint8_t motorId, const CyberGearStatus &status);

signals:
    void enableRequested(uint8_t motorId);
    void stopRequested(uint8_t motorId);
    void speedModeRequested(uint8_t motorId);
    void positionModeRequested(uint8_t motorId);
    void speedRequested(uint8_t motorId, float radS);
    void positionDegRequested(uint8_t motorId, float deg);
    void setZeroRequested(uint8_t motorId);
    void goToZeroRequested(uint8_t motorId);
    void currentLimitRequested(uint8_t motorId, float amps);
    void speedLimitRequested(uint8_t motorId, float radS);
    void stopAllRequested();

private:
    uint8_t currentId() const;
    void setupUi();

    QComboBox *nodeCombo_;
    QLabel *lblPos_, *lblVel_, *lblTorque_, *lblTemp_, *lblFault_, *lblMode_, *lblOnline_;
    QDoubleSpinBox *speedInput_, *posInput_, *curLimitInput_, *spdLimitInput_;
};

} // namespace dac
