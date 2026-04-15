#pragma once
#include "models/common_types.h"
#include <QWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>

namespace dac {

class MotorControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit MotorControlPanel(QWidget *parent = nullptr);

    void setNodeList(const QList<uint32_t> &nodeIds, const QMap<uint32_t, QString> &names);
    void updateMotorState(uint32_t nodeId, const MotorState &state);

signals:
    void enableRequested(uint32_t nodeId, dac::OperationMode mode);
    void disableRequested(uint32_t nodeId);
    void emergencyStopRequested(uint32_t nodeId);
    void faultResetRequested(uint32_t nodeId);
    void velocityRequested(uint32_t nodeId, int32_t rpm);
    void positionRequested(uint32_t nodeId, double degrees, bool absolute);
    void profileVelocityRequested(uint32_t nodeId, uint32_t rpm);
    void releaseBrakeRequested(uint32_t nodeId);
    void lockBrakeRequested(uint32_t nodeId);

    // 批量
    void enableAllRequested(dac::OperationMode mode);
    void disableAllRequested();
    void emergencyStopAllRequested();
    void faultResetAllRequested();

private:
    uint32_t currentNodeId() const;
    void buildNodeSelector(QVBoxLayout *layout);
    void buildModeAndEnable(QVBoxLayout *layout);
    void buildVelocityControl(QVBoxLayout *layout);
    void buildPositionControl(QVBoxLayout *layout);
    void buildBatchButtons(QVBoxLayout *layout);
    void buildBrakeControl(QVBoxLayout *layout);

    QComboBox    *nodeCombo_;
    QComboBox    *modeCombo_;
    QPushButton  *enableBtn_;
    QPushButton  *disableBtn_;
    QPushButton  *estopBtn_;
    QPushButton  *faultResetBtn_;
    QSpinBox     *velocitySpin_;
    QPushButton  *velGoBtn_;
    QPushButton  *velStopBtn_;
    QDoubleSpinBox *positionSpin_;
    QSpinBox     *profileVelSpin_;
    QComboBox    *posTypeCombo_;
    QPushButton  *posGoBtn_;
    QPushButton  *releaseBrakeBtn_;
    QPushButton  *lockBrakeBtn_;

    QLabel       *stateLabel_;

    QMap<uint32_t, QString> nodeNames_;
};

} // namespace dac
