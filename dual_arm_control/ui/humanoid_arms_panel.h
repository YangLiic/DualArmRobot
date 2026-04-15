#pragma once
#include "models/common_types.h"
#include <QWidget>

class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QDoubleSpinBox;
class QVBoxLayout;

namespace dac {

class HumanoidArmsPanel : public QWidget
{
    Q_OBJECT
public:
    explicit HumanoidArmsPanel(QWidget *parent = nullptr);

    void setConnectionState(bool connected, const QString &message);
    void updateArmState(const dac::ArmState &state);

signals:
    void connectRequested(int deviceIndex, int canIndex);
    void disconnectRequested();
    void refreshRequested();
    void jointTargetsRequested(dac::ArmSide side, const QVector<double> &jointTargetsDeg, double velocity);
    void brakeRequested(dac::ArmSide side);
    void clearErrorsRequested(dac::ArmSide side);

private:
    struct ArmWidgets {
        QLabel *stateLabel = nullptr;
        QLabel *poseLabel = nullptr;
        QLabel *errorLabel = nullptr;
        QLabel *lastUpdateLabel = nullptr;
        QPushButton *syncBtn = nullptr;
        QPushButton *clearErrorsBtn = nullptr;
        QPushButton *brakeBtn = nullptr;
        QVector<QSlider*> sliders;
        QVector<QLabel*> targetLabels;
        QVector<QLabel*> actualLabels;
        QVector<double> targetDegrees;
        QVector<double> actualDegrees;
        bool stateReceived = false;
        bool targetsInitialized = false;
    };

    void buildConnectionGroup(QVBoxLayout *layout);
    void buildArmGroup(QVBoxLayout *layout, dac::ArmSide side);
    ArmWidgets &widgetsForSide(dac::ArmSide side);
    const ArmWidgets &widgetsForSide(dac::ArmSide side) const;
    void syncTargetsFromActual(dac::ArmSide side);
    void updateArmStateText(dac::ArmSide side);
    void emitJointTargets(dac::ArmSide side);
    static QString poseText(const QVector<double> &pose);
    static QString errorText(const dac::ArmState &state);

    bool connected_ = false;

    QSpinBox *deviceIndexSpin_ = nullptr;
    QSpinBox *canIndexSpin_ = nullptr;
    QDoubleSpinBox *velocitySpin_ = nullptr;
    QPushButton *connectBtn_ = nullptr;
    QPushButton *disconnectBtn_ = nullptr;
    QPushButton *refreshBtn_ = nullptr;
    QLabel *sdkStateLabel_ = nullptr;

    ArmWidgets leftArm_;
    ArmWidgets rightArm_;
};

} // namespace dac
