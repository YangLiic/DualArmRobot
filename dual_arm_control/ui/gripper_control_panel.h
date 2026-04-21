#pragma once
#include "models/common_types.h"
#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QStackedWidget;

namespace dac {

class GripperControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit GripperControlPanel(QWidget *parent = nullptr);

    void setGripperState(const dac::GripperState &state);
    void setConnectionState(dac::ArmSide side, bool connected, const QString &message);

signals:
    void connectRequested(dac::ArmSide side, const dac::GripperEndpointConfig &cfg);
    void disconnectRequested(dac::ArmSide side);
    void targetRequested(dac::ArmSide side, double normalized);

public slots:
    void refreshSerialPorts();

private:
    struct EndpointUi {
        dac::ArmSide side = dac::ArmSide::Left;
        QLabel *statusLabel = nullptr;
        QLabel *actualLabel = nullptr;
        QLabel *targetLabel = nullptr;
        QLabel *detailLabel = nullptr;
        QLabel *updatedLabel = nullptr;

        QComboBox *transportCombo = nullptr;
        QStackedWidget *transportStack = nullptr;
        QLineEdit *ipEdit = nullptr;
        QSpinBox *tcpPortSpin = nullptr;
        QComboBox *serialPortCombo = nullptr;
        QSpinBox *baudSpin = nullptr;
        QSpinBox *deviceIdSpin = nullptr;
        QSpinBox *speedSpin = nullptr;
        QSpinBox *forceSpin = nullptr;
        QDoubleSpinBox *commandSpin = nullptr;
        QSlider *commandSlider = nullptr;
        QPushButton *connectBtn = nullptr;
        QPushButton *disconnectBtn = nullptr;
        QPushButton *sendBtn = nullptr;
    };

    void setupUi();
    QWidget *buildEndpointCard(const QString &title, EndpointUi &ui, const GripperEndpointConfig &defaults);
    GripperEndpointConfig configFromUi(const EndpointUi &ui) const;
    EndpointUi &endpointUi(dac::ArmSide side);
    void applyTransportUi(EndpointUi &ui);
    void setCommandValue(EndpointUi &ui, double normalized);

    EndpointUi leftUi_;
    EndpointUi rightUi_;
};

} // namespace dac
