#pragma once
#include "models/common_types.h"
#include <QWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>

namespace dac {

class ConnectionPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ConnectionPanel(QWidget *parent = nullptr);

    void setBusState(bool open, const QString &msg);

signals:
    void connectRequested(const dac::BusConfig &cfg);
    void disconnectRequested();
    void refreshPortsRequested();

public slots:
    void refreshPorts();

private slots:
    void onAdapterTypeChanged(int index);

private:
    QComboBox   *adapterCombo_;

    // Serial 参数
    QWidget     *serialParamsWidget_;
    QComboBox   *portCombo_;
    QComboBox   *baudCombo_;

    // VCI 参数
    QWidget     *vciParamsWidget_;
    QComboBox   *vciDeviceCombo_;
    QComboBox   *vciChannelCombo_;
    QComboBox   *vciBitrateCombo_;

    QStackedWidget *paramsStack_;
    QPushButton *connectBtn_;
    QPushButton *disconnectBtn_;
    QPushButton *refreshBtn_;
    QLabel      *statusLabel_;
    bool         connected_ = false;
};

} // namespace dac
