#pragma once
#include "models/common_types.h"
#include <QWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>

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

private:
    QComboBox   *portCombo_;
    QComboBox   *baudCombo_;
    QPushButton *connectBtn_;
    QPushButton *disconnectBtn_;
    QPushButton *refreshBtn_;
    QLabel      *statusLabel_;
    bool         connected_ = false;
};

} // namespace dac
