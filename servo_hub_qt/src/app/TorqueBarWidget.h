#pragma once

#include <QVector>
#include <QWidget>

class TorqueBarWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TorqueBarWidget(QWidget *parent = nullptr);

    void setThresholdPermille(qint16 thresholdPermille);
    void setTorquePermille(qint16 torquePermille);

protected:
    void paintEvent(QPaintEvent *event) override;
    QSize minimumSizeHint() const override;

private:
    qint16 m_thresholdPermille = 500;
    qint16 m_torquePermille = 0;
    QVector<qint16> m_history;
};
