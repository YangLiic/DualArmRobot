#pragma once
#include <QWidget>
#include <QColor>

namespace dac {

/*
 * TorqueGaugeWidget: 实时扭矩条状监控控件。
 *
 * 以更紧凑的横向条形样式显示当前扭矩、碰撞阈值、
 * 历史峰值和在线/使能/保护状态，适合在监控面板中并排展示。
 */
class TorqueGaugeWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(int torqueValue READ torqueValue WRITE setTorqueValue)
public:
    explicit TorqueGaugeWidget(QWidget *parent = nullptr);

    void setNodeLabel(const QString &label);
    void setMaxRange(int maxPermille);
    void setThreshold(int thresholdPermille);

    int  torqueValue() const { return torque_; }

public slots:
    void setTorqueValue(int permille);
    void setOnline(bool online);
    void setEnabled(bool enabled);
    void setCollisionTriggered(bool triggered);
    void setCollisionProtection(bool on);

protected:
    void paintEvent(QPaintEvent *event) override;
    QSize sizeHint() const override { return QSize(420, 132); }
    QSize minimumSizeHint() const override { return QSize(300, 112); }

private:
    void drawSurface(QPainter &p, const QRectF &rect);
    void drawHeader(QPainter &p, const QRectF &rect);
    void drawBar(QPainter &p, const QRectF &rect);
    void drawFooter(QPainter &p, const QRectF &rect);
    void drawStatusPill(QPainter &p, const QRectF &rect, const QString &text,
                        const QColor &activeColor, bool active) const;
    QColor accentColor() const;

    QString nodeLabel_;
    int     torque_ = 0;          // 当前扭矩
    int     peakTorque_ = 0;      // 峰值
    int     maxRange_ = 5000;     // 满量程
    int     threshold_ = 2000;    // 碰撞阈值
    bool    online_ = false;
    bool    enabled_ = false;
    bool    collisionTriggered_ = false;
    bool    collisionProtection_ = false;
};

} // namespace dac
