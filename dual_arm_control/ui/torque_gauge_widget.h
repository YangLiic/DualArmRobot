#pragma once
#include <QWidget>
#include <QTimer>
#include <QColor>

namespace dac {

/*
 * TorqueGaugeWidget: 实时扭矩仪表盘控件。
 *
 * 圆弧仪表样式，显示当前扭矩值(千分比)、碰撞阈值区域、
 * 历史峰值标记、在线/使能/碰撞状态指示。
 * 仿照 interactive_control_cpp.py 中的实时扭矩条形图，
 * 升级为 Qt 圆弧仪表盘。
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
    QSize sizeHint() const override { return QSize(220, 240); }
    QSize minimumSizeHint() const override { return QSize(160, 180); }

private:
    void drawBackground(QPainter &p, const QRectF &rect);
    void drawArc(QPainter &p, const QRectF &rect);
    void drawNeedle(QPainter &p, const QRectF &rect);
    void drawCenter(QPainter &p, const QRectF &rect);
    void drawLabels(QPainter &p, const QRectF &rect);
    void drawStatusLeds(QPainter &p, const QRectF &rect);
    double valueToAngle(int val) const;

    QString nodeLabel_;
    int     torque_ = 0;          // 当前扭矩
    int     peakTorque_ = 0;      // 峰值
    int     maxRange_ = 5000;     // 满量程
    int     threshold_ = 2000;    // 碰撞阈值
    bool    online_ = false;
    bool    enabled_ = false;
    bool    collisionTriggered_ = false;
    bool    collisionProtection_ = false;

    static constexpr double ARC_START = 225.0;
    static constexpr double ARC_SPAN  = -270.0;
};

} // namespace dac
