#include "ui/torque_gauge_widget.h"
#include <QPainter>
#include <QPainterPath>
#include <QtMath>
#include <algorithm>

namespace dac {

TorqueGaugeWidget::TorqueGaugeWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(minimumSizeHint());
}

void TorqueGaugeWidget::setNodeLabel(const QString &label) { nodeLabel_ = label; update(); }
void TorqueGaugeWidget::setMaxRange(int m) { maxRange_ = qMax(1, m); update(); }
void TorqueGaugeWidget::setThreshold(int t) { threshold_ = t; update(); }

void TorqueGaugeWidget::setTorqueValue(int permille)
{
    torque_ = permille;
    int abs_t = qAbs(permille);
    if (abs_t > peakTorque_) peakTorque_ = abs_t;
    update();
}

void TorqueGaugeWidget::setOnline(bool v) { online_ = v; update(); }
void TorqueGaugeWidget::setEnabled(bool v) { enabled_ = v; update(); }
void TorqueGaugeWidget::setCollisionTriggered(bool v) { collisionTriggered_ = v; update(); }
void TorqueGaugeWidget::setCollisionProtection(bool v) { collisionProtection_ = v; update(); }

double TorqueGaugeWidget::valueToAngle(int val) const
{
    double ratio = qBound(0.0, static_cast<double>(qAbs(val)) / maxRange_, 1.0);
    return ARC_START + ratio * ARC_SPAN;
}

void TorqueGaugeWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    int side = qMin(width(), height() - 30);
    QRectF arcRect((width() - side) / 2.0 + 10, 10, side - 20, side - 20);

    drawBackground(p, arcRect);
    drawArc(p, arcRect);
    drawNeedle(p, arcRect);
    drawCenter(p, arcRect);
    drawLabels(p, arcRect);
    drawStatusLeds(p, arcRect);
}

void TorqueGaugeWidget::drawBackground(QPainter &p, const QRectF &rect)
{
    p.fillRect(this->rect(), QColor("#1e1e2e"));

    // 外圈
    QPen pen(QColor("#45475a"), 2);
    p.setPen(pen);
    p.setBrush(QColor("#181825"));
    p.drawEllipse(rect);
}

void TorqueGaugeWidget::drawArc(QPainter &p, const QRectF &rect)
{
    QRectF arcR = rect.adjusted(8, 8, -8, -8);
    double arcW = 12.0;

    // 底色弧（灰）
    QPen bgPen(QColor("#313244"), arcW, Qt::SolidLine, Qt::RoundCap);
    p.setPen(bgPen);
    p.drawArc(arcR, static_cast<int>(ARC_START * 16), static_cast<int>(ARC_SPAN * 16));

    // 安全区（绿色渐变到黄色）
    double threshRatio = qBound(0.0, static_cast<double>(threshold_) / maxRange_, 1.0);
    double safeSpan = threshRatio * ARC_SPAN;
    QPen safePen(QColor("#a6e3a1"), arcW, Qt::SolidLine, Qt::RoundCap);
    p.setPen(safePen);
    p.drawArc(arcR, static_cast<int>(ARC_START * 16), static_cast<int>(safeSpan * 16));

    // 危险区（红色）
    double dangerStart = ARC_START + safeSpan;
    double dangerSpan = ARC_SPAN - safeSpan;
    QPen dangerPen(QColor("#f38ba8"), arcW, Qt::SolidLine, Qt::RoundCap);
    p.setPen(dangerPen);
    p.drawArc(arcR, static_cast<int>(dangerStart * 16), static_cast<int>(dangerSpan * 16));

    // 当前值弧
    int abs_t = qAbs(torque_);
    if (abs_t > 0) {
        double valRatio = qBound(0.0, static_cast<double>(abs_t) / maxRange_, 1.0);
        double valSpan = valRatio * ARC_SPAN;
        QColor valColor = (abs_t >= threshold_) ? QColor("#f38ba8") : QColor("#89b4fa");
        if (collisionTriggered_) valColor = QColor("#f38ba8");
        QPen valPen(valColor, arcW + 2, Qt::SolidLine, Qt::RoundCap);
        p.setPen(valPen);
        p.drawArc(arcR.adjusted(1, 1, -1, -1),
                   static_cast<int>(ARC_START * 16),
                   static_cast<int>(valSpan * 16));
    }
}

void TorqueGaugeWidget::drawNeedle(QPainter &p, const QRectF &rect)
{
    QPointF center = rect.center();
    double radius = rect.width() / 2.0 - 22;

    double angle = valueToAngle(torque_);
    double rad = qDegreesToRadians(angle);

    QPointF tip(center.x() + radius * qCos(rad), center.y() - radius * qSin(rad));

    QPen needlePen(collisionTriggered_ ? QColor("#f38ba8") : QColor("#cdd6f4"), 2.5, Qt::SolidLine, Qt::RoundCap);
    p.setPen(needlePen);
    p.drawLine(center, tip);

    // 峰值标记
    if (peakTorque_ > 0) {
        double peakAngle = valueToAngle(peakTorque_);
        double peakRad = qDegreesToRadians(peakAngle);
        double outerR = radius + 4;
        QPointF peakPt(center.x() + outerR * qCos(peakRad), center.y() - outerR * qSin(peakRad));
        p.setPen(Qt::NoPen);
        p.setBrush(QColor("#f9e2af"));
        p.drawEllipse(peakPt, 3, 3);
    }
}

void TorqueGaugeWidget::drawCenter(QPainter &p, const QRectF &rect)
{
    QPointF center = rect.center();

    // 中心圆
    p.setPen(Qt::NoPen);
    p.setBrush(QColor("#313244"));
    p.drawEllipse(center, 18, 18);

    // 方向箭头
    QString dir = (torque_ >= 0) ? "+" : "-";
    p.setPen(QColor("#cdd6f4"));
    QFont f = font();
    f.setPixelSize(14);
    f.setBold(true);
    p.setFont(f);
    p.drawText(QRectF(center.x() - 10, center.y() - 8, 20, 16), Qt::AlignCenter, dir);
}

void TorqueGaugeWidget::drawLabels(QPainter &p, const QRectF &rect)
{
    // 标题
    QFont titleFont = font();
    titleFont.setPixelSize(13);
    titleFont.setBold(true);
    p.setFont(titleFont);
    p.setPen(QColor("#cdd6f4"));
    p.drawText(QRectF(rect.left(), rect.bottom() - 6, rect.width(), 20), Qt::AlignCenter, nodeLabel_);

    // 数值
    QFont valFont = font();
    valFont.setPixelSize(18);
    valFont.setBold(true);
    p.setFont(valFont);

    int abs_t = qAbs(torque_);
    QColor numColor = (abs_t >= threshold_) ? QColor("#f38ba8") : QColor("#cdd6f4");
    if (collisionTriggered_) numColor = QColor("#f38ba8");
    p.setPen(numColor);

    QString valStr = QStringLiteral("%1‰ (%2%)")
                         .arg(torque_)
                         .arg(abs_t / 10.0, 0, 'f', 1);
    QPointF center = rect.center();
    p.drawText(QRectF(rect.left(), center.y() + 22, rect.width(), 22), Qt::AlignCenter, valStr);

    // 峰值
    QFont smallFont = font();
    smallFont.setPixelSize(10);
    p.setFont(smallFont);
    p.setPen(QColor("#7f849c"));
    QString peakStr = QStringLiteral("峰值: %1‰").arg(peakTorque_);
    p.drawText(QRectF(rect.left(), center.y() + 44, rect.width(), 14), Qt::AlignCenter, peakStr);

    // 碰撞触发闪烁文字
    if (collisionTriggered_) {
        QFont warnFont = font();
        warnFont.setPixelSize(14);
        warnFont.setBold(true);
        p.setFont(warnFont);
        p.setPen(QColor("#f38ba8"));
        p.drawText(QRectF(rect.left(), rect.bottom() + 12, rect.width(), 20),
                   Qt::AlignCenter, QStringLiteral("碰撞触发!"));
    }
}

void TorqueGaugeWidget::drawStatusLeds(QPainter &p, const QRectF &rect)
{
    double ledY = rect.top() + 6;
    double ledX = rect.right() - 50;
    double ledR = 5;
    double gap = 14;

    auto drawLed = [&](double x, double y, const QColor &color, bool on) {
        p.setPen(Qt::NoPen);
        p.setBrush(on ? color : QColor("#45475a"));
        p.drawEllipse(QPointF(x, y), ledR, ledR);
    };

    // 在线
    drawLed(ledX, ledY, QColor("#a6e3a1"), online_);
    p.setPen(QColor("#7f849c"));
    QFont ledFont = font();
    ledFont.setPixelSize(9);
    p.setFont(ledFont);
    p.drawText(QPointF(ledX + 8, ledY + 3), QStringLiteral("在线"));

    // 使能
    drawLed(ledX, ledY + gap, QColor("#89b4fa"), enabled_);
    p.drawText(QPointF(ledX + 8, ledY + gap + 3), QStringLiteral("使能"));

    // 碰撞保护
    drawLed(ledX, ledY + gap * 2, QColor("#f9e2af"), collisionProtection_);
    p.drawText(QPointF(ledX + 8, ledY + gap * 2 + 3), QStringLiteral("保护"));
}

} // namespace dac
