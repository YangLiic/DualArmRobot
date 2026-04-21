#include "ui/torque_gauge_widget.h"
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QSizePolicy>

namespace dac {

namespace {

constexpr qreal kOuterRadius = 16.0;
constexpr qreal kInnerRadius = 11.0;

} // namespace

TorqueGaugeWidget::TorqueGaugeWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(minimumSizeHint());
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setAttribute(Qt::WA_TranslucentBackground);
}

void TorqueGaugeWidget::setNodeLabel(const QString &label)
{
    nodeLabel_ = label;
    update();
}

void TorqueGaugeWidget::setMaxRange(int maxPermille)
{
    maxRange_ = qMax(1, maxPermille);
    update();
}

void TorqueGaugeWidget::setThreshold(int thresholdPermille)
{
    threshold_ = qBound(0, thresholdPermille, maxRange_);
    update();
}

void TorqueGaugeWidget::setTorqueValue(int permille)
{
    torque_ = permille;
    peakTorque_ = qMax(peakTorque_, qAbs(permille));
    update();
}

void TorqueGaugeWidget::setOnline(bool online)
{
    online_ = online;
    update();
}

void TorqueGaugeWidget::setEnabled(bool enabled)
{
    enabled_ = enabled;
    update();
}

void TorqueGaugeWidget::setCollisionTriggered(bool triggered)
{
    collisionTriggered_ = triggered;
    update();
}

void TorqueGaugeWidget::setCollisionProtection(bool on)
{
    collisionProtection_ = on;
    update();
}

QColor TorqueGaugeWidget::accentColor() const
{
    const int absTorque = qAbs(torque_);
    if (collisionTriggered_) {
        return QColor("#c8604f");
    }
    if (absTorque >= threshold_) {
        return QColor("#c8844c");
    }
    return QColor("#577fa7");
}

void TorqueGaugeWidget::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const QRectF outerRect = rect().adjusted(1, 1, -1, -1);
    const QRectF contentRect = outerRect.adjusted(14, 12, -14, -12);

    drawSurface(p, outerRect);
    drawHeader(p, contentRect);
    drawBar(p, contentRect);
    drawFooter(p, contentRect);
}

void TorqueGaugeWidget::drawSurface(QPainter &p, const QRectF &rect)
{
    QLinearGradient surface(rect.topLeft(), rect.bottomLeft());
    surface.setColorAt(0.0, QColor("#fffdfa"));
    surface.setColorAt(1.0, QColor("#f3ece3"));

    QPainterPath path;
    path.addRoundedRect(rect, kOuterRadius, kOuterRadius);

    p.setPen(Qt::NoPen);
    p.fillPath(path, surface);

    p.setPen(QPen(QColor("#d8cfc1"), 1.0));
    p.drawPath(path);

    QPainterPath highlight;
    highlight.addRoundedRect(rect.adjusted(1.5, 1.5, -1.5, -rect.height() * 0.46), kOuterRadius, kOuterRadius);
    p.fillPath(highlight, QColor(255, 255, 255, 48));
}

void TorqueGaugeWidget::drawHeader(QPainter &p, const QRectF &rect)
{
    const QColor titleColor("#2b3b4c");
    const QColor mutedColor("#718096");
    const QColor valueColor = accentColor();
    const int absTorque = qAbs(torque_);
    const QString direction = torque_ >= 0 ? QStringLiteral("+") : QStringLiteral("-");

    QFont titleFont = font();
    titleFont.setPixelSize(15);
    titleFont.setBold(true);
    p.setFont(titleFont);
    p.setPen(titleColor);
    p.drawText(QRectF(rect.left(), rect.top(), rect.width() * 0.58, 22),
               Qt::AlignLeft | Qt::AlignVCenter, nodeLabel_);

    QFont valueFont = font();
    valueFont.setPixelSize(18);
    valueFont.setBold(true);
    p.setFont(valueFont);
    p.setPen(valueColor);
    p.drawText(QRectF(rect.left() + rect.width() * 0.58, rect.top(), rect.width() * 0.42, 22),
               Qt::AlignRight | Qt::AlignVCenter,
               QStringLiteral("%1%2‰").arg(direction).arg(absTorque));

    QFont subFont = font();
    subFont.setPixelSize(11);
    p.setFont(subFont);
    p.setPen(mutedColor);
    p.drawText(QRectF(rect.left() + rect.width() * 0.58, rect.top() + 22, rect.width() * 0.42, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QStringLiteral("额定 %1%").arg(absTorque / 10.0, 0, 'f', 1));

    const qreal pillY = rect.top() + 34.0;
    const qreal pillH = 22.0;
    qreal x = rect.left();
    drawStatusPill(p, QRectF(x, pillY, 74, pillH), QStringLiteral("在线"),
                   QColor("#6f9b78"), online_);
    x += 82.0;
    drawStatusPill(p, QRectF(x, pillY, 74, pillH), QStringLiteral("使能"),
                   QColor("#5f86af"), enabled_);
    x += 82.0;
    drawStatusPill(p, QRectF(x, pillY, 92, pillH), QStringLiteral("保护"),
                   collisionTriggered_ ? QColor("#c8604f") : QColor("#a88553"),
                   collisionProtection_ || collisionTriggered_);
}

void TorqueGaugeWidget::drawBar(QPainter &p, const QRectF &rect)
{
    const int absTorque = qAbs(torque_);
    const double valueRatio = qBound(0.0, static_cast<double>(absTorque) / maxRange_, 1.0);
    const double thresholdRatio = qBound(0.0, static_cast<double>(threshold_) / maxRange_, 1.0);
    const double peakRatio = qBound(0.0, static_cast<double>(peakTorque_) / maxRange_, 1.0);

    const QRectF trackRect(rect.left(), rect.top() + 64.0, rect.width(), 18.0);

    QPainterPath trackPath;
    trackPath.addRoundedRect(trackRect, 9.0, 9.0);
    p.setPen(Qt::NoPen);
    p.fillPath(trackPath, QColor("#e5ddd0"));

    QPainterPath safePath;
    safePath.addRoundedRect(QRectF(trackRect.left(), trackRect.top(),
                                   trackRect.width() * thresholdRatio, trackRect.height()),
                            9.0, 9.0);
    p.fillPath(safePath, QColor("#d7e0d3"));

    if (thresholdRatio < 1.0) {
        QPainterPath riskPath;
        riskPath.addRoundedRect(QRectF(trackRect.left() + trackRect.width() * thresholdRatio,
                                       trackRect.top(),
                                       trackRect.width() * (1.0 - thresholdRatio),
                                       trackRect.height()),
                                9.0, 9.0);
        p.fillPath(riskPath, QColor("#ead8cf"));
    }

    if (valueRatio > 0.0) {
        QRectF fillRect(trackRect.left(), trackRect.top(), trackRect.width() * valueRatio, trackRect.height());
        QLinearGradient fill(fillRect.topLeft(), fillRect.topRight());
        const QColor accent = accentColor();
        fill.setColorAt(0.0, accent.lighter(120));
        fill.setColorAt(1.0, accent.darker(105));

        QPainterPath fillPath;
        fillPath.addRoundedRect(fillRect, 9.0, 9.0);
        p.fillPath(fillPath, fill);

        p.setPen(Qt::NoPen);
        p.setBrush(accent.lighter(135));
        p.drawEllipse(QPointF(fillRect.right(), fillRect.center().y()), 5.5, 5.5);
    }

    p.setPen(QPen(QColor("#d8cfc1"), 1.0));
    p.drawRoundedRect(trackRect, 9.0, 9.0);

    const qreal thresholdX = trackRect.left() + trackRect.width() * thresholdRatio;
    p.setPen(QPen(QColor("#9b7f58"), 2.0));
    p.drawLine(QPointF(thresholdX, trackRect.top() - 5.0),
               QPointF(thresholdX, trackRect.bottom() + 5.0));

    if (peakTorque_ > 0) {
        const qreal peakX = trackRect.left() + trackRect.width() * peakRatio;
        p.setPen(QPen(QColor("#b58c4d"), 2.0, Qt::DashLine));
        p.drawLine(QPointF(peakX, trackRect.top() - 7.0),
                   QPointF(peakX, trackRect.bottom() + 7.0));
    }

    QFont scaleFont = font();
    scaleFont.setPixelSize(10);
    p.setFont(scaleFont);
    p.setPen(QColor("#7d8a97"));
    p.drawText(QRectF(trackRect.left(), trackRect.bottom() + 6.0, 42, 14),
               Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("0"));
    p.drawText(QRectF(thresholdX - 36.0, trackRect.bottom() + 6.0, 72, 14),
               Qt::AlignCenter | Qt::AlignVCenter,
               QStringLiteral("阈值 %1").arg(threshold_));
    p.drawText(QRectF(trackRect.right() - 62.0, trackRect.bottom() + 6.0, 62, 14),
               Qt::AlignRight | Qt::AlignVCenter,
               QStringLiteral("%1‰").arg(maxRange_));
}

void TorqueGaugeWidget::drawFooter(QPainter &p, const QRectF &rect)
{
    const QColor titleColor("#3e4c5d");
    const QColor mutedColor("#768392");
    const QColor warnColor("#c8604f");
    const qreal footerTop = rect.top() + 96.0;

    QFont infoFont = font();
    infoFont.setPixelSize(11);
    p.setFont(infoFont);
    p.setPen(titleColor);

    const QString directionText = torque_ >= 0 ? QStringLiteral("正向负载") : QStringLiteral("反向负载");
    p.drawText(QRectF(rect.left(), footerTop, rect.width() / 3.0, 16),
               Qt::AlignLeft | Qt::AlignVCenter, directionText);
    p.drawText(QRectF(rect.left() + rect.width() / 3.0, footerTop, rect.width() / 3.0, 16),
               Qt::AlignCenter | Qt::AlignVCenter,
               QStringLiteral("峰值 %1‰").arg(peakTorque_));
    p.drawText(QRectF(rect.left() + rect.width() * 2.0 / 3.0, footerTop, rect.width() / 3.0, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QStringLiteral("保护 %1").arg(collisionProtection_ ? QStringLiteral("已启用") : QStringLiteral("未启用")));

    p.setPen(collisionTriggered_ ? warnColor : mutedColor);
    p.drawText(QRectF(rect.left(), footerTop + 18.0, rect.width(), 16),
               Qt::AlignLeft | Qt::AlignVCenter,
               collisionTriggered_
                   ? QStringLiteral("碰撞保护已触发，请复核当前扭矩和急停状态。")
                   : QStringLiteral("条形监控会持续显示当前扭矩占比、阈值位置和历史峰值。"));
}

void TorqueGaugeWidget::drawStatusPill(QPainter &p, const QRectF &rect, const QString &text,
                                       const QColor &activeColor, bool active) const
{
    const QColor background = active ? activeColor.lighter(185) : QColor("#ece6dd");
    const QColor border = active ? activeColor.lighter(135) : QColor("#d7cdbf");
    const QColor textColor = active ? activeColor.darker(150) : QColor("#8a95a0");

    p.setPen(QPen(border, 1.0));
    p.setBrush(background);
    p.drawRoundedRect(rect, kInnerRadius, kInnerRadius);

    const QPointF dotCenter(rect.left() + 12.0, rect.center().y());
    p.setPen(Qt::NoPen);
    p.setBrush(active ? activeColor : QColor("#c8c0b6"));
    p.drawEllipse(dotCenter, 4.0, 4.0);

    QFont pillFont = font();
    pillFont.setPixelSize(10);
    pillFont.setBold(true);
    p.setFont(pillFont);
    p.setPen(textColor);
    p.drawText(QRectF(rect.left() + 20.0, rect.top(), rect.width() - 24.0, rect.height()),
               Qt::AlignLeft | Qt::AlignVCenter, text);
}

} // namespace dac
