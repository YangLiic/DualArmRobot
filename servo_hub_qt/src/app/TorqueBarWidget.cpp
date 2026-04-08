#include "app/TorqueBarWidget.h"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

TorqueBarWidget::TorqueBarWidget(QWidget *parent)
    : QWidget(parent)
{
    setMinimumHeight(90);
}

void TorqueBarWidget::setThresholdPermille(qint16 thresholdPermille)
{
    const qint16 safeValue = qMax<qint16>(1, qAbs(thresholdPermille));
    if (m_thresholdPermille == safeValue) {
        return;
    }
    m_thresholdPermille = safeValue;
    update();
}

void TorqueBarWidget::setTorquePermille(qint16 torquePermille)
{
    m_torquePermille = torquePermille;
    m_history.append(torquePermille);
    if (m_history.size() > 120) {
        m_history.remove(0, m_history.size() - 120);
    }
    update();
}

void TorqueBarWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF outer = rect().adjusted(10, 10, -10, -16);
    const QRectF trendRect(outer.left(), outer.top(), outer.width(), 34);
    const QRectF barRect(outer.left(), trendRect.bottom() + 10, outer.width(), 24);
    const qreal centerX = barRect.center().x();
    const qreal halfWidth = barRect.width() / 2.0;
    const qreal safeThreshold = qMax<qreal>(1.0, m_thresholdPermille);
    qreal normalized = qBound(-1.0, static_cast<qreal>(m_torquePermille) / safeThreshold, 1.0);
    if (m_torquePermille != 0) {
        const qreal sign = normalized >= 0 ? 1.0 : -1.0;
        normalized = sign * qMax(0.03, std::sqrt(std::abs(normalized)));
    }
    const qreal currentX = centerX + normalized * halfWidth;

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#f7f3ec"));
    painter.drawRoundedRect(trendRect, 10, 10);

    painter.setPen(QPen(QColor("#c7bba7"), 1));
    painter.drawLine(QPointF(trendRect.left(), trendRect.center().y()), QPointF(trendRect.right(), trendRect.center().y()));

    qint16 historyPeak = 20;
    for (qint16 value : std::as_const(m_history)) {
        historyPeak = qMax<qint16>(historyPeak, qAbs(value));
    }

    if (!m_history.isEmpty()) {
        QPainterPath path;
        for (int i = 0; i < m_history.size(); ++i) {
            const qreal x = trendRect.left() + (trendRect.width() * i) / qMax(1, m_history.size() - 1);
            const qreal y = trendRect.center().y() - (static_cast<qreal>(m_history.at(i)) / historyPeak) * (trendRect.height() * 0.42);
            if (i == 0) {
                path.moveTo(x, y);
            } else {
                path.lineTo(x, y);
            }
        }
        painter.setPen(QPen(QColor("#0f766e"), 2));
        painter.drawPath(path);
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#ece4d7"));
    painter.drawRoundedRect(barRect, 12, 12);

    painter.setBrush(QColor("#cfe6df"));
    painter.drawRoundedRect(QRectF(barRect.left(), barRect.top(), halfWidth, barRect.height()), 12, 12);

    painter.setBrush(QColor("#f3d8d2"));
    painter.drawRoundedRect(QRectF(centerX, barRect.top(), halfWidth, barRect.height()), 12, 12);

    painter.setPen(QPen(QColor("#5b6470"), 2));
    painter.drawLine(QPointF(centerX, barRect.top() - 6), QPointF(centerX, barRect.bottom() + 6));

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(m_torquePermille >= 0 ? "#7cc5b8" : "#e7a29b"));
    const QRectF fillRect = m_torquePermille >= 0
        ? QRectF(centerX, barRect.top(), qMax<qreal>(0.0, currentX - centerX), barRect.height())
        : QRectF(currentX, barRect.top(), qMax<qreal>(0.0, centerX - currentX), barRect.height());
    painter.drawRoundedRect(fillRect, 12, 12);

    painter.setPen(QPen(QColor("#1f2937"), 2));
    painter.setBrush(QColor(normalized >= 0 ? "#0f766e" : "#b91c1c"));
    painter.drawEllipse(QPointF(currentX, barRect.center().y()), 9, 9);

    painter.setPen(QColor("#374151"));
    painter.drawText(QRectF(trendRect.left(), trendRect.top() - 2, trendRect.width(), 14), Qt::AlignCenter,
                     QStringLiteral("实时趋势（自动放大显示）"));
    painter.drawText(QRectF(barRect.left(), barRect.top() - 22, barRect.width(), 18), Qt::AlignCenter,
                     QStringLiteral("负向转矩                          当前扭矩                          正向转矩"));

    painter.drawText(QRectF(barRect.left(), barRect.bottom() + 8, 100, 18), Qt::AlignLeft,
                     QStringLiteral("-%1‰").arg(m_thresholdPermille));
    painter.drawText(QRectF(centerX - 50, barRect.bottom() + 8, 100, 18), Qt::AlignCenter,
                     QStringLiteral("0"));
    painter.drawText(QRectF(barRect.right() - 100, barRect.bottom() + 8, 100, 18), Qt::AlignRight,
                     QStringLiteral("+%1‰").arg(m_thresholdPermille));
}

QSize TorqueBarWidget::minimumSizeHint() const
{
    return QSize(280, 90);
}
