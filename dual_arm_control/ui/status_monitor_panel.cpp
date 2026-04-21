#include "ui/status_monitor_panel.h"
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <cmath>

namespace dac {

namespace {

QString metricTitleStyle()
{
    return QStringLiteral("QLabel { color: #7c8794; font-size: 11px; letter-spacing: 0.5px; }");
}

QString metricValueStyle(const QString &color, bool bold = true)
{
    return QStringLiteral("QLabel { color: %1; font-size: 12px; font-weight: %2; }")
        .arg(color)
        .arg(bold ? QStringLiteral("700") : QStringLiteral("500"));
}

} // namespace

StatusMonitorPanel::StatusMonitorPanel(QWidget *parent)
    : QWidget(parent)
{
    auto *vbox = new QVBoxLayout(this);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(14);

    auto *title = new QLabel(QStringLiteral("实时状态监控"));
    title->setStyleSheet(
        "QLabel { font-size: 18px; font-weight: 700; color: #2b3b4c; padding: 6px 4px 0 4px; }");
    vbox->addWidget(title);

    auto *subtitle = new QLabel(QStringLiteral("横向扭矩条会实时显示当前载荷、碰撞阈值和历史峰值，方便在一屏内同时观察两台电机。"));
    subtitle->setWordWrap(true);
    subtitle->setStyleSheet(
        "QLabel { color: #7b8793; font-size: 12px; padding: 0 4px 4px 4px; }");
    vbox->addWidget(subtitle);

    gaugeLayout_ = new QHBoxLayout;
    gaugeLayout_->setContentsMargins(0, 0, 0, 0);
    gaugeLayout_->setSpacing(18);
    vbox->addLayout(gaugeLayout_);
    vbox->addStretch();
}

void StatusMonitorPanel::addMotorGauge(uint32_t nodeId, const QString &label,
                                       int maxRange, int threshold)
{
    QWidget *card = createMotorCard(nodeId, label, maxRange, threshold);
    gaugeLayout_->addWidget(card, 1);
}

QWidget *StatusMonitorPanel::createMotorCard(uint32_t nodeId, const QString &label,
                                             int maxRange, int threshold)
{
    auto *card = new QGroupBox(label);
    card->setMinimumWidth(440);
    card->setStyleSheet(
        "QGroupBox {"
        "  background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #fffdfa, stop:1 #f7f2ea);"
        "  border: 1px solid #d9cfc1;"
        "  border-radius: 14px;"
        "  margin-top: 18px;"
        "  padding-top: 22px;"
        "  color: #2e4154;"
        "  font-size: 15px;"
        "  font-weight: 700;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 14px;"
        "  padding: 0 8px;"
        "  color: #5b7694;"
        "}");

    auto *vbox = new QVBoxLayout(card);
    vbox->setContentsMargins(16, 12, 16, 14);
    vbox->setSpacing(12);

    MotorPanel mp;
    mp.thresholdPermille = threshold;

    mp.gauge = new TorqueGaugeWidget;
    mp.gauge->setNodeLabel(label);
    mp.gauge->setMaxRange(maxRange);
    mp.gauge->setThreshold(threshold);
    vbox->addWidget(mp.gauge);

    auto *details = new QWidget;
    auto *grid = new QGridLayout(details);
    grid->setContentsMargins(4, 0, 4, 0);
    grid->setHorizontalSpacing(18);
    grid->setVerticalSpacing(8);
    grid->setColumnStretch(0, 0);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(2, 0);
    grid->setColumnStretch(3, 1);

    auto makeTitle = [](const QString &text) {
        auto *label = new QLabel(text);
        label->setStyleSheet(metricTitleStyle());
        return label;
    };
    auto makeValue = []() {
        auto *label = new QLabel(QStringLiteral("--"));
        label->setWordWrap(true);
        label->setStyleSheet(metricValueStyle("#314152"));
        return label;
    };

    int row = 0;
    grid->addWidget(makeTitle(QStringLiteral("在线状态")), row, 0);
    mp.onlineLabel = makeValue();
    grid->addWidget(mp.onlineLabel, row, 1);
    grid->addWidget(makeTitle(QStringLiteral("使能状态")), row, 2);
    mp.enabledLabel = makeValue();
    grid->addWidget(mp.enabledLabel, row, 3);
    row++;

    grid->addWidget(makeTitle(QStringLiteral("运行模式")), row, 0);
    mp.modeLabel = makeValue();
    grid->addWidget(mp.modeLabel, row, 1);
    grid->addWidget(makeTitle(QStringLiteral("实时扭矩")), row, 2);
    mp.torqueLabel = makeValue();
    grid->addWidget(mp.torqueLabel, row, 3);
    row++;

    grid->addWidget(makeTitle(QStringLiteral("碰撞保护")), row, 0);
    mp.collisionLabel = makeValue();
    grid->addWidget(mp.collisionLabel, row, 1);
    grid->addWidget(makeTitle(QStringLiteral("故障状态")), row, 2);
    mp.faultLabel = makeValue();
    grid->addWidget(mp.faultLabel, row, 3);

    vbox->addWidget(details);

    panels_[nodeId] = mp;
    return card;
}

void StatusMonitorPanel::updateMotorState(uint32_t nodeId, const dac::MotorState &state)
{
    auto it = panels_.find(nodeId);
    if (it == panels_.end()) {
        return;
    }
    updateCard(*it, state);
}

void StatusMonitorPanel::onTorqueUpdated(uint32_t nodeId, int16_t torquePermille)
{
    auto it = panels_.find(nodeId);
    if (it == panels_.end()) {
        return;
    }

    it->gauge->setTorqueValue(torquePermille);

    const int absTorque = std::abs(static_cast<int>(torquePermille));
    const QString direction = torquePermille >= 0 ? QStringLiteral("+") : QStringLiteral("-");
    const QString valueText = QStringLiteral("%1%2‰ (%3% 额定)")
                                  .arg(direction)
                                  .arg(absTorque)
                                  .arg(absTorque / 10.0, 0, 'f', 1);

    const QString torqueColor = absTorque >= it->thresholdPermille ? QStringLiteral("#c8844c")
                                                                   : QStringLiteral("#37516b");
    it->torqueLabel->setText(valueText);
    it->torqueLabel->setStyleSheet(metricValueStyle(torqueColor));
}

void StatusMonitorPanel::onCollisionDetected(uint32_t nodeId, int16_t torquePermille)
{
    auto it = panels_.find(nodeId);
    if (it == panels_.end()) {
        return;
    }

    it->gauge->setCollisionTriggered(true);
    it->collisionLabel->setText(QStringLiteral("已触发，扭矩 %1‰").arg(torquePermille));
    it->collisionLabel->setStyleSheet(metricValueStyle("#c8604f"));
}

void StatusMonitorPanel::updateCard(MotorPanel &panel, const MotorState &state)
{
    panel.gauge->setOnline(state.online);
    panel.gauge->setEnabled(state.enabled);
    panel.gauge->setCollisionProtection(state.collisionProtectionOn);
    panel.gauge->setCollisionTriggered(state.collisionTriggered);

    panel.onlineLabel->setText(state.online ? QStringLiteral("在线") : QStringLiteral("离线"));
    panel.onlineLabel->setStyleSheet(metricValueStyle(state.online ? "#6f9b78" : "#b66d5d"));

    panel.enabledLabel->setText(state.enabled ? QStringLiteral("已使能") : QStringLiteral("未使能"));
    panel.enabledLabel->setStyleSheet(metricValueStyle(state.enabled ? "#5f86af" : "#7c8794"));

    panel.modeLabel->setText(operationModeText(state.mode));
    panel.modeLabel->setStyleSheet(metricValueStyle("#314152", false));

    if (state.collisionTriggered) {
        panel.collisionLabel->setText(QStringLiteral("碰撞触发"));
        panel.collisionLabel->setStyleSheet(metricValueStyle("#c8604f"));
    } else if (state.collisionProtectionOn) {
        panel.collisionLabel->setText(QStringLiteral("保护已启用"));
        panel.collisionLabel->setStyleSheet(metricValueStyle("#9b7f58"));
    } else {
        panel.collisionLabel->setText(QStringLiteral("保护未启用"));
        panel.collisionLabel->setStyleSheet(metricValueStyle("#7c8794", false));
    }

    if (state.faultCode != 0) {
        panel.faultLabel->setText(QStringLiteral("0x%1 %2")
                                      .arg(state.faultCode, 4, 16, QChar('0'))
                                      .arg(state.faultText));
        panel.faultLabel->setStyleSheet(metricValueStyle("#c8604f"));
    } else {
        panel.faultLabel->setText(QStringLiteral("无故障"));
        panel.faultLabel->setStyleSheet(metricValueStyle("#6f9b78", false));
    }
}

} // namespace dac
