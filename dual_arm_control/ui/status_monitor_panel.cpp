#include "ui/status_monitor_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <cmath>

namespace dac {

StatusMonitorPanel::StatusMonitorPanel(QWidget *parent) : QWidget(parent)
{
    auto *vbox = new QVBoxLayout(this);
    vbox->setContentsMargins(0, 0, 0, 0);

    auto *title = new QLabel(QStringLiteral("实时状态监控"));
    title->setStyleSheet("QLabel { font-size: 15px; font-weight: bold; color: #cdd6f4; padding: 4px; }");
    vbox->addWidget(title);

    gaugeLayout_ = new QHBoxLayout;
    gaugeLayout_->setSpacing(16);
    vbox->addLayout(gaugeLayout_);
    vbox->addStretch();
}

void StatusMonitorPanel::addMotorGauge(uint32_t nodeId, const QString &label,
                                       int maxRange, int threshold)
{
    QWidget *card = createMotorCard(nodeId, label, maxRange, threshold);
    gaugeLayout_->addWidget(card);
}

QWidget *StatusMonitorPanel::createMotorCard(uint32_t nodeId, const QString &label,
                                             int maxRange, int threshold)
{
    auto *card = new QGroupBox(label);
    card->setStyleSheet(
        "QGroupBox { background: #181825; border: 1px solid #45475a; border-radius: 8px; "
        "margin-top: 14px; padding-top: 18px; font-weight: bold; color: #cdd6f4; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; }");

    auto *vbox = new QVBoxLayout(card);

    MotorPanel mp;

    // 扭矩仪表盘
    mp.gauge = new TorqueGaugeWidget;
    mp.gauge->setNodeLabel(label);
    mp.gauge->setMaxRange(maxRange);
    mp.gauge->setThreshold(threshold);
    mp.thresholdPermille = threshold;
    vbox->addWidget(mp.gauge, 0, Qt::AlignCenter);

    // 扭矩条形图（类似 interactive_control_cpp.py 的 bar）
    mp.torqueBar = new QProgressBar;
    mp.torqueBar->setRange(0, maxRange);
    mp.torqueBar->setValue(0);
    mp.torqueBar->setTextVisible(true);
    mp.torqueBar->setFormat(QStringLiteral("0‰ / %1‰").arg(threshold));
    mp.torqueBar->setFixedHeight(22);
    mp.torqueBar->setStyleSheet(
        "QProgressBar { background: #313244; border: 1px solid #45475a; border-radius: 4px; text-align: center; color: #cdd6f4; }"
        "QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
        "stop:0 #a6e3a1, stop:0.6 #f9e2af, stop:1 #f38ba8); border-radius: 3px; }");
    vbox->addWidget(mp.torqueBar);

    // 状态信息网格
    auto *grid = new QGridLayout;
    grid->setSpacing(4);
    auto makeLabel = [](const QString &text) {
        auto *l = new QLabel(text);
        l->setStyleSheet("QLabel { color: #7f849c; font-size: 11px; }");
        return l;
    };
    auto makeValue = []() {
        auto *l = new QLabel("-");
        l->setStyleSheet("QLabel { color: #cdd6f4; font-size: 11px; font-weight: bold; }");
        return l;
    };

    int row = 0;
    grid->addWidget(makeLabel(QStringLiteral("在线")), row, 0);
    mp.onlineLabel = makeValue(); grid->addWidget(mp.onlineLabel, row, 1); row++;

    grid->addWidget(makeLabel(QStringLiteral("使能")), row, 0);
    mp.enabledLabel = makeValue(); grid->addWidget(mp.enabledLabel, row, 1); row++;

    grid->addWidget(makeLabel(QStringLiteral("模式")), row, 0);
    mp.modeLabel = makeValue(); grid->addWidget(mp.modeLabel, row, 1); row++;

    grid->addWidget(makeLabel(QStringLiteral("扭矩")), row, 0);
    mp.torqueLabel = makeValue(); grid->addWidget(mp.torqueLabel, row, 1); row++;

    grid->addWidget(makeLabel(QStringLiteral("碰撞保护")), row, 0);
    mp.collisionLabel = makeValue(); grid->addWidget(mp.collisionLabel, row, 1); row++;

    grid->addWidget(makeLabel(QStringLiteral("故障")), row, 0);
    mp.faultLabel = makeValue(); grid->addWidget(mp.faultLabel, row, 1); row++;

    vbox->addLayout(grid);

    panels_[nodeId] = mp;
    return card;
}

void StatusMonitorPanel::updateMotorState(uint32_t nodeId, const dac::MotorState &state)
{
    auto it = panels_.find(nodeId);
    if (it == panels_.end()) return;
    updateCard(*it, state);
}

void StatusMonitorPanel::onTorqueUpdated(uint32_t nodeId, int16_t torquePermille)
{
    auto it = panels_.find(nodeId);
    if (it == panels_.end()) return;

    it->gauge->setTorqueValue(torquePermille);
    int abs_t = std::abs(static_cast<int>(torquePermille));
    it->torqueBar->setValue(abs_t);
    it->torqueBar->setFormat(QStringLiteral("%1‰ / %2‰")
                                 .arg(abs_t).arg(it->thresholdPermille));

    QString dir = (torquePermille >= 0) ? "+" : "-";
    it->torqueLabel->setText(QStringLiteral("%1%2‰ (%3%额定)")
                                 .arg(dir).arg(abs_t).arg(abs_t / 10.0, 0, 'f', 1));
}

void StatusMonitorPanel::onCollisionDetected(uint32_t nodeId, int16_t torquePermille)
{
    auto it = panels_.find(nodeId);
    if (it == panels_.end()) return;

    it->gauge->setCollisionTriggered(true);
    it->collisionLabel->setText(QStringLiteral("触发! 扭矩=%1‰").arg(torquePermille));
    it->collisionLabel->setStyleSheet("QLabel { color: #f38ba8; font-size: 11px; font-weight: bold; }");
}

void StatusMonitorPanel::updateCard(MotorPanel &panel, const MotorState &state)
{
    panel.gauge->setOnline(state.online);
    panel.gauge->setEnabled(state.enabled);
    panel.gauge->setCollisionProtection(state.collisionProtectionOn);
    panel.gauge->setCollisionTriggered(state.collisionTriggered);

    panel.onlineLabel->setText(state.online ? QStringLiteral("在线") : QStringLiteral("离线"));
    panel.onlineLabel->setStyleSheet(state.online
        ? "QLabel { color: #a6e3a1; font-size: 11px; font-weight: bold; }"
        : "QLabel { color: #f38ba8; font-size: 11px; font-weight: bold; }");

    panel.enabledLabel->setText(state.enabled ? QStringLiteral("已使能") : QStringLiteral("未使能"));
    panel.enabledLabel->setStyleSheet(state.enabled
        ? "QLabel { color: #a6e3a1; font-size: 11px; font-weight: bold; }"
        : "QLabel { color: #7f849c; font-size: 11px; }");

    panel.modeLabel->setText(operationModeText(state.mode));

    QString collisionText;
    if (state.collisionTriggered) {
        collisionText = QStringLiteral("碰撞触发!");
        panel.collisionLabel->setStyleSheet("QLabel { color: #f38ba8; font-size: 11px; font-weight: bold; }");
    } else if (state.collisionProtectionOn) {
        collisionText = QStringLiteral("已启用");
        panel.collisionLabel->setStyleSheet("QLabel { color: #a6e3a1; font-size: 11px; font-weight: bold; }");
    } else {
        collisionText = QStringLiteral("未启用");
        panel.collisionLabel->setStyleSheet("QLabel { color: #7f849c; font-size: 11px; }");
    }
    panel.collisionLabel->setText(collisionText);

    if (state.faultCode != 0) {
        panel.faultLabel->setText(QStringLiteral("0x%1 %2")
                                      .arg(state.faultCode, 4, 16, QChar('0'))
                                      .arg(state.faultText));
        panel.faultLabel->setStyleSheet("QLabel { color: #f38ba8; font-size: 11px; font-weight: bold; }");
    } else {
        panel.faultLabel->setText(QStringLiteral("无"));
        panel.faultLabel->setStyleSheet("QLabel { color: #7f849c; font-size: 11px; }");
    }
}

} // namespace dac
