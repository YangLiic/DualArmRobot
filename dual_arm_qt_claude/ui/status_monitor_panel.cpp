/**
 * StatusMonitorPanel 实现
 */
#include "status_monitor_panel.h"
#include "../services/motor_service.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QGroupBox>

namespace dar {

StatusMonitorPanel::StatusMonitorPanel(MotorService *motorService, QWidget *parent)
    : QWidget(parent)
    , motorService_(motorService)
{
    setupUi();

    refreshTimer_ = new QTimer(this);
    refreshTimer_->setInterval(500);
    connect(refreshTimer_, &QTimer::timeout, this, &StatusMonitorPanel::refreshAll);
}

void StatusMonitorPanel::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(12);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // 控制按钮
    auto *ctrlGroup = new QGroupBox(QStringLiteral("📊 实时状态监控"), this);
    ctrlGroup->setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; }");
    auto *ctrlLayout = new QHBoxLayout(ctrlGroup);

    startMonitorBtn_ = new QPushButton(QStringLiteral("▶  开始监控"));
    startMonitorBtn_->setObjectName("startMonitorBtn");
    startMonitorBtn_->setMinimumHeight(38);
    stopMonitorBtn_ = new QPushButton(QStringLiteral("⏹  停止监控"));
    stopMonitorBtn_->setMinimumHeight(38);
    stopMonitorBtn_->setEnabled(false);

    connect(startMonitorBtn_, &QPushButton::clicked, this, &StatusMonitorPanel::onStartMonitoringClicked);
    connect(stopMonitorBtn_, &QPushButton::clicked, this, &StatusMonitorPanel::onStopMonitoringClicked);

    updateLabel_ = new QLabel(QStringLiteral("未启动"));
    updateLabel_->setStyleSheet("color: #888; font-size: 12px;");

    ctrlLayout->addWidget(startMonitorBtn_);
    ctrlLayout->addWidget(stopMonitorBtn_);
    ctrlLayout->addStretch();
    ctrlLayout->addWidget(updateLabel_);
    mainLayout->addWidget(ctrlGroup);

    // 状态表格
    statusTable_ = new QTableWidget(0, 10);
    statusTable_->setHorizontalHeaderLabels({
        QStringLiteral("节点 ID"),
        QStringLiteral("在线"),
        QStringLiteral("状态"),
        QStringLiteral("模式"),
        QStringLiteral("使能"),
        QStringLiteral("转矩 (‰)"),
        QStringLiteral("电流 (A)"),
        QStringLiteral("故障"),
        QStringLiteral("碰撞"),
        QStringLiteral("更新时间")
    });

    statusTable_->horizontalHeader()->setStretchLastSection(true);
    statusTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    statusTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    statusTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    statusTable_->setAlternatingRowColors(true);
    statusTable_->verticalHeader()->setVisible(false);

    // 设置行高
    statusTable_->verticalHeader()->setDefaultSectionSize(36);

    // 设置表头样式
    statusTable_->setStyleSheet(
        "QTableWidget { gridline-color: #3a3a4a; font-size: 13px; }"
        "QHeaderView::section { background-color: #2a2a3a; color: #e0e0ff; "
        "  padding: 8px; font-weight: bold; border: 1px solid #3a3a4a; }"
        "QTableWidget::item { padding: 4px 8px; }"
        "QTableWidget::item:alternate { background-color: #1e1e2e; }"
    );

    mainLayout->addWidget(statusTable_, 1);
}

int StatusMonitorPanel::findOrCreateRow(uint32_t nodeId)
{
    auto it = nodeRowMap_.find(nodeId);
    if (it != nodeRowMap_.end()) return it.value();

    int row = statusTable_->rowCount();
    statusTable_->insertRow(row);
    nodeRowMap_[nodeId] = row;

    // 初始化 ID 列
    auto *idItem = new QTableWidgetItem(
        QStringLiteral("0x%1").arg(nodeId, 3, 16, QChar('0')));
    idItem->setTextAlignment(Qt::AlignCenter);
    idItem->setFont(QFont("Monospace", 11, QFont::Bold));
    statusTable_->setItem(row, 0, idItem);

    return row;
}

void StatusMonitorPanel::onMotorStateChanged(uint32_t nodeId, const dar::MotorState &state)
{
    int row = findOrCreateRow(nodeId);

    auto setCell = [this, row](int col, const QString &text,
                               const QString &color = QString()) {
        auto *item = statusTable_->item(row, col);
        if (!item) {
            item = new QTableWidgetItem();
            item->setTextAlignment(Qt::AlignCenter);
            statusTable_->setItem(row, col, item);
        }
        item->setText(text);
        if (!color.isEmpty()) {
            item->setForeground(QColor(color));
        }
    };

    // 在线
    setCell(1, state.online ? "✅" : "❌", state.online ? "#22c55e" : "#ef4444");

    // 状态
    QString stateColor = "#888";
    if (state.faultCode != 0) stateColor = "#ef4444";
    else if (state.collisionTriggered) stateColor = "#f59e0b";
    else if (state.enabled) stateColor = "#22c55e";
    else if (state.online) stateColor = "#3b82f6";
    setCell(2, state.stateText(), stateColor);

    // 模式
    setCell(3, state.modeText());

    // 使能
    setCell(4, state.enabled ? "✅ 是" : "❌ 否",
            state.enabled ? "#22c55e" : "#888");

    // 转矩
    setCell(5, QString::number(state.torque),
            std::abs(state.torque) > 800 ? "#f59e0b" : "#e0e0ff");

    // 电流
    setCell(6, QString::number(state.current, 'f', 2));

    // 故障
    if (state.faultCode != 0) {
        setCell(7, QStringLiteral("⚠️ %1").arg(state.faultText), "#ef4444");
    } else {
        setCell(7, QStringLiteral("正常"), "#22c55e");
    }

    // 碰撞
    setCell(8, state.collisionTriggered ? "🛑 触发" : "正常",
            state.collisionTriggered ? "#ef4444" : "#22c55e");

    // 更新时间
    setCell(9, state.lastUpdateTime.toString("HH:mm:ss.zzz"), "#888");
}

void StatusMonitorPanel::refreshAll()
{
    auto states = motorService_->allMotorStates();
    for (auto it = states.begin(); it != states.end(); ++it) {
        onMotorStateChanged(it.key(), it.value());
    }
    updateLabel_->setText(QStringLiteral("最后更新: %1")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
}

void StatusMonitorPanel::onStartMonitoringClicked()
{
    auto nodeIds = motorService_->motorNodeIds();
    for (auto nodeId : nodeIds) {
        motorService_->startMonitoring(nodeId);
    }
    refreshTimer_->start();

    startMonitorBtn_->setEnabled(false);
    stopMonitorBtn_->setEnabled(true);
    updateLabel_->setText(QStringLiteral("监控中..."));
    updateLabel_->setStyleSheet("color: #22c55e; font-size: 12px;");
}

void StatusMonitorPanel::onStopMonitoringClicked()
{
    auto nodeIds = motorService_->motorNodeIds();
    for (auto nodeId : nodeIds) {
        motorService_->stopMonitoring(nodeId);
    }
    refreshTimer_->stop();

    startMonitorBtn_->setEnabled(true);
    stopMonitorBtn_->setEnabled(false);
    updateLabel_->setText(QStringLiteral("已停止"));
    updateLabel_->setStyleSheet("color: #888; font-size: 12px;");
}

}  // namespace dar
