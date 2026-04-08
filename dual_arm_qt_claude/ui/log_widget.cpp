/**
 * LogWidget 实现
 */
#include "log_widget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollBar>
#include <QDateTime>

namespace dar {

LogWidget::LogWidget(QWidget *parent)
    : QWidget(parent)
{
    setupUi();
}

void LogWidget::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(8);
    mainLayout->setContentsMargins(20, 20, 20, 20);

    // 工具栏
    auto *toolbar = new QHBoxLayout();

    filterCombo_ = new QComboBox();
    filterCombo_->addItems({"全部", "INFO+", "WARNING+", "ERROR+"});
    connect(filterCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LogWidget::onFilterChanged);

    autoScrollCheck_ = new QCheckBox(QStringLiteral("自动滚动"));
    autoScrollCheck_->setChecked(true);
    connect(autoScrollCheck_, &QCheckBox::stateChanged,
            this, &LogWidget::onAutoScrollChanged);

    clearBtn_ = new QPushButton(QStringLiteral("🗑️  清除"));
    clearBtn_->setFixedWidth(80);
    connect(clearBtn_, &QPushButton::clicked, this, &LogWidget::clear);

    toolbar->addWidget(new QLabel(QStringLiteral("📋 日志 | 过滤:")));
    toolbar->addWidget(filterCombo_);
    toolbar->addStretch();
    toolbar->addWidget(autoScrollCheck_);
    toolbar->addWidget(clearBtn_);
    mainLayout->addLayout(toolbar);

    // 日志文本
    logText_ = new QTextEdit();
    logText_->setReadOnly(true);
    logText_->setFont(QFont("Monospace", 10));
    logText_->setStyleSheet(
        "QTextEdit {"
        "  background-color: #0d0d1a;"
        "  color: #c0c0d0;"
        "  border: 1px solid #2a2a3a;"
        "  border-radius: 6px;"
        "  padding: 8px;"
        "  selection-background-color: #3b82f6;"
        "}"
    );

    mainLayout->addWidget(logText_, 1);
}

void LogWidget::appendLog(const dar::LogEntry &entry)
{
    if (entry.level < filterLevel_) return;

    // 限制日志数量
    logCount_++;
    if (logCount_ > 5000) {
        // 清掉前半部分
        QTextCursor cursor = logText_->textCursor();
        cursor.movePosition(QTextCursor::Start);
        cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, 2500);
        cursor.removeSelectedText();
        logCount_ = 2500;
    }

    QString color;
    QString icon;
    switch (entry.level) {
    case LogEntry::Debug:
        color = "#666680"; icon = "🔹"; break;
    case LogEntry::Info:
        color = "#8888cc"; icon = "ℹ️"; break;
    case LogEntry::Warning:
        color = "#f59e0b"; icon = "⚠️"; break;
    case LogEntry::Error:
        color = "#ef4444"; icon = "❌"; break;
    case LogEntry::Critical:
        color = "#ff0040"; icon = "🔴"; break;
    }

    QString html = QStringLiteral(
        "<span style='color: #555;'>%1</span> "
        "<span style='color: %2; font-weight: bold;'>%3 [%4]</span> "
        "<span style='color: #7a7aaa;'>[%5]</span> "
        "<span style='color: %2;'>%6</span><br>")
        .arg(entry.timestamp.toString("HH:mm:ss.zzz"))
        .arg(color)
        .arg(icon)
        .arg(entry.levelText())
        .arg(entry.source)
        .arg(entry.message.toHtmlEscaped());

    logText_->append(html);

    if (autoScroll_) {
        QScrollBar *sb = logText_->verticalScrollBar();
        sb->setValue(sb->maximum());
    }
}

void LogWidget::clear()
{
    logText_->clear();
    logCount_ = 0;
}

void LogWidget::onAutoScrollChanged(int state)
{
    autoScroll_ = (state == Qt::Checked);
}

void LogWidget::onFilterChanged(int index)
{
    switch (index) {
    case 0: filterLevel_ = LogEntry::Debug; break;
    case 1: filterLevel_ = LogEntry::Info; break;
    case 2: filterLevel_ = LogEntry::Warning; break;
    case 3: filterLevel_ = LogEntry::Error; break;
    }
}

}  // namespace dar
