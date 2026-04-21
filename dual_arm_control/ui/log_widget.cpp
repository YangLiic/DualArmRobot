#include "ui/log_widget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>
#include <QScrollBar>

namespace dac {

LogWidget::LogWidget(QWidget *parent) : QWidget(parent)
{
    auto *vbox = new QVBoxLayout(this);
    vbox->setContentsMargins(0, 0, 0, 0);

    auto *toolbar = new QHBoxLayout;
    clearBtn_ = new QPushButton(QStringLiteral("清空日志"));
    toolbar->addStretch();
    toolbar->addWidget(clearBtn_);
    vbox->addLayout(toolbar);

    logView_ = new QTextEdit;
    logView_->setReadOnly(true);
    logView_->setFont(QFont("Monospace", 9));
    logView_->setStyleSheet(
        "QTextEdit {"
        "  background: #fffdfa;"
        "  color: #314152;"
        "  border: 1px solid #d8cfc1;"
        "  border-radius: 10px;"
        "  padding: 4px;"
        "}");
    vbox->addWidget(logView_);

    connect(clearBtn_, &QPushButton::clicked, this, &LogWidget::clear);
}

void LogWidget::appendLog(dac::LogLevel level, const QString &message)
{
    QString color;
    switch (level) {
    case LogLevel::Info:     color = "#688a74"; break;
    case LogLevel::Warning:  color = "#b07a44"; break;
    case LogLevel::Error:    color = "#bf6655"; break;
    case LogLevel::Critical: color = "#bf6655"; break;
    }
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    QString html = QStringLiteral("<span style='color:#7c8894;'>%1</span> "
                                  "<span style='color:%2;font-weight:bold;'>[%3]</span> %4")
                       .arg(ts, color, logLevelText(level), message.toHtmlEscaped());
    logView_->append(html);

    // 限制行数
    if (logView_->document()->blockCount() > maxLines_) {
        QTextCursor c = logView_->textCursor();
        c.movePosition(QTextCursor::Start);
        c.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, logView_->document()->blockCount() - maxLines_);
        c.removeSelectedText();
    }
    logView_->verticalScrollBar()->setValue(logView_->verticalScrollBar()->maximum());
}

void LogWidget::clear()
{
    logView_->clear();
}

} // namespace dac
