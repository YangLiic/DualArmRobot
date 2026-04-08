/**
 * LogWidget - 日志面板
 */
#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include "../models/common_types.h"

namespace dar {

class LogWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LogWidget(QWidget *parent = nullptr);

public slots:
    void appendLog(const dar::LogEntry &entry);
    void clear();

private slots:
    void onAutoScrollChanged(int state);
    void onFilterChanged(int index);

private:
    void setupUi();

    QTextEdit *logText_;
    QPushButton *clearBtn_;
    QComboBox *filterCombo_;
    QCheckBox *autoScrollCheck_;

    LogEntry::Level filterLevel_ = LogEntry::Debug;
    bool autoScroll_ = true;
    int logCount_ = 0;
};

}  // namespace dar
