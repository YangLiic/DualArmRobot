#pragma once
#include "models/common_types.h"
#include <QWidget>
#include <QTextEdit>
#include <QPushButton>

namespace dac {

class LogWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LogWidget(QWidget *parent = nullptr);

public slots:
    void appendLog(dac::LogLevel level, const QString &message);
    void clear();

private:
    QTextEdit   *logView_;
    QPushButton *clearBtn_;
    int maxLines_ = 2000;
};

} // namespace dac
