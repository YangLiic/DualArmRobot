#include "ui/ze300_control_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QMessageBox>

namespace dac {

Ze300ControlPanel::Ze300ControlPanel(QWidget *parent) : QWidget(parent)
{
    setupUi();
}

uint16_t Ze300ControlPanel::currentAddr() const
{
    return (uint16_t)nodeCombo_->currentData().toUInt();
}

void Ze300ControlPanel::setNodeList(const QList<uint16_t> &addrs, const QMap<uint16_t, QString> &names)
{
    nodeCombo_->clear();
    for (auto addr : addrs) {
        QString label = QStringLiteral("0x%1 - %2").arg(addr, 2, 16, QChar('0')).arg(names.value(addr));
        nodeCombo_->addItem(label, (uint)addr);
    }
}

void Ze300ControlPanel::updateZe300State(uint16_t devAddr, const Ze300Status &s)
{
    // 只更新当前选中节点
    if (currentAddr() != devAddr) return;

    lblTemperature_->setText(QStringLiteral("%1 °C").arg(s.temperatureC, 0, 'f', 1));
    lblQCurrent_->setText(QStringLiteral("%1 A").arg(s.qCurrentA, 0, 'f', 3));
    lblSpeed_->setText(QStringLiteral("%1 rpm").arg(s.speedRpm, 0, 'f', 2));
    lblPosition_->setText(QStringLiteral("%1 (%2°)").arg(s.singleTurnCount).arg(s.singleTurnDeg, 0, 'f', 2));
    lblMultiTurn_->setText(QStringLiteral("%1 (%2°)").arg(s.multiTurnCount).arg(s.totalTurnDeg, 0, 'f', 2));
    lblVoltage_->setText(QStringLiteral("%1 V").arg(s.busVoltageV, 0, 'f', 2));
    lblBusCurrent_->setText(QStringLiteral("%1 A").arg(s.busCurrentA, 0, 'f', 2));
    lblRunMode_->setText(QString::number(s.runMode));
    lblBrake_->setText(s.brakeState ? QStringLiteral("闭合") : QStringLiteral("断开"));
    lblOnline_->setText(s.online ? QStringLiteral("✅ 在线") : QStringLiteral("❌ 离线"));
    lblOnline_->setStyleSheet(s.online
        ? "QLabel { color: #688a74; font-weight: bold; }"
        : "QLabel { color: #bf6655; font-weight: bold; }");

    if (s.fault) {
        lblFault_->setText(QStringLiteral("⚠ 0x%1").arg(s.faultCode, 2, 16, QChar('0')));
        lblFault_->setStyleSheet("QLabel { color: #bf6655; font-weight: bold; }");
    } else {
        lblFault_->setText(QStringLiteral("无故障"));
        lblFault_->setStyleSheet("QLabel { color: #688a74; }");
    }
}

void Ze300ControlPanel::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // ===== 节点选择 =====
    auto *topGroup = new QGroupBox(QStringLiteral("ZE300 电机控制"));
    auto *topLayout = new QHBoxLayout;
    topLayout->addWidget(new QLabel(QStringLiteral("节点:")));
    nodeCombo_ = new QComboBox;
    nodeCombo_->setMinimumWidth(180);
    topLayout->addWidget(nodeCombo_);
    lblOnline_ = new QLabel(QStringLiteral("--"));
    topLayout->addWidget(lblOnline_);
    topLayout->addStretch();

    auto *btnFreeAll = new QPushButton(QStringLiteral("全部自由态"));
    btnFreeAll->setStyleSheet(
        "QPushButton { background: #f9e2af; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }"
        "QPushButton:hover { background: #fab387; }");
    connect(btnFreeAll, &QPushButton::clicked, this, &Ze300ControlPanel::freeOutputAllRequested);
    topLayout->addWidget(btnFreeAll);
    topGroup->setLayout(topLayout);
    mainLayout->addWidget(topGroup);

    // ===== 实时状态 =====
    auto *statusGroup = new QGroupBox(QStringLiteral("实时状态"));
    auto *statusGrid = new QGridLayout;
    int row = 0;

    auto addStatusRow = [&](const QString &label) -> QLabel* {
        statusGrid->addWidget(new QLabel(label), row, 0);
        auto *val = new QLabel(QStringLiteral("--"));
        val->setMinimumWidth(120);
        statusGrid->addWidget(val, row, 1);
        row++;
        return val;
    };

    lblTemperature_ = addStatusRow(QStringLiteral("温度:"));
    lblQCurrent_    = addStatusRow(QStringLiteral("Q轴电流:"));
    lblSpeed_       = addStatusRow(QStringLiteral("转速:"));
    lblPosition_    = addStatusRow(QStringLiteral("单圈位置:"));
    lblMultiTurn_   = addStatusRow(QStringLiteral("多圈位置:"));
    lblVoltage_     = addStatusRow(QStringLiteral("母线电压:"));
    lblBusCurrent_  = addStatusRow(QStringLiteral("母线电流:"));
    lblRunMode_     = addStatusRow(QStringLiteral("运行模式:"));
    lblFault_       = addStatusRow(QStringLiteral("故障:"));
    lblBrake_       = addStatusRow(QStringLiteral("抱闸:"));

    statusGroup->setLayout(statusGrid);
    mainLayout->addWidget(statusGroup);

    // ===== 运动控制 =====
    auto *ctrlGroup = new QGroupBox(QStringLiteral("运动控制"));
    auto *ctrlLayout = new QVBoxLayout;

    // 速度
    {
        auto *row = new QHBoxLayout;
        row->addWidget(new QLabel(QStringLiteral("速度 (rpm):")));
        speedInput_ = new QDoubleSpinBox;
        speedInput_->setRange(-3000, 3000);
        speedInput_->setDecimals(1);
        speedInput_->setValue(0);
        row->addWidget(speedInput_);
        auto *btn = new QPushButton(QStringLiteral("发送"));
        btn->setStyleSheet("QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
        connect(btn, &QPushButton::clicked, this, [this]() { emit speedRequested(currentAddr(), speedInput_->value()); });
        row->addWidget(btn);
        auto *btnStop = new QPushButton(QStringLiteral("停止"));
        btnStop->setStyleSheet("QPushButton { background: #f38ba8; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
        connect(btnStop, &QPushButton::clicked, this, [this]() { emit speedRequested(currentAddr(), 0.0f); });
        row->addWidget(btnStop);
        ctrlLayout->addLayout(row);
    }

    // 绝对位置
    {
        auto *row = new QHBoxLayout;
        row->addWidget(new QLabel(QStringLiteral("绝对位置 (°):")));
        absPosInput_ = new QDoubleSpinBox;
        absPosInput_->setRange(-99999, 99999);
        absPosInput_->setDecimals(2);
        row->addWidget(absPosInput_);
        auto *btn = new QPushButton(QStringLiteral("执行"));
        btn->setStyleSheet("QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
        connect(btn, &QPushButton::clicked, this, [this]() { emit absPositionRequested(currentAddr(), absPosInput_->value()); });
        row->addWidget(btn);
        ctrlLayout->addLayout(row);
    }

    // 相对位置
    {
        auto *row = new QHBoxLayout;
        row->addWidget(new QLabel(QStringLiteral("相对位置 (°):")));
        relPosInput_ = new QDoubleSpinBox;
        relPosInput_->setRange(-99999, 99999);
        relPosInput_->setDecimals(2);
        row->addWidget(relPosInput_);
        auto *btn = new QPushButton(QStringLiteral("执行"));
        btn->setStyleSheet("QPushButton { background: #89b4fa; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
        connect(btn, &QPushButton::clicked, this, [this]() { emit relPositionRequested(currentAddr(), relPosInput_->value()); });
        row->addWidget(btn);
        ctrlLayout->addLayout(row);
    }

    // 快捷按钮行
    {
        auto *row = new QHBoxLayout;
        auto makeBtn = [&](const QString &text, const QString &bg, auto slot) {
            auto *btn = new QPushButton(text);
            btn->setStyleSheet(QStringLiteral("QPushButton { background: %1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }").arg(bg));
            connect(btn, &QPushButton::clicked, this, slot);
            row->addWidget(btn);
        };
        makeBtn(QStringLiteral("回原点"), "#94e2d5", [this]() { emit goOriginRequested(currentAddr()); });
        makeBtn(QStringLiteral("设零点"), "#f9e2af", [this]() { emit setZeroRequested(currentAddr()); });
        makeBtn(QStringLiteral("自由态"), "#cba6f7", [this]() { emit freeOutputRequested(currentAddr()); });
        makeBtn(QStringLiteral("清故障"), "#a6e3a1", [this]() { emit clearFaultRequested(currentAddr()); });
        makeBtn(QStringLiteral("重启"), "#f38ba8", [this]() { emit rebootRequested(currentAddr()); });
        ctrlLayout->addLayout(row);
    }

    ctrlGroup->setLayout(ctrlLayout);
    mainLayout->addWidget(ctrlGroup);

    // ===== 抱闸控制 =====
    auto *brakeGroup = new QGroupBox(QStringLiteral("抱闸控制"));
    auto *brakeRow = new QHBoxLayout;
    auto *btnBrakeClose = new QPushButton(QStringLiteral("闭合抱闸"));
    auto *btnBrakeOpen = new QPushButton(QStringLiteral("断开抱闸"));
    auto *btnBrakeRead = new QPushButton(QStringLiteral("读取状态"));
    btnBrakeClose->setStyleSheet("QPushButton { background: #f9e2af; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    btnBrakeOpen->setStyleSheet("QPushButton { background: #a6e3a1; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 5px 12px; }");
    connect(btnBrakeClose, &QPushButton::clicked, this, [this]() { emit brakeCloseRequested(currentAddr()); });
    connect(btnBrakeOpen, &QPushButton::clicked, this, [this]() { emit brakeOpenRequested(currentAddr()); });
    connect(btnBrakeRead, &QPushButton::clicked, this, [this]() { emit brakeReadRequested(currentAddr()); });
    brakeRow->addWidget(btnBrakeClose);
    brakeRow->addWidget(btnBrakeOpen);
    brakeRow->addWidget(btnBrakeRead);
    brakeGroup->setLayout(brakeRow);
    mainLayout->addWidget(brakeGroup);

    // ===== 参数设置 =====
    auto *paramGroup = new QGroupBox(QStringLiteral("参数设置"));
    auto *paramLayout = new QGridLayout;
    int prow = 0;

    paramLayout->addWidget(new QLabel(QStringLiteral("位置最大速度 (rpm):")), prow, 0);
    posMaxSpeedInput_ = new QDoubleSpinBox;
    posMaxSpeedInput_->setRange(0, 5000);
    posMaxSpeedInput_->setDecimals(1);
    posMaxSpeedInput_->setValue(100);
    paramLayout->addWidget(posMaxSpeedInput_, prow, 1);
    auto *btnPosMaxSpd = new QPushButton(QStringLiteral("设置"));
    connect(btnPosMaxSpd, &QPushButton::clicked, this, [this]() { emit posMaxSpeedRequested(currentAddr(), posMaxSpeedInput_->value()); });
    paramLayout->addWidget(btnPosMaxSpd, prow, 2);
    prow++;

    paramLayout->addWidget(new QLabel(QStringLiteral("最大电流 (A):")), prow, 0);
    maxCurrentInput_ = new QDoubleSpinBox;
    maxCurrentInput_->setRange(0, 50);
    maxCurrentInput_->setDecimals(2);
    maxCurrentInput_->setValue(5.0);
    paramLayout->addWidget(maxCurrentInput_, prow, 1);
    auto *btnMaxCur = new QPushButton(QStringLiteral("设置"));
    connect(btnMaxCur, &QPushButton::clicked, this, [this]() { emit maxCurrentRequested(currentAddr(), maxCurrentInput_->value()); });
    paramLayout->addWidget(btnMaxCur, prow, 2);
    prow++;

    paramLayout->addWidget(new QLabel(QStringLiteral("速度加速度 (rpm/s):")), prow, 0);
    speedAccelInput_ = new QDoubleSpinBox;
    speedAccelInput_->setRange(0, 50000);
    speedAccelInput_->setDecimals(1);
    speedAccelInput_->setValue(1000);
    paramLayout->addWidget(speedAccelInput_, prow, 1);
    auto *btnAccel = new QPushButton(QStringLiteral("设置"));
    connect(btnAccel, &QPushButton::clicked, this, [this]() { emit speedAccelRequested(currentAddr(), speedAccelInput_->value()); });
    paramLayout->addWidget(btnAccel, prow, 2);

    paramGroup->setLayout(paramLayout);
    mainLayout->addWidget(paramGroup);

    // ===== MIT 运控 (暂未启用) =====
    auto *mitGroup = new QGroupBox(QStringLiteral("MIT 运控 (暂未启用)"));
    mitGroup->setEnabled(false);
    auto *mitLayout = new QGridLayout;
    int mrow = 0;

    auto addMitInput = [&](const QString &label, double min, double max, double val, int dec) -> QDoubleSpinBox* {
        mitLayout->addWidget(new QLabel(label), mrow, 0);
        auto *spin = new QDoubleSpinBox;
        spin->setRange(min, max);
        spin->setDecimals(dec);
        spin->setValue(val);
        mitLayout->addWidget(spin, mrow, 1);
        mrow++;
        return spin;
    };

    mitPosInput_    = addMitInput(QStringLiteral("位置 (rad):"), -100, 100, 0, 3);
    mitVelInput_    = addMitInput(QStringLiteral("速度 (rad/s):"), -50, 50, 0, 3);
    mitKpInput_     = addMitInput(QStringLiteral("Kp [0~500]:"), 0, 500, 0, 1);
    mitKdInput_     = addMitInput(QStringLiteral("Kd [0~5]:"), 0, 5, 0, 2);
    mitTorqueInput_ = addMitInput(QStringLiteral("力矩 (Nm):"), -20, 20, 0, 3);

    auto *btnMit = new QPushButton(QStringLiteral("发送 MIT 指令"));
    btnMit->setStyleSheet("QPushButton { background: #cba6f7; color: #1e1e2e; font-weight: bold; border-radius: 4px; padding: 6px 16px; }");
    connect(btnMit, &QPushButton::clicked, this, [this]() {
        emit mitControlRequested(currentAddr(),
                                  mitPosInput_->value(), mitVelInput_->value(),
                                  mitKpInput_->value(), mitKdInput_->value(),
                                  mitTorqueInput_->value());
    });
    mitLayout->addWidget(btnMit, mrow, 0, 1, 2);

    mitGroup->setLayout(mitLayout);
    mainLayout->addWidget(mitGroup);

    mainLayout->addStretch();
}

} // namespace dac
