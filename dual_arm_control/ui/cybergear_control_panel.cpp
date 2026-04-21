#include "ui/cybergear_control_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QStringList>
#include <cmath>

namespace dac {

namespace {

QString formatFaultText(const CyberGearStatus &s)
{
    QStringList faults;
    if (s.uncalibrated) faults << QStringLiteral("未标定");
    if (s.hallFault) faults << QStringLiteral("HALL");
    if (s.magneticFault) faults << QStringLiteral("磁编");
    if (s.overTempFault) faults << QStringLiteral("过温");
    if (s.overCurrentFault) faults << QStringLiteral("过流");
    if (s.underVoltageFault) faults << QStringLiteral("欠压");
    return faults.isEmpty() ? QStringLiteral("正常") : faults.join(QStringLiteral(" | "));
}

} // namespace

CyberGearControlPanel::CyberGearControlPanel(QWidget *parent) : QWidget(parent)
{
    setupUi();
}

uint8_t CyberGearControlPanel::currentId() const { return (uint8_t)nodeCombo_->currentData().toUInt(); }

void CyberGearControlPanel::setNodeList(const QList<uint8_t> &ids, const QMap<uint8_t, QString> &names)
{
    nodeCombo_->clear();
    for (auto id : ids) {
        nodeCombo_->addItem(QStringLiteral("0x%1 - %2").arg(id, 2, 16, QChar('0')).arg(names.value(id)), (uint)id);
    }
}

void CyberGearControlPanel::updateCgState(uint8_t motorId, const CyberGearStatus &s)
{
    if (currentId() != motorId) return;
    lblPos_->setText(QStringLiteral("%1 rad (%2°)").arg(s.positionRad, 0, 'f', 3).arg(s.positionRad * 180.0f / M_PI, 0, 'f', 1));
    lblVel_->setText(QStringLiteral("%1 rad/s (%2 RPM)").arg(s.velocityRadS, 0, 'f', 2).arg(s.velocityRadS * 60.0f / (2*M_PI), 0, 'f', 1));
    lblTorque_->setText(QStringLiteral("%1 Nm").arg(s.torqueNm, 0, 'f', 3));
    lblTemp_->setText(QStringLiteral("%1 °C").arg(s.temperature, 0, 'f', 1));
    lblMode_->setText(QStringLiteral("%1 / %2").arg(cgRunModeText(s.runMode), cgMotorStateText(s.motorState)));
    lblOnline_->setText(s.online ? QStringLiteral("✅ 在线") : QStringLiteral("❌ 离线"));
    lblOnline_->setStyleSheet(s.online ? "QLabel{color:#688a74;font-weight:bold;}" : "QLabel{color:#bf6655;font-weight:bold;}");
    if (s.hasFault) {
        lblFault_->setText(QStringLiteral("⚠ %1").arg(formatFaultText(s)));
        lblFault_->setStyleSheet("QLabel{color:#bf6655;font-weight:bold;}");
    } else {
        lblFault_->setText(formatFaultText(s));
        lblFault_->setStyleSheet("QLabel{color:#688a74;}");
    }
}

void CyberGearControlPanel::setupUi()
{
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0,0,0,0);

    // ===== 节点选择 =====
    auto *topGroup = new QGroupBox(QStringLiteral("CyberGear 电机控制"));
    auto *topLayout = new QHBoxLayout;
    topLayout->addWidget(new QLabel(QStringLiteral("节点:")));
    nodeCombo_ = new QComboBox; nodeCombo_->setMinimumWidth(180);
    topLayout->addWidget(nodeCombo_);
    lblOnline_ = new QLabel(QStringLiteral("--"));
    topLayout->addWidget(lblOnline_);
    topLayout->addStretch();
    auto *btnStopAll = new QPushButton(QStringLiteral("全部停止"));
    btnStopAll->setStyleSheet("QPushButton{background:#f38ba8;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
    connect(btnStopAll, &QPushButton::clicked, this, &CyberGearControlPanel::stopAllRequested);
    topLayout->addWidget(btnStopAll);
    topGroup->setLayout(topLayout);
    mainLayout->addWidget(topGroup);

    // ===== 实时状态 =====
    auto *statusGroup = new QGroupBox(QStringLiteral("实时状态"));
    auto *sg = new QGridLayout; int row = 0;
    auto addRow = [&](const QString &lbl) -> QLabel* {
        sg->addWidget(new QLabel(lbl), row, 0);
        auto *v = new QLabel(QStringLiteral("--")); v->setMinimumWidth(150);
        sg->addWidget(v, row, 1); row++; return v;
    };
    lblPos_    = addRow(QStringLiteral("位置:"));
    lblVel_    = addRow(QStringLiteral("速度:"));
    lblTorque_ = addRow(QStringLiteral("力矩:"));
    lblTemp_   = addRow(QStringLiteral("温度:"));
    lblMode_   = addRow(QStringLiteral("模式:"));
    lblFault_  = addRow(QStringLiteral("故障:"));
    statusGroup->setLayout(sg);
    mainLayout->addWidget(statusGroup);

    // ===== 模式与控制 =====
    auto *ctrlGroup = new QGroupBox(QStringLiteral("运动控制"));
    auto *ctrlLayout = new QVBoxLayout;

    // 模式按钮
    {
        auto *r = new QHBoxLayout;
        auto *btnEnable = new QPushButton(QStringLiteral("使能"));
        btnEnable->setStyleSheet("QPushButton{background:#a6e3a1;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnEnable, &QPushButton::clicked, this, [this](){emit enableRequested(currentId());});
        auto *btnStop = new QPushButton(QStringLiteral("停止"));
        btnStop->setStyleSheet("QPushButton{background:#f38ba8;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnStop, &QPushButton::clicked, this, [this](){emit stopRequested(currentId());});
        auto *btnSpd = new QPushButton(QStringLiteral("速度模式"));
        btnSpd->setStyleSheet("QPushButton{background:#89b4fa;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnSpd, &QPushButton::clicked, this, [this](){emit speedModeRequested(currentId());});
        auto *btnPos = new QPushButton(QStringLiteral("位置模式"));
        btnPos->setStyleSheet("QPushButton{background:#cba6f7;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnPos, &QPushButton::clicked, this, [this](){emit positionModeRequested(currentId());});
        r->addWidget(btnEnable); r->addWidget(btnStop); r->addWidget(btnSpd); r->addWidget(btnPos);
        ctrlLayout->addLayout(r);
    }

    // 速度
    {
        auto *r = new QHBoxLayout;
        r->addWidget(new QLabel(QStringLiteral("速度 (rad/s):")));
        speedInput_ = new QDoubleSpinBox; speedInput_->setRange(-30,30); speedInput_->setDecimals(2);
        r->addWidget(speedInput_);
        auto *btn = new QPushButton(QStringLiteral("发送"));
        btn->setStyleSheet("QPushButton{background:#89b4fa;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btn, &QPushButton::clicked, this, [this](){emit speedRequested(currentId(), speedInput_->value());});
        r->addWidget(btn);
        auto *btnZ = new QPushButton(QStringLiteral("停速"));
        btnZ->setStyleSheet("QPushButton{background:#f38ba8;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnZ, &QPushButton::clicked, this, [this](){emit speedRequested(currentId(), 0.0f);});
        r->addWidget(btnZ);
        ctrlLayout->addLayout(r);
    }

    // 位置
    {
        auto *r = new QHBoxLayout;
        r->addWidget(new QLabel(QStringLiteral("位置 (°):")));
        posInput_ = new QDoubleSpinBox; posInput_->setRange(-720,720); posInput_->setDecimals(1);
        r->addWidget(posInput_);
        auto *btn = new QPushButton(QStringLiteral("执行"));
        btn->setStyleSheet("QPushButton{background:#cba6f7;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btn, &QPushButton::clicked, this, [this](){emit positionDegRequested(currentId(), posInput_->value());});
        r->addWidget(btn);
        ctrlLayout->addLayout(r);
    }

    // 零位
    {
        auto *r = new QHBoxLayout;
        auto *btnSetZ = new QPushButton(QStringLiteral("设当前零位"));
        btnSetZ->setStyleSheet("QPushButton{background:#f9e2af;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnSetZ, &QPushButton::clicked, this, [this](){emit setZeroRequested(currentId());});
        auto *btnGoZ = new QPushButton(QStringLiteral("回到零位"));
        btnGoZ->setStyleSheet("QPushButton{background:#94e2d5;color:#1e1e2e;font-weight:bold;border-radius:4px;padding:5px 12px;}");
        connect(btnGoZ, &QPushButton::clicked, this, [this](){emit goToZeroRequested(currentId());});
        r->addWidget(btnSetZ); r->addWidget(btnGoZ);
        ctrlLayout->addLayout(r);
    }

    ctrlGroup->setLayout(ctrlLayout);
    mainLayout->addWidget(ctrlGroup);

    // ===== 参数 =====
    auto *paramGroup = new QGroupBox(QStringLiteral("参数设置"));
    auto *pg = new QGridLayout; int pr = 0;

    pg->addWidget(new QLabel(QStringLiteral("电流限制 (A):")), pr, 0);
    curLimitInput_ = new QDoubleSpinBox; curLimitInput_->setRange(0,27); curLimitInput_->setDecimals(1); curLimitInput_->setValue(5);
    pg->addWidget(curLimitInput_, pr, 1);
    auto *btnCL = new QPushButton(QStringLiteral("设置"));
    connect(btnCL, &QPushButton::clicked, this, [this](){emit currentLimitRequested(currentId(), curLimitInput_->value());});
    pg->addWidget(btnCL, pr, 2); pr++;

    pg->addWidget(new QLabel(QStringLiteral("速度限制 (rad/s):")), pr, 0);
    spdLimitInput_ = new QDoubleSpinBox; spdLimitInput_->setRange(0,30); spdLimitInput_->setDecimals(1); spdLimitInput_->setValue(0.5);
    pg->addWidget(spdLimitInput_, pr, 1);
    auto *btnSL = new QPushButton(QStringLiteral("设置"));
    connect(btnSL, &QPushButton::clicked, this, [this](){emit speedLimitRequested(currentId(), spdLimitInput_->value());});
    pg->addWidget(btnSL, pr, 2);

    paramGroup->setLayout(pg);
    mainLayout->addWidget(paramGroup);

    mainLayout->addStretch();
}

} // namespace dac
