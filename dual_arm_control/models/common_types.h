#pragma once
#include <QString>
#include <QDateTime>
#include <QMetaType>
#include <QVector>
#include <cstdint>

namespace dac {

// ==================== 请求优先级 ====================
enum class Priority {
    Critical,    // 急停、失能、故障复位
    Control,     // 使能、速度设定、位置设定
    Monitoring   // 扭矩读取、状态查询
};

// ==================== 运行模式 ====================
enum class OperationMode : uint8_t {
    ProfilePosition     = 1,
    Velocity            = 3,
    Homing              = 6,
    CyclicSyncPosition  = 8,
    CyclicSyncVelocity  = 9
};

inline QString operationModeText(OperationMode m) {
    switch (m) {
    case OperationMode::ProfilePosition:    return QStringLiteral("位置模式");
    case OperationMode::Velocity:           return QStringLiteral("速度模式");
    case OperationMode::Homing:             return QStringLiteral("回零模式");
    case OperationMode::CyclicSyncPosition: return QStringLiteral("循环同步位置");
    case OperationMode::CyclicSyncVelocity: return QStringLiteral("循环同步速度");
    }
    return QStringLiteral("未知模式");
}

// ==================== CAN 帧 ====================
struct CanFrame {
    uint32_t canId = 0;
    uint8_t  data[8] = {};
    uint8_t  dlc = 0;
    uint64_t sequence = 0;
};

// ==================== 通信请求 ====================
struct CommRequest {
    uint64_t requestId = 0;
    uint32_t canId = 0;
    uint8_t  payload[8] = {};
    uint8_t  payloadLen = 0;
    Priority priority = Priority::Monitoring;
    int      timeoutMs = 350;
    bool     expectResponse = true;

    // SDO 匹配字段
    uint16_t sdoIndex = 0;
    uint8_t  sdoSubindex = 0;
    uint32_t responseCanId = 0;
};

// ==================== 通信结果 ====================
struct CommResult {
    uint64_t requestId = 0;
    bool     success = false;
    QString  errorMessage;
    uint8_t  data[8] = {};
    uint8_t  dlc = 0;
    int      latencyMs = 0;
};

// ==================== 电机状态 ====================
struct MotorState {
    uint32_t nodeId = 0;
    bool     online = false;
    bool     enabled = false;
    OperationMode mode = OperationMode::Velocity;

    int16_t  torquePermille = 0;       // 千分比 (6077h)
    double   phaseCurrentAmp = 0.0;    // 相电流 (A)
    int32_t  positionDeviation = 0;    // 位置偏差 (60F4h)
    int32_t  velocityRpm = 0;

    uint16_t faultCode = 0;
    QString  faultText;

    bool     collisionTriggered = false;
    bool     collisionProtectionOn = false;

    QDateTime lastUpdateTime;
};

// ==================== 碰撞保护配置 ====================
struct CollisionConfig {
    uint16_t torqueLimitPermille = 1200;
    uint16_t triggerTorquePermille = 900;
    double   triggerCurrentAmp = 0.0;
    int32_t  triggerPositionDeviation = 0;
    int      consecutiveSamples = 3;
    int      pollIntervalMs = 20;
    bool     useQuickStop = true;
};

// ==================== 总线配置 ====================
enum class AdapterType {
    Serial,     // CH340 串口 USB-CAN (0xAA...0x7A 帧协议)
    VCI         // 创芯科技 / ZLG VCI 协议 (CANalyst-II 等)
};

struct BusConfig {
    AdapterType adapterType = AdapterType::Serial;
    QString  devicePath;        // Serial: 串口名(如 /dev/ttyUSB0)；VCI: 设备序号(如 "0")
    int      baudRate = 9600;   // Serial: 串口波特率
    int      canBitrate = 1000; // VCI: CAN 波特率 (kbps)，默认 1Mbps
    int      canChannel = 1;    // VCI: CAN 通道号 (0 或 1)
    int      timeoutMs = 100;
};

// ==================== 日志级别 ====================
enum class LogLevel { Info, Warning, Error, Critical };

inline QString logLevelText(LogLevel l) {
    switch (l) {
    case LogLevel::Info:     return QStringLiteral("INFO");
    case LogLevel::Warning:  return QStringLiteral("WARN");
    case LogLevel::Error:    return QStringLiteral("ERROR");
    case LogLevel::Critical: return QStringLiteral("CRIT");
    }
    return {};
}

// ==================== 双臂机械臂 ====================
enum class ArmSide : uint8_t {
    Left = 0,
    Right = 1
};

inline QString armSideText(ArmSide side) {
    switch (side) {
    case ArmSide::Left:  return QStringLiteral("左臂");
    case ArmSide::Right: return QStringLiteral("右臂");
    }
    return QStringLiteral("未知");
}

struct ArmState {
    ArmSide side = ArmSide::Left;
    bool    initialized = false;
    bool    online = false;

    QVector<double> jointPositionsRad;  // 7关节，单位 rad
    QVector<double> tcpPose;            // [x, y, z, rx, ry, rz]
    QVector<int32_t> errorCodes;        // 7关节错误字
    QString errorSummary;

    QDateTime lastUpdateTime;
};

} // namespace dac

Q_DECLARE_METATYPE(dac::ArmSide)
Q_DECLARE_METATYPE(dac::ArmState)
