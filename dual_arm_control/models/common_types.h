#pragma once
#include <QString>
#include <QDateTime>
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
struct BusConfig {
    QString  devicePath;
    int      baudRate = 9600;
    int      timeoutMs = 100;
};

// ==================== 日志条目 ====================
enum class LogLevel { Info, Warning, Error, Critical };

struct LogEntry {
    QDateTime  timestamp;
    LogLevel   level = LogLevel::Info;
    QString    source;
    QString    message;
};

inline QString logLevelText(LogLevel l) {
    switch (l) {
    case LogLevel::Info:     return QStringLiteral("INFO");
    case LogLevel::Warning:  return QStringLiteral("WARN");
    case LogLevel::Error:    return QStringLiteral("ERROR");
    case LogLevel::Critical: return QStringLiteral("CRIT");
    }
    return {};
}

} // namespace dac
