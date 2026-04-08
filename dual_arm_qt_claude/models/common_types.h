/**
 * DualArmRobot Qt 上位机 - 公共数据类型定义
 *
 * 本文件定义了跨层共享的所有数据结构，包括：
 * - 总线配置
 * - 通信请求/结果
 * - 设备状态缓存
 * - 优先级枚举
 */
#pragma once

#include <QObject>
#include <QString>
#include <QDateTime>
#include <QVariant>
#include <QMetaType>
#include <cstdint>
#include <functional>

namespace dar {  // DualArmRobot

// ==================== 优先级 ====================
enum class Priority {
    Critical   = 0,  // 急停、失能、故障复位
    Control    = 1,  // 使能、速度/位置设定
    Monitoring = 2   // 扭矩读取、状态查询等
};

// ==================== 总线类型 ====================
enum class BusType {
    UsbCan,
    Serial,
    Tcp
};

// ==================== 运行模式 ====================
enum class OperationMode : uint8_t {
    ProfilePosition    = 1,
    Velocity           = 3,
    Homing             = 6,
    CyclicSyncPosition = 8,
    CyclicSyncVelocity = 9
};

// ==================== 协议类型 ====================
enum class ProtocolType {
    CanopenMotor,
    ModbusTcp,
    CustomSerial
};

// ==================== CAN 帧 ====================
struct CanFrame {
    uint32_t can_id = 0;
    uint8_t  data[8] = {};
    uint8_t  dlc = 0;
    uint64_t sequence = 0;
};

// ==================== 总线配置 ====================
struct BusConfig {
    QString  busId;         // 唯一标识，如 "can0"
    BusType  busType = BusType::UsbCan;
    QString  devicePath;    // e.g. "/dev/ttyUSB0"
    int      baudRate = 9600;
    ProtocolType protocol = ProtocolType::CanopenMotor;
    bool     isEnabled = true;
};

// ==================== 通信请求 ====================
struct CommRequest {
    uint64_t     requestId = 0;
    QString      busId;
    uint32_t     deviceId = 0;   // CAN node ID
    ProtocolType protocolType = ProtocolType::CanopenMotor;
    QString      commandType;    // "writeSDO", "readSDO", "nmt", etc.
    Priority     priority = Priority::Monitoring;
    int          timeoutMs = 500;
    QByteArray   payload;        // 编码后的报文数据
    int          retryCount = 0;
    int          maxRetries = 1;

    // 响应匹配器：判断收到的 CAN 帧是否是本请求的响应
    std::function<bool(const CanFrame&)> responseMatcher;

    // 期望响应的数据长度（用于 readSDO 等）
    size_t expectedResponseLen = 0;
};

// ==================== 通信结果 ====================
struct CommResult {
    uint64_t   requestId = 0;
    bool       success = false;
    int        errorCode = 0;
    QString    errorMessage;
    CanFrame   responseFrame;
    QByteArray parsedData;
    qint64     latencyMs = 0;
};

// ==================== 电机状态缓存 ====================
struct MotorState {
    uint32_t      nodeId = 0;
    bool          online = false;
    bool          enabled = false;
    OperationMode mode = OperationMode::Velocity;
    int32_t       velocity = 0;       // RPM
    double        position = 0.0;     // 度
    int16_t       torque = 0;         // 千分比
    double        current = 0.0;      // 安培
    uint16_t      faultCode = 0;
    QString       faultText;
    bool          collisionTriggered = false;
    QDateTime     lastUpdateTime;
    uint16_t      statusWord = 0;

    // 返回状态字符串
    QString stateText() const {
        if (!online) return QStringLiteral("离线");
        if (faultCode != 0) return QStringLiteral("故障 (0x%1)").arg(faultCode, 4, 16, QChar('0'));
        if (collisionTriggered) return QStringLiteral("碰撞触发");
        if (enabled) return QStringLiteral("使能");
        return QStringLiteral("就绪");
    }

    QString modeText() const {
        switch (mode) {
            case OperationMode::ProfilePosition: return QStringLiteral("位置模式");
            case OperationMode::Velocity: return QStringLiteral("速度模式");
            case OperationMode::Homing: return QStringLiteral("回零模式");
            default: return QStringLiteral("未知");
        }
    }
};

// ==================== 日志条目 ====================
struct LogEntry {
    enum Level { Debug, Info, Warning, Error, Critical };

    Level     level = Info;
    QDateTime timestamp;
    QString   source;
    QString   message;

    QString levelText() const {
        switch (level) {
            case Debug:    return "DEBUG";
            case Info:     return "INFO";
            case Warning:  return "WARN";
            case Error:    return "ERROR";
            case Critical: return "CRIT";
        }
        return "?";
    }
};

// ==================== 轮询任务 ====================
struct PollTask {
    QString    taskId;
    uint32_t   nodeId = 0;
    QString    paramName;      // "torque", "position", "statusWord", etc.
    int        intervalMs = 100;
    Priority   priority = Priority::Monitoring;
    bool       active = true;
};

}  // namespace dar

Q_DECLARE_METATYPE(dar::CanFrame)
Q_DECLARE_METATYPE(dar::CommRequest)
Q_DECLARE_METATYPE(dar::CommResult)
Q_DECLARE_METATYPE(dar::MotorState)
Q_DECLARE_METATYPE(dar::LogEntry)
Q_DECLARE_METATYPE(dar::BusConfig)
