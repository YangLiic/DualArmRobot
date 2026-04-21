#pragma once
#include <QString>
#include <QDateTime>
#include <QMetaType>
#include <QVector>
#include <cstdint>
#include <array>

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

// ==================== 协议类型 ====================
enum class ProtocolType {
    CANopenSDO,   // ino motor: SDO index+subindex 匹配
    ZE300Cmd,     // ZE300: data[0] 命令码匹配
    CyberGear     // CyberGear: 29-bit ext ID cmd_type 匹配
};

// ==================== CAN 帧 ====================
struct CanFrame {
    uint32_t canId = 0;
    uint8_t  data[8] = {};
    uint8_t  dlc = 0;
    bool     isExtended = false;  // true = 29-bit 扩展帧 (CyberGear)
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

    ProtocolType protocolType = ProtocolType::CANopenSDO;

    // SDO 匹配字段 (CANopenSDO)
    uint16_t sdoIndex = 0;
    uint8_t  sdoSubindex = 0;
    uint32_t responseCanId = 0;

    // ZE300 匹配字段
    uint8_t  ze300ExpectedCmd = 0;   // 期望响应的 data[0] 命令码

    // CyberGear 匹配字段
    uint8_t  cgExpectedCmdType = 0;  // 期望响应的 cmd_type (ext_id[28:24])
    uint8_t  cgMotorId = 0;          // 电机 CAN ID
    uint8_t  cgMasterId = 0;         // 主机 CAN ID
    bool     isExtendedFrame = false; // 是否使用扩展帧
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

inline constexpr int kArmJointCount = 7;
inline constexpr double kDefaultArmJointMinDeg = -180.0;
inline constexpr double kDefaultArmJointMaxDeg = 180.0;
inline constexpr double kArmLimitInputMinDeg = -720.0;
inline constexpr double kArmLimitInputMaxDeg = 720.0;

inline QString armSideText(ArmSide side) {
    switch (side) {
    case ArmSide::Left:  return QStringLiteral("左臂");
    case ArmSide::Right: return QStringLiteral("右臂");
    }
    return QStringLiteral("未知");
}

struct ArmJointLimits {
    QVector<double> minDegrees = QVector<double>(kArmJointCount, kDefaultArmJointMinDeg);
    QVector<double> maxDegrees = QVector<double>(kArmJointCount, kDefaultArmJointMaxDeg);
};

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

// ==================== ZE300 电机 ====================
enum class Ze300Cmd : uint8_t {
    Reboot            = 0x00,
    ReadVersion       = 0xA0,
    ReadQCurrent      = 0xA1,
    ReadSpeed         = 0xA2,
    ReadPosition      = 0xA3,
    ReadQuick         = 0xA4,
    ReadStatus        = 0xAE,
    ClearFault        = 0xAF,
    ReadMotorInfo     = 0xB0,
    SetZero           = 0xB1,
    SetPosMaxSpeed    = 0xB2,
    SetMaxQCurrent    = 0xB3,
    SetSpeedAccel     = 0xB5,
    TorqueControl     = 0xC0,
    SpeedControl      = 0xC1,
    AbsPositionCtrl   = 0xC2,
    RelPositionCtrl   = 0xC3,
    GoOriginShortest  = 0xC4,
    BrakeControl      = 0xCE,
    FreeOutput        = 0xCF,
    MitLimitCfg       = 0xF0,
    MitReadState      = 0xF1,
};

struct Ze300Status {
    float qCurrentA     = 0.0f;
    float speedRpm      = 0.0f;
    float busVoltageV   = 0.0f;
    float busCurrentA   = 0.0f;
    float temperatureC  = 0.0f;

    uint16_t singleTurnCount = 0;
    int32_t  multiTurnCount  = 0;
    float    singleTurnDeg   = 0.0f;
    float    totalTurnDeg    = 0.0f;

    uint8_t  runMode    = 0;
    uint8_t  faultCode  = 0;
    bool     fault      = false;

    uint8_t  polePairs      = 0;
    float    torqueConstant = 0.0f;
    uint8_t  gearRatio      = 0;

    float    mitPosRad    = 0.0f;
    float    mitVelRadS   = 0.0f;
    float    mitTorqueNm  = 0.0f;
    bool     mitModeActive = false;

    bool     brakeState   = false; // true=closed
    bool     online       = false;
    QDateTime lastUpdateTime;
};

// ==================== CyberGear 电机 ====================
enum class CgCmd : uint8_t {
    MotorControl  = 1,
    Feedback      = 2,
    Enable        = 3,
    Stop          = 4,
    SetZero       = 6,
    ChangeCnId    = 7,
    RamRead       = 17,
    RamWrite      = 18,
    FaultFeedback = 21,
};

enum class CgRunMode : uint8_t {
    Motion   = 0,  // MIT 运控
    Position = 1,
    Speed    = 2,
    Current  = 3,
};

inline QString cgRunModeText(CgRunMode m) {
    switch (m) {
    case CgRunMode::Motion:   return QStringLiteral("运控");
    case CgRunMode::Position: return QStringLiteral("位置");
    case CgRunMode::Speed:    return QStringLiteral("速度");
    case CgRunMode::Current:  return QStringLiteral("电流");
    }
    return QStringLiteral("未知");
}

enum class CgMotorState : uint8_t {
    Reset = 0,
    Cali  = 1,
    Motor = 2,
    Unknown = 3,
};

inline QString cgMotorStateText(CgMotorState s) {
    switch (s) {
    case CgMotorState::Reset:   return QStringLiteral("复位");
    case CgMotorState::Cali:    return QStringLiteral("标定");
    case CgMotorState::Motor:   return QStringLiteral("运行");
    case CgMotorState::Unknown: return QStringLiteral("未知");
    }
    return QStringLiteral("未知");
}

namespace CgParam {
    constexpr uint16_t RunMode      = 0x7005;
    constexpr uint16_t IqRef        = 0x7006;
    constexpr uint16_t SpdRef       = 0x700A;
    constexpr uint16_t LimitTorque  = 0x700B;
    constexpr uint16_t LocRef       = 0x7016;
    constexpr uint16_t LimitSpd     = 0x7017;
    constexpr uint16_t LimitCur     = 0x7018;
    constexpr uint16_t MechPos      = 0x7019;
    constexpr uint16_t MechVel      = 0x701B;
    constexpr uint16_t Vbus         = 0x701C;
}

struct CyberGearStatus {
    float positionRad  = 0.0f;
    float velocityRadS = 0.0f;
    float torqueNm     = 0.0f;
    float temperature  = 0.0f;
    uint8_t motorId    = 0;
    bool hasFault      = false;
    bool atLimit       = false;
    bool enabled       = false;
    CgRunMode runMode  = CgRunMode::Speed;
    CgMotorState motorState = CgMotorState::Unknown;
    bool uncalibrated      = false;
    bool hallFault         = false;
    bool magneticFault     = false;
    bool overTempFault     = false;
    bool overCurrentFault  = false;
    bool underVoltageFault = false;
    bool online        = false;
    QDateTime lastUpdateTime;
};

// ==================== Inspire 夹爪 ====================
enum class GripperTransport : uint8_t {
    TCP = 0,
    RTU = 1,
};

inline QString gripperTransportText(GripperTransport t) {
    switch (t) {
    case GripperTransport::TCP: return QStringLiteral("Modbus TCP");
    case GripperTransport::RTU: return QStringLiteral("485 串口");
    }
    return QStringLiteral("未知");
}

struct GripperEndpointConfig {
    GripperTransport transport = GripperTransport::TCP;

    // TCP 参数
    QString ip = QStringLiteral("192.168.123.211");
    int     tcpPort = 6000;

    // 485 串口参数（按 demo_485.py 协议）
    QString serialPort = QStringLiteral("/dev/ttyUSB0");
    int     serialBaudRate = 115200;
    int     slaveId = 1;

    // 通用控制参数
    int speed = 500;
    int force = 500;
    int pollIntervalMs = 100;
};

struct GripperState {
    ArmSide side = ArmSide::Left;
    bool connected = false;
    bool online = false;
    GripperTransport transport = GripperTransport::TCP;
    double targetNormalized = 0.0;
    double actualNormalized = 0.0;
    QVector<int> fingerAngles;
    int statusCode = 0;
    QString detailText;
    QDateTime lastUpdateTime;
};

} // namespace dac

Q_DECLARE_METATYPE(dac::ArmSide)
Q_DECLARE_METATYPE(dac::ArmState)
Q_DECLARE_METATYPE(dac::Ze300Status)
Q_DECLARE_METATYPE(dac::CyberGearStatus)
Q_DECLARE_METATYPE(dac::GripperTransport)
Q_DECLARE_METATYPE(dac::GripperEndpointConfig)
Q_DECLARE_METATYPE(dac::GripperState)
