/****************************************************************
 * Inovance Servo SDK - CANopen Implementation
 **/
#include "inovance_servo.h"
#include <unistd.h>
#include <cstring>
#include <cmath>

namespace pg
{

// CANopen 对象字典地址
constexpr uint16_t OD_CONTROL_WORD = 0x6040;
constexpr uint16_t OD_STATUS_WORD = 0x6041;
constexpr uint16_t OD_OPERATION_MODE = 0x6060;
constexpr uint16_t OD_TARGET_POSITION = 0x607A;
constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;
constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;
constexpr uint16_t OD_PROFILE_ACCELERATION = 0x6083;
constexpr uint16_t OD_PROFILE_DECELERATION = 0x6084;

// 编码器参数
constexpr int32_t ENCODER_RESOLUTION = 8388608; // 23位编码器 (2^23)
constexpr double PULSES_PER_DEGREE = ENCODER_RESOLUTION / 360.0;

InovanceServo::InovanceServo(const std::string& port_name, int baud_rate, uint32_t node_id)
    : CanInterfaceUsb(port_name, baud_rate, 100), 
      node_id_(node_id), 
      motor_enabled_(false),
      current_mode_(OperationMode::VELOCITY),
      direction_inverted_(false)
{
    std::cout << "🔧 Inovance Servo SDK 初始化" << std::endl;
    std::cout << "   节点 ID: 0x" << std::hex << node_id_ << std::dec << std::endl;
}

void InovanceServo::setDirectionInverted(bool inverted)
{
    direction_inverted_ = inverted;
    std::cout << "⚙️  方向设置: " << (inverted ? "反转" : "正常") << std::endl;
}

InovanceServo::~InovanceServo()
{
    if (motor_enabled_) {
        stop();
    }
}

void InovanceServo::decode()
{
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    
    get_can_id(can_id);
    get_data(data, dlc);
    
    // 处理响应 (可扩展，如状态字监控)
}

// ==================== 内部辅助函数 ====================

bool InovanceServo::writeSDO(uint16_t index, uint8_t subindex, const uint8_t* data, size_t len)
{
    uint8_t frame[8];
    
    // SDO 写命令
    if (len == 1) {
        frame[0] = 0x2F; // 1字节数据
    } else if (len == 2) {
        frame[0] = 0x2B; // 2字节数据
    } else if (len == 4) {
        frame[0] = 0x23; // 4字节数据
    } else {
        std::cerr << "不支持的数据长度: " << len << std::endl;
        return false;
    }
    
    // 对象字典索引 (Little Endian)
    frame[1] = index & 0xFF;
    frame[2] = (index >> 8) & 0xFF;
    frame[3] = subindex;
    
    // 数据
    memcpy(&frame[4], data, len);
    
    // 填充剩余字节
    if (len < 4) {
        memset(&frame[4 + len], 0, 4 - len);
    }
    
    if (can_send(node_id_, frame, 8) < 0) {
        return false;
    }
    
    usleep(10000); // 10ms 延迟
    return true;
}

bool InovanceServo::setOperationMode(OperationMode mode)
{
    uint8_t mode_value = static_cast<uint8_t>(mode);
    
    if (!writeSDO(OD_OPERATION_MODE, 0x00, &mode_value, 1)) {
        std::cerr << "❌ 设置运行模式失败" << std::endl;
        return false;
    }
    
    current_mode_ = mode;
    
    const char* mode_name = "";
    switch (mode) {
        case OperationMode::PROFILE_POSITION: mode_name = "位置模式"; break;
        case OperationMode::VELOCITY: mode_name = "速度模式"; break;
        case OperationMode::HOMING: mode_name = "回零模式"; break;
        default: mode_name = "未知模式"; break;
    }
    
    std::cout << "✅ 设置为 " << mode_name << std::endl;
    return true;
}

bool InovanceServo::sendControlWord(ControlWord cmd)
{
    uint16_t cmd_value = static_cast<uint16_t>(cmd);
    uint8_t data[2] = {
        static_cast<uint8_t>(cmd_value & 0xFF),
        static_cast<uint8_t>((cmd_value >> 8) & 0xFF)
    };
    
    return writeSDO(OD_CONTROL_WORD, 0x00, data, 2);
}

// ==================== 基础控制 ====================

bool InovanceServo::enable()
{
    return enable(OperationMode::VELOCITY);
}

bool InovanceServo::enable(OperationMode mode)
{
    std::cout << "\n🚀 --- 开始使能流程 ---" << std::endl;
    
    // 1. 设置运行模式
    if (!setOperationMode(mode)) {
        return false;
    }
    usleep(50000);
    
    // 2. 状态机跳转: Shutdown
    if (!sendControlWord(ControlWord::SHUTDOWN)) {
        std::cerr << "❌ Shutdown 失败" << std::endl;
        return false;
    }
    usleep(100000);
    
    // 3. Switch On
    if (!sendControlWord(ControlWord::SWITCH_ON)) {
        std::cerr << "❌ Switch On 失败" << std::endl;
        return false;
    }
    usleep(100000);
    
    // 4. Enable Operation (锁轴)
    if (!sendControlWord(ControlWord::ENABLE_OPERATION)) {
        std::cerr << "❌ Enable Operation 失败" << std::endl;
        return false;
    }
    usleep(100000);
    
    motor_enabled_ = true;
    std::cout << "✅ 电机已使能 (已锁轴)" << std::endl;
    
    // 5. NMT 启动
    nmtStart();
    
    return true;
}

bool InovanceServo::disable()
{
    std::cout << "\n⏸️  --- 失能电机 ---" << std::endl;
    
    if (!sendControlWord(ControlWord::SHUTDOWN)) {
        std::cerr << "❌ 失能失败" << std::endl;
        return false;
    }
    
    motor_enabled_ = false;
    std::cout << "✅ 电机已失能" << std::endl;
    return true;
}

bool InovanceServo::faultReset()
{
    std::cout << "\n🔄 --- 错误复位 ---" << std::endl;
    
    // 错误复位命令：控制字 0x6040 = 0x0080
    if (!sendControlWord(ControlWord::FAULT_RESET)) {
        std::cerr << "❌ 错误复位失败" << std::endl;
        return false;
    }
    
    usleep(100000);
    
    // 复位后需要清除复位位 (回到 0x0000)
    if (!sendControlWord(ControlWord::DISABLE_VOLTAGE)) {
        std::cerr << "❌ 清除复位位失败" << std::endl;
        return false;
    }
    
    std::cout << "✅ 错误复位完成" << std::endl;
    motor_enabled_ = false;
    
    return true;
}

bool InovanceServo::quickStop()
{
    std::cout << "\n⚠️  --- 急停 ---" << std::endl;
    
    if (!sendControlWord(ControlWord::QUICK_STOP)) {
        std::cerr << "❌ 急停失败" << std::endl;
        return false;
    }
    
    std::cout << "✅ 急停完成" << std::endl;
    return true;
}

bool InovanceServo::stop()
{
    std::cout << "\n🛑 --- 停止电机 ---" << std::endl;
    
    // 1. 速度/位置设为 0
    if (current_mode_ == OperationMode::VELOCITY) {
        setVelocity(0);
    } else if (current_mode_ == OperationMode::PROFILE_POSITION) {
        // 位置模式不需要特别处理，直接失能
    }
    
    usleep(100000);
    
    // 2. 失能
    disable();
    usleep(100000);
    
    // 3. NMT 预操作
    nmtPreOperational();
    
    return true;
}

// ==================== 速度控制 ====================

bool InovanceServo::setVelocity(int32_t rpm)
{
    if (!motor_enabled_) {
        std::cerr << "❌ 电机未使能，无法设置速度" << std::endl;
        return false;
    }
    
    if (current_mode_ != OperationMode::VELOCITY) {
        std::cerr << "❌ 当前非速度模式，无法设置速度" << std::endl;
        return false;
    }
    
    // 应用方向反转
    int32_t actual_rpm = direction_inverted_ ? -rpm : rpm;
    
    std::cout << "🎯 设置速度: " << rpm << " RPM" 
              << (direction_inverted_ ? " (方向已反转)" : "") << std::endl;
    
    // 换算 RPM 到编码器脉冲/秒
    // 60 RPM = 0x00800000 (8388608 pulses/sec)
    // 使用 int64_t 避免溢出 (500 * 8388608 > int32_t max)
    int32_t encoder_value = static_cast<int32_t>((static_cast<int64_t>(actual_rpm) * ENCODER_RESOLUTION) / 60);
    
    uint8_t data[4] = {
        static_cast<uint8_t>((encoder_value >> 0) & 0xFF),
        static_cast<uint8_t>((encoder_value >> 8) & 0xFF),
        static_cast<uint8_t>((encoder_value >> 16) & 0xFF),
        static_cast<uint8_t>((encoder_value >> 24) & 0xFF)
    };
    
    if (!writeSDO(OD_TARGET_VELOCITY, 0x00, data, 4)) {
        std::cerr << "❌ 设置速度失败" << std::endl;
        return false;
    }
    
    return true;
}

// ==================== 位置控制 ====================

bool InovanceServo::setPosition(double degrees, bool absolute)
{
    // 角度转脉冲
    int32_t pulses = static_cast<int32_t>(degrees * PULSES_PER_DEGREE);
    
    if (!setPositionPulse(pulses, absolute)) {
        return false;
    }
    
    // 自动启动位置运动
    return startPositionMove(!absolute);  // absolute=false 表示 relative=true
}

bool InovanceServo::setPositionPulse(int32_t pulses, bool absolute)
{
    if (!motor_enabled_) {
        std::cerr << "❌ 电机未使能，无法设置位置" << std::endl;
        return false;
    }
    
    if (current_mode_ != OperationMode::PROFILE_POSITION) {
        std::cerr << "❌ 当前非位置模式，无法设置位置" << std::endl;
        return false;
    }
    
    // 应用方向反转
    int32_t actual_pulses = direction_inverted_ ? -pulses : pulses;
    
    double degrees = pulses / PULSES_PER_DEGREE;
    std::cout << "🎯 设置目标位置: " << degrees << "° (" << actual_pulses << " pulses)" 
              << (absolute ? " [绝对]" : " [相对]")
              << (direction_inverted_ ? " (方向已反转)" : "") << std::endl;
    
    pulses = actual_pulses;  // 使用反转后的值
    
    uint8_t data[4] = {
        static_cast<uint8_t>((pulses >> 0) & 0xFF),
        static_cast<uint8_t>((pulses >> 8) & 0xFF),
        static_cast<uint8_t>((pulses >> 16) & 0xFF),
        static_cast<uint8_t>((pulses >> 24) & 0xFF)
    };
    
    if (!writeSDO(OD_TARGET_POSITION, 0x00, data, 4)) {
        std::cerr << "❌ 设置目标位置失败" << std::endl;
        return false;
    }
    
    return true;
}

bool InovanceServo::startPositionMove(bool relative)
{
    if (!motor_enabled_) {
        std::cerr << "❌ 电机未使能" << std::endl;
        return false;
    }
    
    std::cout << "▶️  启动位置运动 (" << (relative ? "相对" : "绝对") << ")" << std::endl;
    
    // 控制字说明：
    // bit0-3: Enable Operation (0x0F)
    // bit4: New setpoint (1 = 启动新位置)
    // bit5: Change set immediately (1 = 立即执行)
    // bit6: Relative (1 = 相对位置, 0 = 绝对位置)
    
    uint8_t control_word;
    if (relative) {
        control_word = 0x7F;  // 0b01111111: 相对位置 + 立即执行 + 新目标
    } else {
        control_word = 0x3F;  // 0b00111111: 绝对位置 + 立即执行 + 新目标
    }
    
    uint8_t data[2] = {control_word, 0x00};
    
    if (!writeSDO(OD_CONTROL_WORD, 0x00, data, 2)) {
        std::cerr << "❌ 启动位置运动失败" << std::endl;
        return false;
    }
    
    usleep(50000);
    
    // 复位 bit 4 (New setpoint)，保持其他位
    data[0] = 0x0F;
    writeSDO(OD_CONTROL_WORD, 0x00, data, 2);
    
    return true;
}

bool InovanceServo::setProfileVelocity(uint32_t rpm)
{
    std::cout << "⚙️  设置运动速度限制: " << rpm << " RPM" << std::endl;
    
    // Profile Velocity (0x6081) 单位: 脉冲/秒
    // 换算: RPM -> pulses/sec (使用 uint64_t 避免溢出)
    uint32_t pulses_per_sec = static_cast<uint32_t>((static_cast<uint64_t>(rpm) * ENCODER_RESOLUTION) / 60);
    
    uint8_t data[4] = {
        static_cast<uint8_t>((pulses_per_sec >> 0) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec >> 8) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec >> 16) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec >> 24) & 0xFF)
    };
    
    if (!writeSDO(OD_PROFILE_VELOCITY, 0x00, data, 4)) {
        std::cerr << "❌ 设置运动速度失败" << std::endl;
        return false;
    }
    
    return true;
}

bool InovanceServo::setProfileAcceleration(uint32_t acc)
{
    std::cout << "⚙️  设置加速度: " << acc << " RPM/s" << std::endl;
    
    // Profile Acceleration (0x6083) 单位: 脉冲/秒² (使用 uint64_t 避免溢出)
    uint32_t pulses_per_sec2 = static_cast<uint32_t>((static_cast<uint64_t>(acc) * ENCODER_RESOLUTION) / 60);
    
    uint8_t data[4] = {
        static_cast<uint8_t>((pulses_per_sec2 >> 0) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec2 >> 8) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec2 >> 16) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec2 >> 24) & 0xFF)
    };
    
    if (!writeSDO(OD_PROFILE_ACCELERATION, 0x00, data, 4)) {
        std::cerr << "❌ 设置加速度失败" << std::endl;
        return false;
    }
    
    return true;
}

bool InovanceServo::setProfileDeceleration(uint32_t dec)
{
    std::cout << "⚙️  设置减速度: " << dec << " RPM/s" << std::endl;
    
    // Profile Deceleration (0x6084) 单位: 脉冲/秒² (使用 uint64_t 避免溢出)
    uint32_t pulses_per_sec2 = static_cast<uint32_t>((static_cast<uint64_t>(dec) * ENCODER_RESOLUTION) / 60);
    
    uint8_t data[4] = {
        static_cast<uint8_t>((pulses_per_sec2 >> 0) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec2 >> 8) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec2 >> 16) & 0xFF),
        static_cast<uint8_t>((pulses_per_sec2 >> 24) & 0xFF)
    };
    
    if (!writeSDO(OD_PROFILE_DECELERATION, 0x00, data, 4)) {
        std::cerr << "❌ 设置减速度失败" << std::endl;
        return false;
    }
    
    return true;
}

// ==================== NMT 管理 ====================

bool InovanceServo::nmtStart()
{
    uint8_t data[2] = {0x01, 0x00};
    
    if (can_send(0x000, data, 2) < 0) {
        std::cerr << "❌ NMT Start 失败" << std::endl;
        return false;
    }
    
    std::cout << "✅ NMT 已启动 (PDO 开始)" << std::endl;
    return true;
}

bool InovanceServo::nmtPreOperational()
{
    uint8_t data[2] = {0x80, 0x00};
    
    if (can_send(0x000, data, 2) < 0) {
        std::cerr << "❌ NMT Pre-Operational 失败" << std::endl;
        return false;
    }
    
    std::cout << "✅ NMT 预操作 (PDO 停止)" << std::endl;
    return true;
}

// ==================== 抱闸控制 ====================

bool InovanceServo::releaseBrake()
{
    std::cout << "\n🔓 --- 松闸 (释放抱闸) ---" << std::endl;
    std::cout << "⚠️  警告：电机将失去保持力！" << std::endl;
    
    // H0d.26 (强制开启抱闸)
    // 通讯地址: 0x200D, 子索引 0x1B (27)
    // 值: 2 = 强制松闸
    uint8_t data[2] = {0x02, 0x00};  // 值 = 2
    
    if (!writeSDO(0x200D, 0x1B, data, 2)) {
        std::cerr << "❌ 松闸失败" << std::endl;
        return false;
    }
    
    std::cout << "✅ 松闸成功，电机可手动转动" << std::endl;
    return true;
}

bool InovanceServo::lockBrake()
{
    std::cout << "\n🔒 --- 锁闸 (恢复抱闸) ---" << std::endl;
    
    // H0d.26 (强制开启抱闸)
    // 值: 0 = 无强制（恢复正常）
    uint8_t data[2] = {0x00, 0x00};  // 值 = 0
    
    if (!writeSDO(0x200D, 0x1B, data, 2)) {
        std::cerr << "❌ 锁闸失败" << std::endl;
        return false;
    }
    
    std::cout << "✅ 锁闸成功" << std::endl;
    return true;
}

} // namespace pg
