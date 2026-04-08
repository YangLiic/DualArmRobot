/****************************************************************
 * Inovance Servo SDK - CANopen Implementation
 **/
#include "inovance_servo.h"
#include <chrono>
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
constexpr uint16_t OD_POSITION_DEVIATION = 0x60F4;
constexpr uint16_t OD_TARGET_TORQUE = 0x6071;
constexpr uint16_t OD_MAX_TORQUE = 0x6072;
constexpr uint16_t OD_ACTUAL_TORQUE = 0x6077;
constexpr uint16_t OD_TORQUE_RAMP = 0x6087;
constexpr uint16_t OD_TARGET_VELOCITY = 0x60FF;
constexpr uint16_t OD_PROFILE_VELOCITY = 0x6081;
constexpr uint16_t OD_PROFILE_ACCELERATION = 0x6083;
constexpr uint16_t OD_PROFILE_DECELERATION = 0x6084;
constexpr uint16_t OD_FORWARD_TORQUE_LIMIT = 0x60E0;
constexpr uint16_t OD_REVERSE_TORQUE_LIMIT = 0x60E1;
constexpr uint16_t OD_AVERAGE_LOAD = 0x200B;
constexpr uint8_t OD_AVERAGE_LOAD_SUB = 0x0D;
constexpr uint16_t OD_PHASE_CURRENT = 0x200B;
constexpr uint8_t OD_PHASE_CURRENT_SUB = 0x19;

// 编码器参数
constexpr int32_t ENCODER_RESOLUTION = 8388608; // 23位编码器 (2^23)
constexpr double PULSES_PER_DEGREE = ENCODER_RESOLUTION / 360.0;

InovanceServo::InovanceServo(const std::string& port_name, int baud_rate, uint32_t node_id)
    : CanInterfaceUsb(port_name, baud_rate, 100), 
      node_id_(node_id), 
      response_id_(node_id >= 0x80 ? node_id - 0x80 : node_id),
      motor_enabled_(false),
      current_mode_(OperationMode::VELOCITY),
      direction_inverted_(false),
      actual_torque_available_(false),
      phase_current_available_(false),
      position_deviation_available_(false),
      collision_thread_running_(false),
      collision_protection_enabled_(false),
      collision_triggered_(false)
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
    disableCollisionProtection();
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

bool InovanceServo::writeSDO(uint16_t index, uint8_t subindex, const uint8_t* data, size_t len, bool wait_response)
{
    std::lock_guard<std::mutex> lock(io_mutex_);
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
    
    const uint64_t sequence = getFrameSequence();
    if (can_send(node_id_, frame, 8) < 0) {
        return false;
    }

    if (!wait_response) {
        usleep(10000);
        return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
    while (std::chrono::steady_clock::now() < deadline) {
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
        if (remaining_ms <= 0) {
            break;
        }

        CanFrame rx{};
        if (!waitForNextFrame(sequence, rx, remaining_ms)) {
            break;
        }

        if (rx.can_id != response_id_) {
            continue;
        }
        if (rx.dlc < 4) {
            continue;
        }
        if (rx.data[1] != (index & 0xFF) || rx.data[2] != ((index >> 8) & 0xFF) || rx.data[3] != subindex) {
            continue;
        }
        if (rx.data[0] == 0x80) {
            std::cerr << "❌ SDO写入被驱动器拒绝: 0x" << std::hex << index << ":" << static_cast<int>(subindex) << std::dec << std::endl;
            return false;
        }
        if (rx.data[0] == 0x60) {
            return true;
        }
    }

    std::cerr << "❌ SDO写入超时: 0x" << std::hex << index << ":" << static_cast<int>(subindex) << std::dec << std::endl;
    return false;
}

bool InovanceServo::readSDO(uint16_t index, uint8_t subindex, uint8_t* data, size_t len, int timeout_ms, bool log_error)
{
    if (!(len == 1 || len == 2 || len == 4)) {
        if (log_error) {
            std::cerr << "不支持的数据长度: " << len << std::endl;
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(io_mutex_);

    uint8_t frame[8] = {
        0x40,
        static_cast<uint8_t>(index & 0xFF),
        static_cast<uint8_t>((index >> 8) & 0xFF),
        subindex,
        0x00, 0x00, 0x00, 0x00
    };

    uint64_t sequence = getFrameSequence();
    if (can_send(node_id_, frame, 8) < 0) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
        if (remaining_ms <= 0) {
            break;
        }

        CanFrame rx{};
        if (!waitForNextFrame(sequence, rx, remaining_ms)) {
            break;
        }
        sequence = rx.sequence;

        if (rx.can_id != response_id_) {
            continue;
        }
        if (rx.dlc < 4) {
            continue;
        }
        if (rx.data[1] != (index & 0xFF) || rx.data[2] != ((index >> 8) & 0xFF) || rx.data[3] != subindex) {
            continue;
        }
        if (rx.data[0] == 0x80) {
            if (log_error) {
                std::cerr << "❌ SDO读取被驱动器拒绝: 0x" << std::hex << index << ":" << static_cast<int>(subindex) << std::dec << std::endl;
            }
            return false;
        }

        const uint8_t expected_cmd = (len == 1) ? 0x4F : ((len == 2) ? 0x4B : 0x43);
        if (rx.data[0] != expected_cmd) {
            continue;
        }

        memcpy(data, &rx.data[4], len);
        return true;
    }

    if (log_error) {
        std::cerr << "❌ SDO读取超时: 0x" << std::hex << index << ":" << static_cast<int>(subindex) << std::dec << std::endl;
    }
    return false;
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
    
    return writeSDO(OD_CONTROL_WORD, 0x00, data, 2, false);
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

void InovanceServo::stopCollisionProtectionThread()
{
    collision_protection_enabled_ = false;
    collision_thread_running_ = false;
    if (collision_thread_.joinable()) {
        collision_thread_.join();
    }
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
    
    if (!writeSDO(OD_TARGET_VELOCITY, 0x00, data, 4, false)) {
        std::cerr << "❌ 设置速度失败" << std::endl;
        return false;
    }
    
    return true;
}

bool InovanceServo::setMaxTorqueLimit(uint16_t permille)
{
    uint8_t data[2] = {
        static_cast<uint8_t>(permille & 0xFF),
        static_cast<uint8_t>((permille >> 8) & 0xFF)
    };
    return writeSDO(OD_MAX_TORQUE, 0x00, data, 2);
}

bool InovanceServo::setDirectionalTorqueLimits(uint16_t forward_permille, uint16_t reverse_permille)
{
    uint8_t forward_data[2] = {
        static_cast<uint8_t>(forward_permille & 0xFF),
        static_cast<uint8_t>((forward_permille >> 8) & 0xFF)
    };
    uint8_t reverse_data[2] = {
        static_cast<uint8_t>(reverse_permille & 0xFF),
        static_cast<uint8_t>((reverse_permille >> 8) & 0xFF)
    };
    return writeSDO(OD_FORWARD_TORQUE_LIMIT, 0x00, forward_data, 2)
        && writeSDO(OD_REVERSE_TORQUE_LIMIT, 0x00, reverse_data, 2);
}

bool InovanceServo::setTorqueRamp(uint32_t permille_per_second)
{
    uint8_t data[4] = {
        static_cast<uint8_t>((permille_per_second >> 0) & 0xFF),
        static_cast<uint8_t>((permille_per_second >> 8) & 0xFF),
        static_cast<uint8_t>((permille_per_second >> 16) & 0xFF),
        static_cast<uint8_t>((permille_per_second >> 24) & 0xFF)
    };
    return writeSDO(OD_TORQUE_RAMP, 0x00, data, 4);
}

int16_t InovanceServo::readActualTorquePermille()
{
    uint8_t data[2];
    if (!readSDO(OD_ACTUAL_TORQUE, 0x00, data, 2, 100, false)) {
        return 0;
    }
    return static_cast<int16_t>(data[0] | (data[1] << 8));
}

double InovanceServo::readPhaseCurrentAmp()
{
    uint8_t data[2];
    if (!readSDO(OD_PHASE_CURRENT, OD_PHASE_CURRENT_SUB, data, 2, 100, false)) {
        return 0.0;
    }
    const uint16_t centi_amp = static_cast<uint16_t>(data[0] | (data[1] << 8));
    return static_cast<double>(centi_amp) / 100.0;
}

double InovanceServo::readAverageLoadPercent()
{
    uint8_t data[2];
    if (!readSDO(OD_AVERAGE_LOAD, OD_AVERAGE_LOAD_SUB, data, 2, 100, false)) {
        return 0.0;
    }
    const uint16_t deci_percent = static_cast<uint16_t>(data[0] | (data[1] << 8));
    return static_cast<double>(deci_percent) / 10.0;
}

int32_t InovanceServo::readPositionDeviation()
{
    uint8_t data[4];
    if (!readSDO(OD_POSITION_DEVIATION, 0x00, data, 4, 100, false)) {
        return 0;
    }
    return static_cast<int32_t>(
        data[0]
        | (data[1] << 8)
        | (data[2] << 16)
        | (data[3] << 24));
}

bool InovanceServo::enableCollisionProtection(
    uint16_t torque_limit_permille,
    uint16_t trigger_torque_permille,
    double trigger_current_amp,
    int32_t trigger_position_deviation,
    int consecutive_samples,
    int poll_interval_ms,
    bool use_quick_stop)
{
    collision_config_.torque_limit_permille = torque_limit_permille;
    collision_config_.forward_torque_limit_permille = 0;
    collision_config_.reverse_torque_limit_permille = 0;
    collision_config_.trigger_torque_permille = trigger_torque_permille;
    collision_config_.trigger_current_amp = trigger_current_amp;
    collision_config_.trigger_position_deviation = trigger_position_deviation;
    collision_config_.consecutive_samples = std::max(1, consecutive_samples);
    collision_config_.poll_interval_ms = std::max(5, poll_interval_ms);
    collision_config_.use_quick_stop = use_quick_stop;
    collision_triggered_ = false;

    if (!setMaxTorqueLimit(torque_limit_permille)) {
        std::cerr << "❌ 设置最大转矩限制失败" << std::endl;
        return false;
    }

    uint8_t probe2[2];
    uint8_t probe4[4];
    actual_torque_available_ = readSDO(OD_ACTUAL_TORQUE, 0x00, probe2, 2, 150, false);
    phase_current_available_ = readSDO(OD_PHASE_CURRENT, OD_PHASE_CURRENT_SUB, probe2, 2, 150, false);
    position_deviation_available_ = readSDO(OD_POSITION_DEVIATION, 0x00, probe4, 4, 150, false);

    if (!collision_thread_.joinable()) {
        collision_thread_running_ = true;
        collision_thread_ = std::thread([this]() {
            collisionProtectionLoop();
        });
    }

    collision_protection_enabled_ =
        actual_torque_available_ || phase_current_available_ || position_deviation_available_;

    std::cout << "🛡️  碰撞保护已启用: 转矩限制=" << torque_limit_permille
              << "‰, 触发转矩=" << trigger_torque_permille << "‰";
    if (trigger_current_amp > 0.0) {
        std::cout << ", 触发电流=" << trigger_current_amp << "A";
    }
    if (trigger_position_deviation > 0) {
        std::cout << ", 位置偏差阈值=" << trigger_position_deviation;
    }
    std::cout << std::endl;

    if (!collision_protection_enabled_) {
        std::cout << "⚠️  当前驱动未返回可用监控对象，已降级为“仅转矩限制”，不会执行自动碰撞停机。" << std::endl;
    } else {
        std::cout << "   可用监控: "
                  << (actual_torque_available_ ? "actual_torque " : "")
                  << (phase_current_available_ ? "phase_current " : "")
                  << (position_deviation_available_ ? "position_deviation" : "")
                  << std::endl;
    }
    return true;
}

void InovanceServo::disableCollisionProtection()
{
    stopCollisionProtectionThread();
}

void InovanceServo::collisionProtectionLoop()
{
    int consecutive_hits = 0;

    while (collision_thread_running_) {
        if (!collision_protection_enabled_ || !motor_enabled_) {
            consecutive_hits = 0;
            usleep(50000);
            continue;
        }

        const int16_t actual_torque = actual_torque_available_ ? readActualTorquePermille() : 0;
        const double phase_current = phase_current_available_ ? readPhaseCurrentAmp() : 0.0;
        const int32_t position_deviation = position_deviation_available_ ? readPositionDeviation() : 0;

        bool triggered = false;
        if (actual_torque_available_
            && std::abs(actual_torque) >= static_cast<int>(collision_config_.trigger_torque_permille)) {
            triggered = true;
        }
        if (phase_current_available_
            && collision_config_.trigger_current_amp > 0.0
            && phase_current >= collision_config_.trigger_current_amp) {
            triggered = true;
        }
        if (position_deviation_available_
            && collision_config_.trigger_position_deviation > 0
            && std::abs(position_deviation) >= collision_config_.trigger_position_deviation) {
            triggered = true;
        }

        if (triggered) {
            consecutive_hits++;
        } else {
            consecutive_hits = 0;
        }

        if (consecutive_hits >= collision_config_.consecutive_samples) {
            collision_triggered_ = true;
            collision_protection_enabled_ = false;
            std::cerr << "\n🛑 碰撞保护触发: 实际转矩=" << actual_torque
                      << "‰, 相电流=" << phase_current
                      << "A, 位置偏差=" << position_deviation << std::endl;
            if (collision_config_.use_quick_stop) {
                quickStop();
            } else {
                stop();
            }
            consecutive_hits = 0;
        }

        usleep(collision_config_.poll_interval_ms * 1000);
    }
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
    
    if (!writeSDO(OD_CONTROL_WORD, 0x00, data, 2, false)) {
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
