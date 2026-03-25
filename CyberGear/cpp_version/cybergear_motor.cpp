/****************************************************************
 * CyberGear Motor Driver - Implementation
 **/
#include "cybergear_motor.h"
#include <cmath>
#include <thread>
#include <chrono>

namespace cybergear
{

// ==================== 构造/析构 ====================

CyberGearMotor::CyberGearMotor(const std::string& port, int baud,
                                uint8_t motor_id, uint8_t master_id)
    : CyberGearCanUsb(port, baud, 100),
      motor_id_(motor_id), master_id_(master_id),
      run_mode_(MODE_MOTION), enabled_(false),
      limit_min_(-M_PI), limit_max_(M_PI), limit_enabled_(true)
{
    init_can();
    
    std::cout << "🔧 CyberGear Motor 初始化" << std::endl;
    std::cout << "   电机 ID: 0x" << std::hex << (int)motor_id_ 
              << ", 主机 ID: 0x" << (int)master_id_ << std::dec << std::endl;
    std::cout << "   行程限制: ±180° (±π rad), 已启用" << std::endl;
}

CyberGearMotor::~CyberGearMotor()
{
    if (enabled_) {
        stop();
    }
}

// ==================== 内部函数 ====================

uint32_t CyberGearMotor::make_ext_id(uint8_t cmd_id, uint16_t option, uint8_t target_id)
{
    // 29-bit CAN ID: cmd_id[28:24] | option[23:8] | target_id[7:0]
    return ((uint32_t)cmd_id << 24) | ((uint32_t)option << 8) | (uint32_t)target_id;
}

void CyberGearMotor::send_command(uint8_t cmd_id, uint16_t option, 
                                   const uint8_t* data, size_t len)
{
    uint32_t ext_id = make_ext_id(cmd_id, option, motor_id_);
    can_send_ext(ext_id, data, len);
    
    // 等待一小段时间让电机处理
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void CyberGearMotor::write_float_param(uint16_t addr, float value)
{
    uint8_t data[8] = {0x00};
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    memcpy(&data[4], &value, 4);
    send_command(CMD_RAM_WRITE, master_id_, data, 8);
}

void CyberGearMotor::write_uint8_param(uint16_t addr, uint8_t value)
{
    uint8_t data[8] = {0x00};
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    data[4] = value;
    send_command(CMD_RAM_WRITE, master_id_, data, 8);
}

int CyberGearMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

float CyberGearMotor::uint_to_float(uint16_t x, float x_min, float x_max)
{
    float span = x_max - x_min;
    return (float)x / 65535.0f * span + x_min;
}

// ==================== 行程限制内部检查 ====================

void CyberGearMotor::check_limit_in_speed_mode()
{
    if (!limit_enabled_ || !enabled_ || run_mode_ != MODE_SPEED) return;
    
    bool at_lim;
    float pos;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        at_lim = status_.at_limit;
        pos = status_.position;
    }
    
    if (!at_lim) return;
    
    // 超限：立即发速度=0 指令
    float zero_spd = 0.0f;
    uint8_t data[8] = {0x00};
    data[0] = ADDR_SPD_REF & 0xFF;
    data[1] = (ADDR_SPD_REF >> 8) & 0xFF;
    memcpy(&data[4], &zero_spd, 4);
    send_command(CMD_RAM_WRITE, master_id_, data, 8);
    
    if (!isSilentMode()) {
        const char* dir = (pos >= limit_max_) ? "最大" : "最小";
        float lim_deg = (pos >= limit_max_) ? limit_max_ : limit_min_;
        printf("⛔ 触发%s限位 (%.1f°), 已强制停速!\n", dir, lim_deg * 180.0f / M_PI);
    }
}

// ==================== CAN 帧接收回调 ====================

void CyberGearMotor::on_can_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
{
    uint8_t receive_id = can_id & 0xFF;          // 目标 (主机)
    uint8_t motor_can_id = (can_id >> 8) & 0xFF; // 来源 (电机)
    uint8_t cmd_type = (can_id >> 24) & 0x3F;    // 通信类型
    
    if (receive_id != master_id_) return;
    if (motor_can_id != motor_id_) return;
    
    switch (cmd_type) {
        case CMD_FEEDBACK:
            process_feedback(can_id, data);
            // 速度模式下实时检查是否触发限位
            if (enabled_ && run_mode_ == MODE_SPEED) {
                check_limit_in_speed_mode();
            }
            break;
        case CMD_RAM_READ:
            process_param_response(data);
            break;
        case CMD_FAULT_FEEDBACK:
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                status_.has_fault = true;
            }
            if (!isSilentMode()) {
                std::cerr << "⚠️  电机故障反馈!" << std::endl;
            }
            break;
        default:
            break;
    }
}

void CyberGearMotor::process_feedback(uint32_t can_id, const uint8_t* data)
{
    uint16_t raw_pos  = (data[0] << 8) | data[1];
    uint16_t raw_vel  = (data[2] << 8) | data[3];
    uint16_t raw_torq = (data[4] << 8) | data[5];
    uint16_t raw_temp = (data[6] << 8) | data[7];
    
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.position    = uint_to_float(raw_pos, P_MIN, P_MAX);
    status_.velocity    = uint_to_float(raw_vel, V_MIN, V_MAX);
    status_.torque      = uint_to_float(raw_torq, T_MIN, T_MAX);
    status_.temperature = (float)raw_temp;
    status_.motor_id    = motor_id_;
    
    // 更新限位状态（反馈帧的位置相对于硬件零位）
    if (limit_enabled_) {
        status_.at_limit = (status_.position <= limit_min_ || status_.position >= limit_max_);
    }
}

void CyberGearMotor::process_param_response(const uint8_t* data)
{
    uint16_t index = (data[1] << 8) | data[0];
    float float_val;
    memcpy(&float_val, &data[4], sizeof(float));
    
    if (!isSilentMode()) {
        printf("   参数 [0x%04X] = %f\n", index, float_val);
    }
    
    // 注意: 当前固件的 type 17 回复数据不可靠，
    // 实际位置/速度应从反馈帧 (type 2) 获取
}

// ==================== 基础控制 ====================

void CyberGearMotor::enable()
{
    uint8_t data[8] = {0x00};
    send_command(CMD_ENABLE, master_id_, data, 8);
    enabled_ = true;
    
    if (!isSilentMode()) {
        std::cout << "✅ 电机使能" << std::endl;
    }
}

void CyberGearMotor::stop()
{
    uint8_t data[8] = {0x00};
    send_command(CMD_STOP, master_id_, data, 8);
    enabled_ = false;
    
    if (!isSilentMode()) {
        std::cout << "⏹️  电机停止" << std::endl;
    }
}

void CyberGearMotor::setRunMode(RunMode mode)
{
    run_mode_ = mode;
    write_uint8_param(ADDR_RUN_MODE, (uint8_t)mode);
    
    if (!isSilentMode()) {
        const char* mode_names[] = {"运控", "位置", "速度", "电流"};
        std::cout << "⚙️  运行模式: " << mode_names[mode] << std::endl;
    }
}

// ==================== 行程限制 ====================

void CyberGearMotor::setTravelLimit(float min_rad, float max_rad)
{
    if (min_rad >= max_rad) {
        std::cerr << "❌ 行程限制无效: min 必须小于 max" << std::endl;
        return;
    }
    // 行程限制不能超出电机硬件限幅
    limit_min_ = std::max(min_rad, P_MIN);
    limit_max_ = std::min(max_rad, P_MAX);
    
    if (!isSilentMode()) {
        printf("📐 行程限制已设置: [%.1f°, +%.1f°] / [%.3f, %.3f] rad\n",
               limit_min_ * 180.0f / M_PI, limit_max_ * 180.0f / M_PI,
               limit_min_, limit_max_);
    }
}

void CyberGearMotor::enableTravelLimit(bool enable)
{
    limit_enabled_ = enable;
    if (!isSilentMode()) {
        std::cout << "📐 行程限制: " << (enable ? "已启用" : "已禁用") << std::endl;
    }
}

bool CyberGearMotor::isAtLimit()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_.at_limit;
}

// ==================== 速度模式 ====================

void CyberGearMotor::enableSpeedMode()
{
    // 重置限位状态，防止上次残留的 at_limit=true 阻止运动
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.at_limit = false;
    }
    
    stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    setRunMode(MODE_SPEED);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    setCurrentLimit(5.0f);
    
    if (!isSilentMode()) {
        std::cout << "✅ 速度模式已就绪" << std::endl;
        if (limit_enabled_) {
            printf("   行程保护: [%.1f°, +%.1f°]\n",
                   limit_min_ * 180.0f / M_PI, limit_max_ * 180.0f / M_PI);
        }
    }
}

void CyberGearMotor::setSpeed(float speed_rad_s)
{
    // 检查当前位置是否允许继续运动
    if (limit_enabled_) {
        float cur_pos;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            cur_pos = status_.position;
        }
        // 如果已在限位，只允许反向运动
        if (cur_pos >= limit_max_ && speed_rad_s > 0) {
            if (!isSilentMode()) {
                std::cerr << "⛔ 已在最大限位 (" << limit_max_ * 180.0f / M_PI 
                          << "°), 拒绝正向速度指令" << std::endl;
            }
            return;
        }
        if (cur_pos <= limit_min_ && speed_rad_s < 0) {
            if (!isSilentMode()) {
                std::cerr << "⛔ 已在最小限位 (" << limit_min_ * 180.0f / M_PI 
                          << "°), 拒绝负向速度指令" << std::endl;
            }
            return;
        }
    }
    
    // 速度范围限幅
    if (speed_rad_s > V_MAX) speed_rad_s = V_MAX;
    if (speed_rad_s < V_MIN) speed_rad_s = V_MIN;
    
    write_float_param(ADDR_SPD_REF, speed_rad_s);
    
    if (!isSilentMode()) {
        std::cout << "🎯 速度: " << speed_rad_s << " rad/s ("
                  << speed_rad_s * 60.0f / (2.0f * M_PI) << " RPM)" << std::endl;
    }
}

void CyberGearMotor::setSpeedRPM(float rpm)
{
    float rad_s = rpm * 2.0f * M_PI / 60.0f;
    setSpeed(rad_s);
}

void CyberGearMotor::setCurrentLimit(float current)
{
    if (current > IQ_MAX) current = IQ_MAX;
    if (current < 0) current = 0;
    write_float_param(ADDR_LIMIT_CUR, current);
}

// ==================== 位置模式 ====================

void CyberGearMotor::enablePositionMode()
{
    stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    setRunMode(MODE_POSITION);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    setSpeedLimit(2.0f);
    setCurrentLimit(5.0f);
    
    if (!isSilentMode()) {
        std::cout << "✅ 位置模式已就绪" << std::endl;
        if (limit_enabled_) {
            printf("   行程保护: [%.1f°, +%.1f°]\n",
                   limit_min_ * 180.0f / M_PI, limit_max_ * 180.0f / M_PI);
        }
    }
}

void CyberGearMotor::setPosition(float position_rad)
{
    // 行程限制检查：超出范围则截断到边界
    if (limit_enabled_) {
        if (position_rad > limit_max_) {
            if (!isSilentMode()) {
                std::cerr << "⚠️  目标位置 " << position_rad * 180.0f / M_PI
                          << "° 超出最大限位 " << limit_max_ * 180.0f / M_PI
                          << "°, 已截断" << std::endl;
            }
            position_rad = limit_max_;
        }
        if (position_rad < limit_min_) {
            if (!isSilentMode()) {
                std::cerr << "⚠️  目标位置 " << position_rad * 180.0f / M_PI
                          << "° 超出最小限位 " << limit_min_ * 180.0f / M_PI
                          << "°, 已截断" << std::endl;
            }
            position_rad = limit_min_;
        }
    }
    
    // 电机硬件限幅
    if (position_rad > P_MAX) position_rad = P_MAX;
    if (position_rad < P_MIN) position_rad = P_MIN;
    
    write_float_param(ADDR_LOC_REF, position_rad);
    
    if (!isSilentMode()) {
        std::cout << "🎯 目标位置: " << position_rad << " rad ("
                  << position_rad * 180.0f / M_PI << "°)" << std::endl;
    }
}

void CyberGearMotor::setPositionDeg(float degrees)
{
    float rad = degrees * M_PI / 180.0f;
    setPosition(rad);
}

void CyberGearMotor::setSpeedLimit(float limit_rad_s)
{
    if (limit_rad_s > 30.0f) limit_rad_s = 30.0f;
    if (limit_rad_s < 0) limit_rad_s = 0;
    write_float_param(ADDR_LIMIT_SPD, limit_rad_s);
}

// ==================== 零位操作 ====================

void CyberGearMotor::setMechPositionToZero()
{
    uint8_t data[8] = {0x00};
    data[0] = 0x01;
    send_command(CMD_SET_ZERO, master_id_, data, 8);
    
    // 重置位置状态
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.position = 0.0f;
        status_.at_limit = false;
    }
    
    if (!isSilentMode()) {
        std::cout << "✅ 已设置当前位置为机械零位 (上电期间有效)" << std::endl;
    }
}

void CyberGearMotor::requestFeedback()
{
    // 重发 enable 指令, 电机回复 type 2 反馈帧
    // 对已使能的电机无副作用
    uint8_t data[8] = {0x00};
    send_command(CMD_ENABLE, master_id_, data, 8);
}

void CyberGearMotor::goToZero(float speed_limit)
{
    if (run_mode_ != MODE_POSITION) {
        enablePositionMode();
    }
    
    setSpeedLimit(speed_limit);
    setPosition(0.0f);  // 已经过行程限制检查
    
    if (!isSilentMode()) {
        std::cout << "🏠 正在回零位... (限速 " << speed_limit << " rad/s)" << std::endl;
    }
}

// ==================== 参数读写 ====================

void CyberGearMotor::readParam(uint16_t addr)
{
    uint8_t data[8] = {0x00};
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    send_command(CMD_RAM_READ, master_id_, data, 8);
}

// ==================== 状态 ====================

MotorStatus CyberGearMotor::getStatus()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

} // namespace cybergear
