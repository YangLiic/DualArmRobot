/****************************************************************
 * CyberGear Motor Driver
 * 
 * 小米 CyberGear 微电机控制驱动
 * 
 * 支持功能：
 *   - 使能/停止控制
 *   - 速度模式控制 (rad/s)
 *   - 位置模式控制 (rad / 角度)
 *   - 设置机械零位
 *   - 回到零位
 *   - 参数读写
 **/
#pragma once
#include "cybergear_can_usb.h"
#include <cstring>
#include <mutex>

namespace cybergear
{

// ==================== CyberGear 通信类型 ====================
enum CyberGearCmd : uint8_t {
    CMD_MOTOR_CONTROL   = 1,   // 运控模式指令
    CMD_FEEDBACK        = 2,   // 电机反馈
    CMD_ENABLE          = 3,   // 使能
    CMD_STOP            = 4,   // 停止 (reset)
    CMD_SET_ZERO        = 6,   // 设置机械零位
    CMD_CHANGE_CAN_ID   = 7,   // 修改 CAN ID
    CMD_RAM_READ        = 17,  // 读 RAM 参数
    CMD_RAM_WRITE       = 18,  // 写 RAM 参数
    CMD_FAULT_FEEDBACK  = 21,  // 故障反馈
};

// ==================== 运行模式 ====================
enum RunMode : uint8_t {
    MODE_MOTION   = 0,  // 运控模式 (MIT模式)
    MODE_POSITION = 1,  // 位置模式
    MODE_SPEED    = 2,  // 速度模式
    MODE_CURRENT  = 3,  // 电流模式
};

// ==================== 参数地址 ====================
enum ParamAddr : uint16_t {
    ADDR_RUN_MODE       = 0x7005,
    ADDR_IQ_REF         = 0x7006,
    ADDR_SPD_REF        = 0x700A,
    ADDR_LIMIT_TORQUE   = 0x700B,
    ADDR_CURRENT_KP     = 0x7010,
    ADDR_CURRENT_KI     = 0x7011,
    ADDR_CURRENT_FILTER = 0x7014,
    ADDR_LOC_REF        = 0x7016,
    ADDR_LIMIT_SPD      = 0x7017,
    ADDR_LIMIT_CUR      = 0x7018,
    ADDR_MECH_POS       = 0x7019,
    ADDR_IQF            = 0x701A,
    ADDR_MECH_VEL       = 0x701B,
    ADDR_VBUS           = 0x701C,
    ADDR_ROTATION       = 0x701D,
    ADDR_LOC_KP         = 0x701E,
    ADDR_SPD_KP         = 0x701F,
    ADDR_SPD_KI         = 0x7020,
};

// ==================== 参数范围 ====================
static const float P_MIN  = -12.5f;
static const float P_MAX  = 12.5f;
static const float V_MIN  = -30.0f;
static const float V_MAX  = 30.0f;
static const float T_MIN  = -12.0f;
static const float T_MAX  = 12.0f;
static const float IQ_MAX = 27.0f;
static const float KP_MAX = 500.0f;
static const float KD_MAX = 5.0f;

// ==================== 电机状态 ====================
struct MotorStatus {
    float position;     // 当前位置 (rad)
    float velocity;     // 当前速度 (rad/s)
    float torque;       // 当前力矩 (Nm)
    float temperature;  // 温度
    uint8_t motor_id;
    bool has_fault;
    bool at_limit;      // 是否处于限位触发状态
    
    MotorStatus() : position(0), velocity(0), torque(0), temperature(0),
                    motor_id(0), has_fault(false), at_limit(false) {}
};

// ==================== 电机驱动类 ====================
class CyberGearMotor : public CyberGearCanUsb
{
private:
    uint8_t motor_id_;    // 电机 CAN ID (默认 0x7F)
    uint8_t master_id_;   // 主机 CAN ID (默认 0x00)
    RunMode run_mode_;
    bool enabled_;
    
    // 行程限制 (rad), 默认 ±π (±180°)
    float limit_min_;    // 最小位置限制
    float limit_max_;    // 最大位置限制
    bool  limit_enabled_; // 是否启用限位
    
    MotorStatus status_;
    std::mutex status_mutex_;
    
    // 检查当前位置是否超限，超限时立即发停速指令
    void check_limit_in_speed_mode();
    
    // 构建 29-bit 扩展 CAN ID
    uint32_t make_ext_id(uint8_t cmd_id, uint16_t option, uint8_t target_id);
    
    // 发送命令
    void send_command(uint8_t cmd_id, uint16_t option, const uint8_t* data, size_t len);
    
    // 写 float 参数
    void write_float_param(uint16_t addr, float value);
    
    // 写 uint8 参数
    void write_uint8_param(uint16_t addr, uint8_t value);
    
    // float <-> uint 转换
    static int float_to_uint(float x, float x_min, float x_max, int bits);
    static float uint_to_float(uint16_t x, float x_min, float x_max);
    
    // 收到 CAN 帧时的回调
    void on_can_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc) override;
    
    // 处理反馈帧
    void process_feedback(uint32_t can_id, const uint8_t* data);
    void process_param_response(const uint8_t* data);

public:
    /**
     * @param port     串口设备路径
     * @param baud     串口波特率 (默认 921600)
     * @param motor_id 电机 CAN ID (默认 0x7F)
     * @param master_id 主机 CAN ID (默认 0x00)
     */
    CyberGearMotor(const std::string& port, int baud = 921600,
                   uint8_t motor_id = 0x7F, uint8_t master_id = 0x00);
    ~CyberGearMotor();
    
    // ==================== 基础控制 ====================
    
    /** 使能电机 */
    void enable();
    
    /** 停止电机 (reset) */
    void stop();
    
    /** 设置运行模式 */
    void setRunMode(RunMode mode);
    
    // ==================== 行程限制 ====================
    
    /**
     * 设置行程限制 (rad)，相对于当前零位
     * 默认值: min=-π, max=+π (即 ±180°)
     * @param min 最小位置 (rad)，例如 -M_PI
     * @param max 最大位置 (rad)，例如 +M_PI
     */
    void setTravelLimit(float min_rad, float max_rad);
    
    /** 启用/禁用行程限制 */
    void enableTravelLimit(bool enable);
    
    /** 获取行程限制是否启用 */
    bool isTravelLimitEnabled() const { return limit_enabled_; }
    
    /** 获取行程范围 (min, max) in rad */
    std::pair<float, float> getTravelLimit() const { return {limit_min_, limit_max_}; }
    
    // ==================== 速度模式 ====================
    
    /** 进入速度模式并使能 */
    void enableSpeedMode();
    
    /** 设置速度 (rad/s)，范围 ±30 */
    void setSpeed(float speed_rad_s);
    
    /** 设置速度 (RPM) */
    void setSpeedRPM(float rpm);
    
    /** 设置电流限制 (A)，范围 0-27 */
    void setCurrentLimit(float current);
    
    // ==================== 位置模式 ====================
    
    /** 进入位置模式并使能 */
    void enablePositionMode();
    
    /** 设置目标位置 (rad)，范围 ±12.5 */
    void setPosition(float position_rad);
    
    /** 设置目标位置 (角度) */
    void setPositionDeg(float degrees);
    
    /** 设置位置模式下的速度限制 (rad/s)，范围 0-30 */
    void setSpeedLimit(float limit_rad_s);
    
    // ==================== 零位操作 ====================
    
    /**
     * 设置当前位置为机械零位（上电期间有效，掉电丢失）
     * 设置后反馈帧位置归零，行程限制以此为基准
     */
    void setMechPositionToZero();
    
    /**
     * 主动请求电机反馈帧（触发一次 type 2 响应）
     * 用于获取最新的位置/速度/力矩/温度
     */
    void requestFeedback();
    
    /** 回到零位 (位置模式下移动到 0 rad) */
    void goToZero(float speed_limit = 2.0f);
    
    // ==================== 参数读写 ====================
    
    /** 读取 RAM 参数 */
    void readParam(uint16_t addr);
    
    // ==================== 状态 ====================
    
    /** 获取电机状态 */
    MotorStatus getStatus();
    
    /** 获取使能状态 */
    bool isEnabled() const { return enabled_; }
    
    /** 获取当前运行模式 */
    RunMode getRunMode() const { return run_mode_; }
    
    /** 获取电机 ID */
    uint8_t getMotorId() const { return motor_id_; }
    
    /** 获取当前位置是否处于限位 */
    bool isAtLimit();
    
    /** 检查并强制执行限位（供轮询线程调用） */
    void checkAndEnforceLimit() { check_limit_in_speed_mode(); }
};

} // namespace cybergear
