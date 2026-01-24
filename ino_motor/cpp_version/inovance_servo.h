/****************************************************************
 * Inovance Servo SDK - CANopen Implementation
 * 
 * 支持功能：
 * - 使能/失能控制
 * - 速度模式控制
 * - 位置模式控制
 * - 错误复位
 * - NMT 状态管理
 **/
#pragma once
#include "hw_can_usb.h"
#include <iostream>

namespace pg
{

// 运行模式枚举
enum class OperationMode : uint8_t {
    PROFILE_POSITION = 1,   // 位置模式
    VELOCITY = 3,           // 速度模式
    PROFILE_VELOCITY = 3,   // 速度模式 (别名)
    HOMING = 6,             // 回零模式
    CYCLIC_SYNC_POSITION = 8, // 循环同步位置模式
    CYCLIC_SYNC_VELOCITY = 9  // 循环同步速度模式
};

// 控制字命令
enum class ControlWord : uint16_t {
    SHUTDOWN = 0x0006,
    SWITCH_ON = 0x0007,
    ENABLE_OPERATION = 0x000F,
    DISABLE_VOLTAGE = 0x0000,
    QUICK_STOP = 0x0002,
    FAULT_RESET = 0x0080
};

class InovanceServo : public CanInterfaceUsb
{
private:
    uint32_t node_id_;
    bool motor_enabled_;
    OperationMode current_mode_;
    bool direction_inverted_;  // 方向反转标志
    
    void decode() override;
    
    // 内部辅助函数
    bool setOperationMode(OperationMode mode);
    bool sendControlWord(ControlWord cmd);
    bool writeSDO(uint16_t index, uint8_t subindex, const uint8_t* data, size_t len);
    
public:
    InovanceServo(const std::string& port_name, int baud_rate, uint32_t node_id = 0x601);
    ~InovanceServo();
    
    /**
     * @brief 设置方向反转
     * @param inverted true=反转方向, false=正常方向
     * 
     * 用于统一不同安装方向的电机，使"顺时针"对所有电机表现一致
     */
    void setDirectionInverted(bool inverted);
    
    /**
     * @brief 获取方向反转状态
     */
    bool isDirectionInverted() const { return direction_inverted_; }
    
    // ==================== 基础控制 ====================
    
    /**
     * @brief 使能电机 (速度模式)
     */
    bool enable();
    
    /**
     * @brief 使能电机 (指定模式)
     * @param mode 运行模式
     */
    bool enable(OperationMode mode);
    
    /**
     * @brief 失能电机
     */
    bool disable();
    
    /**
     * @brief 错误复位
     */
    bool faultReset();
    
    /**
     * @brief 急停
     */
    bool quickStop();
    
    /**
     * @brief 停止电机并失能
     */
    bool stop();
    
    // ==================== 速度控制 ====================
    
    /**
     * @brief 设置目标速度 (RPM)
     * @param rpm 转速，正值顺时针，负值逆时针
     */
    bool setVelocity(int32_t rpm);
    
    // ==================== 位置控制 ====================
    
    /**
     * @brief 设置目标位置 (角度)
     * @param degrees 角度，正值顺时针，负值逆时针
     * @param absolute 是否为绝对位置 (默认相对)
     */
    bool setPosition(double degrees, bool absolute = false);
    
    /**
     * @brief 设置目标位置 (脉冲)
     * @param pulses 脉冲数
     * @param absolute 是否为绝对位置
     */
    bool setPositionPulse(int32_t pulses, bool absolute = false);
    
    /**
     * @brief 启动位置运动
     * @param relative 是否为相对位置 (默认 true)
     */
    bool startPositionMove(bool relative = true);
    
    /**
     * @brief 设置位置模式下的运动速度 (Profile Velocity)
     * @param rpm 速度限制 (RPM)
     */
    bool setProfileVelocity(uint32_t rpm);
    
    /**
     * @brief 设置位置模式下的加速度
     * @param acc 加速度 (RPM/s)
     */
    bool setProfileAcceleration(uint32_t acc);
    
    /**
     * @brief 设置位置模式下的减速度
     * @param dec 减速度 (RPM/s)
     */
    bool setProfileDeceleration(uint32_t dec);
    
    // ==================== NMT 管理 ====================
    
    /**
     * @brief NMT 启动 (开始 PDO 通信)
     */
    bool nmtStart();
    
    /**
     * @brief NMT 预操作 (停止 PDO)
     */
    bool nmtPreOperational();
    
    // ==================== 状态查询 ====================
    
    bool isEnabled() const { return motor_enabled_; }
    OperationMode getCurrentMode() const { return current_mode_; }
    
    // ==================== 兼容旧接口 ====================
    
    [[deprecated("使用 enable() 代替")]]
    bool enableMotor() { return enable(); }
    
    [[deprecated("使用 disable() 代替")]]
    bool disableMotor() { return disable(); }
    
    [[deprecated("使用 stop() 代替")]]
    bool stopMotor() { return stop(); }
};

} // namespace pg
