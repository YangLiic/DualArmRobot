/****************************************************************
 * Inovance Servo - Standalone Implementation (No ROS)
 **/
#include "inovance_servo.h"
#include <unistd.h>
#include <cstring>

namespace pg
{

InovanceServo::InovanceServo(const std::string& port_name, int baud_rate, uint32_t node_id)
    : CanInterfaceUsb(port_name, baud_rate, 100), node_id_(node_id), motor_enabled_(false)
{
    std::cout << "Inovance Servo initialized, Node ID: 0x" << std::hex << node_id_ << std::dec << std::endl;
}

InovanceServo::~InovanceServo()
{
    if (motor_enabled_) {
        stopMotor();
    }
}

void InovanceServo::decode()
{
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    
    get_can_id(can_id);
    get_data(data, dlc);
    
    // 处理响应 (可选)
}

bool InovanceServo::setVelocityMode()
{
    uint8_t data[8] = {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
    
    if (can_send(node_id_, data, 8) < 0) {
        std::cerr << "Failed to set velocity mode" << std::endl;
        return false;
    }
    
    std::cout << "✅ 设置为速度模式" << std::endl;
    usleep(50000);
    return true;
}

bool InovanceServo::enableMotor()
{
    std::cout << "\n--- 开始使能流程 ---" << std::endl;
    
    // 1. 设置速度模式
    if (!setVelocityMode()) {
        return false;
    }
    
    // 2. Shutdown (0x06)
    uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    if (can_send(node_id_, data, 8) < 0) {
        std::cerr << "Failed to send Shutdown" << std::endl;
        return false;
    }
    usleep(100000);
    
    // 3. Switch On (0x07)
    data[4] = 0x07;
    if (can_send(node_id_, data, 8) < 0) {
        std::cerr << "Failed to send Switch On" << std::endl;
        return false;
    }
    usleep(100000);
    
    // 4. Enable Operation (0x0F)
    data[4] = 0x0F;
    if (can_send(node_id_, data, 8) < 0) {
        std::cerr << "Failed to send Enable Operation" << std::endl;
        return false;
    }
    usleep(100000);
    
    motor_enabled_ = true;
    std::cout << "✅ 电机应该已锁轴" << std::endl;
    
    // 5. NMT 启动
    nmtStart();
    
    return true;
}

bool InovanceServo::nmtStart()
{
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    if (can_send(0x000, data, 2) < 0) {
        std::cerr << "Failed to send NMT Start" << std::endl;
        return false;
    }
    
    std::cout << "✅ NMT 已启动，PDO 应该开始刷屏" << std::endl;
    return true;
}

bool InovanceServo::nmtPreOperational()
{
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    if (can_send(0x000, data, 2) < 0) {
        std::cerr << "Failed to send NMT Pre-Operational" << std::endl;
        return false;
    }
    
    std::cout << "NMT 预操作" << std::endl;
    return true;
}

bool InovanceServo::setVelocity(int32_t rpm)
{
    if (!motor_enabled_) {
        std::cerr << "Motor not enabled!" << std::endl;
        return false;
    }
    
    std::cout << "\n--- 设置速度: " << rpm << " RPM ---" << std::endl;
    
    int32_t encoder_value;
    if (rpm == 60) {
        encoder_value = 0x00800000;
    } else {
        encoder_value = (rpm * 0x00800000) / 60;
    }
    
    uint8_t data[8];
    data[0] = 0x23;
    data[1] = 0xFF;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = (encoder_value >> 0) & 0xFF;
    data[5] = (encoder_value >> 8) & 0xFF;
    data[6] = (encoder_value >> 16) & 0xFF;
    data[7] = (encoder_value >> 24) & 0xFF;
    
    if (can_send(node_id_, data, 8) < 0) {
        std::cerr << "Failed to set velocity" << std::endl;
        return false;
    }
    
    return true;
}

bool InovanceServo::disableMotor()
{
    uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
    
    if (can_send(node_id_, data, 8) < 0) {
        std::cerr << "Failed to disable motor" << std::endl;
        return false;
    }
    
    motor_enabled_ = false;
    std::cout << "电机已失能" << std::endl;
    return true;
}

bool InovanceServo::stopMotor()
{
    std::cout << "\n--- 停止电机 ---" << std::endl;
    
    // 1. 速度设为 0
    uint8_t data[8] = {0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_send(node_id_, data, 8);
    usleep(100000);
    
    // 2. 失能
    disableMotor();
    usleep(100000);
    
    // 3. NMT 预操作
    nmtPreOperational();
    
    return true;
}

} // namespace pg
