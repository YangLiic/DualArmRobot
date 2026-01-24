/****************************************************************
 * Inovance Servo Test - SDK 演示程序
 * 
 * 编译:
 *   g++ -o servo_test servo_test.cpp \
 *       inovance_servo.cpp hw_can_usb.cpp \
 *       -lpthread -std=c++11
 * 
 * 运行:
 *   ./servo_test
 **/

#include <iostream>
#include <thread>
#include <csignal>
#include <unistd.h>
#include "inovance_servo.h"

using namespace pg;

bool g_running = true;
InovanceServo* g_servo = nullptr;

void signalHandler(int signum) {
    std::cout << "\n收到中断信号 (" << signum << ")" << std::endl;
    g_running = false;
    if (g_servo) {
        g_servo->stop();
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    
    std::string port_name = "/dev/ttyUSB0";
    int baud_rate = 9600;
    uint32_t node_id = 0x601;  // 默认值
    
    // 支持命令行参数指定节点 ID
    if (argc > 1) {
        node_id = std::stoul(argv[1], nullptr, 16);  // 支持 0x602 或 602 格式
    }
    
    // 0x602 电机需要反转方向（根据实际安装方向）
    bool invert_direction = (node_id == 0x602);
    
    std::cout << "=== 汇川伺服 SDK 演示程序 ===" << std::endl;
    std::cout << "用法: ./servo_test [node_id]  例如: ./servo_test 0x602" << std::endl;
    std::cout << "串口: " << port_name << std::endl;
    std::cout << "波特率: " << baud_rate << std::endl;
    std::cout << "节点 ID: 0x" << std::hex << node_id << std::dec << std::endl;
    std::cout << std::endl;
    
    try {
        InovanceServo servo(port_name, baud_rate, node_id);
        g_servo = &servo;
        
        // 设置方向反转（0x602 需要反转以统一方向）
        servo.setDirectionInverted(invert_direction);
        
        // 启动接收线程
        std::thread recv_thread([&]() {
            servo.can_dump();
        });
        
        sleep(1);
        
        // 错误复位（防止驱动器报错）
        std::cout << "🔄 执行错误复位（预防性）..." << std::endl;
        servo.faultReset();
        sleep(1);
        
        // ==================== 速度模式演示 ====================
        std::cout << "\n【演示 1: 速度模式】" << std::endl;
        
        // 1. 使能电机 (速度模式)
        if (!servo.enable(OperationMode::VELOCITY)) {
            std::cerr << "使能失败" << std::endl;
            g_running = false;
        }
        sleep(2);
        
        if (g_running) {
            // 2. 正向旋转 60 RPM
            servo.setVelocity(60);
            std::cout << ">>> 正向旋转 60 RPM (3秒)" << std::endl;
            sleep(3);
            
            // 3. 反向旋转 30 RPM
            servo.setVelocity(-30);
            std::cout << ">>> 反向旋转 30 RPM (3秒)" << std::endl;
            sleep(3);
            
            // 4. 停止
            servo.setVelocity(0);
            sleep(1);
        }
        
        // 5. 失能
        servo.disable();
        sleep(2);
        
        // ==================== 位置模式演示 ====================
        if (g_running) {
            std::cout << "\n【演示 2: 位置模式】" << std::endl;
            
            // 1. 使能电机 (位置模式)
            if (!servo.enable(OperationMode::PROFILE_POSITION)) {
                std::cerr << "使能失败" << std::endl;
                g_running = false;
            }
            sleep(1);
            
            // 设置运动速度限制
            servo.setProfileVelocity(30);        // 限速 30 RPM
            servo.setProfileAcceleration(50);    // 加速度 50 RPM/s
            servo.setProfileDeceleration(50);    // 减速度 50 RPM/s
            sleep(1);
            
            if (g_running) {
                // 2. 顺时针转 90 度（相对位置）
                std::cout << "\n>>> 顺时针转 90° (限速 30 RPM)" << std::endl;
                servo.setPosition(90.0, false);  // false = 相对位置，自动启动运动
                sleep(4);
                
                // 3. 逆时针转回（相对位置）
                std::cout << "\n>>> 逆时针转回 -90° (限速 30 RPM)" << std::endl;
                servo.setPosition(-90.0, false);  // false = 相对位置，自动启动运动
                sleep(4);
            }
        }
        
        // 6. 停止电机
        servo.stop();
        
        g_running = false;
        sleep(1);
        
        std::cout << "\n✅ 演示完成" << std::endl;
        
        recv_thread.detach();
        
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}