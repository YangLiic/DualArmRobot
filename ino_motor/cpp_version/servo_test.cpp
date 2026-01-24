/****************************************************************
 * Inovance Servo Test - Standalone Version (No ROS)
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
        g_servo->stopMotor();
    }
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    
    std::string port_name = "/dev/ttyUSB0";
    int baud_rate = 9600;  // 与 Python 版本保持一致
    uint32_t node_id = 0x601;
    
    std::cout << "=== 汇川伺服电机测试程序 ===" << std::endl;
    std::cout << "串口: " << port_name << std::endl;
    std::cout << "波特率: " << baud_rate << std::endl;
    std::cout << "节点 ID: 0x" << std::hex << node_id << std::dec << std::endl;
    std::cout << std::endl;
    
    try {
        InovanceServo servo(port_name, baud_rate, node_id);
        g_servo = &servo;
        
        // 启动接收线程
        std::thread recv_thread([&]() {
            servo.can_dump();
        });
        
        sleep(1);
        
        // 1. 使能电机
        if (!servo.enableMotor()) {
            std::cerr << "使能电机失败" << std::endl;
            g_running = false;
        }
        sleep(2);
        
        if (g_running) {
            // 2. 设置速度
            if (!servo.setVelocity(60)) {
                std::cerr << "设置速度失败" << std::endl;
            } else {
                std::cout << ">>> 电机正在旋转... (保持 5 秒)" << std::endl;
                sleep(5);
            }
        }
        
        // 3. 停止电机
        servo.stopMotor();
        
        g_running = false;
        sleep(1);
        
        std::cout << "\n✅ 演示结束" << std::endl;
        
        // 注意: recv_thread 会在 can_dump 中阻塞
        // 这里简单处理，实际应用需要优雅退出
        recv_thread.detach();
        
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}