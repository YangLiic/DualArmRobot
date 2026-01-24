/****************************************************************
 * 速度模式测试示例
 * 
 * 编译:
 *   g++ -o velocity_test velocity_test.cpp \
 *       ../inovance_servo.cpp ../hw_can_usb.cpp \
 *       -I.. -lpthread -std=c++11
 **/

#include <iostream>
#include <thread>
#include <csignal>
#include <unistd.h>
#include "../inovance_servo.h"

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
    uint32_t node_id = 0x601;
    
    // 支持命令行参数指定节点 ID
    if (argc > 1) {
        node_id = std::stoul(argv[1], nullptr, 16);
    }
    bool invert_direction = (node_id == 0x602);
    
    std::cout << "=== 汇川伺服 - 速度模式测试 ===" << std::endl;
    std::cout << "用法: ./velocity_test [node_id]  例如: ./velocity_test 0x602" << std::endl;
    std::cout << "串口: " << port_name << std::endl;
    std::cout << "波特率: " << baud_rate << std::endl;
    std::cout << "节点 ID: 0x" << std::hex << node_id << std::dec << std::endl;
    std::cout << std::endl;
    
    try {
        InovanceServo servo(port_name, baud_rate, node_id);
        g_servo = &servo;
        servo.setDirectionInverted(invert_direction);
        
        // 启动接收线程
        std::thread recv_thread([&]() {
            servo.can_dump();
        });
        
        sleep(1);
        
        // 错误复位（预防性）
        std::cout << "🔄 执行错误复位..." << std::endl;
        servo.faultReset();
        sleep(1);
        
        // 1. 使能电机 (速度模式)
        if (!servo.enable(OperationMode::VELOCITY)) {
            std::cerr << "使能失败" << std::endl;
            g_running = false;
        }
        sleep(2);
        
        if (g_running) {
            // 2. 正向旋转 60 RPM
            servo.setVelocity(60);
            std::cout << ">>> 正向旋转 60 RPM (5秒)" << std::endl;
            sleep(5);
            
            // 3. 反向旋转 30 RPM
            servo.setVelocity(-30);
            std::cout << ">>> 反向旋转 30 RPM (5秒)" << std::endl;
            sleep(5);
            
            // 4. 停止
            servo.setVelocity(0);
            sleep(1);
        }
        
        // 5. 失能
        servo.stop();
        
        g_running = false;
        sleep(1);
        
        std::cout << "\n✅ 测试完成" << std::endl;
        
        recv_thread.detach();
        
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
