/****************************************************************
 * 错误复位测试示例
 * 
 * 当驱动器报错时，使用此程序进行错误复位
 * 
 * 编译:
 *   g++ -o fault_reset_test fault_reset_test.cpp \
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

void signalHandler(int signum) {
    std::cout << "\n收到中断信号 (" << signum << ")" << std::endl;
    g_running = false;
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
    
    std::cout << "=== 汇川伺服 - 错误复位 ===" << std::endl;
    std::cout << "用法: ./fault_reset_test [node_id]  例如: ./fault_reset_test 0x602" << std::endl;
    std::cout << "串口: " << port_name << std::endl;
    std::cout << "波特率: " << baud_rate << std::endl;
    std::cout << "节点 ID: 0x" << std::hex << node_id << std::dec << std::endl;
    std::cout << std::endl;
    
    try {
        InovanceServo servo(port_name, baud_rate, node_id);
        
        // 启动接收线程
        std::thread recv_thread([&]() {
            servo.can_dump();
        });
        
        sleep(1);
        
        // 执行错误复位
        if (!servo.faultReset()) {
            std::cerr << "错误复位失败" << std::endl;
            return 1;
        }
        
        sleep(2);
        
        std::cout << "\n✅ 错误复位完成，可以重新使能电机" << std::endl;
        
        g_running = false;
        sleep(1);
        
        recv_thread.detach();
        
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
