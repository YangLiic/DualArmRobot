/****************************************************************
 * CyberGear USB-CAN Adapter - Extended Frame Support
 * 
 * 支持 CAN 2.0B 扩展帧（29-bit ID），用于小米 CyberGear 电机通信。
 * 帧格式: AA [Type] [DLC] [00 00] [ID3 ID2 ID1 ID0] [Data×8] 7A
 *   - Type bit5 = 1 表示扩展帧
 **/
#pragma once
#include <string>
#include <vector>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <functional>
#include <mutex>

namespace cybergear
{

class CyberGearCanUsb
{
private:
    std::string port_name_;
    int baud_rate_;
    int time_out_;
    int serial_fd_;
    bool running_;
    bool silent_mode_;
    std::mutex write_mutex_;    // 保护串口写操作，防止多线程并发

    // 最近收到的帧
    uint32_t rx_can_id_;
    uint8_t rx_data_[8];
    uint8_t rx_dlc_;

    int open_serial();
    int parse_frame(const uint8_t* buffer, size_t len);

protected:
    // 子类实现：处理收到的 CAN 帧
    virtual void on_can_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc) = 0;

public:
    CyberGearCanUsb(const std::string& port_name, int baud_rate, int time_out = 100);
    virtual ~CyberGearCanUsb();

    // 初始化
    int init_can();

    // 发送扩展帧 (29-bit CAN ID)
    int can_send_ext(uint32_t ext_can_id, const uint8_t* data, size_t size);

    // 接收循环（阻塞，通常在单独线程运行）
    int can_recv_loop();

    // 接收单帧（带超时，返回是否收到帧）
    bool can_recv_once(uint32_t& can_id, uint8_t* data, uint8_t& dlc, int timeout_ms = 100);

    // 控制
    void setSilentMode(bool silent) { silent_mode_ = silent; }
    bool isSilentMode() const { return silent_mode_; }
    void stopRunning() { running_ = false; }
    bool isRunning() const { return running_; }

    // 获取最近收到的帧数据
    uint32_t getRxCanId() const { return rx_can_id_; }
    const uint8_t* getRxData() const { return rx_data_; }
    uint8_t getRxDlc() const { return rx_dlc_; }
};

} // namespace cybergear
