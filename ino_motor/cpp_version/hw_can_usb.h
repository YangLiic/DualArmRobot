/****************************************************************
 * USB-CAN Adapter - Standalone Version (No ROS dependency)
 **/
#pragma once
#include <string>
#include <vector>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <condition_variable>
#include <deque>
#include <mutex>

namespace pg
{

struct CanFrame
{
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    uint64_t sequence;
};

class CanInterfaceUsb
{
private:
    std::string port_name_;
    int baud_rate_;
    int time_out_;
    int serial_fd_;
    bool running_;
    bool silent_mode_;  // 静默模式：不打印接收消息
    mutable std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::deque<CanFrame> frame_queue_;
    uint64_t frame_sequence_;
    
    uint32_t current_can_id_;
    uint8_t current_data_[8];
    uint8_t current_dlc_;
    
    virtual void decode() = 0;
    
    int open_serial();
    int parse_frame(const uint8_t* buffer, size_t len);
    
public:
    int init_can();
    int can_send(uint32_t can_id, const uint8_t* data, size_t size);
    int can_dump();
    
    void get_can_id(uint32_t &can_id);
    void get_data(uint8_t* data, uint8_t &dlc);
    uint64_t getFrameSequence() const;
    bool waitForNextFrame(uint64_t last_sequence, CanFrame& frame, int timeout_ms);
    
    void setSilentMode(bool silent) { silent_mode_ = silent; }
    bool isSilentMode() const { return silent_mode_; }
    
    void stopRunning() { running_ = false; }
    bool isRunning() const { return running_; }
    
    CanInterfaceUsb(const std::string& port_name, int baud_rate, int time_out);
    ~CanInterfaceUsb();
};

} //namespace pg
