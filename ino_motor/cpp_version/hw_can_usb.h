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

namespace pg
{

class CanInterfaceUsb
{
private:
    std::string port_name_;
    int baud_rate_;
    int time_out_;
    int serial_fd_;
    bool running_;
    
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
    
    CanInterfaceUsb(const std::string& port_name, int baud_rate, int time_out);
    ~CanInterfaceUsb();
};

} //namespace pg
