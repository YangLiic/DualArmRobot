/****************************************************************
 * Copyright (c) Baidu.com, Inc. All Rights Reserved
 *
 * @file hw_can.h
 * @brief communicate with the embedded controller
 * @author Lingfeng Qian, RAL, Baidu
 **/
#pragma once
#include "string"
#include "vector"
#include <unistd.h>
#include <stdint.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <eigen3/Eigen/Dense>
#include "can_utils/lib.h"

namespace pg
{

class CanInterface
{
private:
    std::string port_name_;    // 所使用的can口号
    int baud_rate_;            // baud rate
    int time_out_;             // unit: second
    int socket_can_;           // can raw socket 
	struct sockaddr_can addr_; // can address
    struct canfd_frame frame_; // data frame
    bool running_;
    struct can_filter *rfilter_; // can filter
    // overriden methods
    virtual void decode() = 0; 
public:
    // 主体接口
    int init_can();         // 
    int can_send(uint32_t can_id, const uint8_t* data, size_t size);
    int can_dump();
    // data accessors
    void get_frame(struct canfd_frame &frame);
    void get_can_id(uint32_t &can_id);  // 获取当前frame的can_id
    void config_filter(struct can_filter *rfilter);   // 配置can滤波器
    /** constructor */
    CanInterface(const std::string& port_name, int baud_rate, int time_out);
    ~CanInterface();
};

} //namespace pg
