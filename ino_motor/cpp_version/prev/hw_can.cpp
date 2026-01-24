/****************************************************************
 * Copyright (c) Baidu.com, Inc. All Rights Reserved
 *
 * @file hw_can.h
 * @brief handle can bus message
 * @author Lingfeng Qian, RAL, Baidu
 **/
#include <algorithm>
#include <iostream>
#include <cstring>
#include <ros/ros.h>
#include "robot/hw_can.h"

using namespace std;

namespace pg
{

const uint32_t CAN_FD_ID = 0x80000000;
CanInterface::CanInterface(const std::string& port_name, int baud_rate, int time_out):
            port_name_(port_name), baud_rate_(baud_rate), running_(true), time_out_(time_out) {
    init_can();
}

CanInterface::~CanInterface()
{
    close(socket_can_);
}

void CanInterface::get_frame(struct canfd_frame &frame)
{
    frame = frame_;
}

void CanInterface::get_can_id(uint32_t &can_id)
{
    if (frame_.can_id > CAN_FD_ID){
        can_id = frame_.can_id - CAN_FD_ID;
    }else {
        can_id = frame_.can_id;
    }
}

void CanInterface::config_filter(struct can_filter *rfilter)
{
    rfilter_ = rfilter;
}

int CanInterface::init_can() {
    char physicalName[5];
    struct ifreq ifr;
    if ((socket_can_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return 1;
    }
    // port_name_.copy(ifr.ifr_name, 4);
    std::copy(port_name_.begin(), port_name_.end(), physicalName);
    strncpy(ifr.ifr_name, physicalName, port_name_.size());
    ifr.ifr_name[port_name_.size()] = '\0';
    cout << ifr.ifr_name << endl;
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex) {
        perror("if_nametoindex");   
        return 1;
    }

    memset(&addr_, 0, sizeof(addr_));
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr.ifr_ifindex;

    /* check if the frame fits into the CAN netdevice */
    if (ioctl(socket_can_, SIOCGIFMTU, &ifr) < 0) {
        perror("SIOCGIFMTU");
        return 1;
    }

    /* step4: set can filter */
    // struct can_filter rfilter[4];
    // rfilter[0].can_id   = 0x0CF02000;
    // rfilter[0].can_mask = 0X0FFFF000;
    // rfilter[1].can_id   = 0x08FF0000;
    // rfilter[1].can_mask = 0X0FFF0000;  // 添加can滤波器，1是使能滤波位
    // rfilter[2].can_id   = 0x00F00010;
    // rfilter[2].can_mask = 0X00F000F0;  // 添加can滤波器，1是使能滤波位
    // rfilter[3].can_id   = 0x0CFF00C0; 
    // rfilter[3].can_mask = 0X0FFF00F0;  // 添加can滤波器，1是使能滤波位
    // setsockopt(socket_can_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    if (bind(socket_can_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) { 
        perror("bind");
        return 1;
    }
    return 0;
}

int CanInterface::can_send(uint32_t can_id, const uint8_t* data, size_t size)
{
    int required_mtu = 0;
    frame_.can_id = can_id + CAN_FD_ID;
    frame_.len = size;

    memcpy(frame_.data, data, size*sizeof(uint8_t));
    required_mtu = 16;
    /* step2: send frame */
    if (write(socket_can_, &frame_, required_mtu) != required_mtu) {
        perror("write");
        return 1;
    }
}

int CanInterface::can_dump()
{
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
    struct timeval timeout_config = { 0, 0 };
    struct iovec iov;
    fd_set rdfs;
    int ret = 0;
    struct msghdr msg;
    iov.iov_base = &frame_;
    msg.msg_name = &addr_;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = &ctrlmsg;
    while (running_) {
        FD_ZERO(&rdfs);
        FD_SET(socket_can_, &rdfs);

        timeout_config.tv_sec = time_out_ / 1000.0;
        timeout_config.tv_usec = 0;

        ret = select(socket_can_+1, &rdfs, NULL, NULL, &timeout_config);
        if (ret <= 0) {
            //perror("select");
            // running_ = 0;
            continue;
        }else {
            // currmax: 需要监听的几个通道
            if (FD_ISSET(socket_can_, &rdfs)) {
                /* these settings may be modified by recvmsg() */
                iov.iov_len = sizeof(frame_);
                msg.msg_namelen = sizeof(addr_);
                msg.msg_controllen = sizeof(ctrlmsg);  
                msg.msg_flags = 0;

                recvmsg(socket_can_, &msg, 0); // 在这里进行数据读取
                // ROS_INFO_STREAM("receive a message");
                decode();
            }
        }
    }
}

}