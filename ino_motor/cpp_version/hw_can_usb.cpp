/****************************************************************
 * USB-CAN Adapter - Standalone Implementation (No ROS)
 **/
#include <algorithm>
#include <iostream>
#include <cstring>
#include <sys/select.h>
#include "hw_can_usb.h"

using namespace std;

namespace pg
{

CanInterfaceUsb::CanInterfaceUsb(const std::string& port_name, int baud_rate, int time_out):
            port_name_(port_name), baud_rate_(baud_rate), running_(true), time_out_(time_out), silent_mode_(false) {
    current_can_id_ = 0;
    current_dlc_ = 0;
    memset(current_data_, 0, sizeof(current_data_));
    init_can();
}

CanInterfaceUsb::~CanInterfaceUsb()
{
    running_ = false;  // 确保停止接收循环
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

void CanInterfaceUsb::get_can_id(uint32_t &can_id)
{
    can_id = current_can_id_;
}

void CanInterfaceUsb::get_data(uint8_t* data, uint8_t &dlc)
{
    memcpy(data, current_data_, 8);
    dlc = current_dlc_;
}

int CanInterfaceUsb::open_serial() {
    struct termios tty;
    
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
        perror("open serial port");
        return -1;
    }
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }
    
    speed_t speed;
    switch(baud_rate_) {
        case 9600:    speed = B9600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 921600:  speed = B921600; break;
        case 2000000: speed = B2000000; break;
        default:      speed = B9600; break;  // 默认使用 9600
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    
    tty.c_cc[VTIME] = time_out_ / 100;
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    
    tcflush(serial_fd_, TCIOFLUSH);
    
    return 0;
}

int CanInterfaceUsb::init_can() {
    if (open_serial() < 0) {
        std::cerr << "Failed to open serial port: " << port_name_ << std::endl;
        return -1;
    }
    
    std::cout << "✅ USB-CAN adapter initialized on " << port_name_ 
              << " at " << baud_rate_ << " baud" << std::endl;
    return 0;
}

int CanInterfaceUsb::can_send(uint32_t can_id, const uint8_t* data, size_t size)
{
    if (serial_fd_ < 0) {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }
    
    uint8_t frame[17];
    
    frame[0] = 0xAA;
    frame[1] = 0x00;
    frame[2] = 0x00;
    frame[3] = (uint8_t)size;
    frame[4] = 0x00;
    frame[5] = 0x00;
    frame[6] = (can_id >> 8) & 0xFF;
    frame[7] = can_id & 0xFF;
    
    memcpy(&frame[8], data, size);
    
    if (size < 8) {
        memset(&frame[8 + size], 0, 8 - size);
    }
    
    frame[16] = 0x7A;
    
    ssize_t written = write(serial_fd_, frame, 17);
    if (written != 17) {
        perror("write to serial");
        return -1;
    }
    
    // Debug output
    if (!silent_mode_) {
        printf("发送 [ID:0x%03X]: ", can_id);
        for (int i = 0; i < 17; i++) {
            printf("%02X ", frame[i]);
        }
        printf("\n");
    }
    
    return 0;
}

int CanInterfaceUsb::parse_frame(const uint8_t* buffer, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (buffer[i] == 0xAA && (i + 16) < len) {
            if (buffer[i + 16] == 0x7A) {
                current_dlc_ = buffer[i + 3];
                current_can_id_ = (buffer[i + 6] << 8) | buffer[i + 7];
                memcpy(current_data_, &buffer[i + 8], 8);
                
                if (!silent_mode_) {
                    printf("   └── 收到: ");
                    for (int j = 0; j < 17; j++) {
                        printf("%02X", buffer[i + j]);
                    }
                    printf("\n");
                }
                
                decode();
                
                return i + 17;
            }
        }
    }
    return 0;
}

int CanInterfaceUsb::can_dump()
{
    if (serial_fd_ < 0) {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }
    
    uint8_t buffer[1024];
    fd_set rdfs;
    struct timeval timeout;
    
    while (running_) {
        FD_ZERO(&rdfs);
        FD_SET(serial_fd_, &rdfs);
        
        timeout.tv_sec = time_out_ / 1000;
        timeout.tv_usec = (time_out_ % 1000) * 1000;
        
        int ret = select(serial_fd_ + 1, &rdfs, NULL, NULL, &timeout);
        
        if (ret < 0) {
            // 如果不再运行，正常退出
            if (!running_) break;
            perror("select");
            break;  // select 错误时退出，避免循环报错
        } else if (ret == 0) {
            continue;
        } else {
            if (FD_ISSET(serial_fd_, &rdfs)) {
                ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer));
                if (nbytes > 0) {
                    size_t processed = 0;
                    while (processed < nbytes) {
                        int parsed = parse_frame(&buffer[processed], nbytes - processed);
                        if (parsed > 0) {
                            processed += parsed;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
    }
    
    return 0;
}

} // namespace pg
