/****************************************************************
 * CyberGear USB-CAN Adapter - Implementation
 *
 * 实际适配器串口帧格式（来自 串口协议说明.docx）:
 *
 * 发送帧:
 *   [0-1]   41 54            帧头 ("AT")
 *   [2-5]   ID3 ID2 ID1 ID0  编码后的 CAN ID (4 bytes)
 *   [6]     DLC              数据长度 (1-8)
 *   [7..N]  Data             数据区
 *   [N+1]   0D               帧尾 CR
 *   [N+2]   0A               帧尾 LF
 *
 * CAN ID 编码规则:
 *   适配器会把 CAN ID 左移 3 位，最低 3 位为 "100"
 *   即: encoded_id = (can_id << 3) | 0x04
 *   解码: can_id = encoded_id >> 3
 **/
#include <algorithm>
#include <iostream>
#include <cstring>
#include <sys/select.h>
#include "cybergear_can_usb.h"

namespace cybergear
{

// ==================== 帧格式常量 ====================
static const uint8_t FRAME_HEADER_0 = 0x41;  // 'A'
static const uint8_t FRAME_HEADER_1 = 0x54;  // 'T'
static const uint8_t FRAME_TAIL_0   = 0x0D;  // CR
static const uint8_t FRAME_TAIL_1   = 0x0A;  // LF

// 最大帧长: 2(头) + 4(ID) + 1(DLC) + 8(数据) + 2(尾) = 17
static const int MAX_FRAME_LEN = 17;


CyberGearCanUsb::CyberGearCanUsb(const std::string& port_name, int baud_rate, int time_out)
    : port_name_(port_name), baud_rate_(baud_rate), time_out_(time_out),
      serial_fd_(-1), running_(true), silent_mode_(false)
{
    rx_can_id_ = 0;
    rx_dlc_ = 0;
    memset(rx_data_, 0, sizeof(rx_data_));
}

CyberGearCanUsb::~CyberGearCanUsb()
{
    running_ = false;
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

int CyberGearCanUsb::open_serial()
{
    struct termios tty;
    
    // 用 O_NONBLOCK 打开（和 pyserial 一致），防止阻塞
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        perror("open serial port");
        return -1;
    }
    
    // 切回阻塞模式（和 pyserial 一致）
    int flags = fcntl(serial_fd_, F_GETFL, 0);
    fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK);
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }
    
    // 使用 cfmakeraw 设置原始模式（和 pyserial 内部实现一致）
    cfmakeraw(&tty);
    
    // 波特率
    speed_t speed;
    switch(baud_rate_) {
        case 9600:    speed = B9600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 921600:  speed = B921600; break;
        case 2000000: speed = B2000000; break;
        default:      speed = B921600; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1, 无硬件流控, 使能接收, 本地连接
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    
    // 读超时
    tty.c_cc[VTIME] = 1;  // 100ms
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    
    // 清空收发缓冲区
    tcflush(serial_fd_, TCIOFLUSH);
    
    return 0;
}

int CyberGearCanUsb::init_can()
{
    if (open_serial() < 0) {
        std::cerr << "Failed to open serial port: " << port_name_ << std::endl;
        return -1;
    }
    
    std::cout << "✅ CyberGear CAN-USB initialized on " << port_name_ 
              << " at " << baud_rate_ << " baud" << std::endl;
    return 0;
}

int CyberGearCanUsb::can_send_ext(uint32_t ext_can_id, const uint8_t* data, size_t size)
{
    if (serial_fd_ < 0) {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }
    
    uint8_t dlc = (size > 8) ? 8 : (uint8_t)size;
    
    // 编码 CAN ID: 左移 3 位 + 0x04 (二进制 100)
    uint32_t encoded_id = (ext_can_id << 3) | 0x04;
    
    // 构建帧: AT [ID4] [DLC] [Data] CR LF
    uint8_t frame[MAX_FRAME_LEN];
    int idx = 0;
    
    frame[idx++] = FRAME_HEADER_0;  // 'A'
    frame[idx++] = FRAME_HEADER_1;  // 'T'
    
    // 编码后的 CAN ID (4 bytes, 大端序)
    frame[idx++] = (encoded_id >> 24) & 0xFF;
    frame[idx++] = (encoded_id >> 16) & 0xFF;
    frame[idx++] = (encoded_id >> 8) & 0xFF;
    frame[idx++] = encoded_id & 0xFF;
    
    // DLC
    frame[idx++] = dlc;
    
    // 数据区
    for (int i = 0; i < dlc; i++) {
        frame[idx++] = data[i];
    }
    
    // 帧尾
    frame[idx++] = FRAME_TAIL_0;  // CR
    frame[idx++] = FRAME_TAIL_1;  // LF
    
    int total_len = idx;
    
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        ssize_t written = write(serial_fd_, frame, total_len);
        if (written != total_len) {
            perror("write to serial");
            return -1;
        }
        // 确保数据物理发送到硬件（关键！等效 pyserial 的 flush）
        tcdrain(serial_fd_);
    }
    
    // Debug output
    if (!silent_mode_) {
        printf("发送 [CAN_ID:0x%08X -> 编码:0x%08X]: ", ext_can_id, encoded_id);
        for (int i = 0; i < total_len; i++) {
            printf("%02X ", frame[i]);
        }
        printf("\n");
    }
    
    return 0;
}

int CyberGearCanUsb::parse_frame(const uint8_t* buffer, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        // 查找帧头 "AT" (0x41 0x54)
        if (buffer[i] != FRAME_HEADER_0) continue;
        if (i + 1 >= len || buffer[i + 1] != FRAME_HEADER_1) continue;
        
        // 至少需要: AT(2) + ID(4) + DLC(1) + CR(1) + LF(1) = 9 bytes
        if (i + 8 >= len) return 0;  // 不完整，等待更多数据
        
        // 解析编码后的 CAN ID
        uint32_t encoded_id = ((uint32_t)buffer[i+2] << 24) |
                              ((uint32_t)buffer[i+3] << 16) |
                              ((uint32_t)buffer[i+4] << 8) |
                              (uint32_t)buffer[i+5];
        
        // 解码 CAN ID: 右移 3 位
        uint32_t can_id = encoded_id >> 3;
        
        // DLC
        uint8_t dlc = buffer[i+6];
        if (dlc > 8) {
            continue;  // 无效 DLC，跳过
        }
        
        // 检查是否有足够的数据 + 帧尾
        int frame_len = 2 + 4 + 1 + dlc + 2;  // AT + ID + DLC + Data + CRLF
        if (i + frame_len > len) return 0;  // 不完整
        
        // 检查帧尾
        if (buffer[i + frame_len - 2] != FRAME_TAIL_0 ||
            buffer[i + frame_len - 1] != FRAME_TAIL_1) {
            continue;  // 帧尾不对，跳过
        }
        
        // 解析成功
        rx_can_id_ = can_id;
        rx_dlc_ = dlc;
        memcpy(rx_data_, &buffer[i + 7], dlc);
        if (dlc < 8) {
            memset(&rx_data_[dlc], 0, 8 - dlc);
        }
        
        if (!silent_mode_) {
            printf("   └── 收到 [CAN_ID:0x%08X]: ", can_id);
            for (int j = 0; j < frame_len; j++) {
                printf("%02X ", buffer[i+j]);
            }
            printf("\n");
        }
        
        on_can_frame(rx_can_id_, rx_data_, rx_dlc_);
        
        return i + frame_len;
    }
    return 0;
}

bool CyberGearCanUsb::can_recv_once(uint32_t& can_id, uint8_t* data, uint8_t& dlc, int timeout_ms)
{
    if (serial_fd_ < 0) return false;
    
    uint8_t buffer[256];
    fd_set rdfs;
    struct timeval timeout;
    
    FD_ZERO(&rdfs);
    FD_SET(serial_fd_, &rdfs);
    
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(serial_fd_ + 1, &rdfs, NULL, NULL, &timeout);
    
    if (ret > 0 && FD_ISSET(serial_fd_, &rdfs)) {
        ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer));
        if (nbytes > 0) {
            size_t processed = 0;
            while (processed < (size_t)nbytes) {
                int parsed = parse_frame(&buffer[processed], nbytes - processed);
                if (parsed > 0) {
                    can_id = rx_can_id_;
                    memcpy(data, rx_data_, 8);
                    dlc = rx_dlc_;
                    processed += parsed;
                    return true;
                } else {
                    break;
                }
            }
        }
    }
    
    return false;
}

int CyberGearCanUsb::can_recv_loop()
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
            if (!running_) break;
            perror("select");
            break;
        } else if (ret == 0) {
            continue;
        } else {
            if (FD_ISSET(serial_fd_, &rdfs)) {
                ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer));
                if (nbytes > 0) {
                    size_t processed = 0;
                    while (processed < (size_t)nbytes) {
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

} // namespace cybergear
