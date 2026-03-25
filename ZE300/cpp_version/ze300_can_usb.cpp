#include "ze300_can_usb.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace ze300 {

static const uint8_t FRAME_HEADER_0 = 0x41;
static const uint8_t FRAME_HEADER_1 = 0x54;
static const uint8_t FRAME_TAIL_0 = 0x0D;
static const uint8_t FRAME_TAIL_1 = 0x0A;
static const int MAX_FRAME_LEN = 17;

static inline uint32_t encode_std_id_for_adapter(uint16_t can_id) {
    return ((uint32_t)(can_id & 0x07FF) << 21);
}

static inline bool decode_std_id_from_adapter(uint32_t encoded_id, uint16_t& can_id) {
    if ((encoded_id & 0x04) != 0) {
        return false;
    }
    can_id = (uint16_t)((encoded_id >> 21) & 0x07FF);
    return true;
}

Ze300CanUsb::Ze300CanUsb(const std::string& port_name, int baud_rate, int timeout_ms)
    : port_name_(port_name),
      baud_rate_(baud_rate),
      timeout_ms_(timeout_ms),
      serial_fd_(-1),
      running_(true),
      silent_mode_(false),
      rx_can_id_(0),
      rx_dlc_(0) {
    std::memset(rx_data_, 0, sizeof(rx_data_));
        rx_stream_buf_.reserve(2048);
}

Ze300CanUsb::~Ze300CanUsb() {
    running_ = false;
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

int Ze300CanUsb::open_serial() {
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        perror("open serial port");
        return -1;
    }

    int flags = fcntl(serial_fd_, F_GETFL, 0);
    fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfmakeraw(&tty);

    speed_t speed;
    switch (baud_rate_) {
        case 9600:
            speed = B9600;
            break;
        case 115200:
            speed = B115200;
            break;
        case 230400:
            speed = B230400;
            break;
        case 460800:
            speed = B460800;
            break;
        case 921600:
            speed = B921600;
            break;
        case 2000000:
            speed = B2000000;
            break;
        default:
            speed = B921600;
            break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    return 0;
}

int Ze300CanUsb::init_can() {
    if (open_serial() < 0) {
        std::cerr << "Failed to open serial port: " << port_name_ << std::endl;
        return -1;
    }

    std::cout << "✅ ZE300 CAN-USB initialized on " << port_name_ << " at " << baud_rate_ << " baud" << std::endl;
    return 0;
}

int Ze300CanUsb::can_send_std(uint16_t can_id, const uint8_t* data, size_t size) {
    if (serial_fd_ < 0) {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }

    uint8_t dlc = static_cast<uint8_t>(std::min<size_t>(size, 8));

    uint32_t encoded_id = encode_std_id_for_adapter(can_id);

    uint8_t frame[MAX_FRAME_LEN];
    int idx = 0;
    frame[idx++] = FRAME_HEADER_0;
    frame[idx++] = FRAME_HEADER_1;

    frame[idx++] = (encoded_id >> 24) & 0xFF;
    frame[idx++] = (encoded_id >> 16) & 0xFF;
    frame[idx++] = (encoded_id >> 8) & 0xFF;
    frame[idx++] = encoded_id & 0xFF;

    frame[idx++] = dlc;

    for (int i = 0; i < dlc; ++i) {
        frame[idx++] = data[i];
    }

    frame[idx++] = FRAME_TAIL_0;
    frame[idx++] = FRAME_TAIL_1;

    int total_len = idx;
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        ssize_t written = write(serial_fd_, frame, total_len);
        if (written != total_len) {
            perror("write");
            return -1;
        }
        tcdrain(serial_fd_);
    }

    if (!silent_mode_) {
        printf("TX [ID:0x%03X]: ", can_id);
        for (int i = 0; i < total_len; ++i) {
            printf("%02X ", frame[i]);
        }
        printf("\n");
    }

    return 0;
}

int Ze300CanUsb::parse_frame(const uint8_t* buffer, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (buffer[i] != FRAME_HEADER_0) {
            continue;
        }
        if (i + 1 >= len || buffer[i + 1] != FRAME_HEADER_1) {
            continue;
        }

        if (i + 8 >= len) {
            return 0;
        }

        uint32_t encoded_id = ((uint32_t)buffer[i + 2] << 24) |
                              ((uint32_t)buffer[i + 3] << 16) |
                              ((uint32_t)buffer[i + 4] << 8) |
                              (uint32_t)buffer[i + 5];

        uint16_t can_id = 0;
        if (!decode_std_id_from_adapter(encoded_id, can_id)) {
            continue;
        }

        uint8_t dlc = buffer[i + 6];
        if (dlc > 8) {
            continue;
        }

        int frame_len = 2 + 4 + 1 + dlc + 2;
        if (i + frame_len > len) {
            return 0;
        }

        if (buffer[i + frame_len - 2] != FRAME_TAIL_0 ||
            buffer[i + frame_len - 1] != FRAME_TAIL_1) {
            continue;
        }

        rx_can_id_ = can_id;
        rx_dlc_ = dlc;
        std::memcpy(rx_data_, &buffer[i + 7], dlc);
        if (dlc < 8) {
            std::memset(&rx_data_[dlc], 0, 8 - dlc);
        }

        if (!silent_mode_) {
            printf("RX [ID:0x%03X]: ", can_id);
            for (int j = 0; j < frame_len; ++j) {
                printf("%02X ", buffer[i + j]);
            }
            printf("\n");
        }

        on_can_frame(rx_can_id_, rx_data_, rx_dlc_);
        return i + frame_len;
    }

    return 0;
}

bool Ze300CanUsb::can_recv_once(uint16_t& can_id, uint8_t* data, uint8_t& dlc, int timeout_ms) {
    if (serial_fd_ < 0) {
        return false;
    }

    uint8_t buffer[256];
    fd_set rdfs;
    struct timeval timeout;

    FD_ZERO(&rdfs);
    FD_SET(serial_fd_, &rdfs);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(serial_fd_ + 1, &rdfs, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(serial_fd_, &rdfs)) {
        ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer));
        if (nbytes > 0) {
            rx_stream_buf_.insert(rx_stream_buf_.end(), buffer, buffer + nbytes);

            while (!rx_stream_buf_.empty()) {
                int parsed = parse_frame(rx_stream_buf_.data(), rx_stream_buf_.size());
                if (parsed > 0) {
                    rx_stream_buf_.erase(rx_stream_buf_.begin(), rx_stream_buf_.begin() + parsed);
                    can_id = rx_can_id_;
                    std::memcpy(data, rx_data_, 8);
                    dlc = rx_dlc_;
                    return true;
                }

                if (rx_stream_buf_.size() > 4096) {
                    rx_stream_buf_.clear();
                }
                break;
            }
        }
    }

    return false;
}

int Ze300CanUsb::can_recv_loop() {
    if (serial_fd_ < 0) {
        std::cerr << "Serial port not open" << std::endl;
        return -1;
    }

    uint8_t buffer[1024];

    while (running_) {
        fd_set rdfs;
        struct timeval timeout;

        FD_ZERO(&rdfs);
        FD_SET(serial_fd_, &rdfs);

        timeout.tv_sec = timeout_ms_ / 1000;
        timeout.tv_usec = (timeout_ms_ % 1000) * 1000;

        int ret = select(serial_fd_ + 1, &rdfs, nullptr, nullptr, &timeout);
        if (ret < 0) {
            if (!running_) {
                break;
            }
            perror("select");
            break;
        }

        if (ret == 0) {
            continue;
        }

        if (FD_ISSET(serial_fd_, &rdfs)) {
            ssize_t nbytes = read(serial_fd_, buffer, sizeof(buffer));
            if (nbytes > 0) {
                rx_stream_buf_.insert(rx_stream_buf_.end(), buffer, buffer + nbytes);

                while (!rx_stream_buf_.empty()) {
                    int parsed = parse_frame(rx_stream_buf_.data(), rx_stream_buf_.size());
                    if (parsed > 0) {
                        rx_stream_buf_.erase(rx_stream_buf_.begin(), rx_stream_buf_.begin() + parsed);
                    } else {
                        if (rx_stream_buf_.size() > 4096) {
                            rx_stream_buf_.clear();
                        }
                        break;
                    }
                }
            }
        }
    }

    return 0;
}

}  // namespace ze300
