#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace ze300 {

class Ze300CanUsb {
private:
    std::string port_name_;
    int baud_rate_;
    int timeout_ms_;
    int serial_fd_;
    bool running_;
    bool silent_mode_;
    std::mutex write_mutex_;

    uint16_t rx_can_id_;
    uint8_t rx_data_[8];
    uint8_t rx_dlc_;
    std::vector<uint8_t> rx_stream_buf_;

    int open_serial();
    int parse_frame(const uint8_t* buffer, size_t len);

protected:
    virtual void on_can_frame(uint16_t can_id, const uint8_t* data, uint8_t dlc) = 0;

public:
    Ze300CanUsb(const std::string& port_name, int baud_rate, int timeout_ms = 100);
    virtual ~Ze300CanUsb();

    int init_can();
    int can_send_std(uint16_t can_id, const uint8_t* data, size_t size);

    bool can_recv_once(uint16_t& can_id, uint8_t* data, uint8_t& dlc, int timeout_ms = 100);
    int can_recv_loop();

    void setSilentMode(bool silent) { silent_mode_ = silent; }
    bool isSilentMode() const { return silent_mode_; }
    void stopRunning() { running_ = false; }
    bool isRunning() const { return running_; }
};

}  // namespace ze300
