#include "ze300_motor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

namespace ze300 {

namespace {
constexpr float COUNT_PER_REV = 16384.0f;
constexpr float DEG_PER_COUNT = 360.0f / COUNT_PER_REV;
constexpr float RPM_RAW_SCALE = 100.0f;
constexpr float CURRENT_RAW_SCALE = 1000.0f;
constexpr float VOLT_RAW_SCALE = 100.0f;
constexpr float BUS_CURRENT_RAW_SCALE = 100.0f;
constexpr int MAX_ABS_CAN_ID = 0x7FF;
}  // namespace

int32_t Ze300Motor::to_i32_le(const uint8_t* data) {
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
}

uint32_t Ze300Motor::to_u32_le(const uint8_t* data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

int16_t Ze300Motor::to_i16_le(const uint8_t* data) {
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

uint16_t Ze300Motor::to_u16_le(const uint8_t* data) {
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void Ze300Motor::put_i32_le(uint8_t* dst, int32_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

void Ze300Motor::put_u16_le(uint8_t* dst, uint16_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

uint32_t Ze300Motor::float_to_uint(float x, float x_min, float x_max, int bits) {
    if (x_max <= x_min) {
        return 0;
    }
    x = std::max(x_min, std::min(x, x_max));
    float span = x_max - x_min;
    float normalized = (x - x_min) / span;
    uint32_t max_int = (1u << bits) - 1u;
    return (uint32_t)std::round(normalized * (float)max_int);
}

float Ze300Motor::uint_to_float(uint32_t x, float x_min, float x_max, int bits) {
    if (x_max <= x_min) {
        return x_min;
    }
    uint32_t max_int = (1u << bits) - 1u;
    float normalized = (float)x / (float)max_int;
    return x_min + normalized * (x_max - x_min);
}

Ze300Motor::Ze300Motor(const std::string& port, int usb_baud, uint16_t dev_addr, bool use_host_addr)
    : Ze300CanUsb(port, usb_baud, 100),
      dev_addr_(dev_addr & 0xFF),
      tx_id_(0),
      mit_tx_id_(0),
      use_host_addr_(use_host_addr),
      seq_counter_(0) {
    if (dev_addr_ == 0 || dev_addr_ > 0xFF) {
        throw std::runtime_error("ZE300 dev_addr must be in [0x01, 0xFF]");
    }

    tx_id_ = use_host_addr_ ? (0x100 | dev_addr_) : dev_addr_;
    mit_tx_id_ = 0x400 | tx_id_;

    if (tx_id_ > MAX_ABS_CAN_ID || mit_tx_id_ > MAX_ABS_CAN_ID) {
        throw std::runtime_error("Calculated CAN ID exceeds 11-bit standard frame range");
    }

    mit_limits_.pos_max_rad = 95.5f;
    mit_limits_.vel_max_rad_s = 45.0f;
    mit_limits_.torque_max_nm = 18.0f;

    if (init_can() != 0) {
        throw std::runtime_error("Failed to init ZE300 CAN-USB");
    }

    std::cout << "🔧 ZE300 Motor initialized, DevAddr=0x" << std::hex << dev_addr_ << std::dec
              << ", TX_ID=0x" << std::hex << tx_id_ << std::dec
              << ", MIT_TX_ID=0x" << std::hex << mit_tx_id_ << std::dec << std::endl;
}

void Ze300Motor::send_command(uint8_t cmd, const uint8_t* payload, size_t payload_len) {
    uint8_t data[8] = {0};
    data[0] = cmd;

    size_t n = std::min<size_t>(payload_len, 7);
    for (size_t i = 0; i < n; ++i) {
        data[i + 1] = payload[i];
    }

    uint8_t dlc = (uint8_t)(1 + n);

    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        last_response_.erase(cmd);
    }

    can_send_std(tx_id_, data, dlc);
    std::this_thread::sleep_for(std::chrono::microseconds(300));
}

bool Ze300Motor::wait_response(uint8_t cmd, FrameData& frame, int timeout_ms) {
    std::unique_lock<std::mutex> lock(response_mutex_);

    bool ok = response_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]() {
        return last_response_.find(cmd) != last_response_.end();
    });

    if (!ok) {
        return false;
    }

    frame = last_response_[cmd];
    return true;
}

void Ze300Motor::parse_common_response(uint8_t cmd, const uint8_t* data, uint8_t dlc) {
    std::lock_guard<std::mutex> lock(status_mutex_);

    switch (cmd) {
        case CMD_READ_Q_CURRENT:
        case CMD_TORQUE_CONTROL:
            if (dlc >= 5) {
                status_.q_current_a = (float)to_i32_le(&data[1]) / CURRENT_RAW_SCALE;
            }
            break;

        case CMD_READ_SPEED:
        case CMD_SPEED_CONTROL:
            if (dlc >= 5) {
                status_.speed_rpm = (float)to_i32_le(&data[1]) / RPM_RAW_SCALE;
            }
            break;

        case CMD_READ_POSITION:
        case CMD_ABS_POSITION_CONTROL:
        case CMD_REL_POSITION_CONTROL:
            if (dlc >= 7) {
                status_.single_turn_count = to_u16_le(&data[1]);
                status_.multi_turn_count = to_i32_le(&data[3]);
                status_.single_turn_deg = status_.single_turn_count * DEG_PER_COUNT;
                status_.total_turn_deg = status_.multi_turn_count * DEG_PER_COUNT;
            }
            break;

        case CMD_READ_QUICK:
            if (dlc >= 8) {
                status_.temperature_c = (float)data[1];
                status_.q_current_a = (float)to_i16_le(&data[2]) / CURRENT_RAW_SCALE;
                status_.speed_rpm = (float)to_i16_le(&data[4]) / RPM_RAW_SCALE;
                status_.single_turn_count = to_u16_le(&data[6]);
                status_.single_turn_deg = status_.single_turn_count * DEG_PER_COUNT;
            }
            break;

        case CMD_READ_STATUS:
        case CMD_FREE_OUTPUT:
            if (dlc >= 8) {
                status_.bus_voltage_v = (float)to_u16_le(&data[1]) / VOLT_RAW_SCALE;
                status_.bus_current_a = (float)to_u16_le(&data[3]) / BUS_CURRENT_RAW_SCALE;
                status_.temperature_c = (float)data[5];
                status_.run_mode = data[6];
                status_.fault_code = data[7];
                status_.fault = (status_.fault_code != 0);
            }
            break;

        case CMD_CLEAR_FAULT:
            if (dlc >= 2) {
                status_.fault_code = data[1];
                status_.fault = (status_.fault_code != 0);
            }
            break;

        case CMD_READ_MOTOR_INFO:
            if (dlc >= 7) {
                status_.pole_pairs = data[1];
                float tc = 0.0f;
                std::memcpy(&tc, &data[2], sizeof(float));
                status_.torque_constant = tc;
                status_.gear_ratio = data[6];
            }
            break;

        case CMD_MIT_LIMIT_CFG:
            if (dlc >= 7) {
                uint16_t p = to_u16_le(&data[1]);
                uint16_t v = to_u16_le(&data[3]);
                uint16_t t = to_u16_le(&data[5]);
                mit_limits_.pos_max_rad = (float)p * 0.1f;
                mit_limits_.vel_max_rad_s = (float)v * 0.01f;
                mit_limits_.torque_max_nm = (float)t * 0.01f;
            }
            break;

        case CMD_MIT_READ_STATE:
            if (dlc >= 7) {
                uint16_t pos_u16 = ((uint16_t)data[1] << 8) | data[2];
                uint16_t vel_u12 = ((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4);
                uint16_t tor_u12 = (((uint16_t)data[4] & 0x0F) << 8) | data[5];

                status_.mit_pos_rad =
                    uint_to_float(pos_u16, -mit_limits_.pos_max_rad, mit_limits_.pos_max_rad, 16);
                status_.mit_vel_rad_s =
                    uint_to_float(vel_u12, -mit_limits_.vel_max_rad_s, mit_limits_.vel_max_rad_s, 12);
                status_.mit_torque_nm =
                    uint_to_float(tor_u12, -mit_limits_.torque_max_nm, mit_limits_.torque_max_nm, 12);

                status_.mit_mode_active = (data[6] & 0x01) != 0;
                if (data[6] & 0x02) {
                    status_.fault = true;
                }
            }
            break;

        default:
            break;
    }
}

void Ze300Motor::on_can_frame(uint16_t can_id, const uint8_t* data, uint8_t dlc) {
    if (can_id != dev_addr_) {
        return;
    }

    if (dlc == 0) {
        return;
    }

    uint8_t cmd = data[0];
    parse_common_response(cmd, data, dlc);

    FrameData frame;
    frame.can_id = can_id;
    frame.dlc = dlc;
    frame.data.fill(0);
    for (size_t i = 0; i < std::min<size_t>(dlc, 8); ++i) {
        frame.data[i] = data[i];
    }

    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        frame.seq = ++seq_counter_;
        last_response_[cmd] = frame;
    }
    response_cv_.notify_all();
}

void Ze300Motor::reboot() { send_command(CMD_REBOOT, nullptr, 0); }

void Ze300Motor::freeOutput() { send_command(CMD_FREE_OUTPUT, nullptr, 0); }

void Ze300Motor::readVersion() { send_command(CMD_READ_VERSION, nullptr, 0); }

void Ze300Motor::readQCurrent() { send_command(CMD_READ_Q_CURRENT, nullptr, 0); }

void Ze300Motor::readSpeed() { send_command(CMD_READ_SPEED, nullptr, 0); }

void Ze300Motor::readPosition() { send_command(CMD_READ_POSITION, nullptr, 0); }

void Ze300Motor::readQuickStatus() { send_command(CMD_READ_QUICK, nullptr, 0); }

void Ze300Motor::readStatus() { send_command(CMD_READ_STATUS, nullptr, 0); }

uint8_t Ze300Motor::clearFault() {
    send_command(CMD_CLEAR_FAULT, nullptr, 0);
    FrameData frame;
    if (wait_response(CMD_CLEAR_FAULT, frame, 300) && frame.dlc >= 2) {
        return frame.data[1];
    }
    return 0xFF;
}

void Ze300Motor::readMotorInfo() { send_command(CMD_READ_MOTOR_INFO, nullptr, 0); }

void Ze300Motor::setZero() { send_command(CMD_SET_ZERO, nullptr, 0); }

void Ze300Motor::setPositionMaxSpeedRpm(float rpm) {
    int32_t raw = (int32_t)std::round(rpm * RPM_RAW_SCALE);
    if (raw < 0) {
        raw = 0;
    }
    uint8_t payload[4];
    put_i32_le(payload, raw);
    send_command(CMD_SET_POS_MAX_SPEED, payload, 4);
}

void Ze300Motor::setMaxCurrentA(float current_a) {
    int32_t raw = (int32_t)std::round(current_a * CURRENT_RAW_SCALE);
    if (raw < 0) {
        raw = 0;
    }
    uint8_t payload[4];
    put_i32_le(payload, raw);
    send_command(CMD_SET_MAX_Q_CURRENT, payload, 4);
}

void Ze300Motor::setSpeedAccelerationRpmPerSec(float acc_rpm_s) {
    int32_t raw = (int32_t)std::round(acc_rpm_s * RPM_RAW_SCALE);
    if (raw < 0) {
        raw = 0;
    }
    uint8_t payload[4];
    put_i32_le(payload, raw);
    send_command(CMD_SET_SPEED_ACCEL, payload, 4);
}

void Ze300Motor::setTorqueCurrentA(float current_a) {
    int32_t raw = (int32_t)std::round(current_a * CURRENT_RAW_SCALE);
    uint8_t payload[4];
    put_i32_le(payload, raw);
    send_command(CMD_TORQUE_CONTROL, payload, 4);
}

void Ze300Motor::setSpeedRpm(float rpm) {
    int32_t raw = (int32_t)std::round(rpm * RPM_RAW_SCALE);
    uint8_t payload[4];
    put_i32_le(payload, raw);
    send_command(CMD_SPEED_CONTROL, payload, 4);
}

void Ze300Motor::setAbsolutePositionCount(int32_t count) {
    uint8_t payload[4];
    put_i32_le(payload, count);
    send_command(CMD_ABS_POSITION_CONTROL, payload, 4);
}

void Ze300Motor::setRelativePositionCount(int32_t count) {
    uint8_t payload[4];
    put_i32_le(payload, count);
    send_command(CMD_REL_POSITION_CONTROL, payload, 4);
}

void Ze300Motor::goToOriginShortest() { send_command(CMD_GO_ORIGIN_SHORTEST, nullptr, 0); }

void Ze300Motor::setBrakeClosed(bool closed) {
    uint8_t payload[1] = {closed ? (uint8_t)0x01 : (uint8_t)0x00};
    send_command(CMD_BRAKE_CONTROL, payload, 1);
}

uint8_t Ze300Motor::readBrakeState() {
    uint8_t payload[1] = {0xFF};
    send_command(CMD_BRAKE_CONTROL, payload, 1);

    FrameData frame;
    if (wait_response(CMD_BRAKE_CONTROL, frame, 300) && frame.dlc >= 2) {
        return frame.data[1];
    }
    return 0xFF;
}

void Ze300Motor::readMitLimits() {
    send_command(CMD_MIT_LIMIT_CFG, nullptr, 0);
}

void Ze300Motor::configureMitLimits(float pos_max_rad, float vel_max_rad_s, float torque_max_nm) {
    uint8_t payload[6];

    uint16_t p = (uint16_t)std::max(0, (int)std::round(pos_max_rad * 10.0f));
    uint16_t v = (uint16_t)std::max(0, (int)std::round(vel_max_rad_s * 100.0f));
    uint16_t t = (uint16_t)std::max(0, (int)std::round(torque_max_nm * 100.0f));

    put_u16_le(&payload[0], p);
    put_u16_le(&payload[2], v);
    put_u16_le(&payload[4], t);

    send_command(CMD_MIT_LIMIT_CFG, payload, 6);
}

void Ze300Motor::readMitState() {
    send_command(CMD_MIT_READ_STATE, nullptr, 0);
}

void Ze300Motor::sendMitControl(float pos_rad, float vel_rad_s, float kp, float kd, float torque_nm) {
    uint32_t pos_u16 = float_to_uint(pos_rad, -mit_limits_.pos_max_rad, mit_limits_.pos_max_rad, 16);
    uint32_t vel_u12 = float_to_uint(vel_rad_s, -mit_limits_.vel_max_rad_s, mit_limits_.vel_max_rad_s, 12);
    uint32_t kp_u12 = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint32_t kd_u12 = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint32_t tor_u12 = float_to_uint(torque_nm, -mit_limits_.torque_max_nm, mit_limits_.torque_max_nm, 12);

    uint8_t data[8] = {0};
    data[0] = (uint8_t)((pos_u16 >> 8) & 0xFF);
    data[1] = (uint8_t)(pos_u16 & 0xFF);
    data[2] = (uint8_t)((vel_u12 >> 4) & 0xFF);
    data[3] = (uint8_t)(((vel_u12 & 0x0F) << 4) | ((kp_u12 >> 8) & 0x0F));
    data[4] = (uint8_t)(kp_u12 & 0xFF);
    data[5] = (uint8_t)((kd_u12 >> 4) & 0xFF);
    data[6] = (uint8_t)(((kd_u12 & 0x0F) << 4) | ((tor_u12 >> 8) & 0x0F));
    data[7] = (uint8_t)(tor_u12 & 0xFF);

    can_send_std(mit_tx_id_, data, 8);
}

Ze300Status Ze300Motor::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

bool Ze300Motor::waitCommand(uint8_t cmd, int timeout_ms, std::vector<uint8_t>* payload) {
    FrameData frame;
    bool ok = wait_response(cmd, frame, timeout_ms);
    if (!ok) {
        return false;
    }

    if (payload != nullptr) {
        payload->assign(frame.data.begin(), frame.data.begin() + frame.dlc);
    }
    return true;
}

}  // namespace ze300
