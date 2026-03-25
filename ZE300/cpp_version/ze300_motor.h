#pragma once

#include "ze300_can_usb.h"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

namespace ze300 {

enum Ze300Cmd : uint8_t {
    CMD_REBOOT = 0x00,
    CMD_READ_VERSION = 0xA0,
    CMD_READ_Q_CURRENT = 0xA1,
    CMD_READ_SPEED = 0xA2,
    CMD_READ_POSITION = 0xA3,
    CMD_READ_QUICK = 0xA4,
    CMD_READ_STATUS = 0xAE,
    CMD_CLEAR_FAULT = 0xAF,
    CMD_READ_MOTOR_INFO = 0xB0,
    CMD_SET_ZERO = 0xB1,
    CMD_SET_POS_MAX_SPEED = 0xB2,
    CMD_SET_MAX_Q_CURRENT = 0xB3,
    CMD_SET_Q_SLOPE = 0xB4,
    CMD_SET_SPEED_ACCEL = 0xB5,
    CMD_TORQUE_CONTROL = 0xC0,
    CMD_SPEED_CONTROL = 0xC1,
    CMD_ABS_POSITION_CONTROL = 0xC2,
    CMD_REL_POSITION_CONTROL = 0xC3,
    CMD_GO_ORIGIN_SHORTEST = 0xC4,
    CMD_BRAKE_CONTROL = 0xCE,
    CMD_FREE_OUTPUT = 0xCF,
    CMD_MIT_LIMIT_CFG = 0xF0,
    CMD_MIT_READ_STATE = 0xF1,
};

struct Ze300Status {
    float q_current_a;
    float speed_rpm;
    float bus_voltage_v;
    float bus_current_a;
    float temperature_c;

    uint16_t single_turn_count;
    int32_t multi_turn_count;
    float single_turn_deg;
    float total_turn_deg;

    uint8_t run_mode;
    uint8_t fault_code;
    bool fault;

    uint8_t pole_pairs;
    float torque_constant;
    uint8_t gear_ratio;

    float mit_pos_rad;
    float mit_vel_rad_s;
    float mit_torque_nm;
    bool mit_mode_active;

    Ze300Status()
        : q_current_a(0.0f),
          speed_rpm(0.0f),
          bus_voltage_v(0.0f),
          bus_current_a(0.0f),
          temperature_c(0.0f),
          single_turn_count(0),
          multi_turn_count(0),
          single_turn_deg(0.0f),
          total_turn_deg(0.0f),
          run_mode(0),
          fault_code(0),
          fault(false),
          pole_pairs(0),
          torque_constant(0.0f),
          gear_ratio(0),
          mit_pos_rad(0.0f),
          mit_vel_rad_s(0.0f),
          mit_torque_nm(0.0f),
          mit_mode_active(false) {}
};

class Ze300Motor : public Ze300CanUsb {
private:
    struct MitLimits {
        float pos_max_rad;
        float vel_max_rad_s;
        float torque_max_nm;
    };

    struct FrameData {
        uint16_t can_id;
        uint8_t dlc;
        std::array<uint8_t, 8> data;
        uint64_t seq;
    };

    uint16_t dev_addr_;
    uint16_t tx_id_;
    uint16_t mit_tx_id_;
    bool use_host_addr_;

    mutable std::mutex status_mutex_;
    Ze300Status status_;
    MitLimits mit_limits_;

    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    std::map<uint8_t, FrameData> last_response_;
    uint64_t seq_counter_;

    static int32_t to_i32_le(const uint8_t* data);
    static uint32_t to_u32_le(const uint8_t* data);
    static int16_t to_i16_le(const uint8_t* data);
    static uint16_t to_u16_le(const uint8_t* data);

    static void put_i32_le(uint8_t* dst, int32_t v);
    static void put_u16_le(uint8_t* dst, uint16_t v);

    static uint32_t float_to_uint(float x, float x_min, float x_max, int bits);
    static float uint_to_float(uint32_t x, float x_min, float x_max, int bits);

    void send_command(uint8_t cmd, const uint8_t* payload, size_t payload_len);
    bool wait_response(uint8_t cmd, FrameData& frame, int timeout_ms);

    void parse_common_response(uint8_t cmd, const uint8_t* data, uint8_t dlc);

protected:
    void on_can_frame(uint16_t can_id, const uint8_t* data, uint8_t dlc) override;

public:
    Ze300Motor(const std::string& port,
               int usb_baud = 921600,
               uint16_t dev_addr = 0x01,
               bool use_host_addr = true);
    ~Ze300Motor() override = default;

    uint16_t getDeviceAddr() const { return dev_addr_; }
    uint16_t getTxCanId() const { return tx_id_; }

    void reboot();
    void freeOutput();

    void readVersion();
    void readQCurrent();
    void readSpeed();
    void readPosition();
    void readQuickStatus();
    void readStatus();
    uint8_t clearFault();

    void readMotorInfo();
    void setZero();
    void setPositionMaxSpeedRpm(float rpm);
    void setMaxCurrentA(float current_a);
    void setSpeedAccelerationRpmPerSec(float acc_rpm_s);

    void setTorqueCurrentA(float current_a);
    void setSpeedRpm(float rpm);
    void setAbsolutePositionCount(int32_t count);
    void setRelativePositionCount(int32_t count);
    void goToOriginShortest();

    void setBrakeClosed(bool closed);
    uint8_t readBrakeState();

    void readMitLimits();
    void configureMitLimits(float pos_max_rad, float vel_max_rad_s, float torque_max_nm);
    void readMitState();
    void sendMitControl(float pos_rad, float vel_rad_s, float kp, float kd, float torque_nm);

    Ze300Status getStatus() const;

    bool waitCommand(uint8_t cmd, int timeout_ms = 200, std::vector<uint8_t>* payload = nullptr);
};

}  // namespace ze300
