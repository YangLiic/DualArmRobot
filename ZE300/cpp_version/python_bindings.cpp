#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <atomic>
#include <thread>

#include "ze300_motor.h"

namespace py = pybind11;
using namespace ze300;

class Ze300MotorPython : public Ze300Motor {
public:
    Ze300MotorPython(const std::string& port, int usb_baud, uint16_t dev_addr, bool use_host_addr)
        : Ze300Motor(port, usb_baud, dev_addr, use_host_addr), running_(true) {
        recv_thread_ = std::thread([this]() { this->can_recv_loop(); });
    }

    ~Ze300MotorPython() override {
        running_ = false;
        stopRunning();
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

    Ze300MotorPython(const Ze300MotorPython&) = delete;
    Ze300MotorPython& operator=(const Ze300MotorPython&) = delete;

private:
    std::thread recv_thread_;
    std::atomic<bool> running_;
};

PYBIND11_MODULE(ze300_motor, m) {
    m.doc() = "ZE300 motor C++ driver Python bindings";

    py::class_<Ze300Status>(m, "Ze300Status")
        .def(py::init<>())
        .def_readwrite("q_current_a", &Ze300Status::q_current_a)
        .def_readwrite("speed_rpm", &Ze300Status::speed_rpm)
        .def_readwrite("bus_voltage_v", &Ze300Status::bus_voltage_v)
        .def_readwrite("bus_current_a", &Ze300Status::bus_current_a)
        .def_readwrite("temperature_c", &Ze300Status::temperature_c)
        .def_readwrite("single_turn_count", &Ze300Status::single_turn_count)
        .def_readwrite("multi_turn_count", &Ze300Status::multi_turn_count)
        .def_readwrite("single_turn_deg", &Ze300Status::single_turn_deg)
        .def_readwrite("total_turn_deg", &Ze300Status::total_turn_deg)
        .def_readwrite("run_mode", &Ze300Status::run_mode)
        .def_readwrite("fault_code", &Ze300Status::fault_code)
        .def_readwrite("fault", &Ze300Status::fault)
        .def_readwrite("pole_pairs", &Ze300Status::pole_pairs)
        .def_readwrite("torque_constant", &Ze300Status::torque_constant)
        .def_readwrite("gear_ratio", &Ze300Status::gear_ratio)
        .def_readwrite("mit_pos_rad", &Ze300Status::mit_pos_rad)
        .def_readwrite("mit_vel_rad_s", &Ze300Status::mit_vel_rad_s)
        .def_readwrite("mit_torque_nm", &Ze300Status::mit_torque_nm)
        .def_readwrite("mit_mode_active", &Ze300Status::mit_mode_active);

    py::class_<Ze300MotorPython>(m, "Ze300Motor")
        .def(py::init<const std::string&, int, uint16_t, bool>(),
             py::arg("port"),
             py::arg("usb_baud") = 921600,
             py::arg("dev_addr") = 0x01,
             py::arg("use_host_addr") = true,
             "Create ZE300 driver\n"
             "  port: serial device, e.g. /dev/ttyUSB0\n"
             "  usb_baud: USB-CAN serial baud rate, default 921600\n"
             "  dev_addr: motor CAN ID, default 0x01\n"
             "  use_host_addr: if true use 0x100|DevAddr as host StdID")

        .def("set_silent_mode", &Ze300MotorPython::setSilentMode)

        .def("reboot", &Ze300MotorPython::reboot)
        .def("free_output", &Ze300MotorPython::freeOutput)

        .def("read_version", &Ze300MotorPython::readVersion)
        .def("read_q_current", &Ze300MotorPython::readQCurrent)
        .def("read_speed", &Ze300MotorPython::readSpeed)
        .def("read_position", &Ze300MotorPython::readPosition)
        .def("read_quick_status", &Ze300MotorPython::readQuickStatus)
        .def("read_status", &Ze300MotorPython::readStatus)
        .def("clear_fault", &Ze300MotorPython::clearFault)

        .def("read_motor_info", &Ze300MotorPython::readMotorInfo)
        .def("set_zero", &Ze300MotorPython::setZero)
        .def("set_position_max_speed_rpm", &Ze300MotorPython::setPositionMaxSpeedRpm)
        .def("set_max_current_a", &Ze300MotorPython::setMaxCurrentA)
        .def("set_speed_accel_rpm_s", &Ze300MotorPython::setSpeedAccelerationRpmPerSec)

        .def("set_torque_current_a", &Ze300MotorPython::setTorqueCurrentA)
        .def("set_speed_rpm", &Ze300MotorPython::setSpeedRpm)
        .def("set_absolute_position_count", &Ze300MotorPython::setAbsolutePositionCount)
        .def("set_relative_position_count", &Ze300MotorPython::setRelativePositionCount)
        .def("go_to_origin_shortest", &Ze300MotorPython::goToOriginShortest)

        .def("set_brake_closed", &Ze300MotorPython::setBrakeClosed)
        .def("read_brake_state", &Ze300MotorPython::readBrakeState)

        .def("read_mit_limits", &Ze300MotorPython::readMitLimits)
        .def("configure_mit_limits", &Ze300MotorPython::configureMitLimits)
        .def("read_mit_state", &Ze300MotorPython::readMitState)
        .def("send_mit_control", &Ze300MotorPython::sendMitControl)

        .def("get_status", &Ze300MotorPython::getStatus)
        .def("wait_command",
             [](Ze300MotorPython& motor, uint8_t cmd, int timeout_ms) {
                 std::vector<uint8_t> payload;
                 bool ok = motor.waitCommand(cmd, timeout_ms, &payload);
                 return py::make_tuple(ok, payload);
             },
             py::arg("cmd"),
             py::arg("timeout_ms") = 200)

        .def("get_device_addr", &Ze300MotorPython::getDeviceAddr)
        .def("get_tx_can_id", &Ze300MotorPython::getTxCanId);
}
