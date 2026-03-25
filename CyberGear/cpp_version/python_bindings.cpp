/****************************************************************
 * CyberGear Motor - Python Bindings (pybind11)
 **/
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <atomic>
#include "cybergear_motor.h"
#include <thread>

namespace py = pybind11;
using namespace cybergear;

// Python 包装类：自动管理接收线程和位置轮询线程
class CyberGearMotorPython : public CyberGearMotor {
public:
    CyberGearMotorPython(const std::string& port, int baud,
                          uint8_t motor_id, uint8_t master_id)
        : CyberGearMotor(port, baud, motor_id, master_id),
          poll_running_(false) {
        // 启动接收线程
        recv_thread_ = std::thread([this]() {
            this->can_recv_loop();
        });
    }
    
    ~CyberGearMotorPython() {
        stopPollThread();
        stopRunning();
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }
    
    // 启动位置轮询线程（速度模式下检测限位）
    void startPollThread(int interval_ms = 50) {
        if (poll_running_) return;
        poll_running_ = true;
        poll_thread_ = std::thread([this, interval_ms]() {
            while (poll_running_) {
                if (this->isEnabled() && this->getRunMode() == MODE_SPEED) {
                    // 发送 enable 指令触发 type 2 反馈帧
                    // process_feedback 会更新位置并设置 at_limit
                    this->requestFeedback();
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                    // 如果位置超限，发速度=0
                    this->checkAndEnforceLimit();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        });
    }
    
    void stopPollThread() {
        poll_running_ = false;
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
    }

private:
    std::thread recv_thread_;
    std::thread poll_thread_;
    std::atomic<bool> poll_running_;
};


PYBIND11_MODULE(cybergear_motor, m)
{
    m.doc() = "CyberGear 微电机 C++ 驱动模块";
    
    // 运行模式枚举
    py::enum_<RunMode>(m, "RunMode")
        .value("MOTION",   MODE_MOTION,   "运控模式 (MIT)")
        .value("POSITION", MODE_POSITION, "位置模式")
        .value("SPEED",    MODE_SPEED,    "速度模式")
        .value("CURRENT",  MODE_CURRENT,  "电流模式")
        .export_values();
    
    // 电机状态结构体
    py::class_<MotorStatus>(m, "MotorStatus")
        .def(py::init<>())
        .def_readwrite("position",    &MotorStatus::position)
        .def_readwrite("velocity",    &MotorStatus::velocity)
        .def_readwrite("torque",      &MotorStatus::torque)
        .def_readwrite("temperature", &MotorStatus::temperature)
        .def_readwrite("motor_id",    &MotorStatus::motor_id)
        .def_readwrite("has_fault",   &MotorStatus::has_fault)
        .def_readwrite("at_limit",    &MotorStatus::at_limit);
    
    // 电机驱动类
    py::class_<CyberGearMotorPython>(m, "CyberGearMotor")
        .def(py::init<const std::string&, int, uint8_t, uint8_t>(),
             py::arg("port"),
             py::arg("baud") = 921600,
             py::arg("motor_id") = 0x7F,
             py::arg("master_id") = 0x00,
             "创建 CyberGear 电机驱动\n"
             "  port: 串口设备路径\n"
             "  baud: 串口波特率 (默认 921600)\n"
             "  motor_id: 电机 CAN ID (默认 0x7F)\n"
             "  master_id: 主机 CAN ID (默认 0x00)")
        
        // 基础控制
        .def("enable",  &CyberGearMotorPython::enable,  "使能电机")
        .def("stop",    &CyberGearMotorPython::stop,    "停止电机")
        .def("set_run_mode", &CyberGearMotorPython::setRunMode, "设置运行模式")
        
        // 行程限制
        .def("set_travel_limit", &CyberGearMotorPython::setTravelLimit,
             py::arg("min_rad"), py::arg("max_rad"),
             "设置行程限制 (rad)，相对于零位。默认 ±π (±180°)")
        .def("set_travel_limit_deg",
             [](CyberGearMotorPython& m, float min_deg, float max_deg) {
                 m.setTravelLimit(min_deg * M_PI / 180.0f, max_deg * M_PI / 180.0f);
             },
             py::arg("min_deg"), py::arg("max_deg"),
             "设置行程限制 (角度)，相对于零位。默认 ±180°")
        .def("enable_travel_limit", &CyberGearMotorPython::enableTravelLimit,
             "启用/禁用行程限制")
        .def("is_travel_limit_enabled", &CyberGearMotorPython::isTravelLimitEnabled,
             "获取行程限制是否启用")
        .def("is_at_limit", &CyberGearMotorPython::isAtLimit,
             "获取当前位置是否处于限位")
        
        // 速度模式
        .def("enable_speed_mode",  &CyberGearMotorPython::enableSpeedMode,  "进入速度模式并使能")
        .def("set_speed",          &CyberGearMotorPython::setSpeed,         "设置速度 (rad/s)")
        .def("set_speed_rpm",      &CyberGearMotorPython::setSpeedRPM,      "设置速度 (RPM)")
        .def("set_current_limit",  &CyberGearMotorPython::setCurrentLimit,  "设置电流限制 (A)")
        
        // 位置模式
        .def("enable_position_mode", &CyberGearMotorPython::enablePositionMode, "进入位置模式并使能")
        .def("set_position",         &CyberGearMotorPython::setPosition,        "设置目标位置 (rad)")
        .def("set_position_deg",     &CyberGearMotorPython::setPositionDeg,     "设置目标位置 (角度)")
        .def("set_speed_limit",      &CyberGearMotorPython::setSpeedLimit,      "设置速度限制 (rad/s)")
        
        // 零位
        .def("set_zero",    &CyberGearMotorPython::setMechPositionToZero,
             "设置当前位置为零位 (上电期间有效，掉电丢失)")
        .def("request_feedback", &CyberGearMotorPython::requestFeedback,
             "主动请求反馈帧, 获取最新位置/速度/力矩/温度")
        .def("go_to_zero",  &CyberGearMotorPython::goToZero,
             py::arg("speed_limit") = 2.0f, "回到零位")
        
        // 参数
        .def("read_param", &CyberGearMotorPython::readParam, "读取参数")
        
        // 状态
        .def("get_status",   &CyberGearMotorPython::getStatus,   "获取电机状态")
        .def("is_enabled",   &CyberGearMotorPython::isEnabled,   "获取使能状态")
        .def("get_run_mode", &CyberGearMotorPython::getRunMode,  "获取运行模式")
        .def("get_motor_id", &CyberGearMotorPython::getMotorId,  "获取电机 ID")
        
        // 调试
        .def("set_silent_mode", &CyberGearMotorPython::setSilentMode, "设置静默模式")
        
        // 位置轮询（速度模式限位保护所需）
        .def("start_poll", &CyberGearMotorPython::startPollThread,
             py::arg("interval_ms") = 50,
             "启动位置轮询线程 (速度模式行程保护必需), interval_ms 为轮询间隔毫秒")
        .def("stop_poll",  &CyberGearMotorPython::stopPollThread,
             "停止位置轮询线程");
}
