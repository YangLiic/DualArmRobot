/****************************************************************
 * Python Bindings for Inovance Servo C++ SDK
 * 
 * 使用 pybind11 创建 Python 模块
 **/

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <thread>
#include "inovance_servo.h"

namespace py = pybind11;
using namespace pg;

// 包装类，用于管理接收线程
class InovanceServoPython : public InovanceServo {
private:
    std::thread recv_thread_;

public:
    InovanceServoPython(const std::string& port_name, int baud_rate, uint32_t node_id)
        : InovanceServo(port_name, baud_rate, node_id) {
        // 自动启动接收线程
        recv_thread_ = std::thread([this]() {
            this->can_dump();
        });
    }

    ~InovanceServoPython() {
        // 停止接收循环
        stopRunning();
        // 等待线程结束
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

    // 禁用拷贝
    InovanceServoPython(const InovanceServoPython&) = delete;
    InovanceServoPython& operator=(const InovanceServoPython&) = delete;
};

PYBIND11_MODULE(inovance_servo, m) {
    m.doc() = "Inovance Servo C++ SDK Python Bindings";

    // 运行模式枚举
    py::enum_<OperationMode>(m, "OperationMode")
        .value("PROFILE_POSITION", OperationMode::PROFILE_POSITION, "位置模式")
        .value("VELOCITY", OperationMode::VELOCITY, "速度模式")
        .value("HOMING", OperationMode::HOMING, "回零模式")
        .export_values();

    // InovanceServo 类
    py::class_<InovanceServoPython>(m, "InovanceServo")
        .def(py::init<const std::string&, int, uint32_t>(),
             py::arg("port_name"),
             py::arg("baud_rate"),
             py::arg("node_id") = 0x601,
             "初始化伺服电机\n\n"
             "参数:\n"
             "  port_name: 串口设备 (例如: '/dev/ttyUSB0')\n"
             "  baud_rate: 波特率 (默认: 9600)\n"
             "  node_id: 节点ID (默认: 0x601)")
        
        // 基础控制
        .def("enable", 
             py::overload_cast<>(&InovanceServoPython::enable),
             "使能电机（默认速度模式）")
        .def("enable", 
             py::overload_cast<OperationMode>(&InovanceServoPython::enable),
             py::arg("mode"),
             "使能电机（指定模式）\n\n"
             "参数:\n"
             "  mode: 运行模式 (OperationMode.VELOCITY 或 OperationMode.PROFILE_POSITION)")
        .def("disable", &InovanceServoPython::disable,
             "失能电机")
        .def("stop", &InovanceServoPython::stop,
             "停止电机并失能")
        .def("fault_reset", &InovanceServoPython::faultReset,
             "错误复位")
        .def("quick_stop", &InovanceServoPython::quickStop,
             "急停")
        
        // 速度控制
        .def("set_velocity", &InovanceServoPython::setVelocity,
             py::arg("rpm"),
             "设置速度（RPM）\n\n"
             "参数:\n"
             "  rpm: 转速，正值顺时针，负值逆时针")
        
        // 位置控制
        .def("set_position", &InovanceServoPython::setPosition,
             py::arg("degrees"),
             py::arg("absolute") = false,
             "设置目标位置（角度）\n\n"
             "参数:\n"
             "  degrees: 角度，正值顺时针，负值逆时针\n"
             "  absolute: 是否为绝对位置 (默认: False 相对)")
        .def("set_position_pulse", &InovanceServoPython::setPositionPulse,
             py::arg("pulses"),
             py::arg("absolute") = false,
             "设置目标位置（脉冲）")
        .def("start_position_move", &InovanceServoPython::startPositionMove,
             py::arg("relative") = true,
             "启动位置运动（已在set_position中自动调用）")
        .def("set_profile_velocity", &InovanceServoPython::setProfileVelocity,
             py::arg("rpm"),
             "设置位置模式下的速度限制\n\n"
             "参数:\n"
             "  rpm: 速度限制 (RPM)")
        .def("set_profile_acceleration", &InovanceServoPython::setProfileAcceleration,
             py::arg("acc"),
             "设置位置模式下的加速度")
        .def("set_profile_deceleration", &InovanceServoPython::setProfileDeceleration,
             py::arg("dec"),
             "设置位置模式下的减速度")
        
        // 方向控制
        .def("set_direction_inverted", &InovanceServoPython::setDirectionInverted,
             py::arg("inverted"),
             "设置方向反转\n\n"
             "参数:\n"
             "  inverted: True=反转方向, False=正常方向")
        .def("is_direction_inverted", &InovanceServoPython::isDirectionInverted,
             "获取方向反转状态")
        
        // NMT 管理
        .def("nmt_start", &InovanceServoPython::nmtStart,
             "NMT 启动")
        .def("nmt_pre_operational", &InovanceServoPython::nmtPreOperational,
             "NMT 预操作")
        
        // 调试控制
        .def("set_silent_mode", &InovanceServoPython::setSilentMode,
             py::arg("silent"),
             "设置静默模式（不打印CAN收发消息）")
        
        // 状态查询
        .def("is_enabled", &InovanceServoPython::isEnabled,
             "查询电机是否已使能")
        .def("get_current_mode", &InovanceServoPython::getCurrentMode,
             "获取当前运行模式");
}
