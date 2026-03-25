# CyberGear C++ 驱动 SDK

此 SDK 为小米 CyberGear 微电机提供高性能的 C++ 驱动，并通过 `pybind11` 导出为 Python 模块。

## 架构说明

驱动层采用两层架构设计，确保了通信层与协议层的解耦：

1. **CyberGearCanUsb (通信层)**:
   - 负责底层串口通信 (Linux Termios)。
   - 实现专用的 USB-CAN 适配器协议（AT 帧头、编码后的 29-bit CAN ID、CRLF 帧尾）。
   - 提供异步接收循环，确保实时处理电机反馈帧。

2. **CyberGearMotor (协议层)**:
   - 继承自 `CyberGearCanUsb`。
   - 实现 CyberGear 专有的控制逻辑（位移限制、单位转换等）。
   - 维护电机实时状态（`MotorStatus` 结构体），包含位置、速度、力矩、温度。

3. **Python 绑定层**:
   - 使用 `pybind11` 将 C++ 类导出为 Python 模块。
   - 自动管理接收线程的生命周期。

## Python API 参考

### 1. 初始化

```python
import cybergear_motor

motor = cybergear_motor.CyberGearMotor(
    port="/dev/ttyUSB0",   # 串口设备路径
    baud=921600,           # 固定波特率
    motor_id=0x01,         # 电机 CAN ID
    master_id=0xFD         # 主机标识符 (根据文档建议设为 0xFD)
)
```

### 2. 核心控制接口

| 方法 | 说明 |
| :--- | :--- |
| `enable()` | 使能电机。 |
| `stop()` | 停止电机并进入 Reset 状态。 |
| `set_run_mode(mode)` | 设置运行模式 (`MOTION`, `POSITION`, `SPEED`, `CURRENT`)。 |
| `enable_speed_mode()` | 快捷切换到速度模式并自动使能。 |
| `set_speed(rad_s)` | 设置目标转速 (rad/s)，范围 ±30。 |
| `set_speed_rpm(rpm)` | 设置目标转速 (RPM)。 |
| `enable_position_mode()` | 快捷切换到位置模式并自动使能。 |
| `set_position(rad)` | 设置目标位置 (rad)，范围 ±12.5。 |
| `set_position_deg(deg)` | 设置目标位置 (角度)。 |
| `set_speed_limit(rad_s)` | 设置位置模式下的最大转速限制。 |
| `set_zero()` | 将当前机械位置设为零点。 |
| `go_to_zero(speed_limit)` | 自动回零位。 |
| `set_silent_mode(bool)` | 开启/关闭控制台 Debug 信息打印。 |

### 3. 数据反馈接口

```python
status = motor.get_status()

# status 包含以下属性:
print(f"当前位置: {status.position} rad")
print(f"当前速度: {status.velocity} rad/s")
print(f"当前力矩: {status.torque} Nm")
print(f"当前温度: {status.temperature} ℃")
print(f"故障标志: {status.has_fault}")
```

## 编译指南

```bash
cd CyberGear/cpp_version

# 安装编译依赖
uv pip install pybind11 setuptools

# 编译 C++ 动态链接库 (.so)
python setup.py build_ext --inplace
```

## 电机配置参考 (CyberGear)

| 参数 | 值 |
| :--- | :--- |
| **CAN 波特率** | 1000 kbps |
| **CAN 帧格式** | 扩展帧 (Extended Frame) |
| **默认 ID** | 0x7F (及用户自定义 ID 如 1) |
