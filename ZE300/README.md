# ZE300 电机开发目录

本目录提供 ZE300 电机控制实现，包含：

- `cpp_version/`：C++ 控制包 + `pybind11` Python 绑定
- `interactive_control.py`：交互式控制脚本
- `ZE300用户资料3.02_20250714/`：原始手册与配套资源

## 默认通信参数

- USB-CAN 串口波特率：`921600`
- CAN 波特率：`1000kbps`（驱动器侧参数）
- 设备地址（CAN ID）：`0x01`

## 快速开始

```bash
cd ZE300/cpp_version
python setup.py build_ext --inplace

cd /home/yang/DualArmRobot
python ZE300/interactive_control.py
```

## 环境变量

- `ZE300_PORT`：串口设备，默认 `/dev/ttyUSB0`
- `ZE300_USB_BAUD`：USB-CAN 串口波特率，默认 `921600`
- `ZE300_CAN_BAUD_KBPS`：仅用于显示，默认 `1000`
- `ZE300_CAN_ID`：设备地址，默认 `0x01`
- `ZE300_USE_HOST_ADDR`：是否使用 `0x100|DevAddr` 作为主站发送 ID，默认 `1`

## 说明

- 当前实现遵循 `自定义CAN通信协议_V3.07b0` 的命令格式（标准帧、数据小端）。
- 交互脚本优先覆盖速度、位置、原点、抱闸、故障与 MIT 运控调试流程。
