# ZE300 C++ 控制包

该目录提供 ZE300 的 C++ 驱动与 Python 绑定（`pybind11`），通信方式为 USB-CAN 串口转发。

## 默认参数

- USB-CAN 串口波特率：`921600`
- CAN 波特率：`1000kbps`（由驱动器/上位机配置）
- 设备地址（CAN ID）：`0x01`
- CAN 帧格式：标准帧（11-bit）

## 已实现命令（基于手册 V3.07b0）

- 状态读取：`0xA1` `0xA2` `0xA3` `0xA4` `0xAE`
- 参数设置：`0xB1` `0xB2` `0xB3` `0xB5`
- 控制命令：`0xC0` `0xC1` `0xC2` `0xC3` `0xC4` `0xCE` `0xCF`
- 故障处理：`0xAF`
- MIT 运控：`0xF0` `0xF1` + MIT 8-byte 数据帧

## 编译

```bash
cd ZE300/cpp_version
python setup.py build_ext --inplace
```

## Python 导入示例

```python
import ze300_motor

motor = ze300_motor.Ze300Motor(
    port="/dev/ttyUSB0",
    usb_baud=921600,
    dev_addr=0x01,
)
motor.set_silent_mode(True)
```
