# CyberGear 微电机控制

通过 USB-CAN 适配器控制小米 CyberGear 微电机。

## 使用前准备

### 1. 确认 USB-CAN 端口号

CyberGear 的 USB-CAN 适配器插入后会分配一个 `/dev/ttyUSBx` 设备。在 Linux 中，**编号取决于插入顺序**：先插入的是 `ttyUSB0`，后插入的是 `ttyUSB1`，以此类推。

```bash
# 查看当前所有 USB 串口设备
ls /dev/ttyUSB*

# 如果有多个设备，可以通过物理端口路径区分
ls -la /dev/serial/by-path/
```

> ⚠️ 如果系统同时连接了多个 USB-CAN 适配器（如 ino_motor 的适配器），请确认哪个端口对应 CyberGear。
> 可以先拔掉其他适配器，只留 CyberGear 的来确认。

### 2. 赋予串口权限

每次重新插拔 USB 后需要重新赋权：

```bash
sudo chmod 777 /dev/ttyUSB0
```

### 3. 编译 C++ 模块（首次使用）

```bash
cd CyberGear/cpp_version

# 安装依赖（首次）
uv pip install setuptools pybind11

# 编译
python setup.py build_ext --inplace
```

## 运行交互式控制程序

```bash
source .venv/bin/activate
python CyberGear/interactive_control.py
```

程序启动后会显示菜单：

```
  [1] 速度模式    - 设定转速 (rad/s)
  [2] 位置模式    - 设定目标角度
  [3] 设置零位    - 将当前位置设为机械零位
  [4] 回到零位    - 运动到零位
  [5] 读取状态    - 查看位置/速度/力矩/温度
  [q] 退出
```

### 默认参数

| 参数 | 默认值 | 环境变量 | 说明 |
|------|--------|---------|------|
| 串口 | `/dev/ttyUSB0` | `CYBERGEAR_PORT` | USB-CAN 适配器端口 |
| 电机 CAN ID | `0x01` | `CYBERGEAR_MOTOR_ID` | 电机的 CAN 总线地址 |
| 主机 CAN ID | `0xFD` | `CYBERGEAR_MASTER_ID` | 主机标识 |
| 串口波特率 | `921600` | — | 固定值 |
| CAN 波特率 | `1 Mbps` | — | 适配器内部设置 |

### 通过环境变量指定参数

```bash
# 指定不同的串口
CYBERGEAR_PORT=/dev/ttyUSB1 python CyberGear/interactive_control.py

# 指定不同的电机 CAN ID
CYBERGEAR_MOTOR_ID=0x7F python CyberGear/interactive_control.py

# 同时指定多个参数
CYBERGEAR_PORT=/dev/ttyUSB1 CYBERGEAR_MOTOR_ID=0x02 python CyberGear/interactive_control.py
```

## 安全提示

| 参数 | 安全范围 | 说明 |
|------|---------|------|
| 速度 | ±5 rad/s | 初次测试建议用小值 |
| 位置 | ±12.5 rad (±716°) | CyberGear 硬件限制 |
| 电流限制 | 0-27 A | 默认 5A |

## 目录结构

```
CyberGear/
├── README.md                  # 本文件
├── interactive_control.py     # 交互式控制程序
├── 头部旋转电机-小米/          # 电机资料
│   ├── CyberGear微电机使用说明书.pdf
│   └── 串口协议说明.docx
├── 小米电机专用资料/            # 调试工具和配置软件
└── cpp_version/               # C++ SDK
    ├── cybergear_can_usb.h/cpp   # USB-CAN 通信层
    ├── cybergear_motor.h/cpp     # 电机控制协议
    ├── python_bindings.cpp       # Python 绑定
    ├── setup.py                  # 编译脚本
    └── README.md                 # C++ API 文档
```
