# DualArmRobot

双臂机器人项目

> **重要：每次重新插拔 USB 后，必须先执行串口权限设置！**
> ```bash
> sudo chmod 777 /dev/ttyUSB0
> ```

## ino_motor - 汇川伺服电机驱动

汇川伺服电机控制模块，通过 **CANopen** 协议通信。

### 目录结构

```
ino_motor/
├── py_test/                    # Python 版本 (快速测试)
│   ├── servo_driver_400.py     # 400W 电机 (节点 0x601)
│   └── servo_driver_750.py     # 750W 电机 (节点 0x602)
├── interactive_control_cpp.py  # 交互式控制程序
├── brake_control.py            # 抱闸控制工具
└── cpp_version/                # C++ SDK
```

---

## 快速开始 (Python 版本)

Python 版本用于简单测试，速度模式控制。

### 1. 事先准备

```bash
# 查看 USB 串口设备
ls /dev/ttyUSB*

# 如果是 /dev/ttyUSB0，赋予权限
sudo chmod 777 /dev/ttyUSB0
```

### 2. 进入项目目录

```bash
cd DualArmRobot
```

### 3. UV 环境使用

[UV](https://github.com/astral-sh/uv) 是一个快速的 Python 包管理器。

```bash
# 创建虚拟环境 (首次)
uv venv

# 激活环境
source .venv/bin/activate

# 安装依赖
uv pip install pyserial
```

### 4. 运行测试

```bash
# 激活环境后运行
python ino_motor/py_test/servo_driver_400.py   # 测试 400W 电机
python ino_motor/py_test/servo_driver_750.py   # 测试 750W 电机
```

---

## 交互式控制程序 (C++ SDK)

```bash
cd ino_motor/cpp_version

uv pip install setuptools pybind11
sudo apt install python3-dev

python setup.py build_ext --inplace

cd ../..
python ino_motor/interactive_control_cpp.py
```

### 电机方向说明

| 电机 | 正值 (+) | 负值 (-) |
|------|----------|----------|
| 电机1 (0x601) | 双臂上升 | 双臂下降 |
| 电机2 (0x602) | 机器人右转 (顺时针) | 机器人左转 (逆时针) |

### 安全运动范围

| 模式 | 安全值 | 说明 |
|------|--------|------|
| 速度模式 | ±100 RPM | 非常安全的速度 |
| 位置模式 | ±180° | 实际仅上升（下降）或转动一点角度|

### 还需要后期使用者标定

详见 [ino_motor/cpp_version/README.md](ino_motor/cpp_version/README.md)

---

## 抱闸控制

松闸后电机可手动转动，用于调整机械位置。

```bash
cd ino_motor/cpp_version
python setup.py build_ext --inplace

cd ../..
# 松闸电机1 (0x601)
python ino_motor/brake_control.py release 1

# 松闸电机2 (0x602)
python ino_motor/brake_control.py release 2

# 锁闸 (恢复正常)
python ino_motor/brake_control.py lock 1

python ino_motor/brake_control.py lock 2
# 交互模式
python ino_motor/brake_control.py
```

**警告**：松闸后电机失去保持力，垂直安装时负载会下落！

---
