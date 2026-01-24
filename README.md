# DualArmRobot

双臂机器人项目

## ino_motor - 汇川伺服电机驱动

汇川伺服电机控制模块，通过 **CANopen** 协议通信。

### 目录结构

```
ino_motor/
├── py_test/          # Python 版本 (快速测试)
│   ├── servo_driver_400.py   # 400W 电机 (节点 0x601)
│   └── servo_driver_750.py   # 750W 电机 (节点 0x602)
└── cpp_version/         # C++ 版本 (详见 cpp_version/README.md)
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

> 注意：包名是 `pyserial`，不是 `serial`

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

详见 [ino_motor/cpp_version/README.md](ino_motor/cpp_version/README.md)

---
