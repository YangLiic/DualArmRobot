# DualArmRobot

双臂机器人项目

## CAN-USB 模块说明

本项目使用多个 CAN-USB 模块（CH340 芯片）控制不同的电机组。  
当多个模块同时插入时，`/dev/ttyUSB0`、`/dev/ttyUSB1` 的编号**会随插入顺序变化**。

### 如何区分多个 CAN-USB 模块

```bash
# 1. 查看当前所有 CAN-USB 设备
ls -la /dev/serial/by-path/

# 2. 确认哪个 ttyUSB 对应哪个物理端口
# 例如输出:
#   pci-...-usb-0:1:1.0-port0   -> ../../ttyUSB0
#   pci-...-usb-0:2.1:1.0-port0 -> ../../ttyUSB1
```

### 指定串口（当默认 ttyUSB0 不对时）

所有脚本默认使用 `/dev/ttyUSB0`，可通过环境变量 `INO_MOTOR_PORT` 覆盖：

```bash
# 临时指定（当次运行有效）
INO_MOTOR_PORT=/dev/ttyUSB1 python ino_motor/interactive_control_cpp.py

# 或者 export 后所有脚本生效
export INO_MOTOR_PORT=/dev/ttyUSB1
python ino_motor/interactive_control_cpp.py
python ino_motor/brake_control.py
```

> **提示**：只插一个 CAN-USB 模块时，默认 `/dev/ttyUSB0` 就是对的，无需设置。

---

## 串口权限设置

> **每次重新插拔 USB 后，必须先执行串口权限设置！**

```bash
sudo chmod 777 /dev/ttyUSB0
# 如果用的是 ttyUSB1
sudo chmod 777 /dev/ttyUSB1
```

---

## ino_motor - 汇川伺服电机驱动

汇川伺服电机控制模块，通过 **CANopen** 协议通信。  
一个 CAN-USB 模块连接两个电机（0x601 + 0x602）。

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

# 赋予权限
sudo chmod 777 /dev/ttyUSB0
```

### 2. 进入项目目录

```bash
cd DualArmRobot
```

### 3. 激活项目虚拟环境

项目使用统一的虚拟环境，如尚未创建请参照 [项目主 README](../README.md) 完成环境搭建。

```bash
# 回到项目根目录激活环境
cd DualArmRobot
source .venv/bin/activate
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
cd /home/yang/DualArmRobot/ino_motor/cpp_version
python setup.py build_ext --inplace

cd /home/yang/DualArmRobot
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

## 碰撞保护机制

SDK 共实现了 **4 种保护措施**，当前默认启用了其中 **第 1、2 种**（速度模式），位置模式额外启用第 4 种。

### 四种保护措施一览

| # | 保护措施 | SDO 地址 | 机制 | 当前状态 |
|---|---------|---------|------|---------|
| 1 | **最大转矩限制** | `0x6072` (Max Torque) | 驱动器硬件级钳位，转矩绝对不会超过此值 | ✅ 已启用 |
| 2 | **实际转矩监控** | `0x6077` (Actual Torque) | 软件后台线程轮询实际转矩，连续超阈值 → 停机 | ✅ 已启用 |
| 3 | **相电流监控** | `0x200B:19` (Phase Current) | 软件后台线程轮询相电流，超阈值 → 停机 | ❌ 未启用 |
| 4 | **位置偏差监控** | `0x60F4` (Position Deviation) | 软件后台线程轮询位置偏差，超阈值 → 停机 | ⚠️ 仅位置模式 |

> **注意**：第 3 种（相电流监控）当前 `trigger_current_amp=0.0`，即未启用。如需启用，修改 `enable_collision_protection()` 中的参数即可。

### 千分比单位说明

`0x6077` 和 `0x6072` 的单位是 **千分比（‰，permille）**，相对于电机 **额定转矩**：

| 值 | 含义 |
|----|------|
| `1000‰` | 100% 额定转矩 |
| `2000‰` | 200% 额定转矩（2 倍过载） |
| `5000‰` | 500% 额定转矩（5 倍过载） |

### 当前默认阈值（按电机区分）

| 电机 | 节点 ID | 转矩限制 (0x6072) | 触发停机阈值 (0x6077) | 自动失能阈值 |
|------|---------|-------------------|---------------------|-------------|
| **400W** | 0x601 | 5000‰ (500% 额定) | 5000‰ (500% 额定) | 5000‰ (500% 额定) |
| **750W** | 0x602 | 2000‰ (200% 额定) | 2000‰ (200% 额定) | 2000‰ (200% 额定) |

### 保护触发流程

```
电机运行中
  │
  ├─ 保护1: 驱动器硬件实时钳位转矩 ≤ 限制值 (0x6072)
  │
  ├─ 碰撞保护线程 (C++ 后台, 20ms 轮询):
  │   ├─ 保护2: 读取 0x6077 实际转矩 → 连续 3 次超阈值 → quick_stop
  │   └─ 保护4: 读取 0x60F4 位置偏差 → 连续 3 次超阈值 → quick_stop (仅位置模式)
  │
  └─ TorqueMonitor 线程 (Python 后台, 100ms 轮询):
      ├─ 实时显示: 📊 扭矩(6077h): +150‰ (15.0%额定) |███████ ... |
      └─ 自动失能: 连续 3 次超阈值 → set_velocity(0) + disable()
```

### 碰撞保护 vs 扭矩自动失能的区别

| | 碰撞保护 (C++ 线程) | 扭矩自动失能 (Python TorqueMonitor) |
|--|-------------------|-----------------------------------|
| **实现层** | C++ 后台线程 | Python 后台线程 |
| **轮询间隔** | 20ms（更快） | 100ms |
| **触发动作** | `quick_stop`（急停） | `set_velocity(0)` + `disable()`（失能） |
| **触发后状态** | 电机锁轴（仍使能） | 电机完全失能（松轴） |

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
