# Inspire RH56 仿人五指灵巧手 使用说明

## 目录结构

```
inspire/
├── demo_485.py                  # RS485 串口通信控制demo
├── demo_can.py                  # CAN 通信控制demo（需USB-CAN适配器）
├── demo_modbus.py               # Modbus TCP 以太网控制demo（单设备）
├── demo_modbus_multi-device.py  # Modbus TCP 以太网控制demo（多设备）
├── touch_data.py                # 触觉传感器矩阵数据读取（Modbus TCP）
├── touch_data_ts.py             # 触觉传感器力数据读取（Modbus TCP）
├── USB转RS485通信驱动/           # CH341 USB转串口驱动
└── readme.md                    # 本文件
```

---

## 一、RS485 串口通信

### 1.1 硬件准备

- USB 转 RS485 适配器（如 FT232R / CH341）
- 将适配器的 A、B 线连接到灵巧手的 RS485 接口
- 给灵巧手供电

### 1.2 驱动安装

如果系统未识别 USB 转串口设备，需要安装驱动：

```bash
cd inspire/USB转RS485通信驱动/CH341SER_LINUX
make
sudo make load
```

### 1.3 依赖安装

```bash
pip install pyserial
```

### 1.4 检查串口设备

```bash
# 查看是否识别到串口设备
ls /dev/ttyUSB*
# 预期输出: /dev/ttyUSB0
```

### 1.5 串口权限

```bash
# 临时授权（每次插拔后需要重新执行）
sudo chmod 666 /dev/ttyUSB0

# 永久授权（推荐，需重新登录生效）
sudo usermod -aG dialout $USER
```

### 1.6 运行 demo_485.py

```bash
cd ~/DualArmRobot/inspire

# 自动检测串口（默认波特率 115200，设备ID 1）
python3 demo_485.py

# 手动指定参数
python3 demo_485.py --port /dev/ttyUSB0 --baudrate 115200 --id 1
```

### 1.7 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | 自动检测 | 串口设备名，如 `/dev/ttyUSB0` |
| `--baudrate` | 115200 | 串口波特率 |
| `--id` | 1 | 灵巧手设备 ID |

### 1.8 demo_485.py 执行流程

1. 设置六个手指**运动速度**为 1000（最大）
2. 设置六个手指**抓握力度**为 500（50%）
3. 手指角度设为 0 → **手掌张开**
4. 手指角度设为 1000 → **手掌握拳**
5. 读取温度和实际角度
6. 执行内置动作序列 3

### 1.9 六个自由度对应关系

| 电缸 ID | 1 | 2 | 3 | 4 | 5 | 6 |
|---------|---|---|---|---|---|---|
| 手指 | 大拇指弯曲 | 大拇指旋转 | 食指 | 中指 | 无名指 | 小指 |

### 1.10 寄存器说明

| 寄存器名 | 地址 | 说明 |
|----------|------|------|
| `angleSet` | 1486 | 各自由度角度设置值（0-1000，-1不设置） |
| `forceSet` | 1498 | 各自由度力控阈值（0-1000） |
| `speedSet` | 1522 | 各自由度速度设置值（0-1000） |
| `angleAct` | 1546 | 各自由度角度实际值（只读） |
| `forceAct` | 1582 | 各手指实际受力（只读） |
| `temp` | 1618 | 各电缸温度（只读） |
| `errCode` | 1606 | 故障信息（只读） |
| `statusCode` | 1612 | 状态信息（只读） |
| `actionSeq` | 2320 | 动作序列索引号 |
| `actionRun` | 2322 | 运行当前动作序列 |

---

## 二、以太网 Modbus TCP 通信

### 2.1 硬件准备

- 网线直连电脑与灵巧手的以太网接口
- 给灵巧手供电

### 2.2 网络配置

灵巧手默认 IP 为 `192.168.11.210`，端口 `6000`。
需要将电脑有线网口配置为**同网段**的静态 IP：

```bash
# 查看有线网口名称（通常为 enp49s0 或 eth0）
ip link show

# 手动配置静态 IP（临时，重启失效）
sudo ip addr add 192.168.11.100/24 dev enp49s0
sudo ip link set enp49s0 up

# 或者使用 nmcli 配置（推荐，持久生效）
sudo nmcli connection modify "有线连接 1" \
  ipv4.addresses 192.168.11.100/24 \
  ipv4.method manual
sudo nmcli connection up "有线连接 1"
```

### 2.3 验证连接

```bash
ping 192.168.11.210
```

### 2.4 依赖安装

```bash
pip install pymodbus
```

### 2.5 可用脚本

#### demo_modbus.py — 单设备控制

```bash
python3 demo_modbus.py
```

功能：控制单台灵巧手进行张开/握拳动作。
默认连接 `192.168.11.210:6000`。

#### demo_modbus_multi-device.py — 多设备并行控制

```bash
python3 demo_modbus_multi-device.py
```

功能：同时控制多台灵巧手（默认 IP: `192.168.11.210` 和 `192.168.11.220`），使用多线程并行控制。

#### touch_data.py — 触觉传感器矩阵数据

```bash
python3 touch_data.py
```

功能：读取五指和掌心的触觉传感器矩阵数据，以矩阵形式输出。

#### touch_data_ts.py — 触觉传感器力数据

```bash
python3 touch_data_ts.py
```

功能：持续读取各手指的**法向力**和**切向力**数据，并显示读取频率（Ctrl+C 停止）。

### 2.6 已验证测试结果（2026-04-08）

| 脚本 | 状态 | 备注 |
|------|------|------|
| `demo_modbus.py` | ✅ 通过 | 灵巧手执行张开/握拳动作 |
| `touch_data.py` | ✅ 通过 | 读取频率约 50 Hz，固件版本 0116 |
| `touch_data_ts.py` | ✅ 通过 | 读取频率约 90-97 Hz |

---

## 三、常见问题

### Q: 串口权限不足 (Permission denied)

```bash
sudo chmod 666 /dev/ttyUSB0
```

### Q: 以太网连接失败 (Connection failed / Activation of network connection failed)

这通常是因为有线网口没有配置 IP 地址。需手动配置：

```bash
sudo nmcli connection modify "有线连接 1" \
  ipv4.addresses 192.168.11.100/24 \
  ipv4.method manual
sudo nmcli connection up "有线连接 1"
```

其他排查：
1. 确认网线已插好，网口指示灯亮起
2. 确认电脑网口 IP 与灵巧手在同一网段（192.168.11.x）
3. `ping 192.168.11.210` 测试连通性

### Q: pymodbus ImportError

如果出现 `cannot import name 'ModbusTcpClient' from 'pymodbus.client'`，
说明 pymodbus 版本为 2.x，导入路径不同。脚本中已做兼容处理。

### Q: 读取数据偶尔报错 IndexError

RS485 通信偶尔会出现数据帧不完整的情况，重新运行即可。
