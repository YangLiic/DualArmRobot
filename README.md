# 双臂机器人 (DualArmRobot)

## 环境准备与依赖安装 (首次使用)

> 💡 项目使用 Python 虚拟环境管理依赖，`.venv/` 目录不纳入版本控制。
> 首次 clone 后需按以下步骤搭建环境。

### 系统依赖 (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install python3-venv python3-dev build-essential cmake qt6-base-dev libqt6serialport6-dev
```

### 创建虚拟环境并安装 Python 依赖

```bash
# 创建虚拟环境（仅首次）
python3 -m venv .venv

# 激活虚拟环境
source .venv/bin/activate

# 安装项目依赖
pip install -r requirements.txt
```

> ⚠️ **每次打开新终端都需要重新激活虚拟环境**：`source .venv/bin/activate`

---

## dual_arm_control 上位机

基于 Qt6 的双臂机器人统一控制 GUI，详细文档见 [`dual_arm_control/README.md`](dual_arm_control/README.md)。

### 编译

```bash
cd dual_arm_control
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 运行

```bash
sudo ./dual_arm_control
```

启动后在 GUI 界面中：

1. 设备类型选择 **VCI**
2. 波特率选择 **1000 kbps**
3. 点击 **连接**

---

## 项目结构

```
DualArmRobot/
├── dual_arm_control/    # Qt6 统一控制上位机 (C++)
├── ino_motor/           # 汇川伺服电机驱动 (Python + C++ 扩展)
├── CyberGear/           # 小米 CyberGear 微电机驱动
├── ZE300/               # 瑞龙 ZE300 电机驱动
├── inspire/             # Inspire 灵巧手控制 (RS485 / 网线通讯，非 CAN)
├── HumanoidArms/        # 双臂机械臂 SDK
├── requirements.txt     # Python 依赖清单
└── docs/                # 文档资料
```

---

## 硬件信息参考

*   **转换芯片**：CH340 (QinHeng Electronics)
*   **USB VID:PID**：`1a86:7523`
*   **CAN 协议支持**：支持 CAN 2.0A (标准帧) 与 CAN 2.0B (扩展帧)
