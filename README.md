# 双臂机器人 (DualArmRobot)

本项目旨在通过多个 USB-CAN 适配器实现对多台电机的协同控制，目前共涉及四台电机：

## 电机配置与模块分配

为了保证通信效率和兼容性，项目将电机分为两组，分别连接到两个 USB-CAN 模块：

### 1. Inovance 电机组 (2台)
*   **电机型号**：汇川伺服电机 (Node ID: `0x601`, `0x602`)
*   **共享模块**：模块 A
*   **通信参数**：
    *   **串口波特率**：`9600 bps`
    *   **CAN 总线波特率**：`500 kbps`
*   **SDK 目录**：`ino_motor/`

### 2. 小米 & 瑞龙电机组 (2台)
*   **电机型号**：
    *   小米 **CyberGear** (Node ID: `0x01` )
    *   瑞龙 **ZE300** (待开发)
*   **共享模块**：模块 B
*   **通信参数**：
    *   **串口波特率**：`921600 bps`
    *   **CAN 总线波特率**：`1000 kbps (1 Mbps)`
*   **SDK 目录**：`CyberGear/`

---

## CAN-USB 模块连接与管理

由于两个模块使用的是相同的 CH340 芯片，Linux 下的默认设备名 (`/dev/ttyUSB0`, `/dev/ttyUSB1`) 会随着**插入顺序**而变化。

### 端口确定与权限

1.  **确定序号**：通常先插入的模块对应较小的数值 (`ttyUSB0`)。建议通过以下命令查看连接情况：
    ```bash
    # 查看设备列表
    ls /dev/ttyUSB*

    # 查看物理端口映射（推荐，路径固定）
    ls -la /dev/serial/by-path/
    ```
2.  **赋予权限**：每次重新插拔后，需要对使用的端口赋权：
    ```bash
    sudo chmod 777 /dev/ttyUSB0
    sudo chmod 777 /dev/ttyUSB1
    ```

### 3. 环境准备与编译 (首次使用)

在运行控制程序之前，需要先编译对应的 C++ 驱动模块：

```bash
# 激活环境
source .venv/bin/activate

# 编译 Inovance 驱动
cd ino_motor/cpp_version && python setup.py build_ext --inplace && cd ../..

# 编译 CyberGear 驱动
cd CyberGear/cpp_version && python setup.py build_ext --inplace && cd ../..
```

---

## 运行示例

### 单次执行示例：
```bash
# 启动 Inovance 交互控制 (需指定端口，端口号根据 ls /dev/ttyUSB* 确定)
INO_MOTOR_PORT=/dev/ttyUSB0 python ino_motor/interactive_control_cpp.py

# 启动 CyberGear 交互控制 (需指定端口，端口号根据 ls /dev/ttyUSB* 确定)
CYBERGEAR_PORT=/dev/ttyUSB1 python CyberGear/interactive_control.py
```

---

## 硬件信息参考

*   **转换芯片**：CH340 (QinHeng Electronics)
*   **USB VID:PID**：`1a86:7523`
*   **CAN 协议支持**：支持 CAN 2.0A (标准帧) 与 CAN 2.0B (扩展帧)
