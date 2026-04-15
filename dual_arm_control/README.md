# DualArmControl - 双臂机器人控制上位机

基于 Qt6 的双臂机器人统一控制上位机。当前在同一个 GUI 页面内同时包含两类能力：

- 汇川伺服电机控制：单条 USB-CAN 总线下多节点在线管理
- `HumanoidArms_SDK` 双臂机械臂控制：左右臂 7 关节滑块位置控制、状态读取、清错、刹车

---

## 硬件环境

| 组件 | 说明 |
|------|------|
| USB-CAN 适配器 | CH340 串口转 CAN，17 字节帧协议 / CANalyst-II |
| 400W 电机 | 节点 ID `0x601`，SDO 响应 `0x581` |
| 750W 电机 | 节点 ID `0x602`，SDO 响应 `0x582`，方向反转 |
| HumanoidArms SDK | `../HumanoidArms/HumanoidArms_SDK` |
| 双臂机械臂 | 左臂 7 关节 + 右臂 7 关节 |
| 通信协议 | CANopen SDO（CiA 402 状态机） |
| 串口波特率 | 默认 9600（适配器与主机之间） |

---

## 架构设计

严格按照四层分离架构，详见 `qt.md`。

```
┌─────────────────────────────────────────────────────────────┐
│                        GUI 层 (ui/)                         │
│  MainWindow / ConnectionPanel / MotorControlPanel           │
│  HumanoidArmsPanel / StatusMonitorPanel / LogWidget         │
├─────────────────────────────────────────────────────────────┤
│                     业务服务层 (services/)                    │
│  MotorService / ArmService                                  │
├─────────────────────────────────────────────────────────────┤
│                     协议适配层 (protocols/)                   │
│  CanopenMotorProtocol / HumanoidArmsSdkAdapter             │
├─────────────────────────────────────────────────────────────┤
│                     通信调度层 (communication/)               │
│  CommunicationManager → CanBusWorker → FrameIO              │
│  一条总线一个 worker / 优先级队列 / 串行请求-响应 / 轮询调度     │
└─────────────────────────────────────────────────────────────┘
```

### 核心原则

- **一条物理总线只允许一个 worker 独占**：`/dev/ttyUSB0` 只有一个 `CanBusWorker` 打开管理
- **所有请求统一进入队列**：UI 控制、后台监控、初始化流程全部排队
- **优先级机制**：Critical（急停）> Control（使能/速度）> Monitoring（扭矩轮询）
- **UI 主线程不接触物理设备**：全部通过 signal/slot 异步交互

---

## 目录结构

```
dual_arm_control/
├── CMakeLists.txt                          # Qt6 CMake 构建
├── main.cpp                                # 程序入口
├── models/
│   └── common_types.h                      # 统一数据结构
│       - Priority / OperationMode / CanFrame
│       - CommRequest / CommResult
│       - MotorState / ArmState / CollisionConfig / BusConfig
│       - LogLevel / LogEntry
├── communication/                          # 通信调度层
│   ├── frame_io.h/cpp                      # USB-CAN 17字节帧协议收发
│   ├── can_bus_worker.h/cpp                # 总线 worker (优先级队列/响应匹配)
│   └── communication_manager.h/cpp         # 通信管理器 (线程/轮询/分发)
├── protocols/
│   ├── canopen_motor_protocol.h/cpp        # CANopen SDO 编解码
│   └── humanoid_arms_sdk_adapter.h/cpp     # HumanoidArms SDK 适配层
├── services/
│   ├── motor_service.h/cpp                 # 电机业务服务
│   └── arm_service.h/cpp                   # 双臂机械臂业务服务
└── ui/                                     # Qt GUI 层
    ├── main_window.h/cpp                   # 主窗口 - 组装所有面板 + signal/slot 连接
    ├── connection_panel.h/cpp              # 总线连接面板
    │   - 串口选择 / 波特率 / 连接断开 / 状态指示
    ├── motor_control_panel.h/cpp           # 电机控制面板
    │   - 节点选择 / 模式切换 / 使能失能急停
    │   - 速度控制 / 位置控制 / 抱闸控制
    │   - 批量操作 (全部使能/失能/急停/复位)
    ├── status_monitor_panel.h/cpp          # 实时状态监控面板
    │   - 双电机仪表盘并排显示
    │   - 扭矩条形进度条
    │   - 在线/使能/模式/故障/碰撞状态指示
    ├── torque_gauge_widget.h/cpp           # 扭矩仪表盘自绘控件
    │   - 圆弧仪表 (绿色安全区 / 红色危险区)
    │   - 实时指针 + 峰值标记
    │   - 碰撞触发闪红 + 状态 LED 指示灯
    ├── humanoid_arms_panel.h/cpp           # 同页机械臂控制板块
    │   - SDK 连接/断开
    │   - 左右臂各 7 关节滑块位置控制
    │   - 当前关节角 / 末端位姿 / 错误状态显示
    │   - 清错 / 刹车 / 同步当前姿态
    └── log_widget.h/cpp                    # 系统日志面板
        - 彩色分级日志 (INFO/WARN/ERROR/CRIT)
        - 自动滚动 / 行数限制 / 一键清空
├── third_party/                            # 本地头文件依赖
│   ├── eigen3_pkg/
│   └── boost_pkg/
└── ncurses.h                               # SDK 头文件的本地 shim
```

---

## 编译与运行

### 依赖

- Qt6 (Widgets + Core + SerialPort)
- CMake >= 3.16
- GCC/G++ (C++17)
- `HumanoidArms/HumanoidArms_SDK`

Ubuntu/Debian 安装依赖：

```bash
sudo apt install qt6-base-dev libqt6serialport6-dev cmake g++
```

说明：

- 机械臂 SDK 依赖的 `Eigen` / `Boost property_tree` 头文件已经放在仓库内的 `third_party/`，不要求系统额外安装开发包
- 运行时会直接加载 `../HumanoidArms/HumanoidArms_SDK/usrlib/` 下的 `libHumanoidArms.so` 和 `libcontrolcan.so`

### 编译

```bash
cd /home/yang/DualArmRobot/dual_arm_control
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 运行

```bash
sudo ./dual_arm_control
```

说明：

- 使用 CANalyst-II 或 `HumanoidArms_SDK` 时，建议直接用 `sudo` 启动
- 机械臂 SDK 的 `can_init()`/`Exit()` 由 GUI 内的 `HumanoidArms` 板块控制

---

## 使用流程

### 1. 连接总线

1. 插入 USB-CAN 适配器
2. 在「总线连接」面板选择串口（如 `/dev/ttyUSB0`）
3. 波特率选 `9600`
4. 点击「连接」

连接成功后，系统自动开始轮询两个电机的扭矩数据（100ms 间隔）。

### 2. 使能电机

1. 在「电机选择」下拉框选择目标节点
2. 选择控制模式（速度模式 / 位置模式）
3. 点击「使能」

使能完成后自动启用碰撞保护：
- 400W 电机 (0x601)：阈值 500‰ (50% 额定转矩)
- 750W 电机 (0x602)：阈值 200‰ (20% 额定转矩)

### 3. 控制运动

**速度模式：**
- 输入目标速度（RPM），点击「执行」
- 点击「停止」设为 0 RPM

**位置模式：**
- 输入目标角度和限速
- 选择相对/绝对模式，点击「执行」

### 4. 实时监控

切换到「实时监控」标签页，可看到：
- 两个电机的圆弧扭矩仪表盘（实时更新）
- 扭矩条形进度条（绿→黄→红渐变）
- 在线/使能/碰撞保护状态 LED
- 峰值扭矩记录

### 5. 碰撞保护

碰撞检测逻辑（与原版 `interactive_control_cpp.py` 一致）：
- 每 100ms 读取电机实际扭矩 (`6077h`)
- 当扭矩绝对值连续 3 次 >= 阈值时触发
- 触发后自动执行 quick stop (急停)
- 仪表盘闪红 + 弹出告警对话框
- 需要手动「故障复位」后重新使能

### 6. 批量操作

- 「全部使能」/ 「全部失能」/ 「全部急停」/ 「全部复位」
- 对所有已注册节点同时生效

### 7. HumanoidArms 机械臂板块

主页面左侧新增了 `HumanoidArms` 控制板块，与原有电机控制在同一个页面内显示。

使用流程：

1. 在板块顶部设置 `deviceIndex` 和 `canIndex`
2. 点击「连接 SDK」
3. 点击「刷新状态」读取左右臂当前关节角、末端位姿和错误状态
4. 如需避免误动作，可先点击「同步当前姿态」，将滑块目标同步到当前真实姿态
5. 拖动左臂或右臂任一关节滑块，释放后会按整臂 7 关节目标调用 `moveJ_ToJoint(...)`
6. 如有故障可点击「清错」，如需立即停车可点击「刹车」

当前机械臂板块已接入的 SDK 能力：

- `can_init()` / `Exit()`
- `get_joint()`
- `get_Pos()`
- `get_mechanicalarm_status()`
- `moveJ_ToJoint()`
- `clear_elc_error()`
- `brake()`

说明：

- 滑块当前先按 `±180°` 配置范围，后续可再按真实关节限位细化
- GUI 显示使用角度 `deg`，内部下发前会自动转换为 SDK 所需的弧度 `rad`
- 机械臂 SDK 当前是自管 CAN，不并入现有 `CommunicationManager`

---

## 信号流

```
用户点击按钮
  → MotorControlPanel 发出 signal (enableRequested / velocityRequested ...)
    → MotorService 封装业务请求
      → CommunicationManager 分发到 CanBusWorker
        → CanBusWorker 在专属线程中串口收发
          → 协议层解析响应
            → MotorService 更新状态缓存，发出 signal
              → StatusMonitorPanel / TorqueGaugeWidget 更新显示
```

```
扭矩轮询（后台自动）
  → CommunicationManager 定时器触发
    → CanBusWorker 发送 SDO 读取 6077h
      → MotorService::onTorqueResult 解析
        → 更新 torquePermille → emit torqueUpdated
          → TorqueGaugeWidget 指针 + 条形图更新
        → 碰撞检测 → 连续超阈值 → emit collisionDetected
          → 自动急停 + 弹窗告警
```

---

## 预配置电机参数

| 参数 | 400W (0x601) | 750W (0x602) |
|------|-------------|-------------|
| 方向反转 | 否 | 是 |
| 碰撞转矩限制 | 500‰ (50% 额定) | 200‰ (20% 额定) |
| 碰撞触发阈值 | 500‰ | 200‰ |
| 连续采样次数 | 3 | 3 |
| 急停方式 | Quick Stop | Quick Stop |
| 仪表盘满量程 | 6000‰ | 3000‰ |
| 编码器分辨率 | 2^23 = 8388608 | 2^23 = 8388608 |

---

## 从 ino_motor 迁移说明

本项目从 `ino_motor/cpp_version/` 的功能迁移而来，主要变化：

| 原版 (ino_motor) | 新框架 (dual_arm_control) |
|---|---|
| `InovanceServo` 继承 `CanInterfaceUsb`，设备直接持有串口 | 四层分离，设备不接触串口 |
| 单设备单线程，各自打开串口 | 一条总线一个 worker，多节点共享 |
| Python `TorqueMonitor` 后台线程轮询 | `CommunicationManager` 统一调度轮询 |
| 终端 `\r` 覆盖显示扭矩条 | Qt 圆弧仪表盘 + 条形进度条 |
| 碰撞保护在 C++ 层 `collisionProtectionLoop` | `MotorService::onTorqueResult` 集中检测 |
| 无优先级，控制和监控互相阻塞 | Critical > Control > Monitoring 优先级队列 |

---

## 后续扩展

按 `qt.md` 规划，后续可在不推翻现有架构的前提下扩展：

- **新设备**：新增 `GripperService` / `SensorService`，接入 `CommunicationManager`
- **新协议**：新增 `ModbusProtocol` / `CustomSerialProtocol`，不影响通信层
- **新总线**：新增 `SerialBusWorker` / `TcpBusWorker`，各自独占线程
- **日志增强**：文件日志、原始报文录包、数据回放
- **安全增强**：联锁策略、温度监控、多条件碰撞判断
