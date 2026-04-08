# servo_hub_qt

基于 `qt.md` 的首版 Qt Widgets 上位机重构工程。

## 目标

- 一个 USB-CAN 口只由一个 `CanBusWorker` 独占
- 多电机节点通过统一调度层排队通信
- UI 只通过 `MotorService` 交互，不直接碰串口
- 首版支持同一总线下的两台汇川伺服节点 `0x601` / `0x602`

## 目录

```text
servo_hub_qt/
├── CMakeLists.txt
├── README.md
└── src/
    ├── app/             # MainWindow 与 GUI 交互
    ├── communication/   # CommunicationManager / CanBusWorker / 编解码
    ├── models/          # 共享数据模型与表格模型
    ├── protocols/       # CANopen 协议适配
    └── services/        # MotorService 业务层
```

## 依赖

- Qt 6 或 Qt 5
- `Qt::Core`
- `Qt::Widgets`
- `Qt::SerialPort`
- CMake 3.16+

## Ubuntu 22.04 安装依赖

优先推荐 Qt 6：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config qt6-base-dev libqt6serialport6-dev
```

如果你想改用 Qt 5，也可以装：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config qtbase5-dev libqt5serialport5-dev
```

## 构建

```bash
cd /home/yang/DualArmRobot/servo_hub_qt
cmake -S . -B build
cmake --build build
./build/servo_hub_qt
```

## 当前实现边界

- 已实现单进程、多线程、每条总线一个 worker
- 已实现请求优先级、监控轮询、批量安全操作和基础日志
- 已实现 Qt 页面：总线连接、电机表格、单机控制、批量控制、日志视图
- 本机当前环境缺少 `cmake` 和 Qt 开发工具链，所以本次未能在这里完成编译验证

## 串口权限

临时测试时可以执行：

```bash
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
```

但这只是临时方案，重新插拔设备后通常还要再执行一次。

更推荐的长期方案是把当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

执行后重新登录终端或重启一次，再检查：

```bash
groups
```

如果串口仍然权限不稳定，再考虑补 `udev` 规则，而不是长期依赖 `chmod 777`。
