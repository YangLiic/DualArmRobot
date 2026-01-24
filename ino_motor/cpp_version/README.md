# 汇川伺服电机 C++ 驱动

基于 USB-CAN 适配器的汇川伺服电机控制程序（无 ROS 依赖）。

## 编译

```bash
cd /home/yang/DualArmRobot/ino_motor/cpp_version
g++ -o servo_test servo_test.cpp inovance_servo.cpp hw_can_usb.cpp -lpthread -std=c++11
```

## 运行

```bash
# 确保有串口权限
sudo chmod 777 /dev/ttyUSB0

# 运行测试程序
./servo_test
```

按 `Ctrl+C` 可安全停止电机并退出。

## 文件说明

| 文件 | 角色 | 作用 |
|------|------|------|
| `hw_can_usb.cpp` | 硬件通信层 | USB-CAN 适配器驱动，负责串口初始化、CAN 帧打包 (AA...7A 格式) 与解析、数据收发 |
| `inovance_servo.cpp` | 电机控制层 | 汇川伺服电机逻辑，封装 CANopen 协议：使能/失能、速度模式设置、NMT 状态管理 |
| `servo_test.cpp` | 应用入口 | 测试程序 main 函数，演示电机使能 → 旋转 → 停止的完整流程 |

## 架构

```
servo_test.cpp (应用层)
       │
       ▼
inovance_servo.cpp (控制层，继承自 CanInterfaceUsb)
       │
       ▼
hw_can_usb.cpp (硬件层，串口 ↔ CAN)
       │
       ▼
   /dev/ttyUSB0
```

## 配置

在 `servo_test.cpp` 中修改：

```cpp
std::string port_name = "/dev/ttyUSB0";  // 串口设备
int baud_rate = 9600;                     // 波特率
uint32_t node_id = 0x601;                 // CAN 节点 ID
```
