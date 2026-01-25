# Inovance Servo C++ SDK

CANopen 协议汇川伺服电机控制库。

## 编译

```bash
# C++ 程序
g++ -o servo_test servo_test.cpp inovance_servo.cpp hw_can_usb.cpp -lpthread -std=c++11

# Python 模块
python setup.py build_ext --inplace
```

## 使用

### C++

```cpp
#include "inovance_servo.h"
using namespace pg;

InovanceServo servo("/dev/ttyUSB0", 9600, 0x601);
std::thread recv([&]() { servo.can_dump(); });

servo.faultReset();
servo.enable(OperationMode::VELOCITY);
servo.setVelocity(60);
sleep(3);
servo.stop();
```

### Python

```python
import inovance_servo

servo = inovance_servo.InovanceServo("/dev/ttyUSB0", 9600, 0x601)
servo.set_silent_mode(True)
servo.fault_reset()
servo.enable(inovance_servo.OperationMode.VELOCITY)
servo.set_velocity(60)
```

---

## API

### 构造函数

```cpp
InovanceServo(const std::string& port, int baud, uint32_t node_id = 0x601)
```

### 基础控制

| 方法 | 说明 |
|------|------|
| `enable()` | 使能 (默认速度模式) |
| `enable(OperationMode mode)` | 使能 (指定模式) |
| `disable()` | 失能 |
| `stop()` | 停止并失能 |
| `faultReset()` | 错误复位 |
| `quickStop()` | 急停 |

### 速度控制

```cpp
servo.enable(OperationMode::VELOCITY);
servo.setVelocity(60);   // +60 RPM 顺时针
servo.setVelocity(-30);  // -30 RPM 逆时针
servo.setVelocity(0);    // 停止
```

### 位置控制

```cpp
servo.enable(OperationMode::PROFILE_POSITION);
servo.setProfileVelocity(30);       // 限速 30 RPM
servo.setProfileAcceleration(50);   // 加速度
servo.setProfileDeceleration(50);   // 减速度
servo.setPosition(90.0, false);     // 相对 +90°
servo.setPosition(180.0, true);     // 绝对 180°
```

| 方法 | 参数 | 说明 |
|------|------|------|
| `setPosition(deg, abs)` | deg: 角度, abs: 绝对/相对 | 设置目标位置 |
| `setPositionPulse(p, abs)` | p: 脉冲数 | 按脉冲设置位置 |
| `setProfileVelocity(rpm)` | rpm | 运动速度限制 |
| `setProfileAcceleration(a)` | a | 加速度 |
| `setProfileDeceleration(d)` | d | 减速度 |
| `startPositionMove()` | - | 启动运动 (setPosition 已自动调用) |

### 方向控制

```cpp
servo.setDirectionInverted(true);   // 0x602 需要反转
bool inv = servo.isDirectionInverted();
```

### 状态查询

```cpp
bool en = servo.isEnabled();
OperationMode m = servo.getCurrentMode();
```

### NMT 管理

```cpp
servo.nmtStart();           // 启动 PDO (enable 自动调用)
servo.nmtPreOperational();  // 停止 PDO (stop 自动调用)
```

### 抱闸控制

```cpp
servo.releaseBrake();  // 松闸，电机可手动转动
servo.lockBrake();     // 锁闸，恢复正常
```

**警告**：松闸后电机将失去保持力，垂直安装时负载会下落！

### 调试控制

```cpp
servo.setSilentMode(true);  // 关闭收发打印
```

---

## 枚举

```cpp
enum class OperationMode : uint8_t {
    PROFILE_POSITION = 1,   // 位置模式
    VELOCITY = 3,           // 速度模式
    HOMING = 6              // 回零模式
};
```

---

## 单位

| 物理量 | 单位 | 换算 |
|--------|------|------|
| 速度 | RPM | - |
| 位置 | 度 | 360° = 8388608 pulses (23bit 编码器) |

---

## 文件

| 文件 | 说明 |
|------|------|
| `inovance_servo.h/cpp` | SDK 核心 |
| `hw_can_usb.h/cpp` | USB-CAN 驱动 |
| `servo_test.cpp` | 演示程序 |
| `python_bindings.cpp` | pybind11 绑定 |
| `setup.py` | Python 编译脚本 |

---

## 示例

### 速度模式

```cpp
InovanceServo servo("/dev/ttyUSB0", 9600, 0x601);
std::thread recv([&]() { servo.can_dump(); });

servo.faultReset();
sleep(1);

servo.enable(OperationMode::VELOCITY);
servo.setVelocity(60);
sleep(5);
servo.stop();
```

### 位置模式

```cpp
InovanceServo servo("/dev/ttyUSB0", 9600, 0x601);
std::thread recv([&]() { servo.can_dump(); });

servo.faultReset();
sleep(1);

servo.enable(OperationMode::PROFILE_POSITION);
servo.setProfileVelocity(30);
servo.setPosition(90.0, false);  // 转 90°
sleep(3);
servo.setPosition(-90.0, false); // 转回
sleep(3);
servo.stop();
```

### 多电机

```cpp
InovanceServo m1("/dev/ttyUSB0", 9600, 0x601);
InovanceServo m2("/dev/ttyUSB0", 9600, 0x602);
m2.setDirectionInverted(true);  // 统一方向
```

---

## 注意事项

1. 接收线程 `can_dump()` 必须启动
2. 使能前等待初始化 (~1s)
3. 程序退出前调用 `stop()`
4. 0x602 电机需要 `setDirectionInverted(true)`
