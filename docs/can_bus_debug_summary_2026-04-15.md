# CAN Bus 排查总结（2026-04-15）

## 背景

本次排查涉及两套系统共用同一块 USB-CAN 适配器：

- `dual_arm_control`
- `HumanoidArms/HumanoidArms_SDK_单编`

目标是确认：

- `VCI` 是否已经建立通信
- `dual_arm_control` 这套电机是否真的在线
- `HumanoidArms/Ti` 电机是否也在同一条 CAN 总线上
- 两套系统能否直接共线运行

---

## 结论摘要

### 1. VCI 硬件本身正常

已经确认 `libcontrolcan.so` 可以正常完成以下操作：

- `VCI_FindUsbDevice2`
- `VCI_OpenDevice`
- `VCI_InitCAN`
- `VCI_StartCAN`
- `VCI_ReadBoardInfo`

已识别设备：

- `device_index=0`
- `can_num=2`
- `serial='31F10002B0E'`
- `hw='CAN-Linux'`

说明 USB-CAN 设备本身没有问题。

### 2. 合法 CAN 通道只有 0 和 1

实测结果：

- `channel=0`：可初始化
- `channel=1`：可初始化
- `channel=2`：初始化失败

说明这块板卡只有两路 CAN，对应通道号 `0` 和 `1`。

### 3. `dual_arm_control` 预设的 `0x601/0x602` 节点在本次测试总线上没有响应

通过 `dual_arm_control/tools/vci_diag.py` 测试：

- `channel=0 @ 1000kbps`：`0x601/0x602` 无响应
- `channel=1 @ 1000kbps`：`0x601/0x602` 无响应
- `scan 1-50`：无 CANopen SDO 响应
- `listen 5s`：无任何原始 CAN 帧

这说明：

- 当前测试时接到该通道上的总线对 `dual_arm_control` 这套 CANopen 扫描来说是静默的
- 至少在该时刻，不存在可被 `vci_diag.py` 按当前假设识别的 `0x601/0x602` 节点

### 4. `HumanoidArms/Ti` 的 smoke test 在 `device_index=0, can_index=1` 上可工作

可用命令：

```bash
cd /home/yang/DualArmRobot/HumanoidArms/HumanoidArms_SDK_单编/src
sudo env LD_LIBRARY_PATH=/home/yang/DualArmRobot/HumanoidArms/HumanoidArms_SDK_单编/usrlib ./sdk_smoke_test left 0 1
```

在总线干净时，该命令可以成功输出：

- `Init CAN Success!`
- `Joint angles (rad)`
- `Joint angles (deg)`
- `TCP pose [x y z rx ry rz]`

说明：

- `HumanoidArms/Ti` 这一套确实在用同一块 VCI 设备
- 当前能工作的口位是 `device_index=0, can_index=1`

### 5. 两套系统当前不能直接并在同一条物理 CAN bus 上

已观察到的关键现象：

- 当 `/home/yang/DualArmRobot/dual_arm_control` 那套电机的 CAN 线接入总线后，`sdk_smoke_test` 会出现：
  - `start device_1 CAN_2 fail`
  - 一串 `ID 发送失败`
  - 最后出现 `segmentation fault`
- 当拔掉 `dual_arm_control` 那套电机的 CAN 线后，`sdk_smoke_test` 恢复正常，可以读出真实关节角与 TCP 位姿

这说明：

- 问题不在 VCI 硬件本身
- 问题也不在 `sdk_smoke_test` 的基本启动流程
- 根因更接近于物理总线冲突，或者两套设备不适合当前接法直接共线运行

---

## 当前问题的根因判断

目前最可能的根因是以下几类之一：

1. 两套设备协议不一致
   - `dual_arm_control` 当前按 CANopen / CiA402 假设去通讯
   - `HumanoidArms/Ti` 这一套很可能不是完全相同的协议栈

2. 波特率或 CAN 参数不一致
   - 一侧使用标准 `1Mbps`
   - 另一侧可能使用其他波特率，或 `1Mbps-非标`

3. 节点规划冲突
   - 节点地址、CAN ID 或报文格式不兼容

4. 终端电阻或接线问题
   - 两端终端电阻数量不对
   - `CANH/CANL/GND` 接法不一致
   - 某一侧设备掉电但收发器仍挂在线上，拖坏总线

5. SDK 失败路径不够健壮
   - `sdk_smoke_test` 在 CAN 启动失败后仍继续访问内部对象
   - 因此会出现 `segmentation fault`

---

## 已完成的工具与代码改动

### 1. 增强了 `dual_arm_control/tools/vci_diag.py`

已增加以下能力：

- 支持直接测试指定节点
- 支持扫描节点地址范围，如 `--scan 1-50`
- 支持监听原始 CAN 帧，如 `--listen 5`

用途：

- 验证 `VCI` 是否真的建立通信
- 验证总线上是否存在 CANopen SDO 响应
- 验证总线是否存在任何原始 CAN 帧活动

### 2. 修正了 `dual_arm_control` 里“假使能”的问题

已改成：

- 发送使能链路时等待 SDO 回包
- 读取 `StatusWord`
- 只有真正进入 `Operation Enabled` 才认定为使能成功

目的：

- 避免 UI 在驱动器无响应时仍显示“已使能”

---

## 当前可用启动方式

### HumanoidArms / Ti 电机

左臂：

```bash
cd /home/yang/DualArmRobot/HumanoidArms/HumanoidArms_SDK_单编/src
sudo env LD_LIBRARY_PATH=/home/yang/DualArmRobot/HumanoidArms/HumanoidArms_SDK_单编/usrlib ./sdk_smoke_test left 0 1
```

右臂：

```bash
cd /home/yang/DualArmRobot/HumanoidArms/HumanoidArms_SDK_单编/src
sudo env LD_LIBRARY_PATH=/home/yang/DualArmRobot/HumanoidArms/HumanoidArms_SDK_单编/usrlib ./sdk_smoke_test right 0 1
```

注意：

- 需要 `sudo`
- `LD_LIBRARY_PATH` 需要指向 `usrlib`
- 当前确认可用的是 `can_index=1`

### dual_arm_control

当前不建议与 `HumanoidArms/Ti` 共挂在同一条物理 CAN 总线上做联调。

若单独测试 `dual_arm_control`，应先确保：

- 总线只挂该套设备
- 节点地址、协议模式、波特率一致
- 再使用 `vci_diag.py` 验证总线是否真的有响应

---

## 当前已知风险

1. `HumanoidArms_SDK_单编/src/main.cpp` 中很多调用仍使用 `0,0`

这意味着：

- 主程序默认可能还在走 `can_index=0`
- 与当前 smoke test 验证通过的 `can_index=1` 不一致

2. `build_sdk_smoke_test.sh` 当前在本机无法重新编译

原因：

- 缺少系统头文件：
  - `Eigen`
  - `Boost`
  - `ncurses`

但仓库中已有现成可执行文件：

- `src/sdk_smoke_test`

所以目前可以先直接运行现成二进制，不必依赖重新编译。

3. `HumanoidArms` SDK 在 CAN 启动失败路径上存在崩溃风险

表现为：

- 前面已经报 `start device... fail`
- 后面仍继续执行
- 最终出现 `segmentation fault`

---

## 建议的后续处理

### 建议一：物理隔离两套系统

最稳的做法：

- `HumanoidArms/Ti` 单独占一条 CAN
- `dual_arm_control` 单独占另一条 CAN

这是当前最可靠、最省时间的方案。

### 建议二：如果必须共线，统一检查以下项目

- 波特率是否一致
- 协议是否一致
- 节点地址是否冲突
- 终端电阻是否正确
- 接线是否一致
- 掉电设备是否仍拖住总线

### 建议三：后续代码建议

- 将 `HumanoidArms_SDK_单编/src/main.cpp` 默认 CAN 口改成 `device_index=0, can_index=1`
- 给 `HumanoidArms` SDK 的失败路径补保护，避免初始化失败后继续读写导致崩溃
- 为两套系统各自补一份明确的启动说明，避免再次混线

---

## 一句话总结

本次问题的本质不是 `VCI` 坏了，也不是 `Ti` 电机本身不通，而是：

`HumanoidArms/Ti` 在 `device_index=0, can_index=1` 上可以正常工作，但只要把 `dual_arm_control` 那套电机并到同一条物理 CAN bus 上，就会引发总线冲突，导致 Ti 侧启动失败、发送失败，甚至崩溃。`
