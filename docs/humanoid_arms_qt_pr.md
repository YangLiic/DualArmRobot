# PR: 在现有 GUI 页面中新增 `HumanoidArms_SDK_单编` 机械臂控制板块

## 背景

当前 `dual_arm_control` 主要服务汇川电机控制。现在需要在现有 GUI 主页面内新增一个机械臂控制板块，专门给 `HumanoidArms/HumanoidArms_SDK_单编` 使用，用于双臂机械臂的日常调试和基础操作。

## 需求

- 在现有 GUI 主页面中新增一个单独板块，专门给 `HumanoidArms_SDK_单编` 使用。
- 机械臂是双臂结构，左臂、右臂分开展示。
- 每只手臂 7 个关节分别提供滑块，用于关节位置控制。
- 滑块修改的是单关节目标值，但下发时按 SDK 接口要求组装为整臂 7 关节目标，通过 `moveJ_ToJoint(...)` 发送。
- 页面中同时显示左右臂当前关节角、末端位姿、错误状态。
- 保持与现有 Qt 架构一致：UI 层 -> `ArmService` -> `HumanoidArmsSdkAdapter`，不要让 UI 直接调用 SDK。

## GUI 设计

- 在现有主窗口中新增 `HumanoidArmsPanel` 分区
- 页面结构建议：
  - 板块顶部：SDK 连接/断开、初始化状态、日志提示
  - 板块左侧：左臂 7 个关节滑块 + 当前值显示
  - 板块右侧：右臂 7 个关节滑块 + 当前值显示
  - 板块底部：当前末端位姿、错误状态、刹车/清错按钮
- 建议滑块对人机显示使用“角度 deg”，内部统一换算为 SDK 需要的“rad”
- 建议使用“拖动仅更新显示，松开后发送”或短延时合并发送，避免滑块连续拖动时频繁下发 CAN 命令

## SDK 已确认能力

首版直接相关：

- `can_init()` / `Exit()`
- `moveJ_ToJoint`
- `get_joint`
- `get_Pos`
- `get_mechanicalarm_status`
- `clear_elc_error`
- `brake`

SDK 里还能看到的其他能力，可作为后续扩展：

- `moveJ_ToPos`：关节空间到笛卡尔目标位姿
- `moveL_ToPos`：线性运动到目标位姿
- `robot_fk` / `robot_ik`：正逆运动学
- `get_motor_position` / `set_motor_position`：底层电机位置读写
- `set_current_mode`：电流模式
- `dragMod`：拖拽模式
- `Check_CAN_interface` / `detectArmOnSocket`：CAN 接口识别
- `mechanical_arm_origin_socketcan`：回零相关接口
- `view_robot_config_information` / `ip_address`：配置和环境信息查看

## 本次范围

- 先做同页独立分区的双臂关节控制板块
- 首版聚焦“关节位置滑块控制 + 状态读取 + 清错/刹车”
- 暂不纳入拖拽示教、笛卡尔控制、电流模式、底层电机直写

## TODO

- [ ] 新增 `ArmService`，统一封装双臂连接、状态查询、关节控制、刹车、清错
- [ ] 新增 `HumanoidArmsSdkAdapter`，封装 `HumanoidArms_SDK_单编` 的初始化、退出和接口调用
- [ ] 在现有 GUI 主页面中新增机械臂控制板块，不新开独立页面
- [ ] 为左臂、右臂分别实现 7 个关节滑块和数值显示
- [ ] 维护左右臂各自的目标关节数组，单滑块变化时按整臂 7 关节目标发送
- [ ] 增加当前关节角刷新和末端位姿刷新
- [ ] 增加错误状态显示、清错按钮、刹车按钮
- [ ] 明确每个关节滑块的上下限，避免超出机械臂真实关节范围
- [ ] 处理 `sudo` 启动、`LD_LIBRARY_PATH`、SDK 生命周期和设备独占问题

## 验收标准

- Qt 现有主页面中有单独的 `HumanoidArms` 控制板块
- 左右臂各 7 个关节都可通过滑块完成位置控制
- 页面可读取并显示左右臂当前关节角和末端位姿
- 页面可执行错误查询、清错、刹车
- 新增机械臂模块后，不影响现有 `dual_arm_control` 电机功能

## 风险

- SDK 是“库内自管 CAN”，和当前统一通信调度层不是同一路子，首版建议先独立封装，不强行并入原有 `CommunicationManager`
- 滑块直接控制时，命令下发频率容易过高，需要节流
- 关节真实运动范围目前还需要结合机械臂参数进一步确认，不能先写死为通用范围
