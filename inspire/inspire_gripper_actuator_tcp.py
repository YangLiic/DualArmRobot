#!/usr/bin/env python3
"""
inspire_gripper_actuator_tcp - 因时(Inspire)夹爪执行器节点 (被控模式, Modbus TCP)

接收控制指令驱动夹爪，同时发布状态反馈。
用于推理/回放场景：机器人末端的 Inspire 夹爪接收指令执行动作。

订阅 (相对名，由 namespace 决定完整路径):
  gripper_cmd    (std_msgs/Float64)  — 目标位置 0.0~1.0

发布 (相对名):
  gripper_state  (std_msgs/Float64MultiArray)  — 当前 6 个手指 angle 值

参数:
  ip:           Modbus TCP IP地址（默认 192.168.123.211）
  port:         Modbus TCP 端口（默认 6000）
  state_freq:   状态发布频率 Hz（默认 10.0）
  speed:        运动速度 10~1000（默认 500）
  force:        夹持力度 100~1000 g（默认 1000）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Float64MultiArray
from pymodbus.client import ModbusTcpClient

# ==================== 寄存器定义 (来自 demo_modbus.py) ====================
regdict = {
    'ID': 1000,
    'baudrate': 1001,
    'clearErr': 1004,
    'forceClb': 1009,
    'angleSet': 1486,
    'forceSet': 1498,
    'speedSet': 1522,
    'angleAct': 1546,
    'forceAct': 1582,
    'Action': 1600,
    'errCode': 1606,
    'statusCode': 1612,
    'temp': 1618,
    'actionSeq': 2320,
    'actionRun': 2322,
}

# ==================== 手指姿态端点 (来自 test_inspire.py) ====================
# 0.0 -> 大拇指握紧(0)，其余张开(1000)
# 1.0 -> 全部握紧(0)
POSE_AT_0 = [1000, 1000, 1000, 1000, 1000, 0]
POSE_AT_1 = [0, 0, 0, 0, 0, 0]


class InspireGripperActuatorTcpNode(Node):
    """因时夹爪执行器节点 — 接收指令 + 发布状态"""

    def __init__(self):
        super().__init__('inspire_gripper_actuator_tcp')

        # ==================== 参数 ====================
        self.declare_parameter('ip', '192.168.123.211')
        self.declare_parameter('port', 6000)
        self.declare_parameter('state_freq', 10.0)
        self.declare_parameter('speed', 500)
        self.declare_parameter('force', 500)

        ip = self.get_parameter('ip').value
        port = self.get_parameter('port').value
        state_freq = self.get_parameter('state_freq').value
        self.speed = self.get_parameter('speed').value
        self.force = self.get_parameter('force').value

        # ==================== Modbus TCP ====================
        self.client = ModbusTcpClient(host=ip, port=port)
        if not self.client.connect():
            self.get_logger().fatal(f'Modbus TCP 连接失败: {ip}:{port}')
            raise RuntimeError('Modbus TCP connect failed')

        self.get_logger().info(f'Modbus TCP 已连接: {ip}:{port}')

        # 初始化速度和力度
        self.write6('speedSet', [self.speed] * 6)
        self.write6('forceSet', [self.force] * 6)

        # ==================== ROS2 接口 ====================
        self._state_pub = self.create_publisher(
            Float64MultiArray, 'gripper_state', qos_profile_sensor_data)

        self.create_subscription(
            Float64, 'gripper_cmd',
            self._cmd_callback, qos_profile_sensor_data)

        self.create_timer(1.0 / state_freq, self._publish_state)

        # ==================== 日志 ====================
        ns = self.get_namespace()
        self.get_logger().info(
            f'InspireGripperActuator 启动 | {ip}:{port} '
            f'cmd={ns}/gripper_cmd state={ns}/gripper_state '
            f'freq={state_freq}Hz speed={self.speed} force={self.force}')

    # ==================== Modbus 操作 ====================

    def write_register(self, address, values):
        self.client.write_registers(address, values)

    def read_register(self, address, count):
        response = self.client.read_holding_registers(address, count=count)
        return response.registers if response.isError() is False else []

    # ==================== ROS2 回调 ====================

    @staticmethod
    def interpolate_pose(position, pose_start, pose_end):
        """按 0.0~1.0 在 pose_start 和 pose_end 之间线性插值。"""
        position = max(0.0, min(1.0, position))
        target = []
        for start, end in zip(pose_start, pose_end):
            value = round(start + (end - start) * position)
            target.append(max(0, min(1000, value)))
        return target

    def _cmd_callback(self, msg: Float64):
        """接收夹爪指令 0.0~1.0，插值映射到灵巧手 6 个手指。"""
        target_pose = self.interpolate_pose(msg.data, POSE_AT_0, POSE_AT_1)
        self.write6('angleSet', target_pose)

    def _publish_state(self):
        """直接发布夹爪当前 6 个手指 angle 值。"""
        angles = self.read6('angleAct')
        if angles and len(angles) == 6:
            msg = Float64MultiArray()
            msg.data = [float(a) for a in angles]
            self._state_pub.publish(msg)

    # ==================== Modbus 底层操作 ====================

    def write6(self, reg_name, val):
        if reg_name in ['angleSet', 'forceSet', 'speedSet']:
            val_reg = []
            for i in range(6):
                val_reg.append(val[i] & 0xFFFF)
            self.write_register(regdict[reg_name], val_reg)
        else:
            self.get_logger().error(
                'write6 调用错误: reg_name 必须是 angleSet/forceSet/speedSet')

    def read6(self, reg_name):
        if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct', 'Action']:
            val = self.read_register(regdict[reg_name], 6)
            return val
        elif reg_name in ['errCode', 'statusCode', 'temp']:
            val_act = self.read_register(regdict[reg_name], 3)
            results = []
            for v in val_act:
                results.append(v & 0xFF)
                results.append((v >> 8) & 0xFF)
            return results
        else:
            self.get_logger().error('read6 调用错误')
            return []

    def destroy_node(self):
        try:
            self.client.close()
            self.get_logger().info('Modbus TCP 已关闭')
        except Exception:
            pass
        super().destroy_node()


# ==================== main ====================

def main(args=None):
    rclpy.init(args=args)
    try:
        node = InspireGripperActuatorTcpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'inspire_gripper_actuator_tcp 错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
