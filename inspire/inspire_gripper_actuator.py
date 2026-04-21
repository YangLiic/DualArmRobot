#!/usr/bin/env python3
"""
inspire_gripper_actuator - 因时机器人电动夹爪执行器节点 (Modbus RTU)

基于 Modbus RTU 协议控制因时夹爪:
  - 功能码 0x03: 读取寄存器
  - 功能码 0x06: 写入单个寄存器
  - 功能码 0x10: 写入多个寄存器

关键寄存器:
  5  (0x05): CMD_CATCH_MOD     夹取模式, 0=一次夹取, 1=持续力控夹取
  10 (0x0A): CMD_OPENLEN_SET   开口度设置, 0~1000, 写入后立即动作
  11 (0x0B): CMD_SPEED_SET     速度设置, 10~1000
  12 (0x0C): CMD_FORCE_SET     力度设置, 100~1000 (单位: g)
  61 (0x3D): CMD_OPENLEN_ACT   实际开口度 (只读)
  65 (0x41): CMD_STATUS        状态码 (只读)

订阅:  gripper_cmd   (std_msgs/Float64)  目标位置 0.0~1.0
发布:  gripper_state (std_msgs/Float64)  当前位置 0.0~1.0

参数:
  port:       串口路径 (默认 /dev/ttyUSB0)
  baudrate:   波特率 (默认 115200)
  gripper_id: 夹爪 Modbus slave ID (默认 1)
  state_freq: 状态读取/发布频率 Hz (默认 10.0)
  speed:      运动速度 10~1000 (默认 500)
  force:      夹持力度 100~1000 (默认 500)
"""

import struct
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

# ==================== Modbus CRC16 ====================
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _build_modbus(slave_id: int, func: int, payload: bytes) -> bytes:
    frame = bytes([slave_id, func]) + payload
    crc = _crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


_SERIAL_TIMEOUT = 0.05  # 50ms


class InspireGripperActuatorNode(Node):

    def __init__(self):
        super().__init__('inspire_gripper_actuator')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('gripper_id', 1)
        self.declare_parameter('state_freq', 10.0)
        self.declare_parameter('speed', 500)
        self.declare_parameter('force', 500)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self._gid = self.get_parameter('gripper_id').value
        freq = self.get_parameter('state_freq').value
        self._speed = self.get_parameter('speed').value
        self._force = self.get_parameter('force').value

        try:
            self._ser = serial.Serial(
                port=port, baudrate=baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=_SERIAL_TIMEOUT)
        except Exception as e:
            self.get_logger().fatal(f'串口打开失败: {port} — {e}')
            raise

        self._position = 0.0
        self._fail_count = 0

        # ---- 初始化: 开启持续力控夹取 + 预设速度/力度 ----
        self._init_gripper()

        self._state_pub = self.create_publisher(
            Float64, 'gripper_state', qos_profile_sensor_data)
        self.create_subscription(
            Float64, 'gripper_cmd', self._cmd_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / freq, self._timer_cb)

        self.get_logger().info(
            f'InspireGripper (Modbus RTU) 启动 | port={port} '
            f'id={self._gid} freq={freq}Hz speed={self._speed} '
            f'force={self._force} catch_mod=持续力控')

    # ==================== 初始化 ====================

    def _init_gripper(self):
        """开机初始化: 持续力控模式 + 预设速度/力度"""
        time.sleep(0.05)  # 等待串口稳定

        # 1. 开启持续力控夹取 (寄存器 5 = 1)
        ok = self._write_register(5, 1)
        self.get_logger().info(
            f'  持续力控模式: {"✅ 已启用" if ok else "❌ 设置失败"}')

        # 2. 预设速度 (寄存器 11)
        ok = self._write_register(11, self._speed)
        self.get_logger().info(
            f'  速度={self._speed}: {"✅" if ok else "❌"}')

        # 3. 预设力度 (寄存器 12)
        ok = self._write_register(12, self._force)
        self.get_logger().info(
            f'  力度={self._force}: {"✅" if ok else "❌"}')

    # ==================== Modbus 通信 ====================

    def _transact(self, frame: bytes) -> bytes | None:
        """发送 Modbus 帧, 等待应答"""
        try:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()

            buf = bytearray()
            deadline = time.monotonic() + _SERIAL_TIMEOUT
            while time.monotonic() < deadline:
                n = self._ser.in_waiting
                if n > 0:
                    buf.extend(self._ser.read(n))
                    if len(buf) >= 5:
                        time.sleep(0.003)
                        buf.extend(self._ser.read(self._ser.in_waiting))
                        return bytes(buf)
                else:
                    time.sleep(0.0005)
        except Exception:
            pass
        return None

    def _read_register(self, reg: int) -> int | None:
        """Modbus 0x03: 读取单个寄存器"""
        payload = struct.pack('>HH', reg, 1)
        frame = _build_modbus(self._gid, 0x03, payload)
        reply = self._transact(frame)
        if reply is None or len(reply) < 7:
            return None
        if reply[1] & 0x80:
            return None
        return (reply[3] << 8) | reply[4]

    def _write_register(self, reg: int, value: int) -> bool:
        """Modbus 0x06: 写入单个寄存器"""
        payload = struct.pack('>HH', reg, value)
        frame = _build_modbus(self._gid, 0x06, payload)
        reply = self._transact(frame)
        if reply is None or len(reply) < 8:
            return False
        return not (reply[1] & 0x80)

    def _write_registers(self, start_reg: int, values: list[int]) -> bool:
        """Modbus 0x10: 写入多个寄存器"""
        count = len(values)
        payload = struct.pack('>HHB', start_reg, count, count * 2)
        for v in values:
            payload += struct.pack('>H', v)
        frame = _build_modbus(self._gid, 0x10, payload)
        reply = self._transact(frame)
        if reply is None or len(reply) < 8:
            return False
        return not (reply[1] & 0x80)

    # ==================== 回调 ====================

    def _timer_cb(self):
        """定时读取实际开口度 (寄存器 61) 并发布"""
        raw = self._read_register(61)
        if raw is not None:
            self._position = max(0.0, min(1.0, raw / 1000.0))
            self._fail_count = 0
        else:
            self._fail_count += 1
            if self._fail_count == 10:
                self.get_logger().error('夹爪 Modbus 通讯中断, 使用缓存值')
            if self._fail_count % 100 == 0:
                self.get_logger().warn(
                    f'夹爪通讯持续失败 (连续 {self._fail_count} 次)')

        msg = Float64()
        msg.data = self._position
        self._state_pub.publish(msg)

    def _cmd_cb(self, msg: Float64):
        """收到指令: 写入开口度 (寄存器 10), 速度/力度已在初始化时预设"""
        raw_pos = int(max(0.0, min(1.0, msg.data)) * 1000)
        ok = self._write_register(10, raw_pos)
        if not ok:
            self._fail_count += 1

    def destroy_node(self):
        try:
            self._ser.close()
        except Exception:
            pass
        self.get_logger().info('因时夹爪串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = InspireGripperActuatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'inspire_gripper_actuator 错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
