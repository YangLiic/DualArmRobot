import serial
import time
import struct

# ================= 配置区域 =================
# 端口号：根据你的 ls /dev/tty* 结果修改
SERIAL_PORT = '/dev/ttyUSB0' 
# 波特率：适配器与电脑通信的速率（注意不是CAN波特率）
# 大多数国产适配器内部是透传，这里填 115200 或 2000000 (2M) 都可以
BAUD_RATE = 9600 

class InovanceServo:
    def __init__(self, port, baud, node_id=0x601):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.node_id = node_id
        if self.ser.is_open:
            print(f"✅ 串口 {port} 已打开, 节点 ID: 0x{node_id:03X}")

    def close(self):
        self.ser.close()
        print("串口已关闭")

    def send_can_packet(self, can_id, data_bytes):
        """
        将 CAN ID 和数据打包成 AA...7A 格式发送
        :param can_id: 整数, 例如 0x601
        :param data_bytes: 列表或字节串, 例如 [0x2F, 0x60...]
        """
        # 1. 帧头 (AA)
        frame = [0xAA]
        
        # 2. 类型 (00 00) - 固定
        frame.extend([0x00, 0x00])
        
        # 3. 数据长度 (DLC)
        dlc = len(data_bytes)
        frame.append(dlc)
        
        # 4. 扩展位 (00 00) - 固定
        frame.extend([0x00, 0x00])
        
        # 5. CAN ID (2字节, 高位在前? 根据你Windows的日志 06 01 代表 601)
        # 0x601 -> 0x06 0x01
        id_high = (can_id >> 8) & 0xFF
        id_low = can_id & 0xFF
        frame.extend([id_high, id_low])
        
        # 6. 数据 (Data)
        frame.extend(data_bytes)
        
        # 7. 补齐填充 (如果不足8字节，某些适配器可能需要补0，根据你成功的NMT指令，需要补齐)
        # 你的适配器似乎是定长17字节包 (1(AA)+2+1+2+2+8+1(7A) = 17)
        padding_len = 8 - dlc
        if padding_len > 0:
            frame.extend([0x00] * padding_len)
            
        # 8. 帧尾 (7A)
        frame.append(0x7A)
        
        # 发送
        byte_data = bytearray(frame)
        self.ser.write(byte_data)
        
        # 打印调试信息
        hex_str = ' '.join([f'{b:02X}' for b in byte_data])
        print(f"发送 [ID:0x{can_id:03X}]: {hex_str}")
        
        # 简单的接收回显 (非阻塞)
        time.sleep(0.02) # 给一点点时间处理
        while self.ser.in_waiting:
            resp = self.ser.read(self.ser.in_waiting)
            print(f"   └── 收到: {resp.hex().upper()}")

    def enable_motor(self):
        print(f"\n--- 开始使能流程 (节点 0x{self.node_id:03X}) ---")
        # 1. 设置速度模式 (0x6060 = 3)
        self.send_can_packet(self.node_id, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
        
        # 2. 状态机跳转: Shutdown (0x06)
        self.send_can_packet(self.node_id, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        time.sleep(0.1)
        
        # 3. 状态机跳转: Switch On (0x07)
        self.send_can_packet(self.node_id, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
        time.sleep(0.1)
        
        # 4. 状态机跳转: Enable Operation (0x0F) - 此时锁轴
        self.send_can_packet(self.node_id, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        print("✅ 电机应该已锁轴")
        
        # 5. NMT 启动 (0x01) - 激活 PDO
        self.send_can_packet(0x000, [0x01, 0x00])
        print("✅ NMT 已启动，PDO 应该开始刷屏")

    def set_velocity(self, rpm):
        print(f"\n--- 设置速度: {rpm} RPM (节点 0x{self.node_id:03X}) ---")
        # 换算 RPM 到 编码器单位 (23位编码器)
        # 60 RPM = 0x00800000 (8388608)
        encoder_value = (rpm * 0x00800000) // 60
        
        speed_data = [
            0x23, 0xFF, 0x60, 0x00,
            (encoder_value >> 0) & 0xFF,
            (encoder_value >> 8) & 0xFF,
            (encoder_value >> 16) & 0xFF,
            (encoder_value >> 24) & 0xFF
        ]
        self.send_can_packet(self.node_id, speed_data)

    def stop_motor(self):
        print(f"\n--- 停止电机 (节点 0x{self.node_id:03X}) ---")
        # 速度设为 0
        self.send_can_packet(self.node_id, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        # 去使能 (Shutdown)
        self.send_can_packet(self.node_id, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        # NMT 预操作 (停止刷屏)
        self.send_can_packet(0x000, [0x80, 0x00])

# ================= 主程序 =================
if __name__ == "__main__":
    NODE_ID = 0x601 # 400W电机
    
    try:
        servo = InovanceServo(SERIAL_PORT, BAUD_RATE, node_id=NODE_ID)
        
        # 1. 启动
        servo.enable_motor()
        time.sleep(2)
        
        # 2. 旋转
        servo.set_velocity(600) 
        print(">>> 电机正在旋转... (保持 5 秒)")
        time.sleep(5)
        
        # 3. 停止
        servo.stop_motor()
        
        servo.close()
        print("\n✅ 演示结束")
        
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        print("提示：请检查 USB 权限 (sudo chmod 777 /dev/ttyUSB0)")

"""
source .venv/bin/activate
python ino_motor/py_version/servo_driver_400.py
"""