#!/usr/bin/env python3
"""
抱闸控制工具

用法:
    python brake_control.py release [1|2]   # 松闸 (1=0x601, 2=0x602)
    python brake_control.py lock [1|2]      # 锁闸
    python brake_control.py                 # 交互模式
"""

import sys
import os
import time

# 添加 cpp_version 到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'cpp_version'))

try:
    import inovance_servo
except ImportError:
    print("❌ 未找到 inovance_servo 模块")
    print("请先编译: cd ino_motor/cpp_version && python setup.py build_ext --inplace")
    sys.exit(1)

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600

def get_node_id(motor_num):
    """获取节点 ID"""
    if motor_num == 1:
        return 0x601
    elif motor_num == 2:
        return 0x602
    else:
        raise ValueError(f"无效的电机编号: {motor_num}")

def release_brake(motor_num):
    """松闸"""
    node_id = get_node_id(motor_num)
    print(f"\n🔓 松闸: 电机 {motor_num} (节点 0x{node_id:03X})")
    print("⚠️  警告: 电机将失去保持力!")
    
    servo = inovance_servo.InovanceServo(SERIAL_PORT, BAUD_RATE, node_id)
    servo.set_silent_mode(True)
    time.sleep(0.3)
    
    result = servo.release_brake()
    time.sleep(0.3)
    del servo
    
    if result:
        print("✅ 松闸成功，电机可手动转动")
    else:
        print("❌ 松闸失败")
    return result

def lock_brake(motor_num):
    """锁闸"""
    node_id = get_node_id(motor_num)
    print(f"\n🔒 锁闸: 电机 {motor_num} (节点 0x{node_id:03X})")
    
    servo = inovance_servo.InovanceServo(SERIAL_PORT, BAUD_RATE, node_id)
    servo.set_silent_mode(True)
    time.sleep(0.3)
    
    result = servo.lock_brake()
    time.sleep(0.3)
    del servo
    
    if result:
        print("✅ 锁闸成功")
    else:
        print("❌ 锁闸失败")
    return result

def interactive_mode():
    """交互模式"""
    print("=" * 50)
    print("    抱闸控制工具")
    print("=" * 50)
    print("\n命令:")
    print("  r1 / release1  - 松闸电机1 (0x601)")
    print("  r2 / release2  - 松闸电机2 (0x602)")
    print("  l1 / lock1     - 锁闸电机1")
    print("  l2 / lock2     - 锁闸电机2")
    print("  q / quit       - 退出")
    print()
    
    while True:
        try:
            cmd = input("请输入命令: ").strip().lower()
            
            if cmd in ['q', 'quit', 'exit']:
                print("👋 再见")
                break
            elif cmd in ['r1', 'release1']:
                release_brake(1)
            elif cmd in ['r2', 'release2']:
                release_brake(2)
            elif cmd in ['l1', 'lock1']:
                lock_brake(1)
            elif cmd in ['l2', 'lock2']:
                lock_brake(2)
            else:
                print("❓ 未知命令，输入 q 退出")
                
        except KeyboardInterrupt:
            print("\n👋 再见")
            break

def main():
    if len(sys.argv) == 1:
        # 无参数，进入交互模式
        interactive_mode()
    elif len(sys.argv) >= 2:
        action = sys.argv[1].lower()
        motor = int(sys.argv[2]) if len(sys.argv) >= 3 else 1
        
        if action in ['release', 'r']:
            release_brake(motor)
        elif action in ['lock', 'l']:
            lock_brake(motor)
        else:
            print(__doc__)
            sys.exit(1)

if __name__ == "__main__":
    main()
