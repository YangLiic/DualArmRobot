#!/usr/bin/env python3
"""
汇川伺服电机交互式控制程序 - C++ SDK版本

使用方法：
    1. 编译C++模块: cd ino_motor/cpp_version && python setup.py build_ext --inplace
    2. 运行: python ino_motor/interactive_control_cpp.py
    
按 Ctrl+C 退出
"""

import sys
import os
import time

# 添加C++模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'cpp_version'))

try:
    import inovance_servo
except ImportError as e:
    print("❌ 无法导入 inovance_servo 模块")
    print("   请先编译C++模块：")
    print("   cd ino_motor/cpp_version")
    print("   pip install pybind11")
    print("   python setup.py build_ext --inplace")
    print(f"\n错误详情: {e}")
    sys.exit(1)

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def main():
    """主程序"""
    print("=" * 60)
    print("    汇川伺服电机交互式控制程序 (C++ SDK)")
    print("=" * 60)
    print("\n提示：按 Ctrl+C 退出程序\n")
    
    servo = None
    
    try:
        while True:
            # ==================== 步骤1: 选择电机 ====================
            print("\n" + "=" * 60)
            print("【步骤 1】选择电机:")
            print("  [1] 400W 电机 (节点 ID: 0x601)")
            print("  [2] 750W 电机 (节点 ID: 0x602)")
            print("=" * 60)
            
            motor_choice = input("请输入选择 (1 或 2): ").strip()
            
            if motor_choice == '1':
                node_id = 0x601
                motor_name = "400W"
                invert_direction = False
            elif motor_choice == '2':
                node_id = 0x602
                motor_name = "750W"
                invert_direction = True  # 0x602 需要反转方向
            else:
                print("❌ 无效选择，请重新选择")
                continue
            
            print(f"\n✅ 已选择: {motor_name} 电机 (节点 ID: 0x{node_id:03X})")
            
            # 删除之前的伺服对象
            if servo:
                del servo
                time.sleep(0.5)
            
            # 创建新的伺服对象（自动启动接收线程）
            print(f"\n正在初始化电机...")
            servo = inovance_servo.InovanceServo(SERIAL_PORT, BAUD_RATE, node_id)
            
            # 启用静默模式（不打印CAN收发消息，避免干扰用户输入）
            servo.set_silent_mode(True)
            
            # 设置方向反转
            servo.set_direction_inverted(invert_direction)
            
            time.sleep(0.5)
            
            # 错误复位
            print("\n🔄 执行错误复位（预防性）...")
            servo.fault_reset()
            time.sleep(0.5)
            
            # ==================== 步骤2: 选择模式 ====================
            print("\n" + "=" * 60)
            print("【步骤 2】选择控制模式:")
            print("  [1] 位置模式")
            print("  [2] 速度模式")
            print("=" * 60)
            
            mode_choice = input("请输入选择 (1 或 2): ").strip()
            
            if mode_choice == '1':
                mode = inovance_servo.OperationMode.PROFILE_POSITION
                mode_name = "位置模式"
            elif mode_choice == '2':
                mode = inovance_servo.OperationMode.VELOCITY
                mode_name = "速度模式"
            else:
                print("❌ 无效选择，请重新选择")
                continue
            
            print(f"\n✅ 已选择: {mode_name}")
            
            # 使能电机
            print(f"\n正在使能电机...")
            servo.enable(mode)
            time.sleep(0.5)
            
            # ==================== 步骤3: 执行控制 ====================
            if mode == inovance_servo.OperationMode.PROFILE_POSITION:
                # 位置模式
                print("\n" + "=" * 60)
                print("【步骤 3】位置控制")
                print("=" * 60)
                
                # 输入角度
                angle_input = input("请输入目标角度 (例如: +45 或 -90): ").strip()
                try:
                    angle = float(angle_input)
                except ValueError:
                    print("❌ 无效的角度值")
                    servo.disable()
                    continue
                
                # 输入限速
                rpm_input = input("请输入运动速度限制 (RPM, 例如: 30): ").strip()
                try:
                    rpm_limit = int(rpm_input)
                except ValueError:
                    print("❌ 无效的速度值")
                    servo.disable()
                    continue
                
                # 设置速度限制
                servo.set_profile_velocity(rpm_limit)
                time.sleep(0.2)
                
                # 等待用户按Enter
                input("\n✅ 准备就绪，按 Enter 开始执行位置运动...")
                
                # 执行位置运动
                print(f"\n▶️  开始运动到 {angle}° (限速 {rpm_limit} RPM)...")
                servo.set_position(angle, False)  # False = 相对位置
                
                print("\n⏳ 正在运动中...")
                print("   提示：运动完成后按 Enter 继续")
                input()
                
            else:
                # 速度模式
                print("\n" + "=" * 60)
                print("【步骤 3】速度控制")
                print("=" * 60)
                
                # 输入速度
                speed_input = input("请输入目标速度 (RPM, 例如: +100 或 -100): ").strip()
                try:
                    speed = int(speed_input)
                except ValueError:
                    print("❌ 无效的速度值")
                    servo.disable()
                    continue
                
                # 等待用户按Enter
                input("\n✅ 准备就绪，按 Enter 开始执行速度运动...")
                
                # 执行速度控制
                print(f"\n▶️  设置速度为 {speed} RPM...")
                servo.set_velocity(speed)
                
                print(f"\n⏳ 电机正在以 {speed} RPM 旋转...")
                print("   提示：按 Enter 停止电机")
                input()
                
                # 停止
                print("\n⏸️  停止电机...")
                servo.set_velocity(0)
                time.sleep(0.5)
            
            # ==================== 步骤4: 失能 ====================
            print("\n⏸️  失能电机...")
            servo.disable()
            time.sleep(0.5)
            
            print("\n✅ 本次控制完成")
            print("\n" + "=" * 60)
            choice = input("按 Enter 继续下一次控制，或输入 'q' 退出: ").strip().lower()
            if choice == 'q':
                break
    
    except KeyboardInterrupt:
        print("\n\n🛑 收到 Ctrl+C，正在退出...")
        if servo and servo.is_enabled():
            try:
                servo.set_velocity(0)
                time.sleep(0.2)
                servo.disable()
            except:
                pass
    
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if servo:
            del servo
        print("\n👋 程序已退出")


if __name__ == "__main__":
    main()
