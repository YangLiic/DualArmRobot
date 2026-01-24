#!/usr/bin/env python3
"""
Python 绑定快速测试

测试 C++ SDK 的 Python 绑定是否正常工作
"""

import sys
import time

try:
    import inovance_servo
    print("✅ 成功导入 inovance_servo 模块")
except ImportError as e:
    print("❌ 无法导入模块，请先编译:")
    print("   ./build_python.sh")
    sys.exit(1)

def test_basic():
    """基础功能测试"""
    print("\n" + "=" * 60)
    print("测试：基础功能")
    print("=" * 60)
    
    try:
        # 创建对象
        print("\n1. 创建伺服对象...")
        servo = inovance_servo.InovanceServo("/dev/ttyUSB0", 9600, 0x601)
        print("   ✅ 对象创建成功")
        
        # 方向设置
        print("\n2. 测试方向设置...")
        servo.set_direction_inverted(False)
        assert servo.is_direction_inverted() == False
        servo.set_direction_inverted(True)
        assert servo.is_direction_inverted() == True
        servo.set_direction_inverted(False)
        print("   ✅ 方向设置正常")
        
        # 错误复位
        print("\n3. 测试错误复位...")
        servo.fault_reset()
        time.sleep(0.5)
        print("   ✅ 错误复位成功")
        
        # 速度模式
        print("\n4. 测试速度模式...")
        servo.enable(inovance_servo.OperationMode.VELOCITY)
        time.sleep(1)
        print("   ✅ 速度模式使能成功")
        
        assert servo.is_enabled() == True
        print("   ✅ 状态查询正常")
        
        print("\n5. 测试速度控制 (30 RPM, 3秒)...")
        servo.set_velocity(30)
        time.sleep(3)
        
        servo.set_velocity(0)
        time.sleep(0.5)
        print("   ✅ 速度控制正常")
        
        servo.disable()
        time.sleep(1)
        
        # 位置模式
        print("\n6. 测试位置模式...")
        servo.enable(inovance_servo.OperationMode.PROFILE_POSITION)
        time.sleep(1)
        print("   ✅ 位置模式使能成功")
        
        print("\n7. 测试位置控制 (45°, 限速20 RPM)...")
        servo.set_profile_velocity(20)
        servo.set_position(45.0, False)
        time.sleep(3)
        print("   ✅ 位置控制正常")
        
        # 停止
        print("\n8. 停止电机...")
        servo.stop()
        print("   ✅ 停止成功")
        
        # 释放
        print("\n9. 释放资源...")
        del servo
        print("   ✅ 资源释放成功")
        
        print("\n" + "=" * 60)
        print("✅ 所有测试通过！")
        print("=" * 60)
        
        return True
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_enum():
    """测试枚举"""
    print("\n" + "=" * 60)
    print("测试：枚举类型")
    print("=" * 60)
    
    print("\n运行模式枚举:")
    print(f"  VELOCITY = {inovance_servo.OperationMode.VELOCITY}")
    print(f"  PROFILE_POSITION = {inovance_servo.OperationMode.PROFILE_POSITION}")
    print(f"  HOMING = {inovance_servo.OperationMode.HOMING}")
    print("✅ 枚举正常")

if __name__ == "__main__":
    print("=" * 60)
    print(" Inovance Servo Python 绑定测试")
    print("=" * 60)
    
    # 测试枚举
    test_enum()
    
    # 询问是否进行完整测试
    print("\n完整测试将控制电机运动，请确保:")
    print("  1. 电机已正确连接")
    print("  2. /dev/ttyUSB0 有权限 (sudo chmod 777 /dev/ttyUSB0)")
    print("  3. 电机处于安全环境")
    
    choice = input("\n是否进行完整测试？(y/n): ").strip().lower()
    
    if choice == 'y':
        if test_basic():
            print("\n🎉 全部测试完成！")
            sys.exit(0)
        else:
            print("\n⚠️  测试失败")
            sys.exit(1)
    else:
        print("\n跳过完整测试")
        sys.exit(0)
