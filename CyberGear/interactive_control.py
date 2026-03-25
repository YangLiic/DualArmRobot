#!/usr/bin/env python3
"""
CyberGear 微电机交互式控制程序

使用方法：
    1. 编译C++模块: cd CyberGear/cpp_version && python setup.py build_ext --inplace
    2. 运行: python CyberGear/interactive_control.py

按 Ctrl+C 退出
"""

import sys
import os
import time
import math

# 添加C++模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'cpp_version'))

try:
    import cybergear_motor
except ImportError as e:
    print("❌ 无法导入 cybergear_motor 模块")
    print("   请先编译C++模块：")
    print("   cd CyberGear/cpp_version")
    print("   python setup.py build_ext --inplace")
    print(f"\n错误详情: {e}")
    sys.exit(1)

# 默认串口，可通过环境变量覆盖
SERIAL_PORT = os.environ.get('CYBERGEAR_PORT', '/dev/ttyUSB0')
BAUD_RATE = 921600
MOTOR_ID  = int(os.environ.get('CYBERGEAR_MOTOR_ID',  '0x02'), 0)
MASTER_ID = int(os.environ.get('CYBERGEAR_MASTER_ID', '0xFD'), 0)

# 行程限制 (°)，上电后生效
TRAVEL_LIMIT_DEG = 180.0


def rad_to_deg(rad):
    return rad * 180.0 / math.pi

def deg_to_rad(deg):
    return deg * math.pi / 180.0


# ==================== 零位状态管理 ====================

class ZeroManager:
    """
    管理上电后零位设置状态。
    CyberGear 机械零位掉电丢失，因此每次上电必须重新设置零位
    才能解锁运动控制功能。
    """
    def __init__(self):
        self.zero_set = False

    def require_zero(self) -> bool:
        """检查零位是否已设置，未设置则提示用户。"""
        if not self.zero_set:
            print("\n⛔ 运动被拒绝！")
            print("   CyberGear 机械零位掉电丢失，上电后必须重新设置零位。")
            print("   请先选择 [3] 设置零位，再进行运动控制。")
            return False
        return True

    def mark_set(self):
        self.zero_set = True


ZERO_MGR = ZeroManager()


# ==================== 菜单 ====================

def print_menu():
    zero_status = "✅ 已设置" if ZERO_MGR.zero_set else "❌ 未设置 (运动被锁定)"
    print("\n" + "=" * 60)
    print("    CyberGear 微电机控制程序")
    print("=" * 60)
    print(f"  串口: {SERIAL_PORT}  |  电机 ID: 0x{MOTOR_ID:02X}")
    print(f"  行程限制: ±{TRAVEL_LIMIT_DEG}°")
    print(f"  零位状态: {zero_status}")
    print("-" * 60)
    print("  [1] 速度模式")
    print("  [2] 位置模式")
    print("  [3] 设置零位  ← 上电后必须先执行")
    print("  [4] 回到零位")
    print("  [5] 读取状态")
    print("  [q] 退出")
    print("=" * 60)


# ==================== 控制功能 ====================

def speed_mode(motor):
    """速度模式控制"""
    if not ZERO_MGR.require_zero():
        return

    print("\n--- 速度模式 ---")
    print(f"  行程保护: ±{TRAVEL_LIMIT_DEG}° (实时轮询检测)")
    motor.enable_speed_mode()
    time.sleep(0.5)

    speed_input = input("请输入目标速度 (rad/s, 范围 ±5, 或 RPM 如 '30rpm'): ").strip()

    try:
        if speed_input.lower().endswith('rpm'):
            rpm = float(speed_input[:-3])
            speed = rpm * 2 * math.pi / 60.0
            print(f"  = {speed:.2f} rad/s")
        else:
            speed = float(speed_input)
    except ValueError:
        print("❌ 无效输入")
        motor.stop()
        return

    input("\n✅ 准备就绪，按 Enter 开始...")
    print(f"\n▶️  设置速度: {speed:.2f} rad/s ({speed * 60 / (2*math.pi):.1f} RPM)")
    
    # 先设置速度，再启动轮询，避免轮询先发止速导致运动被立即阻止
    motor.set_speed(speed)
    motor.start_poll(50)

    print("   按 Enter 停止...")
    input()

    print("⏸️  停止...")
    motor.set_speed(0)
    time.sleep(0.3)
    motor.stop_poll()
    motor.stop()
    time.sleep(0.3)


def position_mode(motor):
    """位置模式控制"""
    if not ZERO_MGR.require_zero():
        return

    print("\n--- 位置模式 ---")
    print(f"  行程保护: ±{TRAVEL_LIMIT_DEG}°")
    motor.enable_position_mode()
    time.sleep(0.5)

    spd_input = input("请输入运动速度限制 (rad/s, 默认 2.0): ").strip()
    try:
        spd_limit = float(spd_input) if spd_input else 2.0
    except ValueError:
        spd_limit = 2.0
    motor.set_speed_limit(spd_limit)

    pos_input = input(f"请输入目标位置 (角度°, 范围 ±{TRAVEL_LIMIT_DEG}°, 或 rad 如 '1.5rad'): ").strip()
    try:
        if pos_input.lower().endswith('rad'):
            pos_rad = float(pos_input[:-3])
        else:
            pos_rad = deg_to_rad(float(pos_input))
    except ValueError:
        print("❌ 无效输入")
        motor.stop()
        return

    input(f"\n✅ 准备运动到 {rad_to_deg(pos_rad):.1f}° ({pos_rad:.3f} rad)，按 Enter 开始...")
    print(f"\n▶️  运动到目标位置 (限速 {spd_limit:.1f} rad/s)...")
    motor.set_position(pos_rad)

    print("   运动中... 按 Enter 结束")
    input()
    motor.stop()
    time.sleep(0.3)


def set_zero(motor):
    """设置零位 (上电期间有效，掉电丢失)"""
    print("\n--- 设置零位 ---")
    print("⚠️  注意：")
    print("   - 将当前位置设为基准零点 (上电期间有效，掉电后需重新设置)")
    print("   - 行程限制 ±" + str(TRAVEL_LIMIT_DEG) + "° 相对于此零点生效")
    
    confirm = input("\n确认设置当前位置为零位? (y/n): ").strip().lower()
    if confirm == 'y':
        motor.set_zero()
        ZERO_MGR.mark_set()
        print("✅ 零位设置成功！现在可以进行速度/位置控制。")
    else:
        print("已取消")


def go_to_zero(motor):
    """回到零位"""
    if not ZERO_MGR.require_zero():
        return

    print("\n--- 回到零位 ---")
    spd_input = input("请输入回零速度限制 (rad/s, 默认 2.0): ").strip()
    try:
        spd_limit = float(spd_input) if spd_input else 2.0
    except ValueError:
        spd_limit = 2.0

    input("按 Enter 开始回零...")
    motor.go_to_zero(spd_limit)

    print("   回零中... 按 Enter 结束")
    input()
    motor.stop()
    time.sleep(0.3)


def read_status(motor):
    """读取状态 (通过反馈帧获取最新编码器值)"""
    print("\n--- 电机状态 ---")
    # 主动触发反馈帧获取最新数据
    motor.request_feedback()
    time.sleep(0.1)

    status = motor.get_status()
    limit_str = " ⚠️ 触限!" if status.at_limit else ""
    print(f"  位置: {status.position:.4f} rad ({rad_to_deg(status.position):.2f}°){limit_str}")
    print(f"  速度: {status.velocity:.4f} rad/s ({status.velocity * 60 / (2*math.pi):.2f} RPM)")
    print(f"  力矩: {status.torque:.4f} Nm")
    print(f"  温度: {status.temperature:.1f}")
    print(f"  故障: {'是' if status.has_fault else '无'}")
    print(f"  零位: {'已设置' if ZERO_MGR.zero_set else '❌ 未设置'}")

# ==================== 主程序 ====================

def main():
    print_menu()

    motor = None

    try:
        print(f"\n正在连接电机...")
        motor = cybergear_motor.CyberGearMotor(SERIAL_PORT, BAUD_RATE, MOTOR_ID, MASTER_ID)
        motor.set_silent_mode(True)
        motor.set_travel_limit_deg(-TRAVEL_LIMIT_DEG, TRAVEL_LIMIT_DEG)
        time.sleep(0.5)

        print("✅ 连接成功")
        print(f"\n⚠️  重要：请先选择 [3] 设置零位，再进行运动控制！")

        while True:
            print_menu()
            choice = input("请选择: ").strip().lower()

            if choice == '1':
                speed_mode(motor)
            elif choice == '2':
                position_mode(motor)
            elif choice == '3':
                set_zero(motor)
            elif choice == '4':
                go_to_zero(motor)
            elif choice == '5':
                read_status(motor)
            elif choice == 'q':
                break
            else:
                print("❌ 无效选择")

    except KeyboardInterrupt:
        print("\n\n🛑 收到 Ctrl+C，正在退出...")
        if motor and motor.is_enabled():
            try:
                motor.set_speed(0)
                time.sleep(0.2)
                motor.stop()
            except:
                pass

    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if motor:
            try:
                motor.stop()
            except:
                pass
            del motor
        print("\n👋 程序已退出")


if __name__ == "__main__":
    main()
