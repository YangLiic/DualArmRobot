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
import threading
from glob import glob

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

# 默认串口，可通过环境变量 INO_MOTOR_PORT 覆盖
# 否则自动探测 ttyUSB/ttyACM
def detect_serial_port():
    env_port = os.environ.get('INO_MOTOR_PORT')
    if env_port:
        return env_port

    candidates = sorted(glob('/dev/ttyUSB*')) + sorted(glob('/dev/ttyACM*'))
    return candidates[0] if candidates else '/dev/ttyUSB0'


SERIAL_PORT = detect_serial_port()
BAUD_RATE = 9600


# ==================== 实时扭矩监控器 ====================
class TorqueMonitor:
    """
    后台线程持续读取 6077h (Actual Torque) 并实时显示。
    可选：超过阈值自动失能电机。
    """
    def __init__(self, servo):
        self.servo = servo
        self._running = False
        self._thread = None
        self._display_enabled = False
        self._auto_disable_enabled = False
        self._auto_disable_threshold = 1200   # 千分比
        self._auto_disable_samples = 3       # 连续采样次数
        self._poll_interval = 0.1            # 100ms
        self._lock = threading.Lock()
        self._last_torque = 0

    def start_display(self, interval=0.1):
        """开始实时显示扭矩"""
        self._display_enabled = True
        self._poll_interval = interval
        self._ensure_thread()

    def stop_display(self):
        """停止实时显示扭矩"""
        self._display_enabled = False

    def enable_auto_disable(self, threshold_permille, consecutive_samples=3):
        """
        启用扭矩阈值自动失能。
        当实际扭矩绝对值连续 consecutive_samples 次 >= threshold_permille 时，
        自动调用 servo.disable() 失能电机。
        """
        self._auto_disable_threshold = threshold_permille
        self._auto_disable_samples = max(1, consecutive_samples)
        self._auto_disable_enabled = True
        self._ensure_thread()
        print(f"⚡ 扭矩阈值自动失能已启用: 阈值={threshold_permille}‰, 连续采样={consecutive_samples}")

    def disable_auto_disable(self):
        """禁用扭矩阈值自动失能"""
        self._auto_disable_enabled = False

    def get_last_torque(self):
        """获取最近一次读取的扭矩值"""
        return self._last_torque

    def stop(self):
        """停止监控线程"""
        self._running = False
        self._display_enabled = False
        self._auto_disable_enabled = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None

    def _ensure_thread(self):
        if self._thread and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()

    def _monitor_loop(self):
        consecutive_hits = 0
        while self._running:
            if not (self._display_enabled or self._auto_disable_enabled):
                time.sleep(0.1)
                continue

            try:
                torque = self.servo.read_actual_torque_permille()
                self._last_torque = torque
            except Exception:
                time.sleep(0.2)
                continue

            # 实时显示
            if self._display_enabled:
                abs_t = abs(torque)
                bar_len = min(abs_t // 20, 50)  # 每20‰一格, 最多50格
                bar = '█' * bar_len
                direction = '+' if torque >= 0 else '-'
                # \r 回到行首，覆盖更新
                print(f"\r  📊 扭矩(6077h): {direction}{abs_t:4d}‰ ({abs_t/10:.1f}%额定) |{bar:<50s}|", end='', flush=True)

            # 自动失能检查
            if self._auto_disable_enabled:
                if abs(torque) >= self._auto_disable_threshold:
                    consecutive_hits += 1
                else:
                    consecutive_hits = 0

                if consecutive_hits >= self._auto_disable_samples:
                    print(f"\n\n🛑 扭矩阈值自动失能触发! 实际扭矩={torque}‰ >= 阈值{self._auto_disable_threshold}‰")
                    self._auto_disable_enabled = False
                    self._display_enabled = False
                    try:
                        self.servo.set_velocity(0)
                        time.sleep(0.1)
                        self.servo.disable()
                        print("✅ 电机已自动失能")
                    except Exception as e:
                        print(f"❌ 自动失能异常: {e}")
                    consecutive_hits = 0
                    break

            time.sleep(self._poll_interval)


def enable_collision_protection(servo, mode, node_id):
    """自动启用碰撞保护（无交互提示）。"""
    # 400W(0x601): 500%额定=5000‰, 750W(0x602): 200%额定=2000‰
    if node_id == 0x601:
        limit = 5000   # 400W: 500% 额定转矩
        trigger = 5000
    else:
        limit = 2000   # 750W: 200% 额定转矩
        trigger = 2000
    deviation = 0 if mode == inovance_servo.OperationMode.VELOCITY else 100000

    if servo.enable_collision_protection(
        torque_limit_permille=limit,
        trigger_torque_permille=trigger,
        trigger_current_amp=0.0,
        trigger_position_deviation=deviation,
        consecutive_samples=3,
        poll_interval_ms=20,
        use_quick_stop=True,
    ):
        print(f"🛡️  碰撞保护已启用 (转矩限制={limit}‰={limit/10:.0f}%额定, 触发={trigger}‰={trigger/10:.0f}%额定)")
    else:
        print("⚠️  碰撞保护启用失败")


def enable_torque_auto_disable(torque_monitor, node_id):
    """自动启用扭矩阈值失能（无交互提示）。"""
    # 400W(0x601): 500%额定=5000‰, 750W(0x602): 200%额定=2000‰
    if node_id == 0x601:
        threshold = 5000  # 400W: 500% 额定转矩
    else:
        threshold = 2000  # 750W: 200% 额定转矩

    torque_monitor.enable_auto_disable(threshold, consecutive_samples=3)

def main():
    """主程序"""
    print("=" * 60)
    print("    汇川伺服电机交互式控制程序 (C++ SDK)")
    print("=" * 60)
    print("\n提示：按 Ctrl+C 退出程序\n")
    
    servo = None
    torque_monitor = None
    
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
            if torque_monitor:
                torque_monitor.stop()
                torque_monitor = None
            if servo:
                del servo
                time.sleep(0.5)
            
            # 创建新的伺服对象（自动启动接收线程）
            print(f"\n正在初始化电机...")
            print(f"使用串口: {SERIAL_PORT}")
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
            enable_collision_protection(servo, mode, node_id)
            time.sleep(0.2)
            
            # 创建扭矩监控器
            torque_monitor = TorqueMonitor(servo)
            enable_torque_auto_disable(torque_monitor, node_id)
            time.sleep(0.2)
            
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
                
                # 开始实时扭矩显示
                torque_monitor.start_display(interval=0.1)
                
                # 执行位置运动
                print(f"\n▶️  开始运动到 {angle}° (限速 {rpm_limit} RPM)...")
                servo.set_position(angle, False)  # False = 相对位置
                
                print("\n⏳ 正在运动中... (实时扭矩显示中)")
                print("   提示：运动完成后按 Enter 继续")
                input()
                
                # 停止扭矩显示
                torque_monitor.stop_display()
                print()  # 换行，避免覆盖扭矩显示行
                
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
                
                # 开始实时扭矩显示
                torque_monitor.start_display(interval=0.1)
                
                # 执行速度控制
                print(f"\n▶️  设置速度为 {speed} RPM...")
                servo.set_velocity(speed)
                
                print(f"\n⏳ 电机正在以 {speed} RPM 旋转... (实时扭矩显示中)")
                print("   提示：按 Enter 停止电机")
                input()
                
                # 停止扭矩显示
                torque_monitor.stop_display()
                print()  # 换行
                
                # 停止
                print("\n⏸️  停止电机...")
                if servo.was_collision_triggered():
                    print("⚠️  本次运动已触发碰撞保护，跳过继续给速度命令")
                else:
                    servo.set_velocity(0)
                time.sleep(0.5)
            
            # ==================== 步骤4: 失能 ====================
            print("\n⏸️  失能电机...")
            if torque_monitor:
                torque_monitor.stop()
            servo.disable_collision_protection()
            servo.disable()
            time.sleep(0.5)
            
            print("\n✅ 本次控制完成")
            print("\n" + "=" * 60)
            choice = input("按 Enter 继续下一次控制，或输入 'q' 退出: ").strip().lower()
            if choice == 'q':
                break
    
    except KeyboardInterrupt:
        print("\n\n🛑 收到 Ctrl+C，正在退出...")
        if torque_monitor:
            torque_monitor.stop()
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
        if torque_monitor:
            torque_monitor.stop()
        if servo:
            del servo
        print("\n👋 程序已退出")


if __name__ == "__main__":
    main()
