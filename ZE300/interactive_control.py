#!/usr/bin/env python3
"""
ZE300 电机交互式控制

使用方法：
    1. 编译模块:
       cd ZE300/cpp_version && python setup.py build_ext --inplace
    2. 运行:
       python ZE300/interactive_control.py
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "cpp_version"))

try:
    import ze300_motor
except ImportError as exc:
    print("❌ 无法导入 ze300_motor 模块")
    print("请先编译: cd ZE300/cpp_version && python setup.py build_ext --inplace")
    print(f"错误详情: {exc}")
    sys.exit(1)

USB_PORT = os.environ.get("ZE300_PORT", "/dev/ttyUSB0")
USB_BAUD = int(os.environ.get("ZE300_USB_BAUD", "921600"), 0)
CAN_BAUD_KBPS = int(os.environ.get("ZE300_CAN_BAUD_KBPS", "1000"), 0)
CAN_ID = int(os.environ.get("ZE300_CAN_ID", "0x01"), 0)
USE_HOST_ADDR = os.environ.get("ZE300_USE_HOST_ADDR", "1") not in ("0", "false", "False")

COUNT_PER_REV = 16384.0
DEG_PER_COUNT = 360.0 / COUNT_PER_REV

CMD_A3 = 0xA3
CMD_A4 = 0xA4
CMD_AE = 0xAE
CMD_AF = 0xAF
CMD_B1 = 0xB1
CMD_CE = 0xCE
CMD_F1 = 0xF1


def decode_fault_bits(code: int):
    mapping = [
        (0, "电压故障"),
        (1, "电流故障"),
        (2, "温度故障"),
        (3, "编码器故障"),
        (6, "硬件故障"),
        (7, "软件故障"),
    ]
    return [name for bit, name in mapping if code & (1 << bit)]


def deg_to_count(deg: float) -> int:
    return int(round(deg / DEG_PER_COUNT))


def count_to_deg(count: int) -> float:
    return count * DEG_PER_COUNT


def wait_cmd(motor, cmd, timeout_ms=300, retries=3, interval_s=0.02):
    for _ in range(max(1, retries)):
        ok, _ = motor.wait_command(cmd, timeout_ms)
        if ok:
            return True
        time.sleep(interval_s)
    return False


def read_brake_state_stable(motor, retries=4, interval_s=0.03):
    for _ in range(max(1, retries)):
        state = motor.read_brake_state()
        if state in (0x00, 0x01):
            return state
        time.sleep(interval_s)
    return 0xFF


def set_brake_state_verified(motor, closed: bool, retries=3):
    target = 0x01 if closed else 0x00
    for _ in range(max(1, retries)):
        motor.set_brake_closed(closed)
        time.sleep(0.05)
        state = read_brake_state_stable(motor, retries=3, interval_s=0.03)
        if state == target:
            return True, state
    return False, read_brake_state_stable(motor, retries=2, interval_s=0.03)


def print_menu():
    print("\n" + "=" * 64)
    print("ZE300 交互控制")
    print("=" * 64)
    print(f"串口: {USB_PORT}  USB波特率: {USB_BAUD}  CAN波特率: {CAN_BAUD_KBPS} kbps")
    print(f"CAN ID: 0x{CAN_ID:02X}  HostAddr模式: {'开' if USE_HOST_ADDR else '关'}")
    print("-" * 64)
    print("[1] 速度控制 (0xC1)")
    print("[2] 绝对位置控制 (0xC2)")
    print("[3] 相对位置控制 (0xC3)")
    print("[4] 设置当前位置为原点 (0xB1，断电不丢失)")
    print("[5] 最短路径回原点 (0xC4)")
    print("[6] 读取状态 (0xA4 + 0xAE + 0xA3)")
    print("[7] 清故障 (0xAF)")
    print("[8] 抱闸控制 (0xCE)")
    print("[9] MIT 运控指令")
    print("[f] 电机自由态/关闭输出 (0xCF)")
    print("[q] 退出")
    print("=" * 64)


def read_status(motor):
    motor.read_quick_status()
    wait_cmd(motor, CMD_A4)
    motor.read_status()
    wait_cmd(motor, CMD_AE)
    motor.read_position()
    wait_cmd(motor, CMD_A3)

    status = motor.get_status()
    print("\n--- 当前状态 ---")
    print(f"温度: {status.temperature_c:.1f} °C")
    print(f"Q轴电流: {status.q_current_a:.3f} A")
    print(f"转速: {status.speed_rpm:.2f} rpm")
    print(f"单圈位置: {status.single_turn_count} count ({status.single_turn_deg:.2f}°)")
    print(f"多圈位置: {status.multi_turn_count} count ({status.total_turn_deg:.2f}°)")
    print(f"母线电压: {status.bus_voltage_v:.2f} V")
    print(f"母线电流: {status.bus_current_a:.2f} A")
    print(f"运行模式: {status.run_mode}")
    print(f"故障码: 0x{status.fault_code:02X} ({'有故障' if status.fault else '无故障'})")


def speed_control(motor):
    value = input("目标速度 (rpm，可负数，直接回车=0停止): ").strip()
    if value == "":
        rpm = 0.0
    else:
        try:
            rpm = float(value)
        except ValueError:
            print("❌ 输入无效")
            return

    if abs(rpm) < 1e-6:
        motor.set_speed_rpm(0.0)
        wait_cmd(motor, 0xC1, 300, retries=2, interval_s=0.03)
        print("⏸️ 已发送 0 rpm 停止指令")
        return

    input("\n✅ 准备就绪，按 Enter 开始速度控制...")
    motor.set_speed_rpm(rpm)
    time.sleep(0.1)
    motor.read_speed()
    got = wait_cmd(motor, 0xC1, 300, retries=4, interval_s=0.03)
    st = motor.get_status()
    print(f"✅ 已发送速度: {rpm:.2f} rpm, 反馈: {st.speed_rpm:.2f} rpm")

    if not got:
        print("⚠️ 本次未等到 C1 应答，已发送命令但反馈可能延迟")

    if abs(rpm) > 0.5 and abs(st.speed_rpm) < 0.2:
        fault_names = decode_fault_bits(st.fault_code)
        if st.fault_code != 0:
            detail = "、".join(fault_names) if fault_names else "未知故障"
            print(f"⚠️ 当前故障码 0x{st.fault_code:02X}: {detail}")

        brake = read_brake_state_stable(motor)
        if brake == 0x01:
            print("⚠️ 抱闸处于闭合状态，建议先在[8]中断开抱闸")
        elif brake == 0x00:
            print("ℹ️ 抱闸已断开，若仍不转建议先[7]清故障再试")
        else:
            print("⚠️ 抱闸状态读取失败，建议在[8]里重试并观察回读结果")

    input("\n按 Enter 停止速度控制...")
    motor.set_speed_rpm(0.0)
    time.sleep(0.05)
    motor.read_speed()
    wait_cmd(motor, 0xC1, 300, retries=3, interval_s=0.03)
    st2 = motor.get_status()
    print(f"⏸️ 已停止，当前反馈速度: {st2.speed_rpm:.2f} rpm")


def abs_pos_control(motor):
    text = input("目标绝对位置 (支持 'count' 或 'deg'，例: 4096count / 90deg): ").strip().lower()
    try:
        if text.endswith("count"):
            count = int(float(text[:-5]))
        elif text.endswith("deg"):
            count = deg_to_count(float(text[:-3]))
        else:
            count = int(float(text))
    except ValueError:
        print("❌ 输入无效")
        return

    motor.set_absolute_position_count(count)
    print(f"✅ 已发送绝对位置: {count} count ({count_to_deg(count):.2f}°)")


def rel_pos_control(motor):
    text = input("相对位置增量 (支持 'count' 或 'deg'，例: 2048count / -45deg): ").strip().lower()
    try:
        if text.endswith("count"):
            count = int(float(text[:-5]))
        elif text.endswith("deg"):
            count = deg_to_count(float(text[:-3]))
        else:
            count = int(float(text))
    except ValueError:
        print("❌ 输入无效")
        return

    motor.set_relative_position_count(count)
    print(f"✅ 已发送相对位置: {count:+d} count ({count_to_deg(count):+.2f}°)")


def set_zero(motor):
    print("\nℹ️ 0xB1 会将单圈绝对值原点保存到驱动板，断电不丢失。")
    yes = input("确认将当前位置设为原点？(y/n): ").strip().lower()
    if yes != "y":
        print("已取消")
        return
    motor.set_zero()
    ok = wait_cmd(motor, CMD_B1, 500)
    print("✅ 原点设置命令已发送（断电不丢失）" + ("，已收到应答" if ok else "，未等到应答（可能执行较慢）"))


def go_origin(motor):
    motor.go_to_origin_shortest()
    print("✅ 已发送最短路径回原点命令")


def clear_fault(motor):
    code = motor.clear_fault()
    if code == 0xFF:
        print("⚠️ 清故障命令已发送，但未收到应答")
    else:
        print(f"✅ 清故障完成，当前故障码: 0x{code:02X}")


def brake_control(motor):
    print("[1] 抱闸闭合")
    print("[2] 抱闸断开")
    print("[3] 读取抱闸状态")
    choice = input("请选择: ").strip()
    if choice == "1":
        ok, state = set_brake_state_verified(motor, True)
        if ok:
            print("✅ 抱闸已闭合（已回读确认）")
        else:
            print(f"⚠️ 抱闸闭合回读未确认，当前状态码: 0x{state:02X}")
    elif choice == "2":
        ok, state = set_brake_state_verified(motor, False)
        if ok:
            print("✅ 抱闸已断开（已回读确认）")
        else:
            print(f"⚠️ 抱闸断开回读未确认，当前状态码: 0x{state:02X}")
    elif choice == "3":
        state = read_brake_state_stable(motor)
        if state == 0x01:
            print("✅ 抱闸状态: 闭合")
        elif state == 0x00:
            print("✅ 抱闸状态: 断开")
        else:
            print("⚠️ 读取失败或无效状态")
    else:
        print("❌ 无效选择")


def mit_control(motor):
    try:
        pos = float(input("目标位置 (rad): ").strip())
        vel = float(input("目标速度 (rad/s): ").strip())
        kp = float(input("Kp [0~500]: ").strip())
        kd = float(input("Kd [0~5]: ").strip())
        tq = float(input("目标力矩 (Nm): ").strip())
    except ValueError:
        print("❌ 输入无效")
        return

    motor.send_mit_control(pos, vel, kp, kd, tq)
    time.sleep(0.05)
    motor.read_mit_state()
    if wait_cmd(motor, CMD_F1, 300):
        st = motor.get_status()
        print(f"✅ MIT反馈: pos={st.mit_pos_rad:.3f} rad, vel={st.mit_vel_rad_s:.3f} rad/s, tq={st.mit_torque_nm:.3f} Nm")
    else:
        print("⚠️ MIT指令已发出，但未收到F1反馈")


def main():
    print("\n正在连接 ZE300 电机...")
    motor = None
    try:
        motor = ze300_motor.Ze300Motor(
            port=USB_PORT,
            usb_baud=USB_BAUD,
            dev_addr=CAN_ID,
            use_host_addr=USE_HOST_ADDR,
        )
        motor.set_silent_mode(True)
        print("✅ 连接成功")

        while True:
            print_menu()
            cmd = input("请选择: ").strip().lower()

            if cmd == "1":
                speed_control(motor)
            elif cmd == "2":
                abs_pos_control(motor)
            elif cmd == "3":
                rel_pos_control(motor)
            elif cmd == "4":
                set_zero(motor)
            elif cmd == "5":
                go_origin(motor)
            elif cmd == "6":
                read_status(motor)
            elif cmd == "7":
                clear_fault(motor)
            elif cmd == "8":
                brake_control(motor)
            elif cmd == "9":
                mit_control(motor)
            elif cmd == "f":
                motor.free_output()
                print("✅ 已发送自由态命令")
            elif cmd == "q":
                break
            else:
                print("❌ 无效选择")

    except KeyboardInterrupt:
        print("\n🛑 收到 Ctrl+C，退出中...")
    except Exception as exc:
        print(f"\n❌ 运行异常: {exc}")
        raise
    finally:
        if motor is not None:
            try:
                motor.free_output()
            except Exception:
                pass
        print("👋 程序已退出")


if __name__ == "__main__":
    main()
