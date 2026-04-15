#!/usr/bin/env python3

import argparse
import ctypes
import os
import pathlib
import collections
import sys
import time


VCI_USBCAN2 = 4

BITRATE_TIMINGS = {
    20: (0x31, 0x1C),
    50: (0x09, 0x1C),
    100: (0x04, 0x1C),
    125: (0x03, 0x1C),
    250: (0x01, 0x1C),
    500: (0x00, 0x1C),
    1000: (0x00, 0x14),
}


class VCI_BOARD_INFO(ctypes.Structure):
    _fields_ = [
        ("hw_Version", ctypes.c_ushort),
        ("fw_Version", ctypes.c_ushort),
        ("dr_Version", ctypes.c_ushort),
        ("in_Version", ctypes.c_ushort),
        ("irq_Num", ctypes.c_ushort),
        ("can_Num", ctypes.c_ubyte),
        ("str_Serial_Num", ctypes.c_char * 20),
        ("str_hw_Type", ctypes.c_char * 40),
        ("Reserved", ctypes.c_ushort * 4),
    ]


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint),
        ("AccMask", ctypes.c_uint),
        ("Reserved", ctypes.c_uint),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


def cstr(raw):
    return bytes(raw).split(b"\x00", 1)[0].decode(errors="ignore")


def load_library():
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    lib_path = repo_root / "lib" / "libcontrolcan.so"
    if not lib_path.exists():
        raise FileNotFoundError(f"missing library: {lib_path}")

    lib = ctypes.CDLL(str(lib_path))
    lib.VCI_FindUsbDevice2.argtypes = [ctypes.POINTER(VCI_BOARD_INFO)]
    lib.VCI_FindUsbDevice2.restype = ctypes.c_uint
    lib.VCI_OpenDevice.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    lib.VCI_OpenDevice.restype = ctypes.c_uint
    lib.VCI_CloseDevice.argtypes = [ctypes.c_uint, ctypes.c_uint]
    lib.VCI_CloseDevice.restype = ctypes.c_uint
    lib.VCI_ReadBoardInfo.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_BOARD_INFO)]
    lib.VCI_ReadBoardInfo.restype = ctypes.c_uint
    lib.VCI_InitCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_INIT_CONFIG)]
    lib.VCI_InitCAN.restype = ctypes.c_uint
    lib.VCI_StartCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    lib.VCI_StartCAN.restype = ctypes.c_uint
    lib.VCI_ResetCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    lib.VCI_ResetCAN.restype = ctypes.c_uint
    lib.VCI_ClearBuffer.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    lib.VCI_ClearBuffer.restype = ctypes.c_uint
    lib.VCI_GetReceiveNum.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
    lib.VCI_GetReceiveNum.restype = ctypes.c_uint
    lib.VCI_Transmit.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_CAN_OBJ), ctypes.c_uint]
    lib.VCI_Transmit.restype = ctypes.c_uint
    lib.VCI_Receive.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(VCI_CAN_OBJ), ctypes.c_uint, ctypes.c_int]
    lib.VCI_Receive.restype = ctypes.c_uint
    return lib


def enumerate_devices(lib):
    infos = (VCI_BOARD_INFO * 50)()
    count = lib.VCI_FindUsbDevice2(infos)
    print(f"VCI_FindUsbDevice2 count: {count}")
    for idx in range(min(count, 50)):
        info = infos[idx]
        print(
            f"  index={idx} can_num={info.can_Num} serial={cstr(info.str_Serial_Num)!r} hw={cstr(info.str_hw_Type)!r}"
        )
    return count


def open_and_start(lib, device_type, device_index, channel, bitrate):
    open_ret = lib.VCI_OpenDevice(device_type, device_index, 0)
    print(f"VCI_OpenDevice(type={device_type}, index={device_index}) -> {open_ret}")
    if open_ret != 1:
        return False

    timing0, timing1 = BITRATE_TIMINGS[bitrate]
    config = VCI_INIT_CONFIG()
    config.AccCode = 0x00000000
    config.AccMask = 0xFFFFFFFF
    config.Filter = 1
    config.Mode = 0
    config.Timing0 = timing0
    config.Timing1 = timing1

    init_ret = lib.VCI_InitCAN(device_type, device_index, channel, ctypes.byref(config))
    print(f"VCI_InitCAN(channel={channel}, bitrate={bitrate}kbps) -> {init_ret}")
    if init_ret != 1:
        lib.VCI_CloseDevice(device_type, device_index)
        return False

    start_ret = lib.VCI_StartCAN(device_type, device_index, channel)
    print(f"VCI_StartCAN(channel={channel}) -> {start_ret}")
    if start_ret != 1:
        lib.VCI_CloseDevice(device_type, device_index)
        return False

    info = VCI_BOARD_INFO()
    read_ret = lib.VCI_ReadBoardInfo(device_type, device_index, ctypes.byref(info))
    print(
        "VCI_ReadBoardInfo -> "
        f"{read_ret}, can_num={info.can_Num}, serial={cstr(info.str_Serial_Num)!r}, hw={cstr(info.str_hw_Type)!r}"
    )
    return True


def close_device(lib, device_type, device_index, channel):
    lib.VCI_ResetCAN(device_type, device_index, channel)
    lib.VCI_CloseDevice(device_type, device_index)


def response_id(node_id):
    return node_id - 0x80 if node_id >= 0x80 else node_id


def sdo_read(lib, device_type, device_index, channel, node_id, index, subindex, timeout_ms=300):
    lib.VCI_ClearBuffer(device_type, device_index, channel)

    frame = VCI_CAN_OBJ()
    frame.ID = node_id
    frame.SendType = 0
    frame.RemoteFlag = 0
    frame.ExternFlag = 0
    frame.DataLen = 8
    frame.Data[0] = 0x40
    frame.Data[1] = index & 0xFF
    frame.Data[2] = (index >> 8) & 0xFF
    frame.Data[3] = subindex

    tx_ret = lib.VCI_Transmit(device_type, device_index, channel, ctypes.byref(frame), 1)
    if tx_ret != 1:
        return None, f"VCI_Transmit failed: {tx_ret}"

    deadline = time.time() + (timeout_ms / 1000.0)
    expected_id = response_id(node_id)

    while time.time() < deadline:
        rx_count = lib.VCI_GetReceiveNum(device_type, device_index, channel)
        if rx_count:
            rx_count = min(rx_count, 256)
            frames = (VCI_CAN_OBJ * rx_count)()
            got = lib.VCI_Receive(device_type, device_index, channel, frames, rx_count, 0)
            for i in range(got):
                obj = frames[i]
                data = [obj.Data[j] for j in range(obj.DataLen)]
                if obj.ID != expected_id or obj.DataLen < 4:
                    continue
                if data[1] != (index & 0xFF) or data[2] != ((index >> 8) & 0xFF) or data[3] != subindex:
                    continue
                if data[0] == 0x80:
                    abort_code = 0
                    if obj.DataLen >= 8:
                        abort_code = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)
                    return None, f"SDO abort 0x{abort_code:08X}"
                return data, None
        time.sleep(0.01)

    return None, f"timeout waiting for 0x{expected_id:03X}"


def parse_u16(data):
    return data[4] | (data[5] << 8)


def parse_i16(data):
    value = parse_u16(data)
    return value - 0x10000 if value & 0x8000 else value


def parse_node_id(raw):
    return int(raw, 0)


def iter_node_scan_values(spec):
    if "-" in spec:
        start_text, end_text = spec.split("-", 1)
        start = int(start_text, 0)
        end = int(end_text, 0)
        step = 1 if end >= start else -1
        return list(range(start, end + step, step))
    return [int(spec, 0)]


def normalize_node_request(raw):
    value = int(raw, 0)
    if 1 <= value <= 127:
        return value, 0x600 + value, f"addr {value} (req 0x{0x600 + value:03X})"
    return value, value, f"req 0x{value:03X}"


def receive_frames(lib, device_type, device_index, channel):
    rx_count = lib.VCI_GetReceiveNum(device_type, device_index, channel)
    if not rx_count:
        return []

    rx_count = min(rx_count, 256)
    frames = (VCI_CAN_OBJ * rx_count)()
    got = lib.VCI_Receive(device_type, device_index, channel, frames, rx_count, 0)
    result = []
    for i in range(got):
        obj = frames[i]
        data = [obj.Data[j] for j in range(obj.DataLen)]
        result.append((obj.ID, data))
    return result


def listen_for_frames(lib, device_type, device_index, channel, seconds, max_frames):
    deadline = time.time() + seconds
    frames = []
    while time.time() < deadline and len(frames) < max_frames:
        batch = receive_frames(lib, device_type, device_index, channel)
        if batch:
            frames.extend(batch)
            continue
        time.sleep(0.01)

    counter = collections.Counter(frame_id for frame_id, _ in frames)
    return frames[:max_frames], counter


def main():
    parser = argparse.ArgumentParser(description="VCI CANalyst-II diagnostic helper")
    parser.add_argument("--device-type", type=int, default=VCI_USBCAN2)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--channel", type=int, default=0)
    parser.add_argument("--bitrate", type=int, choices=sorted(BITRATE_TIMINGS), default=1000)
    parser.add_argument("--nodes", nargs="*", default=[])
    parser.add_argument(
        "--scan",
        nargs="*",
        default=[],
        help="scan node addresses or raw request IDs, e.g. 1-32 or 0x601-0x610",
    )
    parser.add_argument(
        "--listen",
        type=float,
        default=0.0,
        help="passively listen for raw CAN frames for N seconds after active probes",
    )
    parser.add_argument(
        "--listen-max-frames",
        type=int,
        default=80,
        help="maximum number of raw CAN frames to print in listen mode",
    )
    args = parser.parse_args()

    if os.geteuid() != 0:
        print("warning: current user is not root; CANalyst-II often needs sudo unless udev permissions are fixed")

    try:
        lib = load_library()
    except Exception as exc:
        print(f"failed to load libcontrolcan.so: {exc}")
        return 2

    enumerate_devices(lib)

    if not open_and_start(lib, args.device_type, args.device_index, args.channel, args.bitrate):
        print("open/start failed; VCI communication is NOT established from this process")
        return 1

    print("adapter open/start OK")
    print(f"testing nodes on device_index={args.device_index}, channel={args.channel}, bitrate={args.bitrate}kbps")

    exit_code = 0
    try:
        requested_node_specs = args.nodes if args.nodes else []
        if not requested_node_specs and not args.scan and args.listen <= 0:
            requested_node_specs = ["0x601", "0x602"]

        requested_nodes = []
        for raw_node in requested_node_specs:
            node_addr, request_id, label = normalize_node_request(raw_node)
            requested_nodes.append((node_addr, request_id, label))

        for node_addr, request_id, label in requested_nodes:
            print(f"\nnode {label}")

            status_data, status_err = sdo_read(
                lib, args.device_type, args.device_index, args.channel, request_id, 0x6041, 0x00
            )
            if status_err:
                print(f"  statusword read failed: {status_err}")
                exit_code = 1
                continue

            status_word = parse_u16(status_data)
            print(f"  statusword: 0x{status_word:04X}")

            torque_data, torque_err = sdo_read(
                lib, args.device_type, args.device_index, args.channel, request_id, 0x6077, 0x00
            )
            if torque_err:
                print(f"  torque read failed: {torque_err}")
                exit_code = 1
                continue

            torque = parse_i16(torque_data)
            print(f"  torque: {torque} permille")
            print("  communication confirmed")

        if args.scan:
            print("\nscan mode")
            found = []
            for spec in args.scan:
                for value in iter_node_scan_values(spec):
                    node_addr, request_id, label = normalize_node_request(str(value))
                    status_data, status_err = sdo_read(
                        lib, args.device_type, args.device_index, args.channel, request_id, 0x6041, 0x00, timeout_ms=80
                    )
                    if status_err:
                        continue
                    status_word = parse_u16(status_data)
                    found.append((node_addr, request_id, status_word))
                    print(f"  found {label}, statusword=0x{status_word:04X}")
            if not found:
                print("  no responding nodes found in requested scan range")

        if args.listen > 0:
            print(f"\nlisten mode ({args.listen:.1f}s)")
            frames, counter = listen_for_frames(
                lib,
                args.device_type,
                args.device_index,
                args.channel,
                args.listen,
                args.listen_max_frames,
            )
            if not frames:
                print("  no CAN frames observed on the bus")
            else:
                print(f"  captured {len(frames)} frame(s)")
                print("  most frequent CAN IDs:")
                for frame_id, count in counter.most_common(12):
                    print(f"    0x{frame_id:03X}: {count}")
                print("  sample frames:")
                for frame_id, data in frames[:min(len(frames), 20)]:
                    payload = " ".join(f"{byte:02X}" for byte in data)
                    print(f"    0x{frame_id:03X}  [{len(data)}]  {payload}")
    finally:
        close_device(lib, args.device_type, args.device_index, args.channel)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
