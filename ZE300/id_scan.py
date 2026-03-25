#!/usr/bin/env python3
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "cpp_version"))

import ze300_motor


def probe_one(port: str, usb_baud: int, dev_addr: int, timeout_ms: int, retries: int):
    for use_host_addr in (True, False):
        try:
            motor = ze300_motor.Ze300Motor(
                port=port,
                usb_baud=usb_baud,
                dev_addr=dev_addr,
                use_host_addr=use_host_addr,
            )
            motor.set_silent_mode(True)

            got_ae = False
            got_a4 = False

            for _ in range(retries):
                motor.read_status()
                ok, _ = motor.wait_command(0xAE, timeout_ms)
                if ok:
                    got_ae = True
                    break
                time.sleep(0.01)

            for _ in range(retries):
                motor.read_quick_status()
                ok, _ = motor.wait_command(0xA4, timeout_ms)
                if ok:
                    got_a4 = True
                    break
                time.sleep(0.01)

            if got_ae or got_a4:
                return {
                    "dev_addr": dev_addr,
                    "use_host_addr": use_host_addr,
                    "ae": got_ae,
                    "a4": got_a4,
                }
        except Exception:
            pass

    return None


def main():
    parser = argparse.ArgumentParser(description="ZE300 CAN ID scanner")
    parser.add_argument("--port", default=os.environ.get("ZE300_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--usb-baud", type=int, default=int(os.environ.get("ZE300_USB_BAUD", "921600"), 0))
    parser.add_argument("--start", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--end", type=lambda x: int(x, 0), default=0x20)
    parser.add_argument("--timeout-ms", type=int, default=250)
    parser.add_argument("--retries", type=int, default=2)
    args = parser.parse_args()

    start = max(1, min(args.start, 0xFF))
    end = max(1, min(args.end, 0xFF))
    if end < start:
        start, end = end, start

    print("=== ZE300 CAN ID Scan ===")
    print(f"port={args.port}, usb_baud={args.usb_baud}, range=0x{start:02X}..0x{end:02X}")

    hits = []
    total = end - start + 1

    for idx, addr in enumerate(range(start, end + 1), start=1):
        result = probe_one(args.port, args.usb_baud, addr, args.timeout_ms, args.retries)
        if result:
            hits.append(result)
            mode = "0x100|ID" if result["use_host_addr"] else "ID"
            print(f"[HIT] addr=0x{addr:02X}, mode={mode}, AE={result['ae']}, A4={result['a4']}")
        elif idx % 8 == 0 or idx == total:
            print(f"progress: {idx}/{total}")

    print("--- Scan Result ---")
    if not hits:
        print("NO_RESPONSE")
        return 1

    for item in hits:
        mode = "0x100|ID" if item["use_host_addr"] else "ID"
        print(f"addr=0x{item['dev_addr']:02X}, mode={mode}, AE={item['ae']}, A4={item['a4']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
