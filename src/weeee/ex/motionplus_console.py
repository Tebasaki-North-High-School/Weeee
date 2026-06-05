"""
MotionPlus Console: displays yaw, roll, pitch until Ctrl+C.
Use -s <serial> to target a specific Wiimote.
Exits with diagnostic info if multiple Wiimotes are detected.
"""

import sys
import hid
import argparse

from weeee.core import Wiimote, VENDOR_ID, PRODUCT_ID, PRODUCT_ID_TR


def main() -> None:
    parser = argparse.ArgumentParser(description="MotionPlus Console")
    parser.add_argument("-s", type=str, default=None, help="Serial number of target Wiimote")
    args = parser.parse_args()
    serial: str | None = args.s

    devices = hid.enumerate(VENDOR_ID)
    matches = [d for d in devices if d["product_id"] in (PRODUCT_ID, PRODUCT_ID_TR)]

    if not matches:
        print("No Wiimote found")
        sys.exit(1)

    if serial is not None:
        targets = [d for d in matches if d.get("serial_number") == serial]
        if not targets:
            serials = [d.get("serial_number", "?") for d in matches]
            print(f"Wiimote with serial '{serial}' not found. Available serials: {', '.join(serials)}")
            sys.exit(1)
        target_info = targets[0]
    elif len(matches) > 1:
        print(f"Multiple Wiimotes detected ({len(matches)}):")
        for d in matches:
            sn = d.get("serial_number", "?")
            pid = d["product_id"]
            model = "RVL-CNT-01-TR" if pid == PRODUCT_ID_TR else "RVL-CNT-01"
            print(f"  Serial: {sn}  Product: {model} (0x{pid:04X})")
        print("\nUse -s <serial> to pick one.")
        sys.exit(1)
    else:
        target_info = matches[0]

    sn = target_info.get("serial_number", "?")
    pid = target_info["product_id"]
    model = "RVL-CNT-01-TR" if pid == PRODUCT_ID_TR else "RVL-CNT-01"
    print(f"Connecting to {model} (serial: {sn})...")

    # Use path bytes — hidapi crashes with serial_number on some platforms
    wm = Wiimote(target=target_info["path"], require_motion_plus=True, enable_fusion=True)
    wm.set_reporting_mode(0x35, continuous=True)

    print("yaw_deg,roll_deg,pitch_deg")
    try:
        while True:
            wm.update()
            print(f"{wm.yaw_deg:.3f},{wm.roll_deg:.3f},{wm.pitch_deg:.3f}")
    except KeyboardInterrupt:
        pass
    finally:
        wm.close()


if __name__ == "__main__":
    main()
