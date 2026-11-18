"""Check the dongle's VR mode without a VR runtime: print each controller's orientation and inputs.

Needs pyusb and a libusb backend (`pip install pyusb libusb-package`). Linux: a udev rule for
1209:5151 (or run as root). Windows: the VR interfaces bind to WinUSB on their own.

    python3 host/vrtest.py              # live view of both controllers
    python3 host/vrtest.py --buzz right # short buzz on one controller
    python3 host/vrtest.py --recenter   # current heading of both controllers = yaw 0

Protocol: docs/vr.md.
"""
import argparse
import math
import struct
import sys
import time

VID, PID = 0x1209, 0x5151
SUBCLASS = 0x51
SIDES = {0x01: "left", 0x02: "right"}
FLAGS = ((0x01, "conn"), (0x04, "ori"), (0x08, "gcal"), (0x10, "standby"), (0x20, "deadbat"))
BUTTONS = ("A/X", "B/Y", "stick", "sys/menu")


def parse(r):
    """64-byte state report -> dict (see docs/vr.md)."""
    if len(r) < 64 or r[0] != 0x01:
        return None
    q = struct.unpack_from("<4f", r, 12)
    return dict(flags=r[2], seq=r[3], imu_ts=struct.unpack_from("<I", r, 4)[0],
                dongle_ts=struct.unpack_from("<I", r, 8)[0], quat=q,
                gyro=struct.unpack_from("<3f", r, 28), accel=struct.unpack_from("<3f", r, 40),
                stick=struct.unpack_from("<2h", r, 52), trigger=struct.unpack_from("<H", r, 56)[0],
                grip=struct.unpack_from("<H", r, 58)[0], buttons=r[60], touch=r[61],
                battery=None if r[62] == 255 else r[62])


def euler(q):
    """OpenXR quaternion (x, y, z, w) -> yaw, pitch, roll in degrees (Y-X-Z order: yaw about +Y, + = turned
    left; pitch about +X, + = tipped up; roll about +Z, + = rolled left)."""
    x, y, z, w = q
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * x - y * z))))
    yaw = math.atan2(2 * (x * z + w * y), 1 - 2 * (x * x + y * y))
    roll = math.atan2(2 * (x * y + w * z), 1 - 2 * (x * x + z * z))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def line(side, s):
    if s is None:
        return f"{side:>5}: -"
    if not s["flags"] & 0x01:
        return f"{side:>5}: not connected"
    yaw, pitch, roll = euler(s["quat"])
    btn = "+".join(n for i, n in enumerate(BUTTONS) if s["buttons"] & 1 << i) or "-"
    flags = ",".join(n for b, n in FLAGS if s["flags"] & b)
    bat = "?" if s["battery"] is None else f"{s['battery']}%"
    return (f"{side:>5}: yaw {yaw:7.1f} pitch {pitch:6.1f} roll {roll:7.1f}  stick {s['stick'][0]:6d},{s['stick'][1]:6d}"
            f"  trig {s['trigger']:4d} grip {s['grip']:4d}  btn {btn:<10} touch {s['touch']:02x}  bat {bat:>4}  [{flags}]")


def open_dongle():
    try:
        import usb.core
        import usb.util
    except ImportError:
        sys.exit("needs pyusb: pip install pyusb libusb-package")
    backend = None
    try:
        import libusb_package

        backend = libusb_package.get_libusb1_backend()
    except ImportError:
        pass
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        sys.exit("dongle not found (is it in VR mode?)")
    def endpoint(intf, direction):
        return usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == direction)

    cfg = dev.get_active_configuration()
    ifaces = {}
    for intf in cfg:
        if intf.bInterfaceClass == 0xFF and intf.bInterfaceSubClass == SUBCLASS and intf.bInterfaceProtocol in SIDES:
            ep_in = endpoint(intf, usb.util.ENDPOINT_IN)
            ep_out = endpoint(intf, usb.util.ENDPOINT_OUT)
            try:
                if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                    dev.detach_kernel_driver(intf.bInterfaceNumber)
            except (NotImplementedError, usb.core.USBError):
                pass
            usb.util.claim_interface(dev, intf.bInterfaceNumber)
            ifaces[SIDES[intf.bInterfaceProtocol]] = (ep_in, ep_out)
    if not ifaces:
        sys.exit("dongle found, but not in VR mode (switch in the control panel)")
    return dev, ifaces


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--buzz", choices=("left", "right"), help="buzz one controller for 200 ms")
    ap.add_argument("--recenter", action="store_true", help="make both controllers' current heading yaw 0")
    args = ap.parse_args()
    dev, ifaces = open_dongle()
    import usb.core  # after open_dongle(), which explains how to install pyusb if it is missing

    if args.buzz:
        ifaces[args.buzz][1].write(struct.pack("<BBH", 0x10, 200, 200))
        return
    if args.recenter:
        for _, ep_out in ifaces.values():
            ep_out.write(b"\x11")
        return
    state = {side: None for side in ifaces}
    rate = {side: 0 for side in ifaces}
    counts = {side: 0 for side in ifaces}  # new reports (sequence number changed) since the last print
    last_print = 0.0
    while True:
        for side, (ep_in, _) in ifaces.items():
            try:
                s = parse(bytes(ep_in.read(64, timeout=5)))
            except usb.core.USBTimeoutError:
                continue
            if s:
                if state[side] is None or state[side]["seq"] != s["seq"]:
                    counts[side] += 1
                state[side] = s
        now = time.monotonic()
        if now - last_print >= 0.1:
            dt = now - last_print if last_print else 1
            for side in ifaces:
                rate[side], counts[side] = counts[side] / dt, 0
            last_print = now
            out = "  |  ".join(f"{line(side, state[side])}  {rate[side]:4.0f} Hz" for side in sorted(ifaces))
            if len(ifaces) == 1:
                print("\r" + out, end="", flush=True)
            else:
                print(out)


if __name__ == "__main__":
    main()
