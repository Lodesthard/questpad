"""Check the dongle's VR mode without a VR runtime: a window with each controller's orientation in 3D, its
inputs and the numbers that tell whether the gyro works (rates, drift at rest, report rate and losses).

Needs pyusb and a libusb backend (`pip install pyusb libusb-package`). Linux: a udev rule for
1209:5151 (or run as root). Windows: the VR interfaces bind to WinUSB on their own.

    python3 host/vrtest.py              # window (dongle in VR mode)
    python3 host/vrtest.py --text       # same numbers in the terminal
    python3 host/vrtest.py --sim        # read the simulator instead (cd sim && cargo run --release -- live --auto)
    python3 host/vrtest.py --buzz right # short buzz on one controller
    python3 host/vrtest.py --recenter   # current heading of both controllers = yaw 0

Checking the gyro: set the controllers down for a second (gyro calibrated), hold one pointing ahead and press
Recentre: yaw, pitch and roll read about 0. Turning left raises yaw, tipping the front up raises pitch, rolling
left raises roll. Press Mark, turn a quarter turn and back: "since mark" reads about +90 then about 0. Lying
still, "drift" should stay well under 1 °/min. Protocol: docs/vr.md.
"""
import argparse
import math
import socket
import struct
import sys
import threading
import time

VID, PID = 0x1209, 0x5151
SUBCLASS = 0x51
SIDES = {0x01: "left", 0x02: "right"}
FLAGS = ((0x01, "conn"), (0x04, "ori"), (0x08, "gcal"), (0x10, "standby"), (0x20, "deadbat"))
BUTTONS = ("A/X", "B/Y", "stick", "sys/menu")
SIM_VIEW, SIM_CONTROL = ("127.0.0.1", 26770), ("127.0.0.1", 26771)  # sim/src/live.rs

CMD_RECENTER, CMD_RESET = b"\x11", b"\x12"


def cmd_haptic(amp=200, ms=200):
    return struct.pack("<BBH", 0x10, amp, ms)


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


def qrot(q, v):
    """Rotate vector `v` by quaternion `q` (x, y, z, w)."""
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ---- where the reports come from ----

class DongleError(Exception):
    pass


class UsbSource:
    """The dongle's two VR interfaces over pyusb."""

    def __init__(self):
        try:
            import usb.core
            import usb.util
        except ImportError:
            raise DongleError("needs pyusb: pip install pyusb libusb-package")
        self.usb = usb
        backend = None
        try:
            import libusb_package

            backend = libusb_package.get_libusb1_backend()
        except ImportError:
            pass
        dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
        if dev is None:
            raise DongleError("dongle not found (plugged in, in VR mode?)")

        def endpoint(intf, direction):
            return usb.util.find_descriptor(
                intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == direction)

        self.ifaces = {}
        for intf in dev.get_active_configuration():
            if intf.bInterfaceClass == 0xFF and intf.bInterfaceSubClass == SUBCLASS and intf.bInterfaceProtocol in SIDES:
                try:
                    if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                        dev.detach_kernel_driver(intf.bInterfaceNumber)
                except (NotImplementedError, usb.core.USBError):
                    pass
                usb.util.claim_interface(dev, intf.bInterfaceNumber)
                self.ifaces[SIDES[intf.bInterfaceProtocol]] = (endpoint(intf, usb.util.ENDPOINT_IN),
                                                               endpoint(intf, usb.util.ENDPOINT_OUT))
        if not self.ifaces:
            usb.util.dispose_resources(dev)
            raise DongleError("dongle found, but not in VR mode (switch in the control panel)")
        self.dev = dev

    def read(self):
        """-> [(side, report bytes)] waiting now. Raises DongleError if the dongle went away."""
        out = []
        for side, (ep_in, _) in self.ifaces.items():
            try:
                out.append((side, bytes(ep_in.read(64, timeout=5))))
            except self.usb.core.USBTimeoutError:
                pass
            except self.usb.core.USBError as e:
                raise DongleError(f"dongle lost ({e})")
        return out

    def send(self, side, cmd):
        try:
            self.ifaces[side][1].write(cmd)
        except (KeyError, self.usb.core.USBError):
            pass

    def close(self):
        self.usb.util.dispose_resources(self.dev)


class SimSource:
    """The simulator's live mode: the same 64-byte reports over UDP (sim/src/live.rs)."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(SIM_VIEW)
        except OSError as e:
            raise DongleError(f"can't listen on {SIM_VIEW[0]}:{SIM_VIEW[1]} ({e}): is vrsim_view.py open?")
    def read(self):
        """Wait up to 5 ms for a datagram, then take what is queued (reports arrive every ms: a bounded drain)."""
        out = []
        self.sock.settimeout(0.005)
        for _ in range(256):
            try:
                d, _ = self.sock.recvfrom(256)
            except OSError:  # nothing waiting (or the simulator went away)
                break
            self.sock.setblocking(False)
            if len(d) == 64 and d[0] == 0x01:  # skip the viewer's 0xF0 ground truth
                out.append(("right" if d[2] & 0x02 else "left", d))
        return out

    def send(self, side, cmd):
        self.sock.sendto(f"{side[0].upper()} cmd={cmd.hex()}".encode(), SIM_CONTROL)

    def close(self):
        self.sock.close()


# ---- what the window and the terminal show ----

STILL_GYRO_DPS = 3.0  # below this rotation rate and with |a| near 1 g, the controller counts as lying still
DRIFT_MIN_S = 5.0  # drift shown after this long still


class Stats:
    """One controller: latest report plus the numbers worked out from the stream."""

    def __init__(self, side):
        self.side = side
        self.s = None  # latest report (dict from parse)
        self.count = 0  # new reports since the last rate update
        self.hz = 0.0
        self.lost = 0  # sequence numbers skipped since start (or reset)
        self._lost_s = 0  # ... since the last rate update
        self.lost_pct = 0.0  # share of reports lost over the last second
        self.gap_ms = 0.0  # longest time between two new reports in the last second (dongle clock)
        self._gap_max = 0.0
        self.still_since = None  # (time, yaw) when it started lying still
        self.drift = None  # °/min while still
        self.mark = None  # yaw when Mark was pressed

    def feed(self, s, now):
        prev = self.s
        self.s = s
        if prev is not None and prev["seq"] == s["seq"]:
            return  # keep-alive resend: no new data
        self.count += 1
        if prev is not None:
            gap = ((s["dongle_ts"] - prev["dongle_ts"]) & 0xFFFFFFFF) / 1000
            # Over 100 ms apart: `prev` was stale (reports left waiting in the endpoint when the tool started,
            # or the controller was away), not a loss.
            if gap < 100:
                skipped = (s["seq"] - prev["seq"] - 1) % 256
                self.lost += skipped
                self._lost_s += skipped
                self._gap_max = max(self._gap_max, gap)
        yaw = euler(s["quat"])[0]
        if self.connected() and self.gyro_dps() < STILL_GYRO_DPS and abs(self.accel_g() - 1) < 0.05:
            if self.still_since is None:
                self.still_since = (now, yaw)
            t0, y0 = self.still_since
            if now - t0 >= DRIFT_MIN_S:
                self.drift = wrap180(yaw - y0) / (now - t0) * 60
        else:
            self.still_since = self.drift = None

    def tick(self, dt):
        total = self.count + self._lost_s
        self.lost_pct = 100 * self._lost_s / total if total else 0.0
        self.hz, self.count, self._lost_s = self.count / dt, 0, 0
        self.gap_ms, self._gap_max = self._gap_max, 0.0

    def connected(self):
        return self.s is not None and self.s["flags"] & 0x01

    def gyro_dps(self):
        return math.degrees(math.sqrt(sum(v * v for v in self.s["gyro"]))) if self.s else 0.0

    def accel_g(self):
        return math.sqrt(sum(v * v for v in self.s["accel"])) / 9.80665 if self.s else 0.0

    def since_mark(self):
        return None if self.mark is None or not self.s else wrap180(euler(self.s["quat"])[0] - self.mark)

    def set_mark(self):
        self.mark = euler(self.s["quat"])[0] if self.s else None


def line(st):
    s = st.s
    if s is None:
        return f"{st.side:>5}: -"
    if not s["flags"] & 0x01:
        return f"{st.side:>5}: not connected"
    yaw, pitch, roll = euler(s["quat"])
    btn = "+".join(n for i, n in enumerate(BUTTONS) if s["buttons"] & 1 << i) or "-"
    flags = ",".join(n for b, n in FLAGS if s["flags"] & b)
    bat = "?" if s["battery"] is None else f"{s['battery']}%"
    drift = "" if st.drift is None else f"  drift {st.drift:+.2f}°/min"
    return (f"{st.side:>5}: yaw {yaw:7.1f} pitch {pitch:6.1f} roll {roll:7.1f}  gyro {st.gyro_dps():6.1f}°/s"
            f"  |a| {st.accel_g():4.2f} g  stick {s['stick'][0]:6d},{s['stick'][1]:6d}  trig {s['trigger']:4d}"
            f" grip {s['grip']:4d}  btn {btn:<10} bat {bat:>4}  [{flags}]  {st.hz:4.0f} Hz lost {st.lost_pct:.1f}% ({st.lost}){drift}")


def run_text(src):
    stats = {side: Stats(side) for side in ("left", "right")}
    last = time.monotonic()
    while True:
        for side, r in src.read():
            s = parse(r)
            if s:
                stats[side].feed(s, time.monotonic())
        now = time.monotonic()
        if now - last >= 0.2:
            for st in stats.values():
                st.tick(now - last)
            last = now
            print("  |  ".join(line(st) for st in stats.values() if st.s is not None) or "waiting for reports")


# ---- the window ----

BG, PANEL, GRID, TEXT, DIM, OK, BAD, WARN = ("#15171c", "#1c1f26", "#252932", "#d8dee9", "#7f8796", "#5ccf8a",
                                             "#ff5c5c", "#e6c34a")
COLOURS = {"left": "#5aa9ff", "right": "#ff9f43"}
POS = {"left": (-0.17, -0.06, -0.55), "right": (0.17, -0.06, -0.55)}
# Controller model (metres, controller frame: x right, y up, -z forward).
BOX = [(sx * 0.025, sy * 0.02, sz * 0.06) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count("1") == 1]
NAMES = {"left": ("X", "Y", "stick", "Menu"), "right": ("A", "B", "stick", "Oculus")}
HELP = "keys: r recentre   x reset   m mark   1 / 2 buzz left / right   l clear lost count"
PANEL_H = 318  # stats panel height, in 96-dpi pixels


class Window:
    def __init__(self, root, open_source):
        import tkinter as tk
        import tkinter.font as tkfont

        self.root, self.open_source = root, open_source
        self.stats = {side: Stats(side) for side in ("left", "right")}
        self.lock = threading.Lock()
        self.src = None
        self.error = "connecting..."
        self.last_rate = time.monotonic()
        self.k = root.winfo_fpixels("1i") / 96  # canvas coordinates follow the display scaling, like the fonts
        self.fonts = {(size, bold): tkfont.Font(family="Segoe UI", size=size, weight="bold" if bold else "normal")
                      for size in (9, 10, 11, 12, 18) for bold in (False, True)}
        self.mono = tkfont.Font(family="Consolas", size=10)
        root.title("QuestPad VR test")
        root.configure(bg=BG)
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{min(int(1180 * self.k), sw - 40)}x{min(int(800 * self.k), sh - 120)}+20+20")
        root.minsize(min(int(900 * self.k), sw - 40), min(int(600 * self.k), sh - 120))
        self.canvas = tk.Canvas(root, bg=BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        bar = tk.Frame(root, bg=BG)
        bar.pack(fill="x", padx=10, pady=(0, 2))
        for text, fn in (("Recentre", self.recenter), ("Reset", self.reset), ("Mark", self.mark),
                         ("Buzz left", lambda: self.buzz("left")), ("Buzz right", lambda: self.buzz("right"))):
            tk.Button(bar, text=text, command=fn, bg=PANEL, fg=TEXT, activebackground=GRID, activeforeground=TEXT,
                      relief="flat", padx=10).pack(side="left", padx=(0, 6))
        tk.Label(root, text=HELP, font=self.fonts[9, False], fg=DIM, bg=BG, anchor="w").pack(fill="x", padx=10,
                                                                                           pady=(0, 6))
        keys = {"r": self.recenter, "x": self.reset, "m": self.mark, "1": lambda: self.buzz("left"),
                "2": lambda: self.buzz("right"), "l": self.reset_lost}
        root.bind("<KeyPress>", lambda e: keys.get(e.keysym.lower(), lambda: None)())
        self.running = True
        root.protocol("WM_DELETE_WINDOW", self.close)
        threading.Thread(target=self.reader, daemon=True).start()
        self.tick()

    # ---- reading (own thread: USB reads block) ----

    def reader(self):
        while self.running:
            if self.src is None:
                try:
                    self.src = self.open_source()
                    self.error = None
                except DongleError as e:
                    self.error = str(e)
                    time.sleep(1.0)
                    continue
            try:
                got = self.src.read()
            except DongleError as e:
                self.error = str(e)
                self.src.close()
                self.src = None
                continue
            now = time.monotonic()
            with self.lock:
                for side, r in got:
                    s = parse(r)
                    if s:
                        self.stats[side].feed(s, now)

    def send(self, side, cmd):
        if self.src:
            self.src.send(side, cmd)

    def recenter(self):
        for side in self.stats:
            self.send(side, CMD_RECENTER)

    def reset(self):
        for side in self.stats:
            self.send(side, CMD_RESET)

    def buzz(self, side):
        self.send(side, cmd_haptic())

    def mark(self):
        with self.lock:
            for st in self.stats.values():
                st.set_mark()

    def reset_lost(self):
        with self.lock:
            for st in self.stats.values():
                st.lost = 0

    def close(self):
        self.running = False
        self.root.destroy()

    # ---- drawing ----

    def project(self, p, w, h):
        f = 0.9 * min(w, h * 1.6)
        z = min(p[2], -0.05)
        return w / 2 + f * p[0] / -z, h * 0.5 - f * p[1] / -z

    def draw_scene(self, w, h):
        c = self.canvas
        for i in range(-5, 6):  # floor grid
            c.create_line(*self.project((i * 0.1, -0.3, -0.4), w, h), *self.project((i * 0.1, -0.3, -1.4), w, h),
                          fill=GRID)
        for j in range(11):
            z = -0.4 - j * 0.1
            c.create_line(*self.project((-0.5, -0.3, z), w, h), *self.project((0.5, -0.3, z), w, h), fill=GRID)
        for side, st in self.stats.items():
            pos = POS[side]

            def screen(v, q):
                return self.project(tuple(a + b for a, b in zip(qrot(q, v), pos)), w, h)

            ident = (0.0, 0.0, 0.0, 1.0)  # yaw 0, level: where a recentred controller pointing ahead sits
            c.create_line(*screen((0, 0, -0.06), ident), *screen((0, 0, -0.25), ident), fill=DIM, dash=(3, 3))
            x, y = self.project((pos[0], pos[1] + 0.12, pos[2]), w, h)
            c.create_text(x, y, text=side, fill=COLOURS[side], font=self.fonts[11, True])
            if not st.connected():
                c.create_text(*self.project(pos, w, h), text="not connected", fill=DIM, font=self.fonts[10, False])
                continue
            q = st.s["quat"]
            pts = [screen(v, q) for v in BOX]
            for a, b in EDGES:
                c.create_line(*pts[a], *pts[b], fill=COLOURS[side], width=2)
            c.create_line(*screen((0, 0, -0.06), q), *screen((0, 0, -0.28), q), fill=COLOURS[side], width=3,
                          arrow="last")
            c.create_line(*screen((0, 0.02, 0), q), *screen((0, 0.07, 0), q), fill=COLOURS[side], width=2)

    def draw_panel(self, st, x0, y0, w):
        """Stats of one controller in a box at (x0, y0), `w` wide. Offsets are 96-dpi pixels, scaled by k."""
        c, col, k = self.canvas, COLOURS[st.side], self.k
        c.create_rectangle(x0, y0, x0 + w, y0 + PANEL_H * k, fill=PANEL, outline="")
        x, y = x0 + 14 * k, y0 + 10 * k
        s = st.s

        def text(dx, dy, t, fill=TEXT, size=10, bold=False, anchor="nw", font=None):
            font = font or self.fonts[size, bold]
            c.create_text(x + dx * k, y + dy * k, text=t, fill=fill, anchor=anchor, font=font)
            return font.measure(t) / k  # width, in 96-dpi pixels

        status = ("no report yet" if s is None else "standby" if s["flags"] & 0x10 else
                  "connected" if s["flags"] & 0x01 else "not connected")
        dx = text(0, 0, st.side.upper(), col, 12, True)
        text(dx + 12, 2, status, OK if status == "connected" else WARN, 11)
        if s is None:
            return
        bat = s["battery"]
        text(w / k - 28, 2, "battery ?" if bat is None else f"battery {bat}%",
             BAD if bat is not None and bat < 15 else TEXT, 11, anchor="ne")
        cx = 0
        for bit, name in ((0x04, "orientation"), (0x08, "gyro calibrated"), (0x80, "IMU units read")):
            on = bool(s["flags"] & bit)
            cx += text(cx, 28, ("✓ " if on else "✗ ") + name, OK if on else WARN, 9) + 16
        if s["flags"] & 0x20:
            text(cx, 28, "dead battery", BAD, 9, True)
        if s["flags"] & 0x01 and not s["flags"] & 0x08:
            text(0, 46, "set the controller down, still, for a second: that calibrates the gyro", WARN, 9)

        yaw, pitch, roll = euler(s["quat"])
        sm = st.since_mark()
        for i, (name, v) in enumerate((("yaw", yaw), ("pitch", pitch), ("roll", roll), ("since mark (yaw)", sm))):
            text(i * 125, 66, name, DIM, 9)
            text(i * 125, 80, "—" if v is None else f"{v:+.1f}°", TEXT, 18, True)

        g = [math.degrees(v) for v in s["gyro"]]
        a = [v / 9.80665 for v in s["accel"]]
        text(0, 122, f"gyro  {g[0]:+7.1f} {g[1]:+7.1f} {g[2]:+7.1f} °/s  |ω| {st.gyro_dps():6.1f} °/s", font=self.mono)
        text(0, 142, f"accel {a[0]:+7.2f} {a[1]:+7.2f} {a[2]:+7.2f} g    |a| {st.accel_g():6.2f} g", font=self.mono)
        if st.still_since is None:
            still, fill = "moving (set it down to measure the drift)", DIM
        elif st.drift is None:
            still, fill = f"still for {time.monotonic() - st.still_since[0]:.0f} s: measuring the drift...", TEXT
        else:
            still = f"still: drift {st.drift:+.2f} °/min"
            fill = OK if abs(st.drift) < 1 else WARN if abs(st.drift) < 5 else BAD
        text(0, 164, still, fill, 10, True)
        dx = text(0, 186, f"{st.hz:.0f} reports/s", OK if st.hz > 400 else WARN if st.hz > 0 else BAD)
        dx += text(dx + 18, 186, f"lost {st.lost_pct:.1f} % ({st.lost} in all)",
                   OK if st.lost_pct < 2 else WARN if st.lost_pct < 10 else BAD) + 18
        text(dx + 18, 186, f"longest gap {st.gap_ms:.1f} ms", OK if st.gap_ms < 10 else WARN)

        # inputs: buttons (filled = pressed, outlined = touched), trigger / grip bars, stick
        for i, name in enumerate(NAMES[st.side]):
            bx, by = x + i * 70 * k, y + 214 * k
            pressed, touched = s["buttons"] & 1 << i, i < 3 and s["touch"] & 1 << i
            c.create_rectangle(bx, by, bx + 62 * k, by + 24 * k, fill=col if pressed else PANEL,
                               outline=col if touched or pressed else GRID, width=2)
            c.create_text(bx + 31 * k, by + 12 * k, text=name, fill=BG if pressed else TEXT, font=self.fonts[9, False])
        for i, (name, v, touch) in enumerate((("trigger", s["trigger"], s["touch"] & 0x08), ("grip", s["grip"], 0))):
            by = y + (250 + i * 24) * k
            text(0, 248 + i * 24, name, DIM, 9)
            c.create_rectangle(x + 60 * k, by + 2 * k, x + 260 * k, by + 14 * k, fill=GRID,
                               outline=col if touch else "", width=2)
            c.create_rectangle(x + 60 * k, by + 2 * k, x + (60 + 200 * v / 4095) * k, by + 14 * k, fill=col,
                               outline="")
            text(268, 248 + i * 24, f"{v}", TEXT, 9)
        sx, sy, r = x0 + w - 70 * k, y + 250 * k, 34 * k
        c.create_oval(sx - r, sy - r, sx + r, sy + r, outline=col if s["touch"] & 0x04 else GRID, width=2)
        px, py = sx + (r - 2) * s["stick"][0] / 32000, sy - (r - 2) * s["stick"][1] / 32000
        c.create_oval(px - 6 * k, py - 6 * k, px + 6 * k, py + 6 * k, fill=col, outline="")
        c.create_text(sx, sy - r - 10 * k, text=f"stick {s['stick'][0]:+d} {s['stick'][1]:+d}", fill=DIM,
                      font=self.fonts[9, False])

    def tick(self):
        now = time.monotonic()
        c, k = self.canvas, self.k
        c.delete("all")
        w, h = max(c.winfo_width(), 600), max(c.winfo_height(), 500)
        scene_h = h - (PANEL_H + 14) * k
        with self.lock:
            if now - self.last_rate >= 1.0:
                for st in self.stats.values():
                    st.tick(now - self.last_rate)
                self.last_rate = now
            self.draw_scene(w, scene_h)
            pw = (w - 30) / 2
            for i, st in enumerate(self.stats.values()):
                self.draw_panel(st, 10 + i * (pw + 10), scene_h + 4 * k, pw)
        if self.error:
            c.create_text(w / 2, 24 * k, text=self.error, fill=BAD, font=self.fonts[12, True])
        self.root.after(33, self.tick)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--text", action="store_true", help="print to the terminal instead of opening a window")
    ap.add_argument("--sim", action="store_true", help="read the simulator's live mode over UDP instead of USB")
    ap.add_argument("--buzz", choices=("left", "right"), help="buzz one controller for 200 ms")
    ap.add_argument("--recenter", action="store_true", help="make both controllers' current heading yaw 0")
    args = ap.parse_args()
    open_source = SimSource if args.sim else UsbSource
    if args.text or args.buzz or args.recenter:
        try:
            src = open_source()
        except DongleError as e:
            sys.exit(str(e))
        if args.buzz:
            src.send(args.buzz, cmd_haptic())
        elif args.recenter:
            for side in ("left", "right"):
                src.send(side, CMD_RECENTER)
        else:
            try:
                run_text(src)
            except DongleError as e:
                sys.exit(str(e))
        return
    import tkinter as tk

    root = tk.Tk()
    Window(root, open_source)
    root.mainloop()


if __name__ == "__main__":
    main()
