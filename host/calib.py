"""3DoF calibration panel: the right side of the control panel (gui.py) in DS4 and VR modes.

Shows both controllers in 3D as they turn, with pitch / yaw / roll and the rotation rate about each axis, and
edits each controller's gyro axis map (settings imu_map / imu_map_left: which gyro axis feeds pitch, yaw and
roll, and its sign). Map changes go to the dongle at once without being saved (settings_set persist=0), so the
view, the DS4 gyro and the VR orientation follow them live; "Save to dongle" stores them.

Where the orientation comes from:
  VR mode   the dongle's own fusion, read from its VR interfaces with pyusb (what Monado gets; vrtest.py
            Reader / UsbSource). Without pyusb, or while another program holds those interfaces, the panel's own
            fusion.
  DS4 mode  the panel's own fusion: the dongle's raw IMU samples (serial port), resting offset removed and mapped
            as the firmware does (dsu.py), fused like vr.rs (keep Fusion in step with it). The game fuses the DS4
            gyro itself, so Recentre only moves this view there.

    python3 host/calib.py          sample motion, no dongle (UI work)
    python3 host/calib.py --test   self-test
"""
import math
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

import dsu
import vrtest
from bridge import Bridge
from mapping import Hand, frame_notifications

SIDES = ("left", "right")

# ---- axis maps (settings imu_map, imu_map_left) ----

# Output axes of a map (DS4 frame = OpenXR controller frame: x right, y up, z toward the user). The gyro about
# x is pitch (+ = front tipped up), about y yaw (+ = turned left), about z roll (+ = rolled left).
AXIS_NAMES = ("Pitch", "Yaw", "Roll")
DEFAULT_MAP = Bridge.imu_map(dsu.DEFAULTS, "right")  # settings.rs DEFAULT


def _default_sign(j):
    return -1 if DEFAULT_MAP[j] & 0x80 else 1


def to_logical(m):
    """imu_map -> [(source, sign)] per output axis, relative to the default map: source j = what the default map
    puts on axis j (so the default reads [(0, 1), (1, 1), (2, 1)])."""
    out = []
    for v in m:
        axis = min(v & 3, 2)
        j = next(i for i, d in enumerate(DEFAULT_MAP) if d & 3 == axis)
        out.append((j, (-1 if v & 0x80 else 1) * _default_sign(j)))
    return out


def from_logical(lm):
    """Inverse of to_logical."""
    return tuple(DEFAULT_MAP[j] & 3 | (0x80 if s * _default_sign(j) < 0 else 0) for j, s in lm)


def handedness(lm):
    """+1: a rotation of the default frame. -1: a mirror (an odd number of inversions and swaps). 0: two axes
    read the same source."""
    src = [j for j, _ in lm]
    if sorted(src) != [0, 1, 2]:
        return 0
    swaps = sum(1 for i in range(3) for k in range(i + 1, 3) if src[i] > src[k])
    d = -1 if swaps % 2 else 1
    for _, s in lm:
        d *= s
    return d


# ---- orientation fusion (a port of vr.rs Fusion: Mahony PI filter) ----

G = 9.80665
KP, KI = 0.5, 0.2
BIAS_MAX = 0.0873  # rad/s
LEARN_MAX_RATE = 1.0  # rad/s
GRAVITY_BAND = 0.1 * G  # accelerometer trusted as gravity within this of 1 g...
GRAVITY_STEADY = 10  # ... for this many samples in a row


def _normalize(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (0.0, 0.0, 0.0, 1.0) if n < 1e-6 else (x / n, y / n, z / n, w / n)


def _clamp(v):
    return BIAS_MAX if v > BIAS_MAX else -BIAS_MAX if v < -BIAS_MAX else v


class Fusion:
    """Controller -> world orientation, q = (x, y, z, w) as in the VR report, from gyro (rad/s) and
    accelerometer (m/s²) samples in the OpenXR controller frame, as the dongle computes it in VR mode. Runs 500
    times a second per controller in Python: plain locals."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Start over from gravity at the next sample (also recentres)."""
        self.q, self.init, self.last_ts, self.bias, self.steady = (0.0, 0.0, 0.0, 1.0), False, 0, (0.0,) * 3, 0

    def update(self, a, w, ts):
        ax, ay, az = a
        an = math.sqrt(ax * ax + ay * ay + az * az)
        if not self.init:
            if an < 0.5 * G:
                return
            ux, uy, uz = ax / an, ay / an, az / an  # smallest rotation taking measured up to world +Y
            self.q = (1.0, 0.0, 0.0, 0.0) if 1 + uy < 1e-4 else _normalize(-uz, 0.0, ux, 1 + uy)
            self.init, self.bias, self.last_ts = True, (0.0,) * 3, ts
            self.recenter()
            return
        dt_us, self.last_ts = ts - self.last_ts, ts
        dt = 0.002 if dt_us <= 0 or dt_us > 50_000 else dt_us * 1e-6
        wx, wy, wz = w
        rate = math.sqrt(wx * wx + wy * wy + wz * wz)
        bx, by, bz = self.bias
        wx, wy, wz = wx - bx, wy - by, wz - bz
        qx, qy, qz, qw = self.q
        self.steady = self.steady + 1 if abs(an - G) < GRAVITY_BAND else 0
        if self.steady >= GRAVITY_STEADY:
            # Estimated up (world +Y in the controller frame: qmatrix row 1) vs measured up: e = a/|a| x up.
            vx, vy, vz = 2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)
            ax, ay, az = ax / an, ay / an, az / an
            ex, ey, ez = ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx
            if rate < LEARN_MAX_RATE:
                k = KI * dt
                self.bias = (_clamp(bx - k * ex), _clamp(by - k * ey), _clamp(bz - k * ez))
            wx, wy, wz = wx + KP * ex, wy + KP * ey, wz + KP * ez
        h = 0.5 * dt  # q += dt / 2 * q ⊗ (w, 0)
        self.q = _normalize(qx + h * (qw * wx + qy * wz - qz * wy), qy + h * (qw * wy - qx * wz + qz * wx),
                            qz + h * (qw * wz + qx * wy - qy * wx), qw + h * (-qx * wx - qy * wy - qz * wz))

    def recenter(self):
        """The controller's forward (-Z) projected on the floor becomes world -Z."""
        f = vrtest.qrot(self.q, (0.0, 0.0, -1.0))
        if math.hypot(f[0], f[2]) < 0.2:  # pointing up or down: use the top instead
            u = vrtest.qrot(self.q, (0.0, 1.0, 0.0))
            s = 1.0 if f[1] < 0 else -1.0
            f = (s * u[0], 0.0, s * u[2])
        n = math.hypot(f[0], f[2])
        if n < 1e-3:
            return
        fx, fz = f[0] / n, f[2] / n
        _, yy, _, yw = (0.0, 1.0, 0.0, 0.0) if 1 - fz < 1e-4 else _normalize(0.0, fx, 0.0, 1 - fz)
        x, y, z, w = self.q  # yaw (0, yy, 0, yw) ⊗ q
        self.q = _normalize(yw * x + yy * z, yw * y + yy * w, yw * z - yy * x, yw * w - yy * y)


# ---- sources: what the panel shows ----

FRESH_S = 0.5  # a controller with no sample for this long is shown as not connected


def _reading(conn, quat=(0.0, 0.0, 0.0, 1.0), gyro=(0.0, 0.0, 0.0), gcal=False, battery=None):
    return dict(conn=conn, quat=quat, gyro=gyro, gcal=gcal, battery=battery)


class _HostSide:
    def __init__(self):
        self.hand, self.rest, self.fusion = Hand(), dsu.Rest(), Fusion()
        self.gyro, self.last, self.epoch, self.imu_map = (0.0, 0.0, 0.0), 0.0, 0, DEFAULT_MAP


class HostTracker:
    """The panel's own fusion, fed with every serial frame from the dongle (`feed`, a Worker tap: runs on the
    serial thread, so no sample is lost while the window is busy). Fuses only while `active` (shown)."""

    GYRO_SCALE = math.pi / 180 / dsu.GYRO_PER_DPS
    ACCEL_SCALE = G / dsu.ACCEL_PER_G

    def __init__(self):
        self.lock = threading.Lock()
        self.slots = {}  # link slot -> side
        self.sides = {side: _HostSide() for side in SIDES}
        self._active = False

    @property
    def active(self):
        return self._active

    @active.setter
    def active(self, on):
        if on and not self._active:
            self.reset()  # what it fused before was left behind while it wasn't
        self._active = on

    def feed(self, f):
        kind = f[0]
        if kind == 0xA1:
            slot, entries = frame_notifications(f) or (None, None)
            side = self.slots.get(slot)
            if side is None or not self._active:
                return
            with self.lock:
                hs = self.sides[side]
                ts = hs.hand.imu_ts
                hs.hand.update(entries)
                if hs.hand.imu_ts != ts:
                    self._sample(side, hs)
        elif kind == 0xA0 and len(f) >= 14:
            side = dsu.SIDES.get(int.from_bytes(f[10:14], "little"))
            if side:
                self.slots[f[1]] = side
        elif kind == 0xA2 and len(f) >= 2:
            self.slots.pop(f[1], None)
        elif kind == 0xA9 and len(f) >= 5:
            self.slots = {slot: side for slot, side, _ in Bridge.parse_status(f)["controllers"]}

    def _sample(self, side, hs):
        h, rest = hs.hand, hs.rest
        rest.add(h.accel, h.gyro)
        if rest.epoch != hs.epoch:  # offset re-measured: it replaces the learnt bias (as vr.rs does)
            hs.epoch = rest.epoch
            hs.fusion.bias = (0.0,) * 3
        g = dsu.ds4_frame([v - o for v, o in zip(h.gyro, rest.offset)], side, hs.imu_map)
        a = dsu.ds4_frame(h.accel, side, hs.imu_map)
        gs, as_ = self.GYRO_SCALE, self.ACCEL_SCALE
        hs.gyro = (g[0] * gs, g[1] * gs, g[2] * gs)
        hs.fusion.update((a[0] * as_, a[1] * as_, a[2] * as_), hs.gyro, h.imu_ts)
        hs.last = time.monotonic()

    def set_map(self, side, m):
        """New axis map for one controller: its orientation starts over from gravity (it was in the old axes)."""
        with self.lock:
            hs = self.sides[side]
            hs.imu_map = tuple(m)
            hs.fusion.reset()

    def recenter(self):
        with self.lock:
            for hs in self.sides.values():
                hs.fusion.recenter()

    def reset(self):
        with self.lock:
            for hs in self.sides.values():
                hs.fusion.reset()

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {side: _reading(now - hs.last < FRESH_S and hs.fusion.init, hs.fusion.q, hs.gyro, hs.epoch > 0,
                                   hs.hand.battery or None)
                    for side, hs in self.sides.items()}


class VrTracker:
    """The dongle's own orientation from its VR interfaces: the latest report of each controller (vrtest.Reader;
    `error` says why the interfaces can't be opened, while it retries)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.reports = {}  # side -> (report dict, time)
        self.reader = vrtest.Reader(vrtest.UsbSource, self._feed, retry=1.5)

    def _feed(self, batch, now):
        with self.lock:
            for side, s in batch:
                self.reports[side] = (s, now)

    @property
    def error(self):
        return self.reader.error

    def live(self):
        return self.reader.src is not None

    def send(self, cmd, sides=SIDES):
        for side in sides:
            self.reader.send(side, cmd)

    def snapshot(self):
        now = time.monotonic()
        out = {}
        with self.lock:
            for side in SIDES:
                s, t = self.reports.get(side, (None, 0.0))
                if s is None or now - t > FRESH_S or not s["flags"] & vrtest.FLAG_CONN:
                    out[side] = _reading(False)
                else:
                    out[side] = _reading(bool(s["flags"] & vrtest.FLAG_ORI), s["quat"], s["gyro"],
                                         bool(s["flags"] & vrtest.FLAG_GCAL), s["battery"])
        return out

    def close(self):
        self.reader.close()


class DemoTracker:
    """Sample motion with no dongle (gui.py --demo, calib.py on its own)."""

    def snapshot(self):
        t = time.monotonic()
        out = {}
        for i, side in enumerate(SIDES):
            amp, freq, phase = (0.35, 0.6, 0.5), (0.9, 0.6, 0.5), (i, 2 * i, 3 * i)
            ang = [a * math.sin(f * t + p) for a, f, p in zip(amp, freq, phase)]  # pitch, yaw, roll
            rate = tuple(a * f * math.cos(f * t + p) for a, f, p in zip(amp, freq, phase))
            # yaw (about y), then pitch (x), then roll (z): vrtest.euler's order
            cy, sy, cp, sp, cr, sr = (f(ang[n] / 2) for n in (1, 0, 2) for f in (math.cos, math.sin))
            w, x, y, z = cy * cp, cy * sp, sy * cp, -sy * sp  # qy * qx
            q = (x * cr + y * sr, y * cr - x * sr, z * cr + w * sr, w * cr - z * sr)  # ... * qz, as (x, y, z, w)
            out[side] = _reading(True, q, rate, True, 80 - 30 * i)
        return out


# ---- drawing ----

# Light, like the ttk panel next to it: white surfaces, grey lines, saturated colours only for data.
BG, CARD, LINE, TEXT, DIM = "#ffffff", "#f8fafc", "#e2e8f0", "#0f172a", "#64748b"
OK, WARN, BAD = "#15803d", "#b45309", "#b91c1c"
AXIS_COLOURS = ("#dc2626", "#16a34a", "#2563eb")  # pitch / x red, yaw / y green, roll / z blue (as in 3D tools)
SIDE_COLOURS = {"left": "#0891b2", "right": "#ea580c"}
POS = {"left": (-0.13, -0.01, -0.42), "right": (0.13, -0.01, -0.42)}
FLOOR_Y = -0.12
GIZMO = 0.06  # metres, axis line length each side of the controller's centre
# Coarser than vrtest.py's (the models are drawn small here): Tk's redraw of the faces is the panel's main
# cost, ~40 µs each, so ~200 faces per controller in view.
MODEL_CELL = 1 / 11
CARD_H = (96, 64)  # 96-dpi pixels, one controller's readout: side by side, stacked (compact: one line less)
WIDE = 460  # 96-dpi pixels: readouts side by side from this canvas width, stacked below
RATE_FULL = 300.0  # °/s at a full rate bar
LAYERS = ("floor", "model", "over", "card", "text")  # stacking order, bottom to top


class View:
    """The canvas: both controllers in 3D, and a readout per controller. Items are made once and changed after
    that, and only in what changed (Tk redraws every item an update touches, and creating items costs more
    still); a frame in which nothing visible moved is skipped. The faces of each model have their own pool."""

    def __init__(self, canvas, k, fonts):
        self.c, self.k, self.f = canvas, k, fonts
        self.call, self.path = canvas.tk.call, canvas._w  # the faces' fast path, past tkinter's option handling
        self.models = None  # loaded at the first frame (the panel may never show)
        self.shades = {side: vrtest.shades(colour, 0.35) for side, colour in SIDE_COLOURS.items()}
        self.pools = {side: [] for side in SIDES}  # per side: [face item, last coords (None = hidden), last fill]
        self.items = {}  # key -> other canvas item
        self.opts = {}  # item -> options last set ("xy": coords)
        self.shown, self.drawn = set(), set()  # keys drawn in the last frame / this one
        self.widths = {}  # (font, text) -> width
        self.created = False
        self.last = None  # what the last frame showed

    def width(self, font, text):
        key = (font, text)
        if key not in self.widths:
            self.widths[key] = self.f[font].measure(text)
        return self.widths[key]

    def put(self, key, kind, coords, layer, **opts):
        item = self.items.get(key)
        if item is None:
            item = self.items[key] = getattr(self.c, "create_" + kind)(*coords, tags=layer, **opts)
            self.opts[item] = dict(opts, xy=coords)
            self.created = True
        else:
            old = self.opts[item]
            if old["xy"] != coords:
                self.c.coords(item, *coords)
                old["xy"] = coords
            changed = {o: v for o, v in opts.items() if old.get(o) != v}
            if key not in self.shown:
                changed["state"] = "normal"
            if changed:
                self.c.itemconfigure(item, **changed)
                old.update(changed)
        self.drawn.add(key)

    def frame(self, data, message):
        """Draw `data` (side -> reading), unless it looks the same as last time."""
        k = self.k
        w, h = max(self.c.winfo_width(), 200), max(self.c.winfo_height(), 200)
        key = (w, h, message, tuple((d["conn"], d["gcal"], d["battery"], tuple(round(v * 2000) for v in d["quat"]),
                                     tuple(round(math.degrees(v) / 4) for v in d["gyro"])) for d in data.values()))
        if key == self.last:
            return
        self.last = key
        if self.models is None:
            self.models = {}
            for side, path in vrtest.MODEL_FILES.items():
                try:
                    self.models[side] = vrtest.controller_model(path, MODEL_CELL)
                except (OSError, ValueError, KeyError):  # no assets/: boxes
                    self.models[side] = None
        wide = w >= WIDE * k
        ch = CARD_H[not wide] * k
        scene_h = max(h - (ch + 6 * k) * (1 if wide else 2), 120 * k)
        self._scene(data, w, scene_h, message)
        cw = (w - 10 * k) / 2 if wide else w
        for i, side in enumerate(SIDES):
            x0, y0 = (i * (cw + 10 * k), scene_h + 6 * k) if wide else (0, scene_h + 6 * k + i * (ch + 6 * k))
            self._card(side, data[side], x0, y0, cw, ch, not wide)
        for key in self.shown - self.drawn:
            item = self.items[key]
            self.c.itemconfigure(item, state="hidden")
            self.opts[item]["state"] = "hidden"
        self.shown, self.drawn = self.drawn, set()
        if self.created:
            for layer in LAYERS:
                self.c.tag_raise(layer)
            self.created = False

    @staticmethod
    def _camera(w, h):
        """-> focal length, screen centre (camera at the origin, looking down -z)."""
        return 1.05 * min(w, h * 1.5), w / 2, h * 0.5

    def _project(self, p, w, h):
        f, cx, cy = self._camera(w, h)
        z = min(p[2], -0.05)
        return cx - f * p[0] / z, cy + f * p[1] / z

    def _scene(self, data, w, h, message):
        P, put = self._project, self.put
        for i in range(-4, 5):  # floor grid
            put(("gx", i), "line", (*P((i * 0.1, FLOOR_Y, -0.44), w, h), *P((i * 0.1, FLOOR_Y, -1.2), w, h)), "floor",
                fill=LINE)
        for j in range(9):
            z = -0.44 - j * 0.095
            put(("gz", j), "line", (*P((-0.4, FLOOR_Y, z), w, h), *P((0.4, FLOOR_Y, z), w, h)), "floor", fill=LINE)
        if message:
            put("message", "text", (w / 2, 16 * self.k), "text", text=message, fill=DIM, font=self.f["body"])
        for side in SIDES:
            d, pos, col = data[side], POS[side], SIDE_COLOURS[side]

            def screen(v, q, pos=pos):
                return P(tuple(a + b for a, b in zip(vrtest.qrot(q, v), pos)), w, h)

            ident = (0.0, 0.0, 0.0, 1.0)  # where a recentred controller pointing ahead points
            put(("ref", side), "line", (*screen((0, 0, -0.07), ident), *screen((0, 0, -0.22), ident)), "floor",
                fill=DIM, dash=(3, 3))
            put(("name", side), "text", P((pos[0], pos[1] + 0.105, pos[2]), w, h), "text", text=side.capitalize(),
                fill=col, font=self.f["bold"])
            q = d["quat"]
            if d["conn"] and self.models[side]:
                self._mesh(side, q, pos, w, h)
            else:
                self._mesh(side, None, pos, w, h)
            if not d["conn"]:
                put(("off", side), "text", P(pos, w, h), "text", text="not connected", fill=DIM, font=self.f["body"])
                continue
            if not self.models[side]:
                pts = [screen(v, q) for v in vrtest.BOX]
                for n, (a, b) in enumerate(vrtest.EDGES):
                    put(("box", side, n), "line", (*pts[a], *pts[b]), "model", fill=col, width=2)
            put(("fwd", side), "line", (*screen((0, 0, -0.07), q), *screen((0, 0, -0.22), q)), "over", fill=col,
                width=3, arrow="last")
            for i, axis in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):  # rotation axes: pitch x, yaw y, roll z
                tip = tuple(GIZMO * a for a in axis)
                put(("axis", side, i), "line", (*screen(tuple(-v for v in tip), q), *screen(tip, q)), "over",
                    fill=AXIS_COLOURS[i], width=2, arrow="last", arrowshape=(7, 8, 3))
                put(("axis_name", side, i), "text", screen(tuple(1.3 * v for v in tip), q), "over",
                    text=AXIS_NAMES[i][0], fill=AXIS_COLOURS[i], font=self.f["bold"])

    def _mesh(self, side, q, pos, w, h):
        """The model's faces (vrtest.mesh_faces, far to near) into the side's pool: pool order is stacking order.
        No outline (it doubles Tk's drawing); q None hides them."""
        pool, call, path = self.pools[side], self.call, self.path
        faces = ()
        if q is not None:
            wv, faces = vrtest.mesh_faces(self.models[side], q, pos)
            f, cx, cy = self._camera(w, h)
            sp = [(cx - f * x / min(z, -0.05), cy + f * y / min(z, -0.05)) for x, y, z in wv]
            steps = self.shades[side]
            for n, (i, j, k, light) in enumerate(faces):
                xy, fill = (*sp[i], *sp[j], *sp[k]), steps[light]
                if n == len(pool):
                    pool.append([self.c.create_polygon(*xy, fill=fill, outline="", tags="model"), xy, fill])
                    self.created = True
                    continue
                slot = pool[n]
                if slot[1] is None:
                    call(path, "itemconfigure", slot[0], "-state", "normal")
                if slot[1] != xy:
                    call(path, "coords", slot[0], *xy)
                    slot[1] = xy
                if slot[2] != fill:
                    call(path, "itemconfigure", slot[0], "-fill", fill)
                    slot[2] = fill
        for slot in pool[len(faces):]:
            if slot[1] is not None:
                call(path, "itemconfigure", slot[0], "-state", "hidden")
                slot[1] = None

    def _card(self, side, d, x0, y0, w, h, compact):
        """One controller: status, then pitch / yaw / roll, each with a bar for the rotation rate about that axis
        (centre = still, full = RATE_FULL °/s: which way it swings is the sign to check). Compact: the status
        follows the name, and each axis name its value."""
        k, put, f = self.k, self.put, self.f
        x1, y1, r = x0 + w - 1, y0 + h, 6 * k
        put(("card", side), "polygon", (x0 + r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y1 - r, x1, y1, x1 - r, y1,
                                        x0 + r, y1, x0, y1, x0, y1 - r, x0, y0 + r, x0, y0), "card",
            fill=CARD, outline=LINE, smooth=True)
        x, y, xr = x0 + 12 * k, y0 + 8 * k, x1 - 12 * k
        put(("side", side), "text", (x, y), "text", text=side.upper(), fill=SIDE_COLOURS[side], font=f["bold"],
            anchor="nw")
        if d["conn"] and d["battery"] is not None:
            put(("battery", side), "text", (xr, y + k), "text", text=f"battery {d['battery']}%", anchor="ne",
                font=f["small"], fill=BAD if d["battery"] < 15 else DIM)
        if not d["conn"]:
            status, colour = "not connected", DIM
        elif d["gcal"]:
            status, colour = "connected · ✓ gyro calibrated", OK
        else:
            status, colour = "connected · gyro: set it down 1 s", WARN
        sx, sy = (x + self.width("bold", side.upper()) + 10 * k, y + 2 * k) if compact else (x, y + 20 * k)
        put(("status", side), "text", (sx, sy), "text", text=status, fill=colour, font=f["small"], anchor="nw")
        if not d["conn"]:
            return
        yaw, pitch, roll = vrtest.euler(d["quat"])
        col_w = (xr - x) / 3
        for i, (angle, rate) in enumerate(zip((pitch, yaw, roll), d["gyro"])):
            cx, cy, colour = x + i * col_w, y + (22 if compact else 40) * k, AXIS_COLOURS[i]
            put(("dot", side, i), "oval", (cx, cy + 5 * k, cx + 7 * k, cy + 12 * k), "text", fill=colour, outline="")
            name = AXIS_NAMES[i].upper()
            put(("label", side, i), "text", (cx + 11 * k, cy + 2 * k), "text", text=name, fill=DIM, font=f["small"],
                anchor="nw")
            vx, vy = (cx + 16 * k + self.width("small", name), cy) if compact else (cx, cy + 15 * k)
            put(("angle", side, i), "text", (vx, vy), "text", text=f"{angle:+.1f}°", fill=TEXT, font=f["value"],
                anchor="nw")
            bx0, bx1, by = cx, cx + col_w - 10 * k, cy + (22 if compact else 38) * k
            mid = (bx0 + bx1) / 2
            end = mid + (bx1 - bx0) / 2 * max(-1.0, min(1.0, math.degrees(rate) / RATE_FULL))
            put(("track", side, i), "rectangle", (bx0, by, bx1, by + 4 * k), "text", fill=LINE, outline="")
            put(("rate", side, i), "rectangle", (min(mid, end), by, max(mid, end), by + 4 * k), "text", fill=colour,
                outline="")
            put(("zero", side, i), "line", (mid, by - 2 * k, mid, by + 6 * k), "text", fill=DIM)


# ---- the panel ----

TEST_TEXT = ("Tip the front up: Pitch +. Turn left: Yaw +. Roll left: Roll +. Wrong sign: Invert. Two axes trading "
             "places: pick the other one (or Swap). Applies live, to the chosen controller only.")
HINT = {
    "VR": "Point both controllers ahead, then Recentre: yaw 0 here and in Monado. Set them down 1 s to calibrate "
          "the gyro.",
    "DS4": "The game fuses the DS4 gyro itself: Recentre and Reset only move this view. Set the controllers down "
           "1 s to calibrate the gyro.",
}
SHOWN_IN = ("DS4", "VR")
ONE_MAP = "This firmware has one axis map for both controllers: update it to set them apart."
FRAME_MS = 33


class CalibPanel(ttk.Frame):
    """apply_maps({side: imu_map}, persist, done): send the maps to the dongle; done(ok) on the GUI thread.
    on_visible(bool): show or hide the panel (it wants to be seen in DS4 and VR modes)."""

    def __init__(self, parent, apply_maps, on_visible=lambda v: None, demo=False):
        super().__init__(parent, padding=8)
        self.apply_maps, self.on_visible = apply_maps, on_visible
        self.k = k = parent.winfo_fpixels("1i") / 96  # canvas coordinates follow the display scaling, like fonts
        self.host = HostTracker()
        self.vr = None
        self.demo = DemoTracker() if demo else None
        self.mode = None
        self.job = None  # pending frame
        self.two_maps = True  # firmware with imu_map_left
        self.saved = {side: DEFAULT_MAP for side in SIDES}
        self.maps = {side: to_logical(DEFAULT_MAP) for side in SIDES}
        self.state = None  # (text, colour) of the last send, until the maps change again
        base = tkfont.nametofont("TkDefaultFont").actual()
        fonts = dict(body=(base["family"], base["size"]), small=(base["family"], max(base["size"] - 1, 8)),
                     bold=(base["family"], base["size"] + 1, "bold"), value=("Consolas", base["size"] + 4, "bold"))
        self._build()
        self.view = View(self.canvas, k, {n: tkfont.Font(font=v) for n, v in fonts.items()})

    @property
    def visible(self):
        return self.mode in SHOWN_IN

    # ---- widgets ----

    def _build(self):
        pad = dict(padx=8, pady=4)

        def label(parent, **kw):
            """A label that wraps to the width it is given (and starts narrow, so it never sets the width)."""
            lb = ttk.Label(parent, justify="left", wraplength=int(240 * self.k), **kw)
            lb.bind("<Configure>", lambda e: lb.configure(wraplength=max(e.width, 120)))
            return lb

        # The axis frame is packed first, from the bottom: on a short window the 3D view gives way.
        axes = ttk.LabelFrame(self, text="Gyro axes", padding=6)
        axes.pack(side="bottom", fill="x", **pad)
        view = ttk.LabelFrame(self, text="3DoF calibration", padding=6)
        view.pack(fill="both", expand=True, **pad)
        self.source_var = tk.StringVar()
        self.source_lbl = label(view, textvariable=self.source_var, foreground=DIM)
        self.source_lbl.pack(fill="x")
        self.hint_var = tk.StringVar()
        label(view, textvariable=self.hint_var, foreground=DIM).pack(side="bottom", fill="x", pady=(4, 0))
        act = ttk.Frame(view)
        act.pack(side="bottom", fill="x", pady=(6, 0))
        ttk.Button(act, text="Recentre", command=self.recenter).pack(side="left")
        ttk.Button(act, text="Reset orientation", command=self.reset).pack(side="left", padx=4)
        # Small size request: the canvas takes whatever room the window gives it (gui.py sizes the window).
        self.canvas = tk.Canvas(view, bg=BG, highlightthickness=1, highlightbackground=LINE,
                                width=int(300 * self.k), height=int(260 * self.k))
        self.canvas.pack(fill="both", expand=True, pady=(4, 0))

        top = ttk.Frame(axes)
        top.pack(fill="x")
        ttk.Label(top, text="Controller").pack(side="left")
        self.side_var = tk.StringVar(value="right")
        self.side_btns = [ttk.Radiobutton(top, text=s.capitalize(), value=s, variable=self.side_var,
                                          command=self._refresh) for s in SIDES]
        for b in self.side_btns:
            b.pack(side="left", padx=(8, 0))
        self.copy_btn = ttk.Button(top, text="Copy to the other controller", command=self.copy)
        self.copy_btn.pack(side="right")
        label(axes, text=TEST_TEXT, foreground=DIM).pack(fill="x", pady=(4, 6))
        grid = ttk.Frame(axes)
        grid.pack(fill="x")
        frame_bg = ttk.Style().lookup("TFrame", "background") or self.winfo_toplevel()["bg"]
        self.src_vars, self.inv_vars = [], []
        for i, name in enumerate(AXIS_NAMES):
            dot = tk.Canvas(grid, width=10, height=10, highlightthickness=0, bg=frame_bg)
            dot.create_oval(1, 1, 9, 9, fill=AXIS_COLOURS[i], outline="")
            dot.grid(row=i, column=0, padx=(0, 6))
            ttk.Label(grid, text=f"{name} reads").grid(row=i, column=1, sticky="w", padx=(0, 8), pady=1)
            src, inv = tk.IntVar(), tk.IntVar()
            for j, src_name in enumerate(AXIS_NAMES):
                ttk.Radiobutton(grid, text=src_name, value=j, variable=src,
                                command=lambda i=i: self.pick(i, self.src_vars[i].get())).grid(row=i, column=2 + j,
                                                                                               sticky="w", padx=2)
            ttk.Checkbutton(grid, text="Invert", variable=inv, command=lambda i=i: self.invert(i)).grid(
                row=i, column=5, sticky="w", padx=(12, 0))
            self.src_vars.append(src)
            self.inv_vars.append(inv)
        swaps = ttk.Frame(axes)
        swaps.pack(fill="x", pady=(6, 0))
        ttk.Label(swaps, text="Swap").pack(side="left")
        for a, b in ((0, 2), (0, 1), (1, 2)):
            ttk.Button(swaps, text=f"{AXIS_NAMES[a]} ↔ {AXIS_NAMES[b]}",
                       command=lambda a=a, b=b: self.swap(a, b)).pack(side="left", padx=(6, 0))
        self.warn_var = tk.StringVar()
        self.warn_lbl = label(axes, textvariable=self.warn_var, foreground=WARN)
        self.foot = ttk.Frame(axes)
        self.foot.pack(fill="x", pady=(6, 0))
        self.state_var = tk.StringVar()
        self.state_lbl = label(self.foot, textvariable=self.state_var)
        self.state_lbl.pack(side="top", fill="x", pady=(0, 4))
        ttk.Button(self.foot, text="Save to dongle", command=self.save).pack(side="right")
        ttk.Button(self.foot, text="Revert", command=self.revert).pack(side="right", padx=4)
        ttk.Button(self.foot, text="Default", command=self.default).pack(side="right")
        self._refresh()

    # ---- mode, source, frames ----

    def set_mode(self, mode):
        """Dongle mode from the status ("HID", "DS4", "XInput", "VR"), None when it is away."""
        if mode == self.mode:
            return
        was = self.visible
        self.mode = mode
        if mode == "VR" and self.vr is None and not self.demo:
            self.vr = VrTracker()
        elif mode != "VR" and self.vr is not None:
            self.vr.close()
            self.vr = None
        self.hint_var.set(HINT.get(mode, ""))
        if self.visible != was:
            self.on_visible(self.visible)
            if self.visible and self.job is None:
                self.job = self.after(FRAME_MS, self._tick)

    def source(self):
        """-> (where readings come from, what to say about it, whether that is a warning)."""
        if self.demo:
            return self.demo, "Sample motion (demo)", False
        if self.vr is not None and self.vr.live():
            return self.vr, "Orientation from the dongle: what Monado gets.", False
        if self.mode == "VR":
            why = (self.vr.error if self.vr else "") or "VR interfaces busy"
            return self.host, f"Orientation computed here: {why}.", True
        return self.host, "Orientation computed here from the gyro the game gets.", False

    def _tick(self):
        if not self.visible:
            self.job = None
            self.host.active = False
            return
        src, text, warn = self.source()
        self.host.active = src is self.host
        if self.source_var.get() != text:
            self.source_var.set(text)
            self.source_lbl.configure(foreground=WARN if warn else DIM)
        data = src.snapshot()
        idle = not any(d["conn"] for d in data.values())
        self.view.frame(data, "Waiting for the controllers: press Oculus / Menu to wake them" if idle else "")
        self.job = self.after(FRAME_MS, self._tick)

    def recenter(self):
        self.host.recenter()
        if self.vr:
            self.vr.send(vrtest.CMD_RECENTER)

    def reset(self):
        self.host.reset()
        if self.vr:
            self.vr.send(vrtest.CMD_RESET)

    def close(self):
        if self.vr:
            self.vr.close()

    # ---- the axis maps ----

    def set_settings(self, s):
        """The dongle's settings (as Bridge.settings_get gives them) were read or saved: show its maps."""
        self.two_maps = s.get("imu_maps", 2) == 2
        for side in SIDES:
            self.saved[side] = Bridge.imu_map(s, side)
            self.maps[side] = to_logical(self.saved[side])
            self.host.set_map(side, self.saved[side])
        self.state = None
        self._refresh()

    def _edited(self):
        """Sides an edit applies to: the chosen controller, or both with a one-map firmware."""
        return [self.side_var.get()] if self.two_maps else list(SIDES)

    def pick(self, i, j):
        lm = list(self.maps[self.side_var.get()])
        src = [s for s, _ in lm]
        if src[i] == j:
            return
        if j in src:
            self.swap(i, src.index(j))
        else:  # the map had two axes on one source (set by hand): just take it
            lm[i] = (j, lm[i][1])
            self._change(lm)

    def swap(self, a, b):
        lm = list(self.maps[self.side_var.get()])
        lm[a], lm[b] = lm[b], lm[a]  # each source keeps its sign: what was right stays right where it goes
        self._change(lm)

    def invert(self, i):
        lm = list(self.maps[self.side_var.get()])
        lm[i] = (lm[i][0], -lm[i][1])
        self._change(lm)

    def copy(self):
        side = self.side_var.get()
        self._change(self.maps[side], [SIDES[1 - SIDES.index(side)]])

    def default(self):
        self._change(to_logical(DEFAULT_MAP))

    def revert(self):
        self._change(to_logical(self.saved[self.side_var.get()]))

    def _maps(self):
        return {side: from_logical(lm) for side, lm in self.maps.items()}

    def _change(self, lm, sides=None):
        sides = sides or self._edited()
        if all(self.maps[s] == lm for s in sides):
            self._refresh()
            return
        for side in sides:
            self.maps[side] = list(lm)
            self.host.set_map(side, from_logical(lm))
        self.state = ("Sending...", DIM)
        self._refresh()
        maps = self._maps()

        def done(ok):
            if ok and self.vr:
                self.vr.send(vrtest.CMD_RESET, sides)  # the dongle's orientation was in the old axes too
            if self._maps() == maps:
                self.state = None if ok else ("The dongle didn't take the change.", BAD)
                self._refresh()
        self._send(maps, False, done)

    def save(self):
        maps = self._maps()
        self.state = ("Saving...", DIM)
        self._refresh()

        def done(ok):
            if ok:
                self.saved = dict(maps)
                self.state = ("Saved on the dongle. Controllers reconnect (under a second).", OK)
            else:
                self.state = ("Saving failed: the dongle didn't answer.", BAD)
            self._refresh()
        self._send(maps, True, done)

    def _send(self, maps, persist, done):
        if self.demo:
            done(True)
        else:
            self.apply_maps(maps, persist, done)

    def _refresh(self):
        lm = self.maps[self.side_var.get()]
        for i, (j, sign) in enumerate(lm):
            self.src_vars[i].set(j)
            self.inv_vars[i].set(sign < 0)
        enabled = ["!disabled"] if self.two_maps else ["disabled"]
        for b in (*self.side_btns, self.copy_btn):
            b.state(enabled)
        hand = handedness(lm)
        src = [s for s, _ in lm]
        if hand == 0:
            names = [AXIS_NAMES[i] for i in range(3) if src.count(src[i]) > 1]
            warn, colour = f"{' and '.join(names)} read the same gyro axis: pick another one for one of them.", BAD
        elif hand < 0:
            warn, colour = ("Mirrored axes (an odd number of inversions and swaps): fine for a DS4 game's inverted "
                            "aim, but VR orientation can't follow it (gravity pulls the tilt back). For VR, invert or "
                            "swap one more axis."), WARN
        else:
            warn, colour = ("" if self.two_maps else ONE_MAP), DIM
        self.warn_var.set(warn)
        self.warn_lbl.configure(foreground=colour)
        if warn:  # the warning takes room only while there is one
            self.warn_lbl.pack(fill="x", pady=(6, 0), before=self.foot)
        else:
            self.warn_lbl.pack_forget()
        if self.state:
            text, colour = self.state
        elif self._maps() == self.saved:
            text, colour = "Saved on the dongle.", OK
        else:
            text, colour = "Live preview, not saved: Save to keep it after a restart.", WARN
        self.state_var.set(text)
        self.state_lbl.configure(foreground=colour)


def _selftest():
    assert DEFAULT_MAP == (0x00, 0x82, 0x01)
    assert to_logical(DEFAULT_MAP) == [(0, 1), (1, 1), (2, 1)]
    for m in ((0x00, 0x82, 0x01), (0x01, 0x82, 0x00), (0x80, 0x02, 0x81)):
        assert from_logical(to_logical(m)) == m, m
    assert handedness(to_logical(DEFAULT_MAP)) == 1
    assert handedness([(0, -1), (1, 1), (2, 1)]) == -1  # one axis inverted: a mirror
    assert handedness([(2, 1), (1, 1), (0, 1)]) == -1  # pitch <-> roll: a mirror
    assert handedness([(2, -1), (1, 1), (0, 1)]) == 1  # swap + one inversion: a rotation
    assert handedness([(0, 1), (0, 1), (2, 1)]) == 0
    # Fusion: level and still stays level; a steady yaw rate of 90 °/s for 1 s turns it 90° left...
    f = Fusion()
    for n in range(500):
        f.update((0.0, G, 0.0), (0.0, math.radians(90), 0.0), n * 2000)
    yaw, pitch, roll = vrtest.euler(f.q)
    assert abs(yaw - 90) < 1.5 and abs(pitch) < 0.5 and abs(roll) < 0.5, (yaw, pitch, roll)
    # ... recentring brings the heading back to 0...
    f.recenter()
    assert abs(vrtest.euler(f.q)[0]) < 0.5
    # ... and a start tipped up 30° reads pitch 30, yaw 0 (recentred).
    f.reset()
    t = math.radians(30)
    f.update((0.0, G * math.cos(t), -G * math.sin(t)), (0.0, 0.0, 0.0), 0)
    yaw, pitch, roll = vrtest.euler(f.q)
    assert abs(pitch - 30) < 0.5 and abs(yaw) < 0.5 and abs(roll) < 0.5, (yaw, pitch, roll)
    # The demo's quaternion follows vrtest.euler's convention (x, y, z, w).
    for side, d in DemoTracker().snapshot().items():
        assert abs(sum(v * v for v in d["quat"]) - 1) < 1e-9
    print("calib selftest ok")


if __name__ == "__main__":
    import sys

    if "--test" in sys.argv:
        _selftest()
    else:
        root = tk.Tk()
        root.title("QuestPad calibration (demo)")
        root.geometry("560x820")
        panel = CalibPanel(root, lambda maps, persist, done: done(True), demo=True)
        panel.pack(fill="both", expand=True)
        panel.set_mode("VR")
        root.mainloop()
