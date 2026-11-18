"""Virtual Touch controllers for testing VR mode without hardware: a 3D view and keyboard / mouse control
for the simulator (sim/, `cargo run --release -- live`), which runs the dongle's real firmware code.

    cd sim && cargo run --release -- live      # terminal 1
    python3 host/vrsim_view.py                 # terminal 2

Grey outline = where the virtual controller really is; coloured = the orientation the dongle reports.
Drag with the mouse to turn the selected controller (wheel = roll); keys below. Protocol: docs/vr.md.
"""
import math
import socket
import struct
import time
import tkinter as tk

VIEW = ("127.0.0.1", 26770)  # we listen here for reports
SIM = ("127.0.0.1", 26771)  # the simulator listens here for control
FLAGS = ((0x01, "conn"), (0x04, "ori"), (0x08, "gyro-cal"), (0x10, "standby"))
BUTTONS = ("A/X", "B/Y", "stick", "sys/menu")
HELP = ("Tab / 1 / 2  select controller   drag  turn (wheel: roll)   z  level both   h  buzz\n"
        "a b s m  buttons   t g  trigger / grip   arrows  stick   r  recentre both   x  reset   d  (dis)connect")


# ---- quaternions, (x, y, z, w) like the wire format ----

def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qaxis(axis, deg):
    h = math.radians(deg) / 2
    return (axis[0] * math.sin(h), axis[1] * math.sin(h), axis[2] * math.sin(h), math.cos(h))


def qnorm(q):
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return tuple(v / n for v in q)


def qrot(q, v):
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def ypr(q):
    x, y, z, w = q
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * x - y * z))))
    yaw = math.atan2(2 * (x * z + w * y), 1 - 2 * (x * x + y * y))
    roll = math.atan2(2 * (x * y + w * z), 1 - 2 * (x * x + z * z))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def qangle(a, b):
    d = min(1.0, abs(sum(x * y for x, y in zip(a, b))))
    return math.degrees(2 * math.acos(d))


IDENTITY = (0.0, 0.0, 0.0, 1.0)

# Controller model (metres, controller frame: x right, y up, -z forward).
BOX = [(sx * 0.025, sy * 0.02, sz * 0.06) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count("1") == 1]


class Hand:
    def __init__(self, name, pos, colour):
        self.name, self.pos, self.colour = name, pos, colour
        self.target = IDENTITY
        self.buttons = 0
        self.stick = [0, 0]
        self.trigger = self.grip = 0
        self.connected = True
        self.report = None
        self.truth = IDENTITY
        self.amp = 0
        self.seq = None
        self.count = 0
        self.hz = 0.0


class App:
    def __init__(self, root):
        self.root = root
        root.title("QuestPad VR simulator")
        root.configure(bg="#15171c")
        self.canvas = tk.Canvas(root, width=1120, height=480, bg="#15171c", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.info = tk.Label(root, font=("Consolas", 9), fg="#d8dee9", bg="#15171c", justify="left", anchor="w")
        self.info.pack(fill="x", padx=10)
        tk.Label(root, text=HELP, font=("Consolas", 9), fg="#7f8796", bg="#15171c", justify="left",
                 anchor="w").pack(fill="x", padx=10, pady=(2, 8))
        self.hands = [Hand("left", (-0.17, -0.1, -0.55), "#5aa9ff"), Hand("right", (0.17, -0.1, -0.55), "#ff9f43")]
        self.sel = 1
        self.keys = set()
        self.drag = None
        self.last_data = 0.0
        self.last_rate = time.monotonic()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(VIEW)
        self.sock.setblocking(False)
        root.bind("<KeyPress>", self.key_down)
        root.bind("<KeyRelease>", self.key_up)
        self.canvas.bind("<ButtonPress-1>", lambda e: setattr(self, "drag", (e.x, e.y)))
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<MouseWheel>", lambda e: self.turn_local((0, 0, 1), e.delta / 12))
        self.canvas.bind("<Button-4>", lambda e: self.turn_local((0, 0, 1), 10))
        self.canvas.bind("<Button-5>", lambda e: self.turn_local((0, 0, 1), -10))
        self.tick()

    # ---- input ----

    def send(self, i, text):
        self.sock.sendto(f"{'LR'[i]} {text}".encode(), SIM)

    def key_down(self, e):
        k = e.keysym.lower()
        h = self.hands[self.sel]
        if k == "tab":
            self.sel ^= 1
        elif k in ("1", "2"):
            self.sel = int(k) - 1
        elif k == "r":
            for i in range(2):
                self.send(i, "cmd=11")
        elif k == "x":
            self.send(self.sel, "cmd=12")
        elif k == "h":
            self.send(self.sel, "cmd=10c8c800")  # amplitude 200 for 200 ms
        elif k == "d":
            h.connected = not h.connected
        elif k == "z":
            for hand in self.hands:
                hand.target = IDENTITY
        self.keys.add(k)
        return "break"

    def key_up(self, e):
        self.keys.discard(e.keysym.lower())

    def on_drag(self, e):
        if self.drag:
            dx, dy = e.x - self.drag[0], e.y - self.drag[1]
            h = self.hands[self.sel]
            # Left / right = yaw about world up, up / down = pitch about the controller's own x.
            h.target = qnorm(qmul(qmul(qaxis((0, 1, 0), -dx * 0.4), h.target), qaxis((1, 0, 0), -dy * 0.4)))
        self.drag = (e.x, e.y)

    def turn_local(self, axis, deg):
        h = self.hands[self.sel]
        h.target = qnorm(qmul(h.target, qaxis(axis, deg)))

    def control(self):
        for i, h in enumerate(self.hands):
            if i == self.sel:
                k = self.keys
                h.buttons = sum(bit for bit, key in ((1, "a"), (2, "b"), (4, "s"), (8, "m")) if key in k)
                h.trigger = 4095 if "t" in k else 0
                h.grip = 4095 if "g" in k else 0
                h.stick = [32000 * (("right" in k) - ("left" in k)), 32000 * (("up" in k) - ("down" in k))]
            touch = (h.buttons & 0x03) | (0x04 if h.buttons & 0x04 or any(h.stick) else 0) | (0x08 if h.trigger else 0)
            q = ",".join(f"{v:.6f}" for v in h.target)
            self.send(i, f"q={q} b={h.buttons} t={touch} sx={h.stick[0]} sy={h.stick[1]} tr={h.trigger} "
                         f"gr={h.grip} c={int(h.connected)}")

    # ---- data from the simulator ----

    def receive(self):
        while True:
            try:
                d, _ = self.sock.recvfrom(256)
            except OSError:  # nothing waiting (or the simulator went away)
                return
            self.last_data = time.monotonic()
            if len(d) == 64 and d[0] == 0x01:
                h = self.hands[1 if d[2] & 0x02 else 0]
                r = dict(flags=d[2], seq=d[3], q=struct.unpack_from("<4f", d, 12),
                         stick=struct.unpack_from("<2h", d, 52), trigger=struct.unpack_from("<H", d, 56)[0],
                         grip=struct.unpack_from("<H", d, 58)[0], buttons=d[60], touch=d[61], battery=d[62])
                if r["seq"] != h.seq:
                    h.count += 1
                h.seq, h.report = r["seq"], r
            elif len(d) >= 19 and d[0] == 0xF0:
                h = self.hands[d[1] & 1]
                h.truth, h.amp = struct.unpack_from("<4f", d, 2), d[18]

    # ---- drawing ----

    def project(self, p):
        w, hgt = max(self.canvas.winfo_width(), 100), max(self.canvas.winfo_height(), 100)
        f = 0.9 * min(w, hgt * 1.6)
        z = min(p[2], -0.05)
        return w / 2 + f * p[0] / -z, hgt * 0.42 - f * p[1] / -z

    def draw_controller(self, h, q, colour, dash, width):
        def screen(v):  # a point of the controller model -> canvas
            return self.project(tuple(a + b for a, b in zip(qrot(q, v), h.pos)))

        pts = [screen(v) for v in BOX]
        for a, b in EDGES:
            self.canvas.create_line(*pts[a], *pts[b], fill=colour, width=width, dash=dash)
        self.canvas.create_line(*screen((0, 0, -0.06)), *screen((0, 0, -0.3)), fill=colour, width=width, dash=dash,
                                arrow="last")
        self.canvas.create_line(*self.project(h.pos), *screen((0, 0.05, 0)), fill=colour, width=width, dash=dash)

    def draw(self):
        c = self.canvas
        c.delete("all")
        for i in range(-5, 6):  # floor grid
            x = i * 0.1
            c.create_line(*self.project((x, -0.35, -0.2)), *self.project((x, -0.35, -1.4)), fill="#252932")
        for j in range(13):
            z = -0.2 - j * 0.1
            c.create_line(*self.project((-0.5, -0.35, z)), *self.project((0.5, -0.35, z)), fill="#252932")
        for i, h in enumerate(self.hands):
            self.draw_controller(h, h.truth, "#6b7280", (3, 3), 1)
            if h.report and h.report["flags"] & 0x01:
                self.draw_controller(h, h.report["q"], h.colour, None, 3 if i == self.sel else 2)
            x, y = self.project((h.pos[0], h.pos[1] + 0.13, h.pos[2]))
            label = h.name + ("  (selected)" if i == self.sel else "")
            c.create_text(x, y, text=label, fill=h.colour, font=("Segoe UI", 10, "bold" if i == self.sel else ""))
            if h.amp:
                c.create_text(x, y - 18, text=f"buzz {h.amp}", fill="#ff5c5c", font=("Segoe UI", 10, "bold"))
        if time.monotonic() - self.last_data > 1.0:
            c.create_text(c.winfo_width() / 2, 30, fill="#ff5c5c", font=("Segoe UI", 12, "bold"),
                          text="No data: start the simulator (cd sim && cargo run --release -- live)")

    def text(self):
        lines = []
        for h in self.hands:
            r = h.report
            if not r:
                lines.append(f"{h.name:>5}: -")
                continue
            flags = " ".join(n for b, n in FLAGS if r["flags"] & b) or "disconnected"
            fy, fp, fr = ypr(r["q"])
            ty, tp, tr = ypr(h.truth)
            btn = "+".join(n for i, n in enumerate(BUTTONS) if r["buttons"] & 1 << i) or "-"
            lines.append(f"{h.name:>5}  yaw/pitch/roll {fy:6.1f} {fp:5.1f} {fr:6.1f}  true {ty:6.1f} {tp:5.1f} {tr:6.1f}  "
                         f"err {qangle(r['q'], h.truth):4.1f}°  {h.hz:3.0f} Hz")
            lines.append(f"       {btn:<14} stick {r['stick'][0]:6d},{r['stick'][1]:6d}  trig {r['trigger']:4d}  "
                         f"grip {r['grip']:4d}  bat {r['battery']}%  {flags}")
        self.info.config(text="\n".join(lines))

    def tick(self):
        self.receive()
        self.control()
        now = time.monotonic()
        if now - self.last_rate >= 1.0:
            for h in self.hands:
                h.hz, h.count = h.count / (now - self.last_rate), 0
            self.last_rate = now
        self.draw()
        self.text()
        self.root.after(16, self.tick)


def main():
    root = tk.Tk()
    try:
        App(root)
    except OSError as e:
        raise SystemExit(f"can't listen on {VIEW[0]}:{VIEW[1]} ({e}). Is another viewer open?")
    root.mainloop()


if __name__ == "__main__":
    main()
