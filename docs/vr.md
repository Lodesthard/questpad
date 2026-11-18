# VR mode (3DoF controllers for Monado)

In **VR mode** the dongle stops being a gamepad. It presents the two Touch controllers as two separate tracked
devices for an OpenXR runtime: a fork of [Monado](https://monado.freedesktop.org/) running on an Android phone,
with the dongle (nRF52840 Dongle or nice!nano) plugged into the phone's USB-C port through an OTG adapter. Each
controller sends:

- its **orientation** (3DoF), fused on the dongle from the gyro and accelerometer, as a quaternion,
- its **angular velocity** and **acceleration**, for pose prediction (or for the runtime's own fusion),
- its **inputs**: thumbstick, trigger, grip, A/B or X/Y, stick click, Oculus/Menu, capacitive touch, battery.

The runtime can send **haptics** (amplitude and duration) and **recentre** each controller.

There is no position. The runtime places the controllers with an arm or neck model (as with Daydream or Go
controllers).

## Turn it on
1. Pair the controllers as usual (see the README).
2. Switch to **VR** in the control panel, or step through the modes: hold the dongle's button for 3 s, or hold
   Oculus / Menu with the trigger pulled for 5 s, until you reach VR (order: HID → DS4 → XInput → VR → HID). The
   dongle's LED turns **orange** (nice!nano: 4 blinks at power-up).
3. Plug the dongle into the phone. It runs on its own: pairing, power-off gesture, idle switch-off and low-battery
   buzzes work as in the other modes.

Set the controllers down for a second now and then (before you start is enough): the dongle measures each gyro's
drift while they lie still, which keeps the yaw from wandering. A report's `gyro calibrated` flag says whether
that has happened since power-up.

If the controllers switch off whenever the phone's screen goes off, the phone is suspending its USB port: untick
*Turn controllers off when the computer sleeps* in the control panel's settings (the dongle remembers it).

### Checking it on a PC
`host/vrtest.py` reads both controllers the way a runtime does and prints their orientation and inputs:

```sh
pip install pyusb libusb-package
python3 host/vrtest.py               # live view, ~500 Hz per controller
python3 host/vrtest.py --buzz right
python3 host/vrtest.py --recenter
```

Held naturally and pointed at the screen after `--recenter`, a controller reads yaw 0, pitch 0, roll 0. Turning left
increases yaw, tipping the front up increases pitch and rolling left increases roll. On Linux add a udev rule for
`1209:5151` (or run it as root). On Windows the VR interfaces get WinUSB by themselves.

## USB interface
VR mode keeps the usual ids, `1209:5151`, with **bcdDevice 0x0300** and product string `QuestPad VR`.
The device is a composite of:

| interface | class / subclass / protocol | endpoints | |
|---|---|---|---|
| 0 | `FF / 51 / 01` | interrupt IN (64 B, 1 ms) + interrupt OUT (64 B) | **left** controller |
| 1 | `FF / 51 / 02` | interrupt IN (64 B, 1 ms) + interrupt OUT (64 B) | **right** controller |
| 2, 3 | CDC ACM | | serial port for the control panel (as in HID mode) |

Find the controller interfaces by class / subclass / protocol, not by number. No kernel driver binds to a vendor
interface, so on Android an app can claim them through `UsbManager` without root; on Windows they carry MS OS 2.0
descriptors (WinUSB, device interface GUIDs `{5A1C0F51-3D1E-4B6A-9C51-51E7A0D00001}` left and `…0002}` right).

Each IN endpoint sends one report per controller packet (every 2 ms, so ~500 Hz) while that controller is connected.
When nothing new comes in, it sends the last report again every 100 ms with the same sequence number and a fresh
dongle timestamp (so a disconnected or sleeping controller is still visible).

## State report (IN, 64 bytes, little-endian)
| bytes | type | content |
|---|---|---|
| 0 | u8 | report type, `0x01` = state |
| 1 | u8 | protocol version, `1` |
| 2 | u8 | flags: `0x01` connected, `0x02` right hand, `0x04` orientation valid, `0x08` gyro bias calibrated, `0x10` standby, `0x20` dead battery, `0x40` fresh IMU sample |
| 3 | u8 | sequence number, +1 per new report (wraps) |
| 4..8 | u32 | controller IMU timestamp, µs (controller clock, wraps) |
| 8..12 | u32 | dongle timestamp when the report was built, µs (dongle clock, wraps) |
| 12..28 | f32 × 4 | orientation quaternion **x, y, z, w**, controller → world |
| 28..40 | f32 × 3 | angular velocity, rad/s, **controller frame**, gyro bias removed |
| 40..52 | f32 × 3 | acceleration (specific force), m/s², controller frame: +9.81 on y when level and still |
| 52..56 | i16 × 2 | thumbstick x, y: +x right, +y up, ±32000 full, exactly 0, 0 at rest |
| 56..58 | u16 | trigger, 0 released .. 4095 fully pulled |
| 58..60 | u16 | grip, 0 .. 4095 |
| 60 | u8 | buttons: `0x01` A / X, `0x02` B / Y, `0x04` stick click, `0x08` Oculus (right) / Menu (left) |
| 61 | u8 | touch: `0x01` A / X, `0x02` B / Y, `0x04` stick, `0x08` trigger; `0x10`..`0x80` = finger *near* the same four |
| 62 | u8 | battery %, 255 = unknown |
| 63 | u8 | reserved, 0 |

**Frames** follow OpenXR. World: X right, Y up, Z backward (toward the user). Yaw 0 means the controller pointed at
world −Z when it connected or was last recentred. Controller frame, for a controller held naturally: X right, Y up,
−Z forward (the direction it points). Angular velocity is in the controller frame; rotate it by the orientation to
get it in world space (`ω_world = q · ω · q⁻¹`).

**Fusion.** A Mahony filter on the dongle integrates the gyro at 500 Hz and pulls the tilt toward gravity whenever
the accelerometer reads within 10 % of 1 g (gain 0.5 rad/s, about a 2 s time constant). Roll and pitch are
absolute. Yaw has no reference, so it drifts with whatever gyro bias is left after calibration (not yet measured
on hardware: recentre now and then). When a controller (re)connects, its orientation starts level with gravity and facing −Z.

## Commands (OUT)
| bytes | command |
|---|---|
| `10 aa dd dd` | **haptic**: amplitude `aa` (0 = off, 255 = full), duration `dddd` ms (u16; 0 = until the next haptic command) |
| `11` | **recentre**: the controller's current heading becomes yaw 0 (pitch and roll are kept) |
| `12` | **reset**: start the orientation over from gravity (also recentres) |

The Touch motors have one strength setting and no frequency, so a runtime's frequency is ignored. The dongle's own
buzzes (connect, power-off gesture, low battery) briefly take priority over a haptic command.

## Writing the Monado driver
[`monado/questpad_vr.h`](../monado/questpad_vr.h) parses the state report and builds the commands, in plain C with
no Monado headers. A driver in the fork would typically:

1. **Open the device.** On Android, ask `UsbManager` for permission on `1209:5151` (a `device_filter.xml` with that
   VID/PID lets the system offer to open the app when the dongle is plugged in), open it, and hand the file
   descriptor to native code. There, `libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY)`, then
   `libusb_wrap_sys_device(ctx, fd, &handle)`. Claim the two interfaces with subclass `0x51`. On Linux desktop,
   `libusb_open_device_with_vid_pid` works directly.
2. **Create one `xrt_device` per interface** (protocol `0x01` → left hand, `0x02` → right). A device class such as
   `XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER` / `RIGHT_HAND_CONTROLLER` with the Touch controller interaction profile
   (`XRT_DEVICE_TOUCH_CONTROLLER`) fits the inputs one to one.
3. **Run a reader thread per interface**: `libusb_interrupt_transfer` on the IN endpoint (64 bytes), then
   `questpad_vr_parse`. Skip reports whose `seq` did not change. Push each new orientation into an
   `m_relation_history` stamped with the host time of arrival (minus a few ms of radio and USB latency, to be measured). Mark the
   relation orientation-valid only when `QUESTPAD_VR_FLAG_ORIENTATION` is set and the device is connected.
4. **Map the inputs**:

   | report | OpenXR (Touch profile) |
   |---|---|
   | thumbstick x, y | `/input/thumbstick` |
   | buttons `0x04` | `/input/thumbstick/click` |
   | touch `0x04` | `/input/thumbstick/touch` |
   | trigger | `/input/trigger/value` |
   | touch `0x08` | `/input/trigger/touch` |
   | grip | `/input/squeeze/value` |
   | buttons `0x01` / `0x02` | right `/input/a/click`, `/input/b/click`; left `/input/x/click`, `/input/y/click` |
   | touch `0x01` / `0x02` | the same buttons' `/touch` |
   | buttons `0x08` | right `/input/system/click`; left `/input/menu/click` |
   | — | `/input/grip/pose`, `/input/aim/pose`: the orientation, placed by the arm model |
   | — | `/output/haptic` → `questpad_vr_build_haptic` |

5. **Place the controllers** with Monado's 3DoF helpers (as for other orientation-only controllers): a fixed
   offset from the head, rotated by the head's yaw, then the controller orientation applied about the wrist.
6. **Recentre** both controllers together with the head (e.g. when the user long-presses Oculus or the runtime
   recentres): send `11` to both OUT endpoints and reset the head's yaw at the same moment, so world −Z matches the
   direction the user faces.

If the fork prefers its own fusion (`m_imu_3dof`), feed it `angular_velocity` and `acceleration` (both already in
the controller frame and in SI units) with `imu_ts_us` as the sample time, and ignore the quaternion.
