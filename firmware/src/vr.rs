//! VR mode: the two Touch controllers as two separate 3DoF tracked devices for an OpenXR runtime
//! (a Monado fork on Android: the dongle is plugged into the phone over USB OTG). See docs/vr.md
//! for the full protocol and monado/questpad_vr.h for a C parser.
//!
//! USB: one vendor-specific interface per controller (class 0xFF, subclass 0x51 'Q', protocol
//! 0x01 left / 0x02 right), each with an interrupt IN endpoint (64-byte state reports, one per
//! controller packet, i.e. 500 Hz) and an interrupt OUT endpoint (haptics, recentre). No kernel
//! driver binds to them, so Android's UsbManager / libusb (`libusb_wrap_sys_device`) can claim
//! them directly; on Windows they carry WinUSB MS OS 2.0 descriptors. The serial port for the
//! control panel stays, as in HID mode.
//!
//! Orientation is fused on the dongle (Mahony filter: gyro integrated at 500 Hz, tilt pulled
//! toward gravity while the accelerometer reads about 1 g; yaw drifts only by the residual gyro
//! bias, which `gamepad::Rest` measures whenever a controller lies still). Frames follow OpenXR:
//! world X right, Y up, Z toward the user (backward), yaw 0 = the controller's forward (-Z)
//! pointing at world -Z when it connected or was last recentred. Controller frame: the same axes
//! for a controller held naturally (the DS4 frame of `settings::imu_map`): X right, Y up, -Z
//! forward along the controller's pointing direction.
//!
//! IN report (64 bytes, little-endian):
//!   0      0x01 report type (state)
//!   1      protocol version (1)
//!   2      flags: b0 connected, b1 right hand, b2 orientation valid, b3 gyro bias calibrated,
//!          b4 standby, b5 dead battery, b6 fresh IMU sample in this report
//!   3      sequence number (wraps; increments on every new report)
//!   4..8   controller IMU timestamp, µs (controller clock, low 32 bits)
//!   8..12  dongle timestamp, µs, when the report was built (dongle clock, wraps)
//!   12..28 orientation quaternion f32 x, y, z, w (controller -> world)
//!   28..40 angular velocity f32 x, y, z, rad/s, controller frame (bias removed)
//!   40..52 linear acceleration (specific force) f32 x, y, z, m/s², controller frame (+1 g up at rest)
//!   52..56 thumbstick x, y i16 (+x right, +y up; ±32000 full, 0, 0 at rest)
//!   56..58 trigger u16 (0 released .. 4095 fully pulled)
//!   58..60 grip u16 (0 .. 4095)
//!   60     buttons: b0 A / X, b1 B / Y, b2 stick click, b3 system (Oculus, right) / menu (left)
//!   61     capacitive touch: b0 A / X, b1 B / Y, b2 stick, b3 trigger; b4..b7 finger near the same
//!   62     battery % (255 = unknown)
//!   63     reserved (0)
//!
//! OUT commands (first byte):
//!   0x10 haptic: amplitude u8 (0 = off), duration u16 ms (0 = until the next haptic command)
//!   0x11 recentre yaw: the controller's current heading becomes yaw 0
//!   0x12 reset orientation: start over from gravity (also recentres)

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::msos;
use embassy_usb::Builder;

use crate::gamepad::Side;

pub const REPORT_LEN: usize = 64;
pub const INTERFACE_SUBCLASS: u8 = 0x51;
pub const PROTOCOL_VERSION: u8 = 1;
/// Device release in VR mode, so hosts can tell it from HID mode (same VID:PID).
pub const DEVICE_RELEASE: u16 = 0x0300;

const REPORT_STATE: u8 = 0x01;
const CMD_HAPTIC: u8 = 0x10;
const CMD_RECENTER: u8 = 0x11;
const CMD_RESET: u8 = 0x12;

pub const FLAG_CONNECTED: u8 = 0x01;
pub const FLAG_RIGHT: u8 = 0x02;
pub const FLAG_ORIENTATION: u8 = 0x04;
pub const FLAG_GYRO_CAL: u8 = 0x08;
pub const FLAG_STANDBY: u8 = 0x10;
pub const FLAG_DEAD_BATTERY: u8 = 0x20;
pub const FLAG_IMU_FRESH: u8 = 0x40;

/// WinUSB device interface GUIDs (one per controller interface), for libusb / WinUSB on Windows.
const GUID_LEFT: &[&str] = &["{5A1C0F51-3D1E-4B6A-9C51-51E7A0D00001}"];
const GUID_RIGHT: &[&str] = &["{5A1C0F51-3D1E-4B6A-9C51-51E7A0D00002}"];

/// Set at boot when the dongle runs in VR mode: the radio side then feeds this module.
pub static ENABLED: AtomicBool = AtomicBool::new(false);

pub fn enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

/// Latest report per side (0 left, 1 right).
static REPORTS: [Signal<CriticalSectionRawMutex, [u8; REPORT_LEN]>; 2] = [Signal::new(), Signal::new()];

/// Requests from the USB side to the fusion (radio interrupt side): bit 0 recentre, bit 1 reset.
static REQUESTS: [AtomicU8; 2] = [AtomicU8::new(0), AtomicU8::new(0)];
const REQ_RECENTER: u8 = 0x01;
const REQ_RESET: u8 = 0x02;

// ---- USB ----

pub struct Endpoints<D: embassy_usb::driver::Driver<'static>> {
    pub left_in: D::EndpointIn,
    pub left_out: D::EndpointOut,
    pub right_in: D::EndpointIn,
    pub right_out: D::EndpointOut,
}

/// Call before adding any function (the MS OS descriptor set is device-wide).
pub fn configure(cfg: &mut embassy_usb::Config) {
    cfg.device_release = DEVICE_RELEASE;
}

pub fn build<D: embassy_usb::driver::Driver<'static>>(builder: &mut Builder<'static, D>) -> Endpoints<D> {
    builder.msos_descriptor(msos::windows_version::WIN8_1, 0);
    let mut iface = |protocol: u8, guids: &'static [&'static str]| {
        let mut func = builder.function(0xFF, INTERFACE_SUBCLASS, protocol);
        func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
        func.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
            "DeviceInterfaceGUIDs",
            msos::PropertyData::RegMultiSz(guids),
        ));
        let mut i = func.interface();
        let mut alt = i.alt_setting(0xFF, INTERFACE_SUBCLASS, protocol, None);
        let ep_in = alt.endpoint_interrupt_in(None, REPORT_LEN as u16, 1);
        let ep_out = alt.endpoint_interrupt_out(None, REPORT_LEN as u16, 4);
        (ep_in, ep_out)
    };
    let (left_in, left_out) = iface(0x01, GUID_LEFT);
    let (right_in, right_out) = iface(0x02, GUID_RIGHT);
    Endpoints { left_in, left_out, right_in, right_out }
}

/// Stream state reports for `side`: every new one, else the last one again every 100 ms (so a
/// host sees a disconnected or sleeping controller; an unchanged sequence number = no new data).
pub async fn run_in<E: EndpointIn>(side: Side, mut ep: E) -> ! {
    let mut last = disconnected_report(side, 0);
    loop {
        ep.wait_enabled().await;
        if let Ok(r) = with_timeout(Duration::from_millis(100), REPORTS[side as usize].wait()).await {
            last = r;
        } else {
            last[8..12].copy_from_slice(&now_us().to_le_bytes());
        }
        let _ = ep.write(&last).await;
    }
}

/// Commands for `side`'s controller from the host.
pub async fn run_out<E: EndpointOut>(side: Side, mut ep: E) -> ! {
    let mut buf = [0u8; REPORT_LEN];
    let mut stop_at: Option<Instant> = None;
    loop {
        ep.wait_enabled().await;
        let ev = match stop_at {
            Some(t) => select(ep.read(&mut buf), Timer::at(t)).await,
            None => Either::First(ep.read(&mut buf).await),
        };
        let n = match ev {
            Either::First(Ok(n)) => n,
            Either::First(Err(_)) => continue,
            Either::Second(()) => {
                stop_at = None;
                crate::link::rumble_side(side == Side::Right, 0);
                continue;
            }
        };
        match &buf[..n] {
            &[CMD_HAPTIC, amp, ref rest @ ..] => {
                let ms = if rest.len() >= 2 { u16::from_le_bytes([rest[0], rest[1]]) } else { 0 };
                crate::link::rumble_side(side == Side::Right, amp);
                stop_at = (amp != 0 && ms != 0).then(|| Instant::now() + Duration::from_millis(ms as u64));
            }
            &[CMD_RECENTER, ..] => {
                REQUESTS[side as usize].fetch_or(REQ_RECENTER, Ordering::Relaxed);
            }
            &[CMD_RESET, ..] => {
                REQUESTS[side as usize].fetch_or(REQ_RESET, Ordering::Relaxed);
            }
            _ => {}
        }
    }
}

fn now_us() -> u32 {
    Instant::now().as_micros() as u32
}

// ---- reports (built on the radio interrupt side) ----

/// One controller's inputs, as `gamepad` keeps them.
pub struct Input {
    pub buttons: u8,
    pub touch: u8,
    pub stick: (i16, i16),
    pub trigger: u16,
    pub grip: u16,
    pub battery: Option<u8>,
    pub status: u8,
    pub imu_ts: u32,
    pub gyro_calibrated: bool,
}

fn disconnected_report(side: Side, seq: u8) -> [u8; REPORT_LEN] {
    let mut r = [0u8; REPORT_LEN];
    r[0] = REPORT_STATE;
    r[1] = PROTOCOL_VERSION;
    r[2] = if side == Side::Right { FLAG_RIGHT } else { 0 };
    r[3] = seq;
    r[8..12].copy_from_slice(&now_us().to_le_bytes());
    // Identity orientation.
    r[24..28].copy_from_slice(&1.0f32.to_le_bytes());
    r[62] = 255;
    r
}

/// A controller of `side` joined: start its orientation over from gravity.
pub fn joined(side: Side) {
    fusion(side).init = false;
}

/// The controller of `side` left: report it disconnected.
pub fn dropped(side: Side) {
    let f = fusion(side);
    f.init = false;
    f.seq = f.seq.wrapping_add(1);
    REPORTS[side as usize].signal(disconnected_report(side, f.seq));
}

/// One IMU sample, in the right controller's IMU axes (after `left_to_right_frame`), bias removed:
/// accel 1024 LSB/g, gyro 8.192 LSB/(deg/s); `ts` the controller's µs timestamp.
pub fn imu(side: Side, accel: [i16; 3], gyro: [i16; 3], ts: u32) {
    let (gyro, accel) = to_openxr(gyro, accel);
    const G: f32 = 9.80665;
    const GYRO_SCALE: f32 = core::f32::consts::PI / 180.0 / 8.192;
    const ACCEL_SCALE: f32 = G / 1024.0;
    let w = gyro.map(|v| v as f32 * GYRO_SCALE);
    let a = accel.map(|v| v as f32 * ACCEL_SCALE);
    let f = fusion(side);
    let req = REQUESTS[side as usize].swap(0, Ordering::Relaxed);
    if req & REQ_RESET != 0 {
        f.init = false;
    }
    f.update(a, w, ts);
    if req & REQ_RECENTER != 0 && f.init {
        f.recenter();
    }
    f.gyro = w;
    f.accel = a;
    f.fresh = true;
}

/// Publish `side`'s state after a controller packet.
pub fn publish(side: Side, inp: &Input) {
    let f = fusion(side);
    f.seq = f.seq.wrapping_add(1);
    let mut r = [0u8; REPORT_LEN];
    r[0] = REPORT_STATE;
    r[1] = PROTOCOL_VERSION;
    let mut flags = FLAG_CONNECTED;
    let set = |flags: &mut u8, on: bool, bit: u8| {
        if on {
            *flags |= bit;
        }
    };
    set(&mut flags, side == Side::Right, FLAG_RIGHT);
    set(&mut flags, f.init, FLAG_ORIENTATION);
    set(&mut flags, inp.gyro_calibrated, FLAG_GYRO_CAL);
    set(&mut flags, inp.status & 0x10 != 0, FLAG_STANDBY);
    set(&mut flags, inp.status & 0x01 != 0, FLAG_DEAD_BATTERY);
    set(&mut flags, f.fresh, FLAG_IMU_FRESH);
    f.fresh = false;
    r[2] = flags;
    r[3] = f.seq;
    r[4..8].copy_from_slice(&inp.imu_ts.to_le_bytes());
    r[8..12].copy_from_slice(&now_us().to_le_bytes());
    // Internal order w, x, y, z -> wire order x, y, z, w.
    let q = if f.init { f.q } else { [1.0, 0.0, 0.0, 0.0] };
    for (i, v) in [q[1], q[2], q[3], q[0]].iter().enumerate() {
        r[12 + 4 * i..16 + 4 * i].copy_from_slice(&v.to_le_bytes());
    }
    for i in 0..3 {
        r[28 + 4 * i..32 + 4 * i].copy_from_slice(&f.gyro[i].to_le_bytes());
        r[40 + 4 * i..44 + 4 * i].copy_from_slice(&f.accel[i].to_le_bytes());
    }
    r[52..54].copy_from_slice(&inp.stick.0.to_le_bytes());
    r[54..56].copy_from_slice(&inp.stick.1.to_le_bytes());
    r[56..58].copy_from_slice(&inp.trigger.to_le_bytes());
    r[58..60].copy_from_slice(&inp.grip.to_le_bytes());
    r[60] = inp.buttons & 0x0F;
    r[61] = inp.touch;
    r[62] = inp.battery.unwrap_or(255);
    REPORTS[side as usize].signal(r);
}

/// Controller IMU (right controller's axes) -> OpenXR controller frame, with the DS4 axis map
/// from settings (x right, y up, z toward the user for a controller held naturally).
fn to_openxr(gyro: [i16; 3], accel: [i16; 3]) -> ([i16; 3], [i16; 3]) {
    let m = crate::settings::get().imu_map;
    let pick = |v: &[i16; 3], i: usize| {
        let x = v[((m[i] & 0x03) as usize).min(2)];
        if m[i] & 0x80 != 0 { x.saturating_neg() } else { x }
    };
    ([pick(&gyro, 0), pick(&gyro, 1), pick(&gyro, 2)], [pick(&accel, 0), pick(&accel, 1), pick(&accel, 2)])
}

// ---- orientation fusion ----

/// Proportional gain pulling the tilt toward gravity, rad/s per unit of error (~2 s time constant).
const KP: f32 = 0.5;
/// Accelerometer magnitude window (fraction of 1 g) inside which it is trusted as gravity.
const GRAVITY_TOLERANCE: f32 = 0.1;

#[derive(Clone, Copy)]
struct Fusion {
    /// Controller -> world, w x y z.
    q: [f32; 4],
    init: bool,
    last_ts: u32,
    /// Last sample, OpenXR controller frame: rad/s, m/s².
    gyro: [f32; 3],
    accel: [f32; 3],
    fresh: bool,
    seq: u8,
}

const FUSION: Fusion =
    Fusion { q: [1.0, 0.0, 0.0, 0.0], init: false, last_ts: 0, gyro: [0.0; 3], accel: [0.0; 3], fresh: false, seq: 0 };

/// Per side (only touched from the radio-side interrupts, like `gamepad::HANDS`).
static mut FUSIONS: [Fusion; 2] = [FUSION; 2];

fn fusion(side: Side) -> &'static mut Fusion {
    unsafe { &mut (*core::ptr::addr_of_mut!(FUSIONS))[side as usize] }
}

impl Fusion {
    fn update(&mut self, a: [f32; 3], w: [f32; 3], ts: u32) {
        const G: f32 = 9.80665;
        let an = norm(a);
        if !self.init {
            // Start level with gravity, facing world -Z.
            if an < 0.5 * G {
                return;
            }
            self.q = from_up(scale(a, 1.0 / an));
            self.init = true;
            self.last_ts = ts;
            self.recenter();
            return;
        }
        let dt_us = ts.wrapping_sub(self.last_ts);
        self.last_ts = ts;
        let dt = if dt_us == 0 || dt_us > 50_000 { 0.002 } else { dt_us as f32 * 1e-6 };
        let mut w = w;
        if abs(an - G) < GRAVITY_TOLERANCE * G {
            // Measured up (accel) vs estimated up (world +Y in the controller frame).
            let up = rotate(conj(self.q), [0.0, 1.0, 0.0]);
            let e = cross(scale(a, 1.0 / an), up);
            w = [w[0] + KP * e[0], w[1] + KP * e[1], w[2] + KP * e[2]];
        }
        // q += dt / 2 * q ⊗ (0, w)
        let [qw, qx, qy, qz] = self.q;
        let h = 0.5 * dt;
        let dq = [
            -qx * w[0] - qy * w[1] - qz * w[2],
            qw * w[0] + qy * w[2] - qz * w[1],
            qw * w[1] - qx * w[2] + qz * w[0],
            qw * w[2] + qx * w[1] - qy * w[0],
        ];
        self.q = normalize([qw + h * dq[0], qx + h * dq[1], qy + h * dq[2], qz + h * dq[3]]);
    }

    /// Remove the heading: the controller's forward (-Z) projected on the floor becomes world -Z.
    fn recenter(&mut self) {
        let f = rotate(self.q, [0.0, 0.0, -1.0]);
        let len = sqrt(f[0] * f[0] + f[2] * f[2]);
        if len < 0.2 {
            // Pointing (almost) straight up or down: the heading is undefined, use the top (+Y),
            // which faces away from the user when pointing down and toward them when pointing up.
            let u = rotate(self.q, [0.0, 1.0, 0.0]);
            let s = if f[1] < 0.0 { 1.0 } else { -1.0 };
            return self.apply_heading([s * u[0], 0.0, s * u[2]]);
        }
        self.apply_heading(f);
    }

    /// Rotate about world Y so the horizontal direction `f` maps to world -Z.
    fn apply_heading(&mut self, f: [f32; 3]) {
        let len = sqrt(f[0] * f[0] + f[2] * f[2]);
        if len < 1e-3 {
            return;
        }
        let (fx, fz) = (f[0] / len, f[2] / len);
        // Shortest rotation from (fx, 0, fz) to (0, 0, -1): w = 1 + dot, xyz = cross.
        let yaw = if 1.0 - fz < 1e-4 { [0.0, 0.0, 1.0, 0.0] } else { normalize([1.0 - fz, 0.0, fx, 0.0]) };
        self.q = normalize(mul(yaw, self.q));
    }
}

/// Smallest rotation taking the measured up `u` (unit, controller frame) to world +Y.
fn from_up(u: [f32; 3]) -> [f32; 4] {
    let w = 1.0 + u[1];
    if w < 1e-4 {
        return [0.0, 1.0, 0.0, 0.0]; // upside down: 180° about X
    }
    normalize([w, -u[2], 0.0, u[0]])
}

fn mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn conj(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// v' = q v q*.
fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let r = [q[1], q[2], q[3]];
    let t = scale(cross(r, v), 2.0);
    let c = cross(r, t);
    [v[0] + q[0] * t[0] + c[0], v[1] + q[0] * t[1] + c[1], v[2] + q[0] * t[2] + c[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn scale(v: [f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn norm(v: [f32; 3]) -> f32 {
    sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

fn normalize(q: [f32; 4]) -> [f32; 4] {
    let n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if n2 < 1e-12 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let k = inv_sqrt(n2);
    [q[0] * k, q[1] * k, q[2] * k, q[3] * k]
}

fn abs(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

/// 1 / sqrt(x) for x > 0: bit-trick estimate, then Newton steps (core has no f32::sqrt).
fn inv_sqrt(x: f32) -> f32 {
    let mut y = f32::from_bits(0x5F37_59DF - (x.to_bits() >> 1));
    for _ in 0..3 {
        y *= 1.5 - 0.5 * x * y * y;
    }
    y
}

fn sqrt(x: f32) -> f32 {
    if x <= 0.0 { 0.0 } else { x * inv_sqrt(x) }
}
