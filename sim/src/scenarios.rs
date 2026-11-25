//! Scripted scenarios: virtual controllers move, the firmware code builds reports, and the reports
//! are checked against the ground truth. Each scenario runs in its own process, so the firmware's
//! statics (calibration, fusion state) start fresh.

use crate::ctrl::Controller;
use crate::gamepad::Side;
use crate::math::{self, Quat, Vec3};
use crate::report::Report;
use crate::vr;

const SCENARIOS: &[(&str, &str, fn(&mut Run))] = &[
    ("level_start", "both controllers join lying level: orientation valid, matches truth", level_start),
    ("tilted_start", "joins pitched 30° and rolled -20°: tilt taken from gravity", tilted_start),
    ("turned_start", "joins turned 70°: heading recentred to 0, tilt kept", turned_start),
    ("rotations_right", "yaw / pitch / roll turns and 5 s of mixed motion, right hand", rotations_right),
    ("rotations_left", "same motion on the left hand (its IMU sits rotated 90°)", rotations_left),
    ("bias_calibration", "gyro bias 1.2 / -0.8 / 0.5 °/s, set down 3 s: calibrated, then 2 min still", bias_calibration),
    ("bias_while_moving", "gyro bias, waved around for 60 s, never set down: bias learnt online", bias_while_moving),
    ("bias_held_pose", "gyro bias, held in one pose with hand tremor for 60 s: tilt right, yaw limit", bias_held_pose),
    ("calibration_handover", "bias learnt while moving, then set down: at-rest calibration takes over", calibration_handover),
    ("shake", "3 s of 1.5 g shaking without turning: tilt not dragged along, recovers", shake),
    ("inputs", "buttons, touch, stick, trigger, grip, battery reach the report", inputs),
    ("recenter", "turn 60° and tip up 20°, then OUT 0x11: yaw 0, pitch kept", recenter),
    ("reset", "OUT 0x12: orientation restarts from gravity", reset),
    ("haptic", "OUT 0x10 parsing: amplitude, duration -> stop time, unknown commands ignored", haptic),
    ("disconnect", "right controller drops: its reports say disconnected, left unaffected", disconnect),
    ("report_rate", "one report per controller packet (500 Hz), sequence +1 each", report_rate),
    ("imu_units_reported", "controller with 4x finer gyro / 2x finer accel units, reported in 0x32", imu_units_reported),
    ("imu_units_missing", "the same controller without the 0x32 answer: why the read matters", imu_units_missing),
    ("imu_units_rejected", "implausible 0x32 answers are ignored", imu_units_rejected),
    ("imu_map_per_hand", "left axis map set apart (half turn about yaw): only the left reports change", imu_map_per_hand),
    ("stack_link_join", "stack: paired controller wakes, follows the hopping, joins; MIC, 0x32, VR reports", crate::stack::link_join),
    ("stack_pairing_button", "stack: blank dongle, button tap, DM pairing over the air, record stored, joins", crate::stack::pairing_button),
    ("stack_pairing_timeout", "stack: pairing with no controller gives up after 90 s", crate::stack::pairing_timeout),
    ("stack_pairing_cancel", "stack: a second tap cancels pairing, link resumes", crate::stack::pairing_cancel),
    ("stack_two_controllers", "stack: both hands join; VR reports, combined pad, DS4 / XInput / HID reports", crate::stack::two_controllers),
    ("stack_haptics", "stack: connect buzz, OUT haptic -> 0x97 downlink -> motor, acks", crate::stack::haptics),
    ("stack_power_off_gesture", "stack: Oculus held 3 s, set down: both power off, slots freed, rejoin", crate::stack::power_off_gesture),
    ("stack_mode_gesture", "stack: Oculus + trigger 5 s: next mode stored in flash, restart", crate::stack::mode_gesture),
    ("stack_button_mode", "stack: button held 3 s: next mode, restart, pairing kept", crate::stack::button_mode),
    ("stack_button_forget", "stack: button held 10 s: pairing forgotten, restart", crate::stack::button_forget),
    ("stack_serial", "stack: control panel protocol (ping, status, settings, export, slots, register read, rumble, stats, COBS, power off)", crate::stack::serial_protocol),
    ("stack_firmware_update", "stack: firmware update over serial: bad CRC refused, good one flashed", crate::stack::firmware_update),
    ("stack_wrong_key", "stack: a controller with another link key is never let in", crate::stack::wrong_key),
    ("stack_dropout", "stack: controller goes silent: slot freed, drop event, VR disconnected, rejoin", crate::stack::dropout),
    ("stack_low_battery", "stack: battery at the warning level: 0.8 s buzz on that controller", crate::stack::low_battery),
    ("stack_idle_off", "stack: idle setting 1 min: untouched controller powered off", crate::stack::idle_off),
    ("stack_watchdog", "stack: the watchdog task keeps feeding", crate::stack::watchdog),
    ("stack_vr_motion", "stack: turning controller, VR orientation through radio + link + fusion", crate::stack::vr_motion),
    ("stack_hid_mode", "stack: HID mode sends no VR reports, pad state flows", crate::stack::hid_mode),
    ("stack_sniff", "stack: raw radio from the host (POKE / RX / PEEK): advertisements sniffed", crate::stack::sniff),
    ("stack_dm_host", "stack: host-driven pairing (DM_START / DM_SEND): link, serial read, record written", crate::stack::dm_host),
];

/// Error budget: what each sensor imperfection costs (`budget`; measurements, not pass / fail).
const BUDGET: &[(&str, &str, fn(&mut Run))] = &[
    ("reference", "ideal sensor (bias and noise only)", |r| session(r, |_| {})),
    ("gyro_scale_1pct", "gyro scale +1 % on every axis", |r| session(r, |c| c.gyro_scale = [1.01; 3])),
    ("gyro_scale_mixed", "gyro scale +1 % / -1 % / +0.5 %", |r| session(r, |c| c.gyro_scale = [1.01, 0.99, 1.005])),
    ("misalign_1deg", "sensor axes rotated 1° from the controller's", |r| {
        session(r, |c| c.misalign = math::axis_angle([0.7071, 0.7071, 0.0], 1.0))
    }),
    ("accel_offset_10mg", "accelerometer offset 10 mg per axis", |r| session(r, |c| c.accel_offset_g = [0.01, -0.01, 0.01])),
    ("accel_offset_30mg", "accelerometer offset 30 / -20 / 25 mg", |r| {
        session(r, |c| c.accel_offset_g = [0.03, -0.02, 0.025])
    }),
    ("accel_scale_1pct", "accelerometer scale +1 % / -1 % / 0", |r| session(r, |c| c.accel_scale = [1.01, 0.99, 1.0])),
    ("typical_uncalibrated", "all of the above at typical consumer-MEMS levels", |r| {
        session(r, |c| {
            c.gyro_scale = [1.015, 0.985, 1.01];
            c.misalign = math::axis_angle([0.7071, 0.7071, 0.0], 1.0);
            c.accel_offset_g = [0.02, -0.015, 0.025];
            c.accel_scale = [1.01, 0.99, 1.005];
        })
    }),
];

pub struct Run {
    c: [Controller; 2],
    last: [Option<Report>; 2],
    count: [u32; 2],
    pub checks: Vec<(bool, String)>,
}

const L: usize = 0;
const R: usize = 1;

impl Run {
    fn new() -> Run {
        Run {
            c: [Controller::new(Side::Left, 1), Controller::new(Side::Right, 2)],
            last: [None; 2],
            count: [0; 2],
            checks: Vec::new(),
        }
    }

    /// One 2 ms link frame, both controllers turning at body rates `w`.
    fn frame(&mut self, w: [Vec3; 2]) {
        for i in 0..2 {
            self.c[i].step(w[i]);
        }
        crate::advance_us(crate::ctrl::FRAME_US);
        for i in 0..2 {
            if let Some(r) = self.c[i].take_report() {
                self.last[i] = Some(r);
                self.count[i] += 1;
            }
        }
    }

    fn still(&mut self, secs: f64) {
        for _ in 0..frames(secs) {
            self.frame([[0.0; 3]; 2]);
        }
    }

    /// Turn `side` at `dps` about a world axis (`world`) or its own axis, for `secs`; returns the
    /// largest orientation error seen.
    fn turn(&mut self, side: usize, axis: Vec3, dps: f64, secs: f64, world: bool) -> f64 {
        let mut worst: f64 = 0.0;
        for _ in 0..frames(secs) {
            let a = if world { math::rotate(math::conj(self.c[side].q), axis) } else { axis };
            let mut w = [[0.0; 3]; 2];
            w[side] = a.map(|x| x * dps.to_radians());
            self.frame(w);
            worst = worst.max(self.err(side));
        }
        worst
    }

    fn est(&self, side: usize) -> Quat {
        self.last[side].map_or(math::IDENTITY, |r| r.q)
    }

    fn err(&self, side: usize) -> f64 {
        math::angle_deg(self.est(side), self.c[side].q)
    }

    fn tilt(&self, side: usize) -> f64 {
        math::tilt_deg(self.est(side), self.c[side].q)
    }

    fn report(&self, side: usize) -> Report {
        self.last[side].expect("no report yet")
    }

    pub fn check(&mut self, ok: bool, what: String) {
        self.checks.push((ok, what));
    }

    fn below(&mut self, what: &str, value: f64, limit: f64) {
        self.check(value < limit, format!("{what} {value:.3}° (< {limit}°)"));
    }
}

fn frames(secs: f64) -> usize {
    (secs * 1e6 / crate::ctrl::FRAME_US as f64).round() as usize
}

// ---- scenarios ----

fn level_start(r: &mut Run) {
    r.c[L].join();
    r.c[R].join();
    r.still(1.0);
    for (i, name) in [(L, "left"), (R, "right")] {
        let rep = r.report(i);
        r.check(rep.has(vr::FLAG_CONNECTED) && rep.has(vr::FLAG_ORIENTATION), format!("{name}: connected + orientation valid"));
        r.check(rep.has(vr::FLAG_RIGHT) == (i == R), format!("{name}: right-hand flag {}", rep.has(vr::FLAG_RIGHT)));
        r.check((rep.norm() - 1.0).abs() < 1e-4, format!("{name}: unit quaternion (|q| = {:.6})", rep.norm()));
        let e = r.err(i);
        r.below(&format!("{name}: error"), e, 0.5);
    }
}

fn tilted_start(r: &mut Run) {
    r.c[R].q = math::from_ypr(0.0, 30.0, -20.0);
    r.c[R].join();
    r.still(0.5);
    let e = r.err(R);
    r.below("error", e, 1.0);
}

fn turned_start(r: &mut Run) {
    r.c[R].q = math::from_ypr(70.0, 10.0, 0.0);
    r.c[R].join();
    r.still(0.5);
    let yaw = math::ypr(r.est(R))[0];
    r.check(yaw.abs() < 0.5, format!("reported yaw {yaw:.2}° (recentred, |yaw| < 0.5°)"));
    let t = r.tilt(R);
    r.below("tilt error", t, 1.0);
}

fn rotations(r: &mut Run, s: usize) {
    r.c[s].join();
    rotation_checks(r, s);
}

fn rotation_checks(r: &mut Run, s: usize) {
    r.still(0.5);
    let mut worst = r.turn(s, [0.0, 1.0, 0.0], 90.0, 1.0, true);
    worst = worst.max(r.turn(s, [1.0, 0.0, 0.0], 45.0, 1.0, false));
    worst = worst.max(r.turn(s, [0.0, 0.0, 1.0], 60.0, 1.0, false));
    // Mixed motion: each axis swings at its own frequency, up to ~170 °/s.
    for n in 0..frames(5.0) {
        let t = n as f64 * 0.002;
        let mut w = [[0.0; 3]; 2];
        w[s] = [3.0 * (2.1 * t).sin(), 2.5 * (1.3 * t + 1.0).sin(), 2.0 * (3.7 * t + 2.0).sin()];
        r.frame(w);
        worst = worst.max(r.err(s));
    }
    r.still(1.0);
    r.below("worst error while moving", worst, 4.0);
    let e = r.err(s);
    r.below("final error", e, 2.0);
}

fn imu_map_per_hand(r: &mut Run) {
    // Left map = a half turn about y (x and z negated): a proper rotation, so its fusion still works.
    let mut s = crate::settings::get();
    s.imu_map_left = [0x80, 0x82, 0x81];
    crate::settings::set_volatile(&s);
    r.c[L].join();
    r.c[R].join();
    r.still(0.5);
    // Both pitch up about their own x at 45 °/s: the left reports it about -x, the right about +x.
    let w = [45f64.to_radians(), 0.0, 0.0];
    for _ in 0..frames(0.5) {
        r.frame([w, w]);
    }
    for (i, name, want) in [(L, "left (map applied)", -45.0), (R, "right (default map)", 45.0)] {
        let g = r.report(i).gyro.map(|v| (v as f64).to_degrees());
        let ok = (g[0] - want).abs() < 2.0 && g[1].abs() < 2.0 && g[2].abs() < 2.0;
        r.check(ok, format!("{name} gyro {:+.1} {:+.1} {:+.1} °/s (want {want:+.0} 0 0)", g[0], g[1], g[2]));
    }
    // A layout from before the left map (flash record, or an older panel over USB) sets both maps.
    let mut old = crate::settings::DEFAULT.to_bytes();
    old[12..15].copy_from_slice(&[0x01, 0x82, 0x80]);
    let parsed = crate::settings::Settings::from_bytes_over(&s, &old[..20]).unwrap();
    r.check(parsed.imu_map == [0x01, 0x82, 0x80] && parsed.imu_map_left == [0x01, 0x82, 0x80],
            "20-byte layout: its one map on both hands".into());
}

fn rotations_right(r: &mut Run) {
    rotations(r, R)
}

fn rotations_left(r: &mut Run) {
    rotations(r, L)
}

fn bias_calibration(r: &mut Run) {
    r.c[R].bias_dps = BIAS;
    r.c[R].join();
    r.still(3.0);
    let cal = r.report(R).has(vr::FLAG_GYRO_CAL);
    r.check(cal, "gyro-calibrated flag set after 3 s still".into());
    let yaw0 = math::yaw_diff_deg(r.est(R), r.c[R].q);
    r.still(120.0);
    let drift = math::yaw_diff_deg(r.est(R), r.c[R].q) - yaw0;
    r.below("yaw drift over 2 min after calibration", drift.abs(), 0.5);
    let t = r.tilt(R);
    r.below("tilt error", t, 0.5);
}

const BIAS: Vec3 = [1.2, -0.8, 0.5];

fn bias_norm() -> f64 {
    BIAS.iter().map(|b| b * b).sum::<f64>().sqrt()
}

/// Waves the right controller around (each axis swinging ±30–60°) for `secs`; returns the worst
/// tilt error after `settle` seconds and the yaw drift over the last 10 s.
fn wave(r: &mut Run, secs: f64, settle: f64) -> (f64, f64) {
    let mut worst_tilt: f64 = 0.0;
    let mut yaw_mark = None;
    for n in 0..frames(secs) {
        let t = n as f64 * 0.002;
        let mut w = [[0.0; 3]; 2];
        w[R] = [0.6 * (1.1 * t).sin(), 0.8 * (0.7 * t).sin(), 0.5 * (1.9 * t).sin()];
        r.frame(w);
        if t > settle {
            worst_tilt = worst_tilt.max(r.tilt(R));
        }
        if yaw_mark.is_none() && t >= secs - 10.0 {
            yaw_mark = Some(math::yaw_diff_deg(r.est(R), r.c[R].q));
        }
    }
    let drift = math::yaw_diff_deg(r.est(R), r.c[R].q) - yaw_mark.unwrap_or(0.0);
    (worst_tilt, drift)
}

fn bias_while_moving(r: &mut Run) {
    r.c[R].bias_dps = BIAS;
    r.c[R].join();
    let (tilt, drift) = wave(r, 60.0, 20.0);
    r.check(!r.report(R).has(vr::FLAG_GYRO_CAL), "not calibrated at rest (never still)".into());
    r.below("worst tilt error after 20 s", tilt, 0.5);
    let raw = bias_norm() * 10.0;
    r.check(drift.abs() < 1.0, format!(
        "yaw drift over the last 10 s {:.2}° (uncorrected bias: up to {raw:.0}°; < 1°)",
        drift.abs()
    ));
}

fn bias_held_pose(r: &mut Run) {
    r.c[R].bias_dps = BIAS;
    r.c[R].q = math::from_ypr(0.0, 20.0, 0.0);
    r.c[R].join();
    let start = r.c[R].q;
    let mut worst_tilt: f64 = 0.0;
    let mut yaw_mark = 0.0;
    for n in 0..frames(60.0) {
        let t = n as f64 * 0.002;
        // Hand tremor (a few °/s, too much for the at-rest calibration) around a fixed pose.
        let hold = math::rate_between(r.c[R].q, start, 0.5);
        let mut w = [[0.0; 3]; 2];
        w[R] = [0, 1, 2].map(|i| hold[i] + 0.05 * (7.0 * t + i as f64 * 2.0).sin());
        r.frame(w);
        if t > 20.0 {
            worst_tilt = worst_tilt.max(r.tilt(R));
        }
        if n == frames(40.0) {
            yaw_mark = math::yaw_diff_deg(r.est(R), r.c[R].q);
        }
    }
    let drift = math::yaw_diff_deg(r.est(R), r.c[R].q) - yaw_mark;
    // The bias about the vertical can't be seen through gravity: that part keeps turning the yaw.
    let up = math::rotate(math::conj(start), [0.0, 1.0, 0.0]);
    let vertical = (BIAS[0] * up[0] + BIAS[1] * up[1] + BIAS[2] * up[2]).abs() * 20.0;
    r.check(!r.report(R).has(vr::FLAG_GYRO_CAL), "not calibrated at rest (tremor)".into());
    r.below("worst tilt error after 20 s", worst_tilt, 0.3);
    r.check(drift.abs() < vertical * 1.2 + 0.5, format!(
        "yaw drift over the last 20 s {:.2}° (unobservable vertical bias part: {vertical:.1}°, physics limit)",
        drift.abs()
    ));
}

fn calibration_handover(r: &mut Run) {
    r.c[R].bias_dps = BIAS;
    r.c[R].join();
    wave(r, 30.0, 10.0);
    r.still(3.0);
    r.check(r.report(R).has(vr::FLAG_GYRO_CAL), "calibrated at rest after 3 s still".into());
    let yaw0 = math::yaw_diff_deg(r.est(R), r.c[R].q);
    r.still(60.0);
    let drift = math::yaw_diff_deg(r.est(R), r.c[R].q) - yaw0;
    r.below("yaw drift over the next minute", drift.abs(), 0.3);
    let t = r.tilt(R);
    r.below("tilt error", t, 0.3);
}

fn shake(r: &mut Run) {
    r.c[R].join();
    r.still(1.0);
    let mut worst: f64 = 0.0;
    for n in 0..frames(3.0) {
        let t = n as f64 * 0.002;
        let a = 1.5 * 9.80665 * (2.0 * std::f64::consts::PI * 3.0 * t).sin();
        r.c[R].lin_accel = [a, 0.3 * a, -0.7 * a];
        r.frame([[0.0; 3]; 2]);
        worst = worst.max(r.tilt(R));
    }
    r.c[R].lin_accel = [0.0; 3];
    r.below("worst tilt error while shaking", worst, 3.0);
    r.still(5.0);
    let t = r.tilt(R);
    r.below("tilt error 5 s after", t, 0.5);
}

fn inputs(r: &mut Run) {
    r.c[R].join();
    let c = &mut r.c[R];
    c.buttons = 0x0F;
    c.touch = 0x5A;
    c.stick = (12345, -23456);
    c.trigger = 3000;
    c.grip = 1000;
    c.battery = 42;
    r.still(0.1);
    let rep = r.report(R);
    r.check(rep.buttons == 0x0F, format!("buttons {:#04x} (0x0f)", rep.buttons));
    r.check(rep.touch == 0x5A, format!("touch {:#04x} (0x5a)", rep.touch));
    r.check(rep.stick == (12345, -23456), format!("stick {:?} ((12345, -23456))", rep.stick));
    r.check(rep.trigger == 3000 && rep.grip == 1000, format!("trigger {} grip {} (3000, 1000)", rep.trigger, rep.grip));
    r.check(rep.battery == 42, format!("battery {} (42)", rep.battery));
    r.c[R].buttons = 0x01;
    r.c[R].stick = (0, 0);
    r.still(0.01);
    let rep = r.report(R);
    r.check(rep.buttons == 0x01 && rep.stick == (0, 0), "changes show in the next report".into());
}

fn recenter(r: &mut Run) {
    r.c[R].join();
    r.still(0.5);
    r.turn(R, [0.0, 1.0, 0.0], 60.0, 1.0, true);
    r.turn(R, [1.0, 0.0, 0.0], 20.0, 1.0, false);
    let before = math::ypr(r.est(R));
    r.check((before[0] - 60.0).abs() < 2.0, format!("before: yaw {:.1}° (60°)", before[0]));
    let _ = vr::command(Side::Right, &[0x11]);
    r.still(0.01);
    let after = math::ypr(r.est(R));
    r.check(after[0].abs() < 0.5, format!("after: yaw {:.2}° (0°)", after[0]));
    r.check((after[1] - 20.0).abs() < 1.0, format!("after: pitch {:.2}° (20°)", after[1]));
}

fn reset(r: &mut Run) {
    r.c[R].join();
    r.still(0.5);
    r.turn(R, [0.0, 1.0, 0.0], 45.0, 1.0, true);
    let _ = vr::command(Side::Right, &[0x12]);
    r.still(0.01);
    let yaw = math::ypr(r.est(R))[0];
    r.check(r.report(R).has(vr::FLAG_ORIENTATION), "orientation valid again".into());
    r.check(yaw.abs() < 0.5, format!("yaw {yaw:.2}° (restarted at 0)"));
    let t = r.tilt(R);
    r.below("tilt error", t, 1.0);
}

/// The OUT command parser (what reaches the motor is checked end to end in `stack_haptics`).
fn haptic(r: &mut Run) {
    r.c[R].join();
    r.still(0.1);
    let now = embassy_time::Instant::now();
    let res = vr::command(Side::Right, &[0x10, 200, 100, 0]);
    let ok = matches!(res, Some(Some(t)) if t == now + embassy_time::Duration::from_millis(100));
    r.check(ok, "haptic 200 for 100 ms: stop time = now + 100 ms".into());
    let res = vr::command(Side::Right, &[0x10, 0, 0, 0]);
    r.check(matches!(res, Some(None)), "amplitude 0: no timer".into());
    let res = vr::command(Side::Right, &[0x10, 80]);
    r.check(matches!(res, Some(None)), "no duration: until the next command".into());
    let res = vr::command(Side::Right, &[0x7E, 1, 2]);
    r.check(res.is_none(), "unknown command ignored".into());
}

fn disconnect(r: &mut Run) {
    r.c[L].join();
    r.c[R].join();
    r.still(0.2);
    r.c[R].drop_link();
    r.still(0.2);
    let right = r.report(R);
    let left = r.report(L);
    r.check(!right.has(vr::FLAG_CONNECTED), "right: disconnected".into());
    r.check(left.has(vr::FLAG_CONNECTED) && left.has(vr::FLAG_ORIENTATION), "left: still connected".into());
    r.c[R].join();
    r.still(0.2);
    r.check(r.report(R).has(vr::FLAG_CONNECTED), "right: reconnects".into());
}

fn report_rate(r: &mut Run) {
    r.c[R].join();
    r.still(0.1);
    let start = r.count[R];
    let mut seq = r.report(R).seq;
    let mut gaps = 0;
    for _ in 0..frames(1.0) {
        r.frame([[0.0; 3]; 2]);
        let s = r.report(R).seq;
        if s != seq.wrapping_add(1) {
            gaps += 1;
        }
        seq = s;
    }
    let n = r.count[R] - start;
    r.check(n == 500, format!("{n} reports in 1 s (500)"));
    r.check(gaps == 0, format!("{gaps} sequence gaps"));
}

fn imu_units_reported(r: &mut Run) {
    r.c[R].gyro_lsb_per_dps = 32.768;
    r.c[R].accel_lsb_per_g = 2048.0;
    r.c[R].bias_dps = BIAS;
    r.c[R].join();
    let ok = r.c[R].answer_imu_config();
    r.check(ok, "0x32 answer accepted".into());
    r.still(3.0);
    let rep = r.report(R);
    r.check(rep.has(vr::FLAG_IMU_SCALE), "report flag 0x80 (units from the controller)".into());
    r.check(rep.has(vr::FLAG_GYRO_CAL), "at-rest calibration still works in the new units".into());
    rotation_checks(r, R);
}

fn imu_units_missing(r: &mut Run) {
    r.c[R].gyro_lsb_per_dps = 32.768;
    r.c[R].accel_lsb_per_g = 2048.0;
    r.c[R].join();
    r.still(0.5);
    r.turn(R, [0.0, 1.0, 0.0], 30.0, 1.0, true);
    let yaw = math::ypr(r.est(R))[0];
    r.check(!r.report(R).has(vr::FLAG_IMU_SCALE), "no 0x80 flag".into());
    r.check((yaw - 120.0).abs() < 5.0, format!("a 30° turn reads {yaw:.0}° (4x: the units are wrong without 0x32)"));
}

fn imu_units_rejected(r: &mut Run) {
    r.c[R].join();
    let good = r.c[R].imu_config();
    let mut bad = Vec::new();
    for (name, off, v) in [("zero g/count", 8, 0.0f32), ("NaN °/s/count", 12, f32::NAN), ("100 °/s/count", 12, 100.0)] {
        let mut d = good;
        d[off..off + 4].copy_from_slice(&v.to_le_bytes());
        if crate::gamepad::imu_config(r.c[R].slot, &d) {
            bad.push(name);
        }
    }
    let short = crate::gamepad::imu_config(r.c[R].slot, &good[..12]);
    r.still(0.1);
    r.check(bad.is_empty() && !short, format!("rejected: zero, NaN, out of range, short (accepted: {bad:?})"));
    r.check(!r.report(R).has(vr::FLAG_IMU_SCALE), "reference units kept".into());
}

/// Error-budget session: calibrate at rest, 5 full turns, pitch / roll sweeps, waving, rest.
fn session(r: &mut Run, setup: impl Fn(&mut Controller)) {
    setup(&mut r.c[R]);
    r.c[R].bias_dps = BIAS;
    r.c[R].join();
    r.still(2.0);
    // Start from yaw 0: the first second's drift (before the at-rest calibration) is not the point.
    let _ = vr::command(Side::Right, &[0x11]);
    r.still(0.5);
    r.turn(R, [0.0, 1.0, 0.0], 180.0, 10.0, true);
    r.still(0.5);
    let after_turns = math::yaw_diff_deg(r.est(R), r.c[R].q);
    let tilt_turns = r.tilt(R);
    let mut worst_tilt = tilt_turns;
    for (axis, deg) in [([1.0, 0.0, 0.0], 60.0), ([1.0, 0.0, 0.0], -120.0), ([1.0, 0.0, 0.0], 60.0),
                        ([0.0, 0.0, 1.0], 45.0), ([0.0, 0.0, 1.0], -90.0), ([0.0, 0.0, 1.0], 45.0)] {
        let secs = f64::abs(deg) / 60.0;
        r.turn(R, axis, 60.0 * f64::signum(deg), secs, false);
        worst_tilt = worst_tilt.max(r.tilt(R));
    }
    let (wt, _) = wave(r, 10.0, 0.0);
    worst_tilt = worst_tilt.max(wt);
    r.still(2.0);
    let yaw = math::yaw_diff_deg(r.est(R), r.c[R].q);
    let tilt = r.tilt(R);
    r.check(true, format!("yaw error after 5 turns (1800°) {after_turns:+6.2}°   tilt {tilt_turns:5.2}°"));
    r.check(true, format!("after sweeps + waving: yaw {yaw:+6.2}°   tilt {tilt:5.2}°   worst tilt {worst_tilt:5.2}°"));
}

// ---- runner ----

/// `None`: every scenario, each in a child process. `Some(name)`: just that one, here.
pub fn run(name: Option<&str>) -> i32 {
    run_list(SCENARIOS, "test", name)
}

/// The error budget: each case in a child process, measurements only.
pub fn budget(name: Option<&str>) -> i32 {
    run_list(BUDGET, "budget", name)
}

fn run_list(list: &[(&str, &str, fn(&mut Run))], cmd: &str, name: Option<&str>) -> i32 {
    if let Some(name) = name {
        let Some((_, _, f)) = list.iter().find(|s| s.0 == name) else {
            eprintln!("unknown {cmd} case {name}; known: {}", list.iter().map(|s| s.0).collect::<Vec<_>>().join(", "));
            return 2;
        };
        if !name.starts_with("stack_") {
            vr::ENABLED.store(true, core::sync::atomic::Ordering::Relaxed);
        }
        let mut r = Run::new();
        f(&mut r);
        let mut ok = true;
        for (pass, what) in &r.checks {
            println!("    {} {what}", if *pass { "ok  " } else { "FAIL" });
            ok &= pass;
        }
        return if ok { 0 } else { 1 };
    }
    let exe = std::env::current_exe().expect("own path");
    let mut failed = Vec::new();
    for (name, what, _) in list {
        let out = std::process::Command::new(&exe).args([cmd, name]).output().expect("run scenario");
        let pass = out.status.success();
        println!("{} {name}: {what}", if pass { "PASS" } else { "FAIL" });
        print!("{}", String::from_utf8_lossy(&out.stdout));
        eprint!("{}", String::from_utf8_lossy(&out.stderr));
        if !pass {
            failed.push(*name);
        }
    }
    println!();
    if failed.is_empty() {
        println!("all {} {} passed", list.len(), if cmd == "test" { "scenarios" } else { "cases" });
        0
    } else {
        println!("{} of {} failed: {}", failed.len(), list.len(), failed.join(", "));
        1
    }
}
