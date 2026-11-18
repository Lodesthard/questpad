//! The 64-byte VR state report, parsed the way a runtime does (same fields as monado/questpad_vr.h).

use crate::gamepad::Side;
use crate::math::Quat;

#[derive(Clone, Copy, Debug, Default)]
pub struct Report {
    pub flags: u8,
    pub seq: u8,
    pub imu_ts: u32,
    pub dongle_ts: u32,
    /// w, x, y, z (the wire order is x, y, z, w).
    pub q: Quat,
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub stick: (i16, i16),
    pub trigger: u16,
    pub grip: u16,
    pub buttons: u8,
    pub touch: u8,
    pub battery: u8,
}

impl Report {
    /// The latest report the firmware built for `side`, if a new one is waiting.
    pub fn take(side: Side) -> Option<Report> {
        crate::vr::REPORTS[side as usize].try_take().and_then(|r| Report::parse(&r))
    }

    pub fn parse(r: &[u8; 64]) -> Option<Report> {
        if r[0] != 0x01 || r[1] != crate::vr::PROTOCOL_VERSION {
            return None;
        }
        let f = |o: usize| f32::from_le_bytes([r[o], r[o + 1], r[o + 2], r[o + 3]]);
        let u16_at = |o: usize| u16::from_le_bytes([r[o], r[o + 1]]);
        let u32_at = |o: usize| u32::from_le_bytes([r[o], r[o + 1], r[o + 2], r[o + 3]]);
        Some(Report {
            flags: r[2],
            seq: r[3],
            imu_ts: u32_at(4),
            dongle_ts: u32_at(8),
            q: [f(24) as f64, f(12) as f64, f(16) as f64, f(20) as f64],
            gyro: [f(28), f(32), f(36)],
            accel: [f(40), f(44), f(48)],
            stick: (u16_at(52) as i16, u16_at(54) as i16),
            trigger: u16_at(56),
            grip: u16_at(58),
            buttons: r[60],
            touch: r[61],
            battery: r[62],
        })
    }

    pub fn has(&self, flag: u8) -> bool {
        self.flags & flag != 0
    }

    pub fn norm(&self) -> f64 {
        self.q.iter().map(|x| x * x).sum::<f64>().sqrt()
    }
}
