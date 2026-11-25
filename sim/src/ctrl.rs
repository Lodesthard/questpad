//! A virtual Touch controller: a ground-truth orientation, and the radio packets the real one would
//! send for it (raw IMU counts in its own sensor axes, with bias and noise, plus its inputs).

use crate::gamepad::{self, Side};
use crate::math::{self, Quat, Vec3};
use crate::report::Report;

/// Touch IMU scales (PROTOCOL.md).
const GYRO_LSB_PER_DPS: f64 = 8.192;
const ACCEL_LSB_PER_G: f64 = 1024.0;
const G: f64 = 9.80665;
/// One controller packet (and IMU sample) per 2 ms link frame.
pub const FRAME_US: u64 = 2000;

pub struct Controller {
    pub side: Side,
    /// Link slot (1..4), as the dongle would assign it.
    pub slot: u8,
    pub connected: bool,
    /// Ground truth: controller -> world (OpenXR frame), w x y z.
    pub q: Quat,
    /// Linear acceleration of the controller in the world, m/s² (shaking it).
    pub lin_accel: Vec3,
    /// Gyro resting offset the sensor adds, °/s, in the controller frame (OpenXR axes).
    pub bias_dps: Vec3,
    /// Noise, standard deviation: gyro °/s, accel g.
    pub gyro_noise_dps: f64,
    pub accel_noise_g: f64,
    /// Sensor imperfections (what a factory calibration would correct): per-axis scale factors
    /// (1.0 = exact), the sensor's axes rotated from the controller's (misalignment), accel offset.
    pub gyro_scale: Vec3,
    pub accel_scale: Vec3,
    pub misalign: Quat,
    pub accel_offset_g: Vec3,
    /// Raw resolution the controller sends (the firmware assumes 8.192 / 1024 unless told by 0x32).
    pub gyro_lsb_per_dps: f64,
    pub accel_lsb_per_g: f64,
    pub buttons: u8,
    pub touch: u8,
    pub stick: (i16, i16),
    /// 0 released .. 4095 fully pulled.
    pub trigger: u16,
    pub grip: u16,
    pub battery: u8,
    ts_us: u64,
    rng: u64,
}

impl Controller {
    pub fn new(side: Side, slot: u8) -> Controller {
        Controller {
            side,
            slot,
            connected: false,
            q: math::IDENTITY,
            lin_accel: [0.0; 3],
            bias_dps: [0.0; 3],
            gyro_noise_dps: 0.1, // measured at rest on the hardware (PROTOCOL.md)
            accel_noise_g: 0.003,
            gyro_scale: [1.0; 3],
            accel_scale: [1.0; 3],
            misalign: math::IDENTITY,
            accel_offset_g: [0.0; 3],
            gyro_lsb_per_dps: GYRO_LSB_PER_DPS,
            accel_lsb_per_g: ACCEL_LSB_PER_G,
            buttons: 0,
            touch: 0,
            stick: (0, 0),
            trigger: 0,
            grip: 0,
            battery: 80,
            ts_us: 1_000_000,
            rng: 0x9E37_79B9_7F4A_7C15 ^ slot as u64,
        }
    }

    /// Device type the controller announces when it joins (bit 16: right hand).
    pub fn dev_type(&self) -> u32 {
        if self.side == Side::Right { 0x1301_1101 } else { 0x1300_1101 }
    }

    /// Join the link (what the radio does when the controller's join request arrives).
    pub fn join(&mut self) {
        gamepad::link_join(self.slot, self.dev_type());
        self.connected = true;
    }

    pub fn drop_link(&mut self) {
        gamepad::link_drop(self.slot);
        self.connected = false;
    }

    /// One link frame: turn at body rate `w` (rad/s, controller frame) for 2 ms, then deliver the
    /// packet to the firmware code. Does not advance the clock (the caller does, once per frame).
    pub fn step(&mut self, w: Vec3) {
        self.advance(w);
        if self.connected {
            let p = self.entries(w);
            gamepad::link_notifications(self.slot, &p);
        }
    }

    /// One 2 ms frame of motion (ground truth and the controller's IMU clock), no delivery.
    pub fn advance(&mut self, w: Vec3) {
        self.q = math::integrate(self.q, w, FRAME_US as f64 * 1e-6);
        self.ts_us += FRAME_US;
    }

    /// This controller's answer to register 0x32 (IMU configuration), data bytes only.
    pub fn imu_config(&self) -> [u8; 16] {
        let mut d = [0u8; 16];
        let range_mg = (32768.0 / self.accel_lsb_per_g * 1000.0) as u16;
        let range_dps = (32768.0 / self.gyro_lsb_per_dps) as u16;
        d[0..2].copy_from_slice(&range_mg.to_le_bytes());
        d[2..4].copy_from_slice(&range_dps.to_le_bytes());
        d[4..6].copy_from_slice(&500u16.to_le_bytes());
        d[6..8].copy_from_slice(&500u16.to_le_bytes());
        d[8..12].copy_from_slice(&((1.0 / self.accel_lsb_per_g) as f32).to_le_bytes());
        d[12..16].copy_from_slice(&((1.0 / self.gyro_lsb_per_dps) as f32).to_le_bytes());
        d
    }

    /// Deliver the 0x32 answer as the radio link would after the join.
    pub fn answer_imu_config(&self) -> bool {
        gamepad::imu_config(self.slot, &self.imu_config())
    }

    /// The latest report the dongle built for this controller, if a new one is waiting.
    pub fn take_report(&self) -> Option<Report> {
        Report::take(self.side)
    }

    fn gauss(&mut self) -> f64 {
        // xorshift64*, then the sum of 4 uniforms (close enough to a normal for sensor noise).
        let mut s = 0.0;
        for _ in 0..4 {
            self.rng ^= self.rng >> 12;
            self.rng ^= self.rng << 25;
            self.rng ^= self.rng >> 27;
            let u = (self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64;
            s += u - 0.5;
        }
        s * (3.0f64).sqrt() // 4 uniforms of variance 1/12 -> variance 1/3; scale to 1
    }

    /// Controller frame (OpenXR axes) -> the Touch IMU's raw axes: the inverse of what the firmware
    /// does with the default settings (`left_to_right_frame` for the left hand, then the default
    /// `imu_map`: how the IMU really sits; a changed map then shows in the reports).
    fn to_raw(&self, v: Vec3) -> Vec3 {
        let m = crate::settings::DEFAULT.imu_map;
        let mut right = [0.0; 3];
        for i in 0..3 {
            let axis = ((m[i] & 0x03) as usize).min(2);
            right[axis] = if m[i] & 0x80 != 0 { -v[i] } else { v[i] };
        }
        match self.side {
            // left_to_right_frame(l) = [-l1, l0, l2]
            Side::Left => [right[1], -right[0], right[2]],
            Side::Right => right,
        }
    }

    fn counts(v: f64) -> i16 {
        v.round().clamp(-32768.0, 32767.0) as i16
    }

    /// Notification entries for the current state: what follows the 4-byte log chunk in an uplink,
    /// as `link.rs` passes them on.
    pub fn entries(&mut self, w: Vec3) -> Vec<u8> {
        // Specific force (what an accelerometer reads), in g: acceleration minus gravity, so +1 g
        // up at rest; seen from the controller.
        let f_world = [self.lin_accel[0] / G, 1.0 + self.lin_accel[1] / G, self.lin_accel[2] / G];
        let f_body = math::rotate(math::conj(self.q), f_world);
        let w_dps = w.map(f64::to_degrees);
        // What the (imperfect) sensor measures, in the controller's axes.
        let a_s = math::rotate(self.misalign, f_body);
        let g_s = math::rotate(self.misalign, w_dps);
        let a_meas = [0, 1, 2].map(|i| a_s[i] * self.accel_scale[i] + self.accel_offset_g[i]);
        let g_meas = [0, 1, 2].map(|i| g_s[i] * self.gyro_scale[i] + self.bias_dps[i]);
        let mut accel = [0i16; 3];
        let mut gyro = [0i16; 3];
        let a_raw = self.to_raw(a_meas);
        let g_raw = self.to_raw(g_meas);
        for i in 0..3 {
            let na = self.gauss() * self.accel_noise_g;
            let ng = self.gauss() * self.gyro_noise_dps;
            accel[i] = Self::counts((a_raw[i] + na) * self.accel_lsb_per_g);
            gyro[i] = Self::counts((g_raw[i] + ng) * self.gyro_lsb_per_dps);
        }
        let mut imu = Vec::with_capacity(18);
        imu.extend_from_slice(&self.ts_us.to_le_bytes()[..6]);
        for v in accel.iter().chain(gyro.iter()) {
            imu.extend_from_slice(&v.to_le_bytes());
        }
        let t = 4095 - self.trigger.min(4095);
        let g = 4095 - self.grip.min(4095);
        let analog = [(t & 0xFF) as u8, ((t >> 8) & 0x0F) as u8 | ((g & 0x0F) << 4) as u8, (g >> 4) as u8];
        let mut stick = Vec::new();
        stick.extend_from_slice(&self.stick.0.to_le_bytes());
        stick.extend_from_slice(&self.stick.1.to_le_bytes());
        let entries: [(u8, Vec<u8>); 6] = [
            (0, vec![self.battery]),
            (1, imu),
            (2, stick),
            (3, analog.to_vec()),
            (4, vec![self.buttons | 0x10]), // bit 4 is always set
            (5, vec![self.touch]),
        ];
        let mut out = Vec::new();
        for (n, (id, d)) in entries.iter().enumerate() {
            let len = d.len() as u8;
            let last = (n == entries.len() - 1) as u8;
            out.push(id | (len & 7) << 5);
            out.push((len >> 3) & 0x0F | (n as u8 & 7) << 4 | last << 7);
            out.extend_from_slice(d);
        }
        out
    }
}
