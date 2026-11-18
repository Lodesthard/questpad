//! Virtual Touch controllers over the air: the controller side of the Pulsar protocol, as
//! PROTOCOL.md describes it, against the emulated radio (hw.rs) that the firmware drives.
//!
//! - Pairing mode: advertisements every 102.2 ms on 2402 MHz (FACEB00C / AA), then the DM link on
//!   2426 MHz with the host, answering 1000 us after each host packet (0x14 pairing record, 0x15
//!   reset into the main app).
//! - Main app: search for beacons, follow the hopping (and check it: channel, timestamp), join in
//!   slot 0 (with random backoff), then send one uplink per frame in the assigned slot: IMU and
//!   inputs from `ctrl::Controller`, register read replies; act on requests (haptics, power off,
//!   0x32) and echo the request seq. Uplinks carry the `hw::mic` checksum keyed like the real
//!   AES-CCM (link key, beacon timestamp + session).

use crate::ctrl::Controller;
use crate::gamepad::Side;
use crate::hw::{self, AirPacket};
use crate::math::Vec3;

const ADV_BASE: u32 = 0xFACE_B00C;
const DM_PREFIX: u8 = 0xAA;
const ADV_FREQ: u32 = 2;
const DM_FREQ: u32 = 26;
const ADV_PERIOD_US: u64 = 102_200;
const CHANNELS: [u8; 37] = [
    4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62,
    64, 66, 68, 70, 72, 74, 76, 78,
];
const SLOT_OFFSET_US: [u64; 5] = [350, 575, 875, 1175, 1475];
const BEACON_PREFIX: u8 = 0xF0;
const S0_PULSAR: u8 = 4;
const FRAME_US: u64 = 2000;
const MAX_MISSED: u32 = 24;
const SCAN: [usize; 3] = [0, 17, 36];

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum State {
    /// Switched off (power-off request, or not started); `wake` brings it back.
    Off,
    Advertising,
    Dm,
    Searching,
    Linked,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct AirStats {
    pub adverts: u32,
    pub dm_commands: u32,
    pub beacons: u32,
    /// Beacon on a channel / with a timestamp the hopping rules didn't predict.
    pub hop_errors: u32,
    pub ts_errors: u32,
    pub missed_beacons: u32,
    pub join_requests: u32,
    pub joins: u32,
    pub uplinks: u32,
    pub acks: u32,
    pub requests: u32,
    pub read_replies: u32,
}

pub struct AirController {
    pub c: Controller,
    /// Angular rate (rad/s, controller frame) the controller turns at.
    pub w: Vec3,
    pub dev_id: [u8; 8],
    pub id0: u32,
    pub id1: u32,
    pub pairing: Option<(u32, [u8; 16])>,
    pub state: State,
    pub slot: Option<u8>,
    /// Haptic amplitude the dongle set (register 0x97), and its history (time, amplitude).
    pub motor: u8,
    pub motor_log: Vec<(u64, u8)>,
    pub stats: AirStats,
    // Pairing / DM.
    next_adv: u64,
    adv_listen: Option<(u64, u64)>,
    dm_last: u64,
    reset_after: Option<u64>,
    // Main link.
    hop: u32,
    t_beacon: u64,
    unmapped: u32,
    ts: u64,
    session: u16,
    missed: u32,
    next_tick: Option<u64>,
    scan_i: usize,
    scan_switch: u64,
    last_req: Option<(u8, u8, bool)>,
    seq_seen: u8,
    read_reply: Option<Vec<u8>>,
    off_after_uplink: bool,
    tx: Vec<(u64, AirPacket)>,
    rng: u64,
}

impl AirController {
    pub fn new(side: Side, slot_hint: u8, n: u8) -> AirController {
        let mut c = Controller::new(side, slot_hint);
        c.connected = false;
        AirController {
            c,
            w: [0.0; 3],
            dev_id: [0x51, 0x50, n, 0x0A, 0x11, 0x22, 0x33, side as u8],
            id0: 0x5A3C_9600 | n as u32,
            id1: 0x0F1E_2D00 | n as u32,
            pairing: None,
            state: State::Off,
            slot: None,
            motor: 0,
            motor_log: Vec::new(),
            stats: AirStats::default(),
            next_adv: 0,
            adv_listen: None,
            dm_last: 0,
            reset_after: None,
            hop: 0,
            t_beacon: 0,
            unmapped: 0,
            ts: 0,
            session: 0,
            missed: 0,
            next_tick: None,
            scan_i: 0,
            scan_switch: 0,
            last_req: None,
            seq_seen: 0,
            read_reply: None,
            off_after_uplink: false,
            tx: Vec::new(),
            rng: 0x9E37_79B9 ^ (n as u64) << 17,
        }
    }

    fn rand(&mut self) -> u64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        self.rng
    }

    /// Oculus + Y / Oculus + B: pairing mode (advertise for about a minute).
    pub fn enter_pairing(&mut self, now: u64) {
        self.state = State::Advertising;
        self.next_adv = now + 1000;
        self.tx.clear();
    }

    /// Main app with the stored pairing (power-up, wake by motion or a button).
    pub fn wake(&mut self, now: u64) {
        self.state = State::Searching;
        self.slot = None;
        self.missed = 0;
        self.next_tick = None;
        self.scan_i = 0;
        self.scan_switch = now + 100_000;
        self.tx.clear();
        self.last_req = None;
    }

    pub fn power_off(&mut self) {
        self.state = State::Off;
        self.slot = None;
        self.tx.clear();
        self.next_tick = None;
    }

    /// When this controller next needs to run.
    pub fn next_time(&self) -> Option<u64> {
        let state_due = match self.state {
            State::Advertising => Some(self.next_adv),
            State::Dm => Some(self.reset_after.unwrap_or(self.dm_last + 60_000)),
            State::Searching => Some(self.scan_switch),
            State::Linked => self.next_tick,
            State::Off => None,
        };
        self.tx.iter().map(|x| x.0).chain(state_due).min()
    }

    /// Do whatever is due at `now` (transmissions, timeouts).
    pub fn run(&mut self, now: u64) {
        let mut due = Vec::new();
        self.tx.retain(|(t, p)| {
            if *t <= now {
                due.push(p.clone());
                false
            } else {
                true
            }
        });
        for mut p in due {
            p.start = now;
            p.end = now + hw::air_time_us(p.data.len());
            hw::air_tx(p);
        }
        match self.state {
            State::Advertising if now >= self.next_adv => {
                self.advertise(now);
                self.next_adv = now + ADV_PERIOD_US;
            }
            State::Dm => {
                if let Some(t) = self.reset_after {
                    if now >= t {
                        // 0x15 handled: reboot into the main app with the new pairing.
                        self.reset_after = None;
                        self.wake(now);
                    }
                } else if now >= self.dm_last + 60_000 {
                    self.state = State::Advertising; // host gone
                    self.next_adv = now;
                }
            }
            State::Searching if now >= self.scan_switch => {
                self.scan_i = (self.scan_i + 1) % SCAN.len();
                self.scan_switch = now + 100_000;
            }
            State::Linked => {
                if let Some(t) = self.next_tick {
                    if now >= t {
                        self.frame_missed(now);
                    }
                }
            }
            _ => {}
        }
        if self.off_after_uplink && self.tx.is_empty() {
            self.off_after_uplink = false;
            self.power_off();
        }
    }

    fn send_at(&mut self, at: u64, freq: u32, base: u32, prefix: u8, data: Vec<u8>) {
        self.tx.push((at, AirPacket { freq, base, prefix, data, start: at, end: at }));
    }

    // ---- pairing mode ----

    fn advertise(&mut self, now: u64) {
        let mut p = vec![32u8, 2];
        p.extend_from_slice(&self.c.dev_type().to_le_bytes());
        p.extend_from_slice(&self.id0.to_le_bytes());
        p.extend_from_slice(&self.id1.to_le_bytes());
        p.extend_from_slice(&[0; 4]);
        p.extend_from_slice(b"1WMHH000000000");
        p.push(0);
        let end = now + hw::air_time_us(p.len());
        self.adv_listen = Some((end + 900, end + 2000));
        self.stats.adverts += 1;
        let pkt = AirPacket { freq: ADV_FREQ, base: ADV_BASE, prefix: DM_PREFIX, data: p, start: now, end };
        hw::air_tx(pkt);
    }

    fn dm_host_packet(&mut self, pkt: &AirPacket) {
        let d = &pkt.data;
        if d.len() < 3 {
            return;
        }
        let (len, hdr, seq) = (d[0] as usize, d[1], d[2]);
        let payload = &d[3..(1 + len).min(d.len())];
        self.state = State::Dm;
        self.dm_last = pkt.end;
        // Replies echo the header without the read bit (bit 7 = error).
        let mut reply = vec![2u8, hdr & 0xFE, seq];
        if hdr != 0 {
            self.stats.dm_commands += 1;
            match hdr >> 1 {
                0x14 if payload.len() >= 20 => {
                    let net = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                    let mut key = [0u8; 16];
                    key.copy_from_slice(&payload[4..20]);
                    self.pairing = Some((net, key));
                }
                0x15 => self.reset_after = Some(pkt.end + 1000 + 5000),
                0x03 => {
                    reply = vec![16, hdr & 0xFE, seq];
                    reply.extend_from_slice(b"1WMHH000000000");
                }
                _ => reply[1] = (hdr & 0xFE) | 0x80, // unknown: error
            }
        }
        self.send_at(pkt.end + 1000, DM_FREQ, self.id0, DM_PREFIX, reply);
    }

    // ---- main link ----

    /// A packet the dongle finished sending.
    pub fn on_air(&mut self, pkt: &AirPacket) {
        match self.state {
            State::Advertising => {
                if let Some((a, b)) = self.adv_listen {
                    if pkt.freq == DM_FREQ && pkt.base == self.id0 && pkt.prefix == DM_PREFIX && (a..=b).contains(&pkt.start) {
                        self.dm_host_packet(pkt);
                    }
                }
            }
            State::Dm => {
                if pkt.freq == DM_FREQ && pkt.base == self.id0 && pkt.prefix == DM_PREFIX {
                    self.dm_host_packet(pkt);
                }
            }
            State::Searching => {
                let Some((net, _)) = self.pairing else { return };
                if pkt.base == net && pkt.prefix == BEACON_PREFIX && pkt.freq == CHANNELS[SCAN[self.scan_i]] as u32 {
                    self.hop = (net & 0xFF) % 11 + 5;
                    self.state = State::Linked;
                    self.beacon(pkt, true);
                }
            }
            State::Linked => {
                let Some((net, _)) = self.pairing else { return };
                let expect_ch = (self.unmapped + self.hop) % 37;
                let expect_t = self.t_beacon + FRAME_US;
                if pkt.base == net && pkt.prefix == BEACON_PREFIX && pkt.start.abs_diff(expect_t) <= 60 {
                    if pkt.freq != CHANNELS[expect_ch as usize] as u32 {
                        self.stats.hop_errors += 1;
                        return; // we were listening on the predicted channel: missed
                    }
                    self.beacon(pkt, false);
                }
            }
            State::Off => {}
        }
    }

    fn beacon(&mut self, pkt: &AirPacket, first: bool) {
        let d = &pkt.data;
        if d.len() < 3 + 16 || d[0] != S0_PULSAR {
            return;
        }
        let p = &d[3..];
        let unmapped = p[5] as u32;
        let session = u16::from_le_bytes([p[6], p[7]]);
        let mut tsb = [0u8; 8];
        tsb[..6].copy_from_slice(&p[8..14]);
        let ts = u64::from_le_bytes(tsb);
        if CHANNELS[unmapped as usize] as u32 != pkt.freq {
            self.stats.hop_errors += 1;
        }
        if !first {
            if unmapped != (self.unmapped + self.hop) % 37 {
                self.stats.hop_errors += 1;
            }
            if ts != self.ts + FRAME_US {
                self.stats.ts_errors += 1;
            }
        }
        self.stats.beacons += 1;
        self.unmapped = unmapped;
        self.ts = ts;
        self.session = session;
        self.t_beacon = pkt.start;
        self.missed = 0;
        self.next_tick = Some(pkt.start + FRAME_US + 300);
        self.c.advance(self.w);
        let cl = &p[14..(d[1] as usize).min(p.len())];
        self.downlink(cl, pkt.start);
        self.uplink(pkt.start);
    }

    fn frame_missed(&mut self, now: u64) {
        // Keep hopping blind, as the controllers do.
        self.missed += 1;
        self.stats.missed_beacons += 1;
        self.unmapped = (self.unmapped + self.hop) % 37;
        self.t_beacon += FRAME_US;
        self.ts += FRAME_US;
        self.c.advance(self.w);
        if self.missed > MAX_MISSED {
            self.wake(now); // back to searching (the dongle frees the slot after 1 s)
        } else {
            self.next_tick = Some(self.t_beacon + FRAME_US + 300);
        }
    }

    fn downlink(&mut self, cl: &[u8], now: u64) {
        if cl.len() >= 14 && cl[0] == 1 && cl[2] == 2 && cl[3..11] == self.dev_id {
            self.slot = Some(cl[11]);
            self.stats.joins += 1;
            return;
        }
        if cl.len() >= 2 {
            if let Some(s) = self.slot {
                if cl[1] & (1 << s) != 0 {
                    self.stats.acks += 1;
                }
            }
        }
        if cl.len() < 4 || cl[0] == 0 || (cl[0] == 1 && cl.len() >= 14 && cl[2] == 2) {
            return;
        }
        let (mask, reg, seq, read) = (cl[0], cl[2], cl[3] & 0x0F, cl[3] & 0x10 != 0);
        self.seq_seen = seq; // every controller echoes the seq of any request it saw
        let Some(s) = self.slot else { return };
        if mask & (1 << s) == 0 || self.last_req == Some((reg, seq, read)) {
            return;
        }
        self.last_req = Some((reg, seq, read));
        self.stats.requests += 1;
        let data = &cl[4..];
        match (reg, read) {
            (0x97, false) if !data.is_empty() => {
                self.motor = data[0];
                self.motor_log.push((now, data[0]));
            }
            (0x05, false) => self.off_after_uplink = true,
            (0x32, true) => self.read_reply = Some([&[0x32u8][..], &self.c.imu_config()[..]].concat()),
            (0x24, true) => self.read_reply = Some(vec![0x24, 1, 17, 2, 0]),
            (_, true) => self.read_reply = Some(vec![reg]), // unknown: answered with the error flag below
            _ => {}
        }
    }

    fn uplink(&mut self, t_beacon: u64) {
        let Some((net, key)) = self.pairing else { return };
        let (slot, payload) = match self.slot {
            None => {
                // Join in slot 0, backing off at random so two controllers don't keep colliding.
                if self.rand() % 3 != 0 {
                    return;
                }
                self.stats.join_requests += 1;
                let mut p = vec![0u8, 1];
                p.extend_from_slice(&self.dev_id);
                p.extend_from_slice(&self.c.dev_type().to_le_bytes());
                (0u8, p)
            }
            Some(s) => {
                let p = if let Some(r) = self.read_reply.take() {
                    self.stats.read_replies += 1;
                    let known = matches!(r[0], 0x32 | 0x24);
                    let flags = self.seq_seen | if known { 0 } else { 0x20 };
                    [&[s, r[0], flags][..], &r[1..]].concat()
                } else {
                    let entries = self.c.entries(self.w);
                    [&[s, 0x01, 0x40 | self.seq_seen, 0, 0, 0, 0][..], &entries[..]].concat()
                };
                (s, p)
            }
        };
        let mut nonce = [0u8; 17];
        nonce[9..15].copy_from_slice(&self.ts.to_le_bytes()[..6]);
        nonce[15..17].copy_from_slice(&self.session.to_le_bytes());
        let m = hw::mic(&key, &nonce, S0_PULSAR, &payload);
        let mut data = vec![S0_PULSAR, (payload.len() + 4) as u8, 0];
        data.extend_from_slice(&payload);
        data.extend_from_slice(&m);
        self.stats.uplinks += 1;
        let freq = CHANNELS[self.unmapped as usize] as u32;
        self.send_at(t_beacon + SLOT_OFFSET_US[slot as usize], freq, net, slot + 1, data);
    }
}
