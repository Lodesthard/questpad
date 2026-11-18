//! The nRF52840 peripherals the firmware drives, emulated. The firmware's only access to registers
//! and flash is `crate::hw::{read, write}` (firmware/src/hw.rs); in the simulator that name is this
//! module, so radio.rs, link.rs, pairing.rs, settings.rs, wdt.rs ... run unchanged against:
//!
//! - TIMER0 (1 MHz, 32-bit) and the PPI channels the firmware uses (ch0 COMPARE3 -> a RADIO task,
//!   ch26 / ch27 ADDRESS / END -> CAPTURE1 / CAPTURE2);
//! - RADIO: ramp-up, START / END / DISABLE shorts, logical addresses (BASE / PREFIX / RXADDRESSES /
//!   TXADDRESS), the packet layout from PCNF0, on-air time at 2 Mbit, packets exchanged with the
//!   virtual devices in `world.rs`;
//! - CCM: the link's decryption, with a keyed checksum (`mic`) in place of AES-CCM, so the key, the
//!   per-packet IV (beacon timestamp + session) and the MIC handling are still checked;
//! - NVMC + 1 MB of flash, RNG, WDT, FICR, GPREGRET.
//!
//! Interrupts (RADIO, TIMER0, AAR_CCM) run the firmware's handlers when no critical section is held
//! and no handler is already running, like an NVIC with the three at one priority.
//!
//! The firmware hands buffer addresses to the peripherals as u32 (PACKETPTR, CNFPTR, ...); on the
//! 64-bit host those are the low 32 bits of the real addresses, which `host` completes with the
//! high bits of this program's own statics.

use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

// ---- the firmware's interface ----

pub fn read(addr: u32) -> u32 {
    CHIP.with(|c| c.borrow_mut().read(addr))
}

pub fn write(addr: u32, val: u32) {
    CHIP.with(|c| c.borrow_mut().write(addr, val));
    dispatch();
}

// ---- interrupts and critical sections ----

const IRQ_RADIO: u32 = 1;
const IRQ_TIMER0: u32 = 2;
const IRQ_CCM: u32 = 4;

static PENDING: AtomicU32 = AtomicU32::new(0);
static CS_DEPTH: AtomicU32 = AtomicU32::new(0);
static IN_ISR: AtomicBool = AtomicBool::new(false);

extern "C" {
    fn RADIO();
    fn TIMER0();
    fn AAR_CCM();
}

/// Run pending interrupt handlers, unless masked (critical section) or already in one.
pub fn dispatch() {
    if CS_DEPTH.load(Ordering::Relaxed) > 0 || IN_ISR.load(Ordering::Relaxed) {
        return;
    }
    loop {
        let p = PENDING.swap(0, Ordering::Relaxed);
        if p == 0 {
            break;
        }
        IN_ISR.store(true, Ordering::Relaxed);
        // Lower IRQ number first: RADIO (1), TIMER0 (8), AAR_CCM (15).
        unsafe {
            if p & IRQ_RADIO != 0 {
                RADIO();
            }
            if p & IRQ_TIMER0 != 0 {
                TIMER0();
            }
            if p & IRQ_CCM != 0 {
                AAR_CCM();
            }
        }
        IN_ISR.store(false, Ordering::Relaxed);
    }
}

fn pend(irq: u32) {
    PENDING.fetch_or(irq, Ordering::Relaxed);
}

struct SimCs;
critical_section::set_impl!(SimCs);

unsafe impl critical_section::Impl for SimCs {
    unsafe fn acquire() -> critical_section::RawRestoreState {
        CS_DEPTH.fetch_add(1, Ordering::Relaxed);
    }

    unsafe fn release(_: critical_section::RawRestoreState) {
        if CS_DEPTH.fetch_sub(1, Ordering::Relaxed) == 1 {
            // Interrupts unmasked: whatever became pending runs now.
            dispatch();
        }
    }
}

// ---- simulation interface (world.rs, scenarios) ----

/// A packet on the air: the RAM layout (header bytes from PCNF0, then the payload), where and when.
#[derive(Clone, Debug)]
pub struct AirPacket {
    pub freq: u32,
    pub base: u32,
    pub prefix: u8,
    pub data: Vec<u8>,
    pub start: u64,
    pub end: u64,
}

/// On-air time at 2 Mbit: preamble 2 B + address 5 B (28 us), then header / payload / CRC 3 B.
pub fn air_time_us(ram_bytes: usize) -> u64 {
    28 + (ram_bytes as u64 + 3) * 4
}

/// Keyed checksum standing in for the link's AES-CCM MIC (4 bytes): over the key, the 17-byte
/// nonce part of the CCM configuration (packet counter, direction, 8-byte IV), S0 and the payload.
pub fn mic(key: &[u8], nonce: &[u8], s0: u8, payload: &[u8]) -> [u8; 4] {
    let mut h: u32 = 0x811C_9DC5;
    for &b in key.iter().chain(nonce).chain(std::iter::once(&s0)).chain(payload) {
        h = (h ^ b as u32).wrapping_mul(0x0100_0193);
    }
    h = h.rotate_left(13) ^ 0x5155_4D49;
    h.to_le_bytes()
}

pub fn now() -> u64 {
    CHIP.with(|c| c.borrow().now)
}

pub fn next_event() -> Option<u64> {
    CHIP.with(|c| c.borrow().queue.keys().next().map(|k| k.0))
}

/// Run the peripherals up to time `t` (us), firing events and interrupt handlers in order.
pub fn advance_to(t: u64) {
    loop {
        let fired = CHIP.with(|c| {
            let mut c = c.borrow_mut();
            let key = match c.queue.keys().next() {
                Some(k) if k.0 <= t => *k,
                _ => return false,
            };
            let ev = c.queue.remove(&key).unwrap();
            c.now = key.0;
            c.fire(ev);
            true
        });
        dispatch();
        if !fired {
            break;
        }
    }
    CHIP.with(|c| c.borrow_mut().now = t);
}

/// A virtual device starts transmitting `pkt` now; the radio receives it if it is listening there.
pub fn air_tx(pkt: AirPacket) {
    CHIP.with(|c| c.borrow_mut().air_rx(pkt));
    dispatch();
}

/// Packets the dongle finished transmitting since the last call.
pub fn take_outbox() -> Vec<AirPacket> {
    CHIP.with(|c| std::mem::take(&mut c.borrow_mut().outbox))
}

pub fn with<R>(f: impl FnOnce(&mut Chip) -> R) -> R {
    CHIP.with(|c| f(&mut c.borrow_mut()))
}

// ---- the chip ----

thread_local! {
    static CHIP: RefCell<Chip> = RefCell::new(Chip::new());
}

static mut ANCHOR: [u8; 8] = [0; 8];

/// Full host address for a firmware buffer address (low 32 bits): the candidate nearest this
/// program's own statics (they all sit within a few MB of each other, possibly across a 4 GB line).
fn host(ptr: u32) -> *mut u8 {
    let anchor = core::ptr::addr_of!(ANCHOR) as u64;
    let base = (anchor & !0xFFFF_FFFF) | ptr as u64;
    let best = [base.wrapping_sub(1 << 32), base, base.wrapping_add(1 << 32)]
        .into_iter()
        .min_by_key(|c| c.abs_diff(anchor))
        .unwrap();
    best as *mut u8
}

const RADIO_BASE: u32 = 0x4000_1000;
const TIMER0_BASE: u32 = 0x4000_8000;
const RNG_BASE: u32 = 0x4000_D000;
const CCM_BASE: u32 = 0x4000_F000;
const WDT_BASE: u32 = 0x4001_0000;
const NVMC_BASE: u32 = 0x4001_E000;
const PPI_BASE: u32 = 0x4001_F000;
const GPREGRET: u32 = 0x4000_051C;
const FLASH_SIZE: usize = 1 << 20;
pub const RAMP_US: u64 = 40;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum RState {
    Disabled,
    TxRu,
    TxIdle,
    Tx,
    RxRu,
    RxIdle,
    RxListen,
    RxBusy,
}

#[derive(Clone, Copy, Debug)]
enum Ev {
    RadioReady(u64),
    RadioAddress(u64),
    RadioEnd(u64),
    TimerCompare(u64),
    CcmDone,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct ChipStats {
    pub dongle_tx: u32,
    pub rx_ok: u32,
    /// Device packets the radio wasn't listening for (wrong channel / address, busy, off).
    pub rx_missed: u32,
    pub ccm_runs: u32,
    pub ccm_mic_fail: u32,
    pub flash_erases: u32,
}

pub struct Chip {
    pub now: u64,
    seq: u64,
    queue: BTreeMap<(u64, u64), Ev>,
    // TIMER0
    t_running: bool,
    t_base: u64,
    t_stopped: u32,
    t_cc: [u32; 4],
    t_ev_compare3: u32,
    t_inten: u32,
    t_epoch: u64,
    // PPI
    ppi_en: u32,
    ppi_eep0: u32,
    ppi_tep0: u32,
    // RADIO
    r: HashMap<u32, u32>,
    pub r_state: RState,
    r_inten: u32,
    r_epoch: u64,
    r_latched: u32,
    r_tx: Option<AirPacket>,
    r_rx: Option<(AirPacket, u32)>,
    // CCM
    c: HashMap<u32, u32>,
    c_inten: u32,
    // NVMC / flash
    pub flash: Vec<u8>,
    nvmc_config: u32,
    rng: u64,
    pub wdt_started: bool,
    pub wdt_last_feed: Option<u64>,
    pub wdt_max_gap: u64,
    pub gpregret: u32,
    other: HashMap<u32, u32>,
    pub outbox: Vec<AirPacket>,
    pub stats: ChipStats,
}

impl Chip {
    fn new() -> Chip {
        Chip {
            now: 0,
            seq: 0,
            queue: BTreeMap::new(),
            t_running: false,
            t_base: 0,
            t_stopped: 0,
            t_cc: [0; 4],
            t_ev_compare3: 0,
            t_inten: 0,
            t_epoch: 0,
            ppi_en: 0,
            ppi_eep0: 0,
            ppi_tep0: 0,
            r: HashMap::new(),
            r_state: RState::Disabled,
            r_inten: 0,
            r_epoch: 0,
            r_latched: 0,
            r_tx: None,
            r_rx: None,
            c: HashMap::new(),
            c_inten: 0,
            flash: vec![0xFF; FLASH_SIZE],
            nvmc_config: 0,
            rng: 0x2545_F491_4F6C_DD1D,
            wdt_started: false,
            wdt_last_feed: None,
            wdt_max_gap: 0,
            gpregret: 0,
            other: HashMap::new(),
            outbox: Vec::new(),
            stats: ChipStats::default(),
        }
    }

    fn schedule(&mut self, at: u64, ev: Ev) {
        self.seq += 1;
        self.queue.insert((at, self.seq), ev);
    }

    fn read(&mut self, addr: u32) -> u32 {
        match addr {
            a if (a as usize) < FLASH_SIZE => self.flash_word(a),
            0x1000_0060 => 0x5155_4144, // FICR DEVICEID[0]
            0x1000_0064 => 0x5349_4D31, // FICR DEVICEID[1]
            GPREGRET => self.gpregret,
            a if (RADIO_BASE..RADIO_BASE + 0x1000).contains(&a) => self.radio_read(a - RADIO_BASE),
            a if (TIMER0_BASE..TIMER0_BASE + 0x1000).contains(&a) => self.timer_read(a - TIMER0_BASE),
            a if (CCM_BASE..CCM_BASE + 0x1000).contains(&a) => match a - CCM_BASE {
                0x304 | 0x308 => self.c_inten,
                off => *self.c.get(&off).unwrap_or(&0),
            },
            a if (RNG_BASE..RNG_BASE + 0x1000).contains(&a) => match a - RNG_BASE {
                0x100 => 1, // EVENTS_VALRDY
                0x508 => {
                    self.rng ^= self.rng << 13;
                    self.rng ^= self.rng >> 7;
                    self.rng ^= self.rng << 17;
                    (self.rng >> 24) as u32 & 0xFF
                }
                off => *self.other.get(&(RNG_BASE + off)).unwrap_or(&0),
            },
            a if (NVMC_BASE..NVMC_BASE + 0x1000).contains(&a) => match a - NVMC_BASE {
                0x400 | 0x408 => 1, // READY, READYNEXT
                0x504 => self.nvmc_config,
                off => *self.other.get(&(NVMC_BASE + off)).unwrap_or(&0),
            },
            a => *self.other.get(&a).unwrap_or(&0),
        }
    }

    fn write(&mut self, addr: u32, v: u32) {
        match addr {
            a if (a as usize) < FLASH_SIZE => {
                if self.nvmc_config == 1 {
                    let i = a as usize & !3;
                    let old = self.flash_word(a);
                    self.flash[i..i + 4].copy_from_slice(&(old & v).to_le_bytes()); // flash only clears bits
                }
            }
            GPREGRET => self.gpregret = v,
            a if (RADIO_BASE..RADIO_BASE + 0x1000).contains(&a) => self.radio_write(a - RADIO_BASE, v),
            a if (TIMER0_BASE..TIMER0_BASE + 0x1000).contains(&a) => self.timer_write(a - TIMER0_BASE, v),
            a if (PPI_BASE..PPI_BASE + 0x1000).contains(&a) => match a - PPI_BASE {
                0x504 => self.ppi_en |= v,
                0x508 => self.ppi_en &= !v,
                0x510 => self.ppi_eep0 = v,
                0x514 => self.ppi_tep0 = v,
                off => {
                    self.other.insert(PPI_BASE + off, v);
                }
            },
            a if (CCM_BASE..CCM_BASE + 0x1000).contains(&a) => self.ccm_write(a - CCM_BASE, v),
            a if (WDT_BASE..WDT_BASE + 0x1000).contains(&a) => match a - WDT_BASE {
                0x000 => {
                    self.wdt_started = true;
                    self.wdt_last_feed.get_or_insert(self.now);
                }
                0x600 if v == 0x6E52_4635 => {
                    if let Some(last) = self.wdt_last_feed {
                        self.wdt_max_gap = self.wdt_max_gap.max(self.now - last);
                    }
                    self.wdt_last_feed = Some(self.now);
                }
                off => {
                    self.other.insert(WDT_BASE + off, v);
                }
            },
            a if (NVMC_BASE..NVMC_BASE + 0x1000).contains(&a) => match a - NVMC_BASE {
                0x504 => self.nvmc_config = v,
                0x508 => {
                    if self.nvmc_config == 2 && (v as usize) < FLASH_SIZE {
                        let p = v as usize & !0xFFF;
                        self.flash[p..p + 4096].fill(0xFF);
                        self.stats.flash_erases += 1;
                    }
                }
                off => {
                    self.other.insert(NVMC_BASE + off, v);
                }
            },
            a => {
                self.other.insert(a, v);
            }
        }
    }

    fn flash_word(&self, addr: u32) -> u32 {
        let i = addr as usize & !3;
        u32::from_le_bytes([self.flash[i], self.flash[i + 1], self.flash[i + 2], self.flash[i + 3]])
    }

    // ---- TIMER0 / PPI ----

    fn counter(&self) -> u32 {
        if self.t_running {
            (self.now - self.t_base) as u32
        } else {
            self.t_stopped
        }
    }

    fn timer_read(&mut self, off: u32) -> u32 {
        match off {
            0x14C => self.t_ev_compare3,
            0x304 | 0x308 => self.t_inten,
            0x540..=0x54C => self.t_cc[((off - 0x540) / 4) as usize],
            o => *self.other.get(&(TIMER0_BASE + o)).unwrap_or(&0),
        }
    }

    fn timer_write(&mut self, off: u32, v: u32) {
        match off {
            0x000 => {
                if !self.t_running {
                    self.t_running = true;
                    self.t_base = self.now - self.t_stopped as u64;
                }
                self.rearm_compare();
            }
            0x004 => {
                self.t_stopped = self.counter();
                self.t_running = false;
            }
            0x00C => {
                self.t_base = self.now;
                self.t_stopped = 0;
                self.rearm_compare();
            }
            0x040..=0x04C => {
                let n = ((off - 0x040) / 4) as usize;
                self.t_cc[n] = self.counter();
            }
            0x14C => self.t_ev_compare3 = v,
            0x304 => self.t_inten |= v,
            0x308 => self.t_inten &= !v,
            0x540..=0x54C => {
                let n = ((off - 0x540) / 4) as usize;
                self.t_cc[n] = v;
                if n == 3 {
                    self.rearm_compare();
                }
            }
            o => {
                self.other.insert(TIMER0_BASE + o, v);
            }
        }
    }

    fn rearm_compare(&mut self) {
        self.t_epoch += 1;
        if self.t_running {
            let mut d = self.t_cc[3].wrapping_sub(self.counter()) as u64;
            if d == 0 {
                d = 1 << 32;
            }
            let e = self.t_epoch;
            self.schedule(self.now + d, Ev::TimerCompare(e));
        }
    }

    /// A task triggered through PPI or written by the firmware, by register address.
    fn task(&mut self, addr: u32) {
        if (RADIO_BASE..RADIO_BASE + 0x100).contains(&addr) {
            self.radio_write(addr - RADIO_BASE, 1);
        } else if (TIMER0_BASE..TIMER0_BASE + 0x100).contains(&addr) {
            self.timer_write(addr - TIMER0_BASE, 1);
        }
    }

    fn capture(&mut self, n: usize) {
        self.t_cc[n] = self.counter();
    }

    // ---- RADIO ----

    fn reg(&self, off: u32) -> u32 {
        *self.r.get(&off).unwrap_or(&0)
    }

    fn radio_read(&mut self, off: u32) -> u32 {
        match off {
            0x304 | 0x308 => self.r_inten,
            0x550 => match self.r_state {
                RState::Disabled => 0,
                RState::RxRu => 1,
                RState::RxIdle => 2,
                RState::RxListen | RState::RxBusy => 3,
                RState::TxRu => 9,
                RState::TxIdle => 10,
                RState::Tx => 11,
            },
            o => self.reg(o),
        }
    }

    fn radio_write(&mut self, off: u32, v: u32) {
        match off {
            0x000 if v != 0 => self.radio_ramp_up(RState::TxRu), // TASKS_TXEN
            0x004 if v != 0 => self.radio_ramp_up(RState::RxRu), // TASKS_RXEN
            0x008 if v != 0 => self.radio_start(),
            0x010 if v != 0 => self.radio_disable(),
            0x304 => self.r_inten |= v,
            0x308 => self.r_inten &= !v,
            o => {
                self.r.insert(o, v);
            }
        }
    }

    /// TXEN / RXEN: ramp up into `state` (TxRu / RxRu), READY after `RAMP_US`; ignored unless disabled.
    fn radio_ramp_up(&mut self, state: RState) {
        if self.r_state == RState::Disabled {
            self.r_state = state;
            self.r_epoch += 1;
            let e = self.r_epoch;
            self.schedule(self.now + RAMP_US, Ev::RadioReady(e));
        }
    }

    fn radio_event(&mut self, off: u32, int_bit: u32) {
        self.r.insert(off, 1);
        if self.r_inten & int_bit != 0 {
            pend(IRQ_RADIO);
        }
    }

    fn radio_disable(&mut self) {
        if self.r_state == RState::Disabled {
            return;
        }
        self.r_state = RState::Disabled;
        self.r_epoch += 1;
        self.r_tx = None;
        self.r_rx = None;
        self.radio_event(0x110, 1 << 4);
        if self.reg(0x200) & (1 << 3) != 0 {
            // DISABLED_RXEN
            self.radio_write(0x004, 1);
        }
    }

    /// RAM layout from PCNF0: (header bytes, index of the LENGTH byte).
    fn layout(&self) -> (usize, usize) {
        let p = self.reg(0x514);
        let lflen = p & 0xF;
        let s0 = ((p >> 8) & 1) as usize;
        let s1 = (p >> 16) & 0xF;
        let s1incl = (p >> 20) & 1;
        let hdr = s0 + (lflen > 0) as usize + (s1 > 0 || s1incl != 0) as usize;
        (hdr, s0)
    }

    fn logical(&self, n: u32) -> (u32, u8) {
        let base = if n == 0 { self.reg(0x51C) } else { self.reg(0x520) };
        let prefix = if n < 4 { self.reg(0x524) >> (8 * n) } else { self.reg(0x528) >> (8 * (n - 4)) } as u8;
        (base, prefix)
    }

    fn radio_start(&mut self) {
        match self.r_state {
            RState::TxRu | RState::TxIdle => {
                let ptr = self.reg(0x504);
                let (hdr, lo) = self.layout();
                let mut data = vec![0u8; 260];
                unsafe { std::ptr::copy_nonoverlapping(host(ptr), data.as_mut_ptr(), hdr) };
                let len = (hdr + data[lo] as usize).min(260);
                unsafe { std::ptr::copy_nonoverlapping(host(ptr), data.as_mut_ptr(), len) };
                data.truncate(len);
                let (base, prefix) = self.logical(self.reg(0x52C));
                let start = self.now;
                let end = start + air_time_us(len);
                self.r_tx = Some(AirPacket { freq: self.reg(0x508), base, prefix, data, start, end });
                self.r_state = RState::Tx;
                let e = self.r_epoch;
                self.schedule(start + 28, Ev::RadioAddress(e));
                self.schedule(end, Ev::RadioEnd(e));
            }
            RState::RxRu | RState::RxIdle => {
                self.r_latched = self.reg(0x504);
                self.r_state = RState::RxListen;
            }
            _ => {}
        }
    }

    /// A device's packet starts now.
    fn air_rx(&mut self, pkt: AirPacket) {
        if self.r_state != RState::RxListen || self.reg(0x508) != pkt.freq {
            self.stats.rx_missed += 1;
            return;
        }
        let enabled = self.reg(0x530);
        let Some(n) = (0..8).find(|&n| enabled & (1 << n) != 0 && self.logical(n) == (pkt.base, pkt.prefix)) else {
            self.stats.rx_missed += 1;
            return;
        };
        self.r_state = RState::RxBusy;
        let e = self.r_epoch;
        self.schedule(pkt.start + 28, Ev::RadioAddress(e));
        self.schedule(pkt.end, Ev::RadioEnd(e));
        self.r_rx = Some((pkt, n));
    }

    fn fire(&mut self, ev: Ev) {
        match ev {
            Ev::TimerCompare(e) if e == self.t_epoch => {
                self.t_ev_compare3 = 1;
                if self.ppi_en & 1 != 0 && self.ppi_eep0 == TIMER0_BASE + 0x14C {
                    let t = self.ppi_tep0;
                    self.task(t);
                }
                if self.t_inten & (1 << 19) != 0 {
                    pend(IRQ_TIMER0);
                }
                let e = self.t_epoch;
                self.schedule(self.now + (1 << 32), Ev::TimerCompare(e));
            }
            Ev::RadioReady(e) if e == self.r_epoch => {
                self.r_state = match self.r_state {
                    RState::TxRu => RState::TxIdle,
                    RState::RxRu => RState::RxIdle,
                    s => s,
                };
                self.radio_event(0x100, 1 << 0);
                if self.reg(0x200) & 1 != 0 {
                    self.radio_start();
                }
            }
            Ev::RadioAddress(e) if e == self.r_epoch => {
                if let Some(&(_, n)) = self.r_rx.as_ref() {
                    self.r.insert(0x408, n); // RXMATCH
                }
                if self.ppi_en & (1 << 26) != 0 {
                    self.capture(1);
                }
                self.radio_event(0x104, 1 << 1);
            }
            Ev::RadioEnd(e) if e == self.r_epoch => {
                if self.ppi_en & (1 << 27) != 0 {
                    self.capture(2);
                }
                match self.r_state {
                    RState::Tx => {
                        if let Some(p) = self.r_tx.take() {
                            self.stats.dongle_tx += 1;
                            self.outbox.push(p);
                        }
                        self.r_state = RState::TxIdle;
                    }
                    RState::RxBusy => {
                        if let Some((p, _)) = self.r_rx.take() {
                            unsafe { std::ptr::copy_nonoverlapping(p.data.as_ptr(), host(self.r_latched), p.data.len()) };
                            self.r.insert(0x400, 1); // CRCSTATUS ok
                            self.r.insert(0x548, 40); // RSSISAMPLE
                            self.stats.rx_ok += 1;
                        }
                        self.r_state = RState::RxIdle;
                    }
                    _ => {}
                }
                self.radio_event(0x10C, 1 << 3);
                let shorts = self.reg(0x200);
                if shorts & (1 << 1) != 0 {
                    self.radio_disable(); // END_DISABLE
                } else if shorts & (1 << 5) != 0 {
                    self.radio_start(); // END_START
                }
            }
            Ev::CcmDone => {
                self.c.insert(0x104, 1); // EVENTS_ENDCRYPT
                if self.c_inten & (1 << 1) != 0 {
                    pend(IRQ_CCM);
                }
            }
            _ => {} // stale (epoch moved on)
        }
    }

    // ---- CCM ----

    fn ccm_write(&mut self, off: u32, v: u32) {
        match off {
            0x000 if v != 0 => {
                if self.c.get(&0x200).copied().unwrap_or(0) & 1 != 0 {
                    self.ccm_crypt();
                }
            }
            0x304 => self.c_inten |= v,
            0x308 => self.c_inten &= !v,
            o => {
                self.c.insert(o, v);
            }
        }
    }

    fn ccm_crypt(&mut self) {
        self.stats.ccm_runs += 1;
        let cnf_ptr = host(*self.c.get(&0x508).unwrap_or(&0));
        let mut cnf = [0u8; 33];
        unsafe { std::ptr::copy_nonoverlapping(cnf_ptr, cnf.as_mut_ptr(), 33) };
        let inp = host(*self.c.get(&0x50C).unwrap_or(&0));
        let mut hdr = [0u8; 3];
        unsafe { std::ptr::copy_nonoverlapping(inp, hdr.as_mut_ptr(), 3) };
        let len = hdr[1] as usize;
        let mut ok = false;
        let mut out = vec![hdr[0], 0, hdr[2]];
        if len >= 4 {
            let mut body = vec![0u8; len];
            unsafe { std::ptr::copy_nonoverlapping(inp.add(3), body.as_mut_ptr(), len) };
            let (plain, m) = body.split_at(len - 4);
            ok = mic(&cnf[..16], &cnf[16..33], hdr[0], plain) == m;
            out[1] = (len - 4) as u8;
            out.extend_from_slice(plain);
        }
        if !ok {
            self.stats.ccm_mic_fail += 1;
        }
        let outp = host(*self.c.get(&0x510).unwrap_or(&0));
        unsafe { std::ptr::copy_nonoverlapping(out.as_ptr(), outp, out.len()) };
        self.c.insert(0x400, ok as u32); // MICSTATUS
        // The real CCM runs at about radio speed.
        self.schedule(self.now + 10 + len as u64, Ev::CcmDone);
    }
}
