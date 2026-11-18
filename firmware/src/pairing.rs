//! Pairing record (network id + link key) kept in the last application flash page, so the
//! dongle can host the link on its own from power-up.
//!
//! Self-update and the nice!nano's UF2 bootloader only erase the pages the new image occupies,
//! and memory-*.x stops the image below this page, so the record survives reflashing (the
//! dongle's bootloader DFU may wipe it).

const PAGE: u32 = 0xDF000;
const MAGIC: u32 = 0x3152_5051; // "QPR1"
const WORDS: usize = 7; // magic, network_id, key[16], crc

const NVMC_READY: u32 = 0x4001_E400;
const NVMC_CONFIG: u32 = 0x4001_E504;
const NVMC_ERASEPAGE: u32 = 0x4001_E508;

#[derive(Clone, Copy)]
pub struct Pairing {
    pub network_id: u32,
    pub key: [u8; 16],
}

fn words(p: &Pairing) -> [u32; WORDS] {
    let mut w = [0u32; WORDS];
    w[0] = MAGIC;
    w[1] = p.network_id;
    for i in 0..4 {
        w[2 + i] = u32::from_le_bytes([p.key[4 * i], p.key[4 * i + 1], p.key[4 * i + 2], p.key[4 * i + 3]]);
    }
    w[6] = checksum(&w[..6]);
    w
}

fn checksum(w: &[u32]) -> u32 {
    w.iter().fold(0x811C_9DC5u32, |h, &x| (h ^ x).wrapping_mul(0x0100_0193))
}

/// A fresh network: random id (radio-address friendly) and random 128-bit key, from the hardware RNG.
pub fn generate() -> Pairing {
    let mut key = [0u8; 16];
    for b in key.iter_mut() {
        *b = rng_byte();
    }
    loop {
        let id = u32::from_le_bytes([rng_byte(), rng_byte(), rng_byte(), rng_byte()]);
        // Nordic guidance for radio addresses: plenty of bit transitions, no long runs.
        let transitions = (id ^ (id >> 1)) & 0x7FFF_FFFF;
        let has_run = (0..27).any(|i| (id >> i) & 0x3F == 0 || (id >> i) & 0x3F == 0x3F);
        if transitions.count_ones() >= 10 && !has_run && id != crate::radio::ADV_BASE {
            return Pairing { network_id: id, key };
        }
    }
}

fn rng_byte() -> u8 {
    const RNG: u32 = 0x4000_D000;
    crate::hw::write(RNG + 0x504, 1); // CONFIG: bias correction
    crate::hw::write(RNG + 0x100, 0); // EVENTS_VALRDY
    crate::hw::write(RNG, 1); // TASKS_START
    while crate::hw::read(RNG + 0x100) == 0 {}
    crate::hw::write(RNG + 0x004, 1); // TASKS_STOP
    crate::hw::read(RNG + 0x508) as u8
}

/// Erase the stored pairing (the controllers then need pairing again).
pub fn forget() {
    critical_section::with(|_| {
        crate::hw::write(NVMC_CONFIG, 2);
        crate::hw::write(NVMC_ERASEPAGE, PAGE);
        wait_ready();
        crate::hw::write(NVMC_CONFIG, 0);
    });
}

pub fn load() -> Option<Pairing> {
    let mut w = [0u32; WORDS];
    for (i, x) in w.iter_mut().enumerate() {
        *x = crate::hw::read(PAGE + 4 * i as u32);
    }
    if w[0] != MAGIC || w[6] != checksum(&w[..6]) {
        return None;
    }
    let mut key = [0u8; 16];
    for i in 0..4 {
        key[4 * i..4 * i + 4].copy_from_slice(&w[2 + i].to_le_bytes());
    }
    Some(Pairing { network_id: w[1], key })
}

fn wait_ready() {
    while crate::hw::read(NVMC_READY) == 0 {}
}

/// Erase the page and write the record. The CPU stalls for the ~85 ms erase, so callers
/// stop the radio first.
pub fn store(p: &Pairing) -> bool {
    let w = words(p);
    critical_section::with(|_| {
        crate::hw::write(NVMC_CONFIG, 2);
        crate::hw::write(NVMC_ERASEPAGE, PAGE);
        wait_ready();
        crate::hw::write(NVMC_CONFIG, 1);
        for (i, &x) in w.iter().enumerate() {
            crate::hw::write(PAGE + 4 * i as u32, x);
            wait_ready();
        }
        crate::hw::write(NVMC_CONFIG, 0);
    });
    load().is_some_and(|q| q.network_id == p.network_id && q.key == p.key)
}
