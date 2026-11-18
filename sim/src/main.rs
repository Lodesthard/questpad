//! QuestPad simulator: the dongle firmware on an emulated nRF52840, with virtual Touch controllers,
//! so it can be tested without a dongle or controllers.
//!
//!   cargo run --release -- test            # scripted scenarios with pass / fail checks
//!   cargo run --release -- budget          # what each sensor imperfection costs (measurements)
//!   cargo run --release -- live [--auto]   # real time; reports over UDP for host/vrsim_view.py
//!
//! Firmware modules compiled here unchanged (by path): radio, link, pairing, settings, serial,
//! gamepad, vr, ds4, xinput, ui, wdt, cobs, board. Their register and flash accesses go through
//! `crate::hw`, which here is the peripheral emulator (src/hw.rs). Not simulated: USB (main.rs;
//! the scenarios read the pad state and VR reports, and call the serial handler directly) and the
//! RAM flashing routine of selfupdate.rs (stand-in below).
//!
//! Two levels of scenario: "direct" ones feed `gamepad::link_notifications` like the link does
//! (ctrl.rs: fast, for the IMU / fusion maths), "stack" ones run everything from the radio up with
//! controllers on the air (air.rs, world.rs). Docs: docs/vr.md (Simulator).

#![allow(dead_code, static_mut_refs)]

#[path = "../../firmware/src/board.rs"]
mod board;
#[path = "../../firmware/src/cobs.rs"]
mod cobs;
#[path = "../../firmware/src/ds4.rs"]
mod ds4;
#[path = "../../firmware/src/gamepad.rs"]
mod gamepad;
#[path = "../../firmware/src/link.rs"]
mod link;
#[path = "../../firmware/src/pairing.rs"]
mod pairing;
#[path = "../../firmware/src/radio.rs"]
mod radio;
#[path = "../../firmware/src/serial.rs"]
mod serial;
#[path = "../../firmware/src/settings.rs"]
mod settings;
#[path = "../../firmware/src/ui.rs"]
mod ui;
#[path = "../../firmware/src/vr.rs"]
mod vr;
#[path = "../../firmware/src/wdt.rs"]
mod wdt;
#[path = "../../firmware/src/xinput.rs"]
mod xinput;

mod air;
mod ctrl;
mod hw;
mod live;
mod math;
mod report;
mod scenarios;
mod stack;
mod world;

/// Stand-in for firmware/src/selfupdate.rs: same staging and CRC check; a good image is "flashed"
/// into the emulated flash at 0x1000 and the chip resets (the real one runs a RAM routine for that).
mod selfupdate {
    pub const STAGE_LEN: usize = 160 * 1024;
    static mut STAGE: Vec<u8> = Vec::new();
    static mut LEN: usize = 0;

    pub fn begin(len: usize) -> bool {
        if len > STAGE_LEN || len == 0 {
            return false;
        }
        unsafe {
            STAGE = vec![0xFF; len];
            LEN = len;
        }
        true
    }

    pub fn data(off: usize, bytes: &[u8]) -> bool {
        unsafe {
            if off + bytes.len() > LEN {
                return false;
            }
            STAGE[off..off + bytes.len()].copy_from_slice(bytes);
        }
        true
    }

    pub fn crc32(data: &[u8]) -> u32 {
        let mut crc = 0xFFFF_FFFFu32;
        for &b in data {
            crc ^= b as u32;
            for _ in 0..8 {
                crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
            }
        }
        !crc
    }

    pub fn commit(len: usize, crc: u32) -> bool {
        let img = unsafe { STAGE.clone() };
        if len != unsafe { LEN } || crc32(&img[..len]) != crc {
            return false;
        }
        crate::hw::with(|c| c.flash[0x1000..0x1000 + len].copy_from_slice(&img[..len]));
        cortex_m::peripheral::SCB::sys_reset()
    }
}

/// Advance the firmware's clock (embassy-time mock driver) without the chip (direct scenarios).
pub fn advance_us(us: u64) {
    embassy_time::MockDriver::get().advance(embassy_time::Duration::from_micros(us));
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let code = match args.first().map(String::as_str) {
        Some("test") => scenarios::run(args.get(1).map(String::as_str)),
        Some("budget") => scenarios::budget(args.get(1).map(String::as_str)),
        Some("live") => live::run(&args[1..]),
        _ => {
            eprintln!("usage: questpad-sim test [scenario] | budget [case] | live [--auto] [--view host:port] [--listen host:port] [--serial host:port]");
            2
        }
    };
    std::process::exit(code);
}
