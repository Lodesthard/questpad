//! The whole dongle in simulated time: the emulated chip (hw.rs), the firmware's interrupt-driven
//! code (radio.rs, link.rs, ...), its async tasks (ui.rs, wdt.rs) on the real embassy executor,
//! polled from this thread, and virtual controllers on the air (air.rs). Single-threaded and
//! event-driven, so every run is deterministic.

use std::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::raw::Executor;

use crate::air::AirController;
use crate::gamepad::Side;
use crate::{hw, pairing, radio, serial, settings, ui, vr, wdt};

static PEND: AtomicBool = AtomicBool::new(false);

/// embassy-executor's wake-up hook (no platform feature: the simulator provides it).
#[export_name = "__pender"]
fn pender(_context: *mut ()) {
    PEND.store(true, Ordering::Relaxed);
}

pub struct World {
    pub devs: Vec<AirController>,
    exec: &'static Executor,
    mock_us: u64,
    /// The firmware asked for a system reset (mode change, forget pairing, firmware update).
    pub reset: bool,
}

/// Longest stretch simulated without a look at the executor (its timers).
const MAX_STEP_US: u64 = 500;

impl World {
    /// Power up like firmware/src/main.rs: watchdog, settings, radio, link from the stored
    /// pairing, then the tasks (LED, button, pairing, mode, watchdog). USB is not simulated: the
    /// scenarios read the pad state / VR reports and call the serial handler directly.
    pub fn boot() -> World {
        install_panic_hook();
        wdt::start();
        let mode = settings::load().mode;
        vr::ENABLED.store(mode == settings::Mode::Vr, Ordering::Relaxed);
        radio::init();
        serial::start_stored_link();
        let exec: &'static Executor = Box::leak(Box::new(Executor::new(core::ptr::null_mut())));
        let sp = exec.spawner();
        sp.spawn(ui::led_task(embassy_nrf::pwm::SimplePwm::sim()).unwrap());
        sp.spawn(ui::button_task(embassy_nrf::gpio::Input::sim_button()).unwrap());
        sp.spawn(ui::pair_task().unwrap());
        sp.spawn(ui::mode_task().unwrap());
        sp.spawn(wdt::task().unwrap());
        PEND.store(true, Ordering::Relaxed);
        let mut w = World { devs: Vec::new(), exec, mock_us: 0, reset: false };
        w.poll();
        w
    }

    /// Store a pairing record in flash before `boot`, as a previous pairing would have.
    pub fn store_pairing(network_id: u32, key: [u8; 16]) {
        pairing::store(&pairing::Pairing { network_id, key });
    }

    /// A controller already paired with `network_id` / `key` (not yet awake).
    pub fn add_paired(&mut self, side: Side, network_id: u32, key: [u8; 16]) -> usize {
        let n = self.devs.len() as u8;
        let mut d = AirController::new(side, n + 1, n);
        d.pairing = Some((network_id, key));
        self.devs.push(d);
        self.devs.len() - 1
    }

    pub fn add_unpaired(&mut self, side: Side) -> usize {
        let n = self.devs.len() as u8;
        self.devs.push(AirController::new(side, n + 1, n));
        self.devs.len() - 1
    }

    pub fn now(&self) -> u64 {
        hw::now()
    }

    fn poll(&mut self) {
        let mut rounds = 0;
        while !self.reset && PEND.swap(false, Ordering::Relaxed) && rounds < 64 {
            rounds += 1;
            let exec = self.exec;
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| unsafe { exec.poll() }));
            if let Err(e) = r {
                if e.downcast_ref::<cortex_m::SysReset>().is_some() {
                    self.reset = true;
                } else {
                    std::panic::resume_unwind(e);
                }
            }
        }
    }

    /// Simulate until `t` (us since power-up).
    pub fn run_until(&mut self, t: u64) {
        loop {
            let now = hw::now();
            if now >= t || self.reset {
                break;
            }
            let mut next = (now + MAX_STEP_US).min(t);
            if let Some(e) = hw::next_event() {
                next = next.min(e.max(now));
            }
            for d in &self.devs {
                if let Some(e) = d.next_time() {
                    next = next.min(e.max(now));
                }
            }
            if next > self.mock_us {
                embassy_time::MockDriver::get().advance(embassy_time::Duration::from_micros(next - self.mock_us));
                self.mock_us = next;
            }
            hw::advance_to(next);
            for pkt in hw::take_outbox() {
                for d in &mut self.devs {
                    d.on_air(&pkt);
                }
            }
            for d in &mut self.devs {
                d.run(next);
            }
            // Packets the devices just sent may already have produced dongle traffic.
            for pkt in hw::take_outbox() {
                for d in &mut self.devs {
                    d.on_air(&pkt);
                }
            }
            self.poll();
        }
    }

    pub fn run_for_ms(&mut self, ms: u64) {
        let t = hw::now() + ms * 1000;
        self.run_until(t);
    }

    /// Hold the dongle button for `ms`.
    pub fn press_button(&mut self, ms: u64) {
        embassy_nrf::gpio::BUTTON_PRESSED.store(true, Ordering::Relaxed);
        self.run_for_ms(ms);
        embassy_nrf::gpio::BUTTON_PRESSED.store(false, Ordering::Relaxed);
        self.run_for_ms(50);
    }

    /// LED duty per channel (R, G, B on the dongle), as last set.
    pub fn led(&self) -> [u16; 3] {
        let d = |i: usize| embassy_nrf::pwm::DUTY[i].load(Ordering::Relaxed);
        [d(0), d(1), d(2)]
    }
}

fn install_panic_hook() {
    let default = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if info.payload().downcast_ref::<cortex_m::SysReset>().is_none() {
            default(info);
        }
    }));
}
