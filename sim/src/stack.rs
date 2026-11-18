//! Full-stack scenarios: the firmware from the radio up (radio.rs, link.rs, pairing.rs, settings.rs,
//! ui.rs, serial.rs, gamepad.rs, vr.rs, ds4.rs, xinput.rs, wdt.rs) on the emulated chip, with
//! controllers on the air. Each runs in its own process (scenarios.rs), so flash and statics start
//! blank.

use crate::gamepad::{self, Side};
use crate::math;
use crate::report::Report;
use crate::scenarios::Run;
use crate::world::World;
use crate::{air, cobs, ds4, hw, link, pairing, radio, serial, settings, ui, vr, xinput};

pub const NET: u32 = 0x5A3C_96E1;
pub const KEY: [u8; 16] = [0x51, 0x50, 0x2D, 0x6B, 0x65, 0x79, 0x2D, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38];
const R: usize = 1;

/// Flash as a dongle that was set up before: pairing record, settings with `mode`.
pub fn provision(mode: settings::Mode) {
    World::store_pairing(NET, KEY);
    provision_settings(settings::Settings { mode, ..settings::DEFAULT });
}

pub fn provision_settings(s: settings::Settings) {
    settings::store(&s);
}

/// Run `ms`, keeping the newest VR report per side and every serial event (encoded frames).
pub fn run_collect(w: &mut World, ms: u64, last: &mut [Option<Report>; 2], events: &mut Vec<Vec<u8>>) {
    let end = w.now() + ms * 1000;
    while w.now() < end && !w.reset {
        let step = (w.now() + 2000).min(end);
        w.run_until(step);
        for (i, side) in [Side::Left, Side::Right].into_iter().enumerate() {
            if let Some(r) = Report::take(side) {
                last[i] = Some(r);
            }
        }
        while let Ok(ev) = radio::EVENTS.try_receive() {
            let mut out = [0u8; 300];
            let n = serial::encode_event(ev, &mut out);
            if events.len() < 20_000 {
                events.push(out[..n].to_vec());
            }
        }
    }
}

fn run(w: &mut World, ms: u64) -> [Option<Report>; 2] {
    let mut last = [None, None];
    run_collect(w, ms, &mut last, &mut Vec::new());
    last
}

/// Run `ms`; the serial events sent meanwhile.
fn run_events(w: &mut World, ms: u64) -> Vec<Vec<u8>> {
    let mut events = Vec::new();
    run_collect(w, ms, &mut [None, None], &mut events);
    events
}

/// Boot a provisioned dongle (`mode`) with the given controllers awake.
fn joined(mode: settings::Mode, sides: &[Side]) -> World {
    provision(mode);
    let mut w = World::boot();
    for &s in sides {
        let d = w.add_paired(s, NET, KEY);
        let t = w.now();
        w.devs[d].wake(t);
    }
    run(&mut w, 1000);
    w
}

/// Whether `f` ends in a system reset.
fn resets(f: impl FnOnce()) -> bool {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(()) => false,
        Err(e) if e.downcast_ref::<cortex_m::SysReset>().is_some() => true,
        Err(e) => std::panic::resume_unwind(e),
    }
}

fn cmd(c: &[u8]) -> Vec<u8> {
    let mut out = [0u8; 300];
    let n = serial::handle(c, &mut out);
    out[..n].to_vec()
}

// ---- scenarios ----

pub fn link_join(r: &mut Run) {
    provision(settings::Mode::Vr);
    let mut w = World::boot();
    r.check(link::running(), "link hosted from the stored pairing at power-up".into());
    let d = w.add_paired(Side::Right, NET, KEY);
    let t = w.now();
    w.devs[d].wake(t);
    let last = run(&mut w, 1500);
    let s = w.devs[d].stats;
    let slots = link::slots();
    r.check(slots.len() == 1 && slots[0].1, format!("dongle: 1 controller, right hand ({slots:?})"));
    r.check(w.devs[d].slot.is_some(), format!("controller: joined slot {:?} ({} join requests)", w.devs[d].slot, s.join_requests));
    r.check(s.hop_errors == 0 && s.ts_errors == 0, format!(
        "{} beacons followed, hopping / timestamp errors {} / {}",
        s.beacons, s.hop_errors, s.ts_errors
    ));
    let st = link::stats();
    r.check(st.mic_bad == 0 && st.mic_ok > 500, format!("uplinks decrypted {} (MIC failures {})", st.mic_ok, st.mic_bad));
    r.check(st.late == 0, format!("late beacons {}", st.late));
    match last[R] {
        Some(rep) => {
            r.check(rep.has(vr::FLAG_CONNECTED) && rep.has(vr::FLAG_ORIENTATION), "VR report: connected, orientation".into());
            r.check(rep.has(vr::FLAG_IMU_SCALE), "IMU units read from the controller over the air (0x32)".into());
        }
        None => r.check(false, "no VR report".into()),
    }
    let c = hw::with(|c| c.stats);
    r.check(c.rx_missed < 20, format!("radio: {} packets received, {} missed", c.rx_ok, c.rx_missed));
}

pub fn pairing_button(r: &mut Run) {
    provision_settings(settings::Settings { mode: settings::Mode::Vr, ..settings::DEFAULT });
    let mut w = World::boot();
    r.check(!link::running() && pairing::load().is_none(), "blank dongle: no pairing, no link".into());
    w.run_for_ms(200);
    let [red, green, blue] = w.led();
    r.check(red > 0 && green == 0 && blue == 0, format!("LED red (not paired): {:?}", w.led()));
    let d = w.add_unpaired(Side::Right);
    let t = w.now();
    w.devs[d].enter_pairing(t);
    w.press_button(100);
    r.check(ui::pair_state() != ui::PairState::Idle, format!("button tap: pairing started ({})", ui::pair_state() as u8));
    let deadline = w.now() + 5_000_000;
    while ui::pair_state() != ui::PairState::Done && w.now() < deadline {
        w.run_for_ms(10);
    }
    r.check(ui::pair_state() == ui::PairState::Done, format!(
        "pairing done in {:.2} s ({} DM commands, {} adverts)",
        w.now() as f64 * 1e-6,
        w.devs[d].stats.dm_commands,
        w.devs[d].stats.adverts
    ));
    let stored = pairing::load().map(|p| (p.network_id, p.key));
    r.check(stored.is_some() && stored == w.devs[d].pairing, "record in flash = record the controller received".into());
    let last = run(&mut w, 1500);
    r.check(link::slots().len() == 1, "the new controller joined the link".into());
    r.check(last[R].is_some_and(|x| x.has(vr::FLAG_CONNECTED)), "VR reports flow".into());
    let [red, green, blue] = w.led();
    r.check(red > green && green > 0 && blue == 0, format!("LED solid orange (VR mode, connected): {:?}", w.led()));
}

pub fn pairing_timeout(r: &mut Run) {
    provision_settings(settings::DEFAULT);
    let mut w = World::boot();
    w.press_button(100);
    w.run_for_ms(89_000);
    r.check(ui::pair_state() == ui::PairState::Waiting, "still waiting after 89 s".into());
    w.run_for_ms(2_000);
    r.check(ui::pair_state() == ui::PairState::TimedOut, "timed out after 90 s".into());
    r.check(pairing::load().is_none() && !link::running(), "nothing stored, no link".into());
}

pub fn pairing_cancel(r: &mut Run) {
    provision(settings::Mode::Hid);
    let mut w = World::boot();
    w.press_button(100);
    w.run_for_ms(1000);
    w.press_button(100);
    w.run_for_ms(100);
    r.check(ui::pair_state() == ui::PairState::Cancelled, "second tap cancels".into());
    r.check(link::running(), "back to hosting the stored pairing".into());
}

pub fn two_controllers(r: &mut Run) {
    let mut w = joined(settings::Mode::Vr, &[Side::Left, Side::Right]);
    let slots = link::slots();
    r.check(slots.len() == 2 && slots.iter().filter(|s| s.1).count() == 1, format!("both joined: {slots:?}"));
    let (l, rr) = (0, 1);
    w.devs[l].c.buttons = 0x01; // X
    w.devs[l].c.stick = (0, 32000); // up
    w.devs[rr].c.buttons = 0x03; // A + B
    w.devs[rr].c.trigger = 4095;
    w.devs[rr].c.grip = 4095;
    let last = run(&mut w, 100);
    r.check(last[0].is_some_and(|x| !x.has(vr::FLAG_RIGHT) && x.buttons == 0x01), "left VR report: X".into());
    r.check(last[1].is_some_and(|x| x.has(vr::FLAG_RIGHT) && x.buttons == 0x03 && x.trigger == 4095), "right VR report: A + B, trigger".into());
    let Some(pad) = gamepad::PAD.try_take() else {
        r.check(false, "no pad state".into());
        return;
    };
    use gamepad::*;
    let want = BTN_X | BTN_A | BTN_B | BTN_RB;
    r.check(pad.buttons == want && pad.rt == 255 && pad.ly < -30000, format!(
        "combined pad: buttons {:#06x} ({want:#06x}), RT {}, LY {}",
        pad.buttons, pad.rt, pad.ly
    ));
    let d = ds4::report(&pad);
    r.check(d[5] & 0xF0 == 0x70 && d[6] & 0x0A == 0x0A && d[9] == 255 && d[2] < 10, format!(
        "DS4 report: face {:#04x} (Square Cross Circle), R1 + R2 {:#04x}, R2 {}, LY {}",
        d[5] & 0xF0,
        d[6],
        d[9],
        d[2]
    ));
    let x = xinput::report(&pad);
    r.check(x[3] == 0x10 | 0x20 | 0x40 | 0x02 && x[5] == 255, format!("XInput report: buttons {:#04x}, RT {}", x[3], x[5]));
    let h = gamepad::hid_report(&pad);
    r.check(u16::from_le_bytes([h[0], h[1]]) == want && h[4] == 255, "HID report: buttons, RT".into());
}

pub fn haptics(r: &mut Run) {
    let mut w = joined(settings::Mode::Vr, &[Side::Right]);
    let log = w.devs[0].motor_log.clone();
    let buzz = log.first().map(|x| x.1) == Some(settings::DEFAULT.buzz_amp)
        && log.get(1).is_some_and(|x| x.1 == 0 && x.0 - log[0].0 < 200_000);
    r.check(buzz, format!("connect buzz: {:?}", log.iter().map(|x| x.1).collect::<Vec<_>>()));
    let t0 = w.now();
    let _ = vr::command(Side::Right, &[0x10, 150, 0, 0]);
    run(&mut w, 30);
    let set = w.devs[0].motor_log.iter().find(|x| x.0 >= t0 && x.1 == 150).map(|x| x.0 - t0);
    r.check(w.devs[0].motor == 150, format!("OUT haptic 150 -> controller motor 150 after {:?} us", set));
    let _ = vr::command(Side::Right, &[0x10, 0, 0, 0]);
    run(&mut w, 30);
    r.check(w.devs[0].motor == 0, "OUT haptic 0 -> motor off".into());
    let st = link::stats();
    r.check(st.dl_ok >= 4 && st.dl_fail == 0, format!("downlink requests acked {} (failed {})", st.dl_ok, st.dl_fail));
}

pub fn power_off_gesture(r: &mut Run) {
    let mut w = joined(settings::Mode::Hid, &[Side::Left, Side::Right]);
    w.devs[R].c.buttons = 0x08; // Oculus held
    run(&mut w, 3200);
    w.devs[R].c.buttons = 0;
    let buzzed = w.devs.iter().all(|d| d.motor_log.iter().filter(|x| x.1 > 0).count() >= 2);
    r.check(buzzed, "3 s hold: both controllers buzz (armed)".into());
    run(&mut w, 2500);
    r.check(w.devs.iter().all(|d| d.state == air::State::Off), format!(
        "set down still: both told to power off ({:?})",
        w.devs.iter().map(|d| d.state).collect::<Vec<_>>()
    ));
    run(&mut w, 1500);
    r.check(link::slots().is_empty(), "their slots are freed".into());
    let t = w.now();
    w.devs[R].wake(t);
    run(&mut w, 1000);
    r.check(link::slots().len() == 1, "picked up again: rejoins".into());
}

pub fn mode_gesture(r: &mut Run) {
    let mut w = joined(settings::Mode::Vr, &[Side::Right]);
    w.devs[0].c.buttons = 0x08;
    w.devs[0].c.trigger = 4095;
    run(&mut w, 6500);
    r.check(w.reset, "Oculus + trigger 5 s: the dongle restarts".into());
    let m = settings::load().mode;
    r.check(m == settings::Mode::Hid, format!("new mode in flash: {} (VR -> HID)", m as u8));
}

pub fn button_mode(r: &mut Run) {
    provision(settings::Mode::Ds4);
    let mut w = World::boot();
    w.press_button(3300);
    w.run_for_ms(1000);
    r.check(w.reset, "button held 3 s: restart".into());
    let m = settings::load().mode;
    r.check(m == settings::Mode::XInput, format!("mode DS4 -> XInput ({})", m as u8));
    r.check(pairing::load().is_some(), "pairing kept".into());
}

pub fn button_forget(r: &mut Run) {
    provision(settings::Mode::Hid);
    let mut w = World::boot();
    w.press_button(10_500);
    w.run_for_ms(1500);
    r.check(w.reset, "button held 10 s: restart".into());
    r.check(pairing::load().is_none(), "pairing forgotten".into());
}

pub fn serial_protocol(r: &mut Run) {
    let mut w = joined(settings::Mode::Vr, &[Side::Right]);
    let p = cmd(&[0x05]);
    r.check(p[0] == 0x84 && String::from_utf8_lossy(&p[5..]).starts_with("questpad "), format!(
        "PING -> {:?}",
        String::from_utf8_lossy(&p[5..])
    ));
    let s = cmd(&[0x2E]);
    r.check(s[0] == 0xA9 && s[1] == 3 && s[2] == 1 && s[4] == 1 && s[6] == 1 && s[7] == 80, format!(
        "STATUS: mode {} paired {} controllers {} right {} battery {}",
        s[1], s[2], s[4], s[6], s[7]
    ));
    let g = cmd(&[0x29]);
    r.check(g[0] == 0xA6 && g.len() == 1 + settings::LEN && g[1] == 3, "SETTINGS_GET: 20 bytes, mode VR".into());
    let mut set = g[1..].to_vec();
    set[10] = 99; // LED level
    set.push(1); // persist
    let ok = cmd(&[[0x2A].as_slice(), &set].concat());
    r.check(ok == [0xF0, 0x2A, 1] && settings::get().led_level == 99, "SETTINGS_SET (persist): applied".into());
    r.check(settings::load().led_level == 99, "... and in flash".into());
    // Persisting stops the radio for the flash write and restarts the link: controllers drop and
    // rejoin (existing behaviour, worth knowing when saving from the panel during play).
    r.check(link::slots().is_empty(), "persisting restarts the link (controller dropped)".into());
    run(&mut w, 1000);
    r.check(link::slots().len() == 1, "... and the controller rejoins within 1 s".into());
    let e = cmd(&[0x2C]);
    r.check(e[0] == 0xA8 && e[1] == 1 && e[2..6] == NET.to_le_bytes() && e[6..22] == KEY, "PAIR_EXPORT: the stored record".into());
    let ls = cmd(&[0x27]);
    let ok = ls.len() == 11 && ls[0] == 0xA5 && ls[2] == 1 && ls[3..11] == w.devs[0].dev_id;
    r.check(ok, format!("LINK_SLOTS: slot, right, device id ({ls:?})"));
    if !ok {
        return;
    }
    let slot = ls[1];
    let q = cmd(&[0x26, slot, 0x24, 1]);
    r.check(q == [0x93, 1], "LINK_REQ read 0x24 queued".into());
    let events = run_events(&mut w, 100);
    let reply = events.iter().find(|f| f[0] == 0xA1 && f.len() > 6 && f[4] == slot && f[5] == 0x24 && f[6] & 0x40 == 0);
    r.check(reply.is_some(), "register reply arrives as EV_LINK_DATA".into());
    let _ = cmd(&[0x25, 0, 120]);
    run(&mut w, 30);
    r.check(w.devs[0].motor == 120, "RUMBLE -> right controller motor 120".into());
    let st = cmd(&[0x21]);
    let frames = u32::from_le_bytes([st[1], st[2], st[3], st[4]]);
    r.check(st[0] == 0xA3 && frames > 500, format!("LINK_STATS: {frames} frames"));
    let mut cobs_ok = true;
    for f in events.iter().take(200) {
        let mut enc = [0u8; 300];
        let n = cobs::encode(f, &mut enc);
        let mut dec = [0u8; 300];
        cobs_ok &= !enc[..n].contains(&0) && cobs::decode(&enc[..n], &mut dec) == Some(f.len()) && dec[..f.len()] == f[..];
    }
    r.check(cobs_ok, format!("COBS framing round-trips {} event frames", events.len().min(200)));
    let _ = cmd(&[0x28]);
    run(&mut w, 100);
    r.check(w.devs[0].state == air::State::Off, "POWER_OFF -> controller off".into());
}

pub fn firmware_update(r: &mut Run) {
    let _w = joined(settings::Mode::Hid, &[]);
    let img: Vec<u8> = (0..20_000u32).map(|i| (i * 7 + i / 255) as u8).collect();
    let crc = crate::selfupdate::crc32(&img);
    let len = (img.len() as u32).to_le_bytes();
    r.check(cmd(&[[0x70].as_slice(), &len].concat()) == [0xF0, 0x70, 1], "FW_BEGIN".into());
    let mut ok = true;
    for (i, chunk) in img.chunks(200).enumerate() {
        let off = ((i * 200) as u32).to_le_bytes();
        ok &= cmd(&[[0x71].as_slice(), &off, chunk].concat()) == [0xF0, 0x71, 1];
    }
    r.check(ok, "FW_DATA: 100 chunks".into());
    let bad = cmd(&[[0x72].as_slice(), &len, &(crc ^ 1).to_le_bytes()].concat());
    r.check(bad == [0xF0, 0x72, 0], "FW_COMMIT with a wrong CRC refused".into());
    let reset = resets(|| {
        cmd(&[[0x72].as_slice(), &len, &crc.to_le_bytes()].concat());
    });
    let flashed = hw::with(|c| c.flash[0x1000..0x1000 + img.len()] == img[..]);
    r.check(reset && flashed, "FW_COMMIT with the right CRC: image in flash at 0x1000, restart".into());
}

pub fn wrong_key(r: &mut Run) {
    provision(settings::Mode::Vr);
    let mut w = World::boot();
    let mut bad = KEY;
    bad[0] ^= 0xFF;
    let d = w.add_paired(Side::Right, NET, bad);
    let t = w.now();
    w.devs[d].wake(t);
    run(&mut w, 1500);
    let st = link::stats();
    r.check(link::slots().is_empty() && w.devs[d].slot.is_none(), "controller with another key never joins".into());
    r.check(st.mic_bad > 0 && st.joins == 0, format!("dongle: {} MIC failures, {} joins", st.mic_bad, st.joins));
}

pub fn dropout(r: &mut Run) {
    let mut w = joined(settings::Mode::Vr, &[Side::Right]);
    let slot = w.devs[0].slot;
    w.devs[0].power_off(); // batteries out: no goodbye
    let mut last = [None, None];
    let mut events = Vec::new();
    run_collect(&mut w, 1500, &mut last, &mut events);
    r.check(link::slots().is_empty(), "slot freed after 1 s of silence".into());
    r.check(events.iter().any(|f| f[0] == 0xA2), "EV_LINK_DROP sent to the host".into());
    r.check(last[R].is_some_and(|x| !x.has(vr::FLAG_CONNECTED)), "VR report: disconnected".into());
    let t = w.now();
    w.devs[0].wake(t);
    let last = run(&mut w, 1000);
    r.check(w.devs[0].slot.is_some() && link::slots().len() == 1, format!("back: rejoins (slot {:?} -> {:?})", slot, w.devs[0].slot));
    r.check(last[R].is_some_and(|x| x.has(vr::FLAG_CONNECTED)), "VR report: connected again".into());
}

pub fn low_battery(r: &mut Run) {
    let mut w = joined(settings::Mode::Hid, &[Side::Left, Side::Right]);
    w.devs[R].c.battery = 9;
    let t0 = w.now();
    run(&mut w, 1500);
    let on: Vec<_> = w.devs[R].motor_log.iter().filter(|x| x.0 >= t0).collect();
    let dur = match (on.first(), on.get(1)) {
        (Some(a), Some(b)) if a.1 > 0 && b.1 == 0 => Some(b.0 - a.0),
        _ => None,
    };
    r.check(dur.is_some_and(|d| (700_000..900_000).contains(&d)), format!("battery 9 %: 0.8 s buzz ({dur:?} us)"));
    r.check(w.devs[0].motor_log.iter().all(|x| x.0 < t0), "the other controller stays quiet".into());
}

pub fn idle_off(r: &mut Run) {
    World::store_pairing(NET, KEY);
    provision_settings(settings::Settings { mode: settings::Mode::Hid, idle_min: 1, ..settings::DEFAULT });
    let mut w = World::boot();
    let d = w.add_paired(Side::Right, NET, KEY);
    let t = w.now();
    w.devs[d].wake(t);
    run(&mut w, 55_000);
    r.check(w.devs[d].state == air::State::Linked, "still on after 55 s untouched".into());
    run(&mut w, 8_000);
    r.check(w.devs[d].state == air::State::Off, "powered off after 1 min untouched (idle setting)".into());
}

pub fn watchdog(r: &mut Run) {
    let mut w = joined(settings::Mode::Hid, &[Side::Right]);
    run(&mut w, 20_000);
    let (started, gap) = hw::with(|c| (c.wdt_started, c.wdt_max_gap));
    r.check(started && gap <= 1_100_000, format!("watchdog started, fed at least every {:.2} s (8 s limit)", gap as f64 * 1e-6));
}

pub fn vr_motion(r: &mut Run) {
    let mut w = joined(settings::Mode::Vr, &[Side::Right]);
    let mut worst: f64 = 0.0;
    for (axis, deg, secs) in [([0.0, 1.0, 0.0], 90.0, 1.0), ([1.0, 0.0, 0.0], 45.0, 1.0), ([0.0, 0.0, 1.0], -60.0, 1.0)] {
        w.devs[0].w = axis.map(|a| a * f64::to_radians(deg));
        let end = w.now() + (secs * 1e6) as u64;
        while w.now() < end {
            let last = run(&mut w, 10);
            if let Some(rep) = last[R] {
                worst = worst.max(math::angle_deg(rep.q, w.devs[0].c.q));
            }
        }
    }
    w.devs[0].w = [0.0; 3];
    let last = run(&mut w, 500);
    let fin = last[R].map_or(99.0, |rep| math::angle_deg(rep.q, w.devs[0].c.q));
    r.check(worst < 1.5, format!("over the radio link, worst error while turning {worst:.2}° (< 1.5°, includes up to a frame of latency)"));
    r.check(fin < 0.5, format!("final error {fin:.2}°"));
}

pub fn hid_mode(r: &mut Run) {
    let mut w = joined(settings::Mode::Hid, &[Side::Right]);
    w.devs[0].c.buttons = 0x01;
    let last = run(&mut w, 100);
    r.check(last[R].is_none(), "HID mode: no VR reports".into());
    r.check(gamepad::PAD.try_take().is_some_and(|p| p.buttons == gamepad::BTN_A), "pad state: A".into());
}

fn poke(addr: u32, val: u32) {
    cmd(&[[0x01].as_slice(), &addr.to_le_bytes(), &val.to_le_bytes()].concat());
}

/// Raw radio access as host/sniff_adv.py does it: poke the address, continuous RX, EV_RX frames.
pub fn sniff(r: &mut Run) {
    provision_settings(settings::DEFAULT);
    let mut w = World::boot();
    let d = w.add_unpaired(Side::Left);
    let t = w.now();
    w.devs[d].enter_pairing(t);
    let radio = 0x4000_1000;
    poke(radio + 0x514, 8); // PCNF0: [LEN][payload]
    poke(radio + 0x51C, 0xFACE_B00C); // BASE0
    poke(radio + 0x524, 0xAA); // PREFIX0
    poke(radio + 0x530, 1); // RXADDRESSES
    poke(radio + 0x508, 2); // FREQUENCY 2402
    let _ = cmd(&[0x06, 0, 1, 0]); // LAYOUT: LEN at 0, 1 header byte
    let _ = cmd(&[0x02]); // RX
    let events = run_events(&mut w, 350);
    let rx: Vec<_> = events.iter().filter(|f| f[0] == 0x81).collect();
    let ok = rx.iter().filter(|f| f[5] == 1 && f.len() >= 8 + 14 && f[9] == 2).count();
    r.check(ok >= 3, format!("{} advertisements sniffed (EV_RX, CRC ok, version 2) in 350 ms", ok));
    let peek = cmd(&[[0x07].as_slice(), &(radio + 0x51C).to_le_bytes()].concat());
    r.check(peek.len() == 9 && peek[5..9] == 0xFACE_B00Cu32.to_le_bytes(), "PEEK reads the register back".into());
}

/// Pairing driven from the host, as host/dm.py does: DM_START, DM_SEND (read serial, write record).
pub fn dm_host(r: &mut Run) {
    provision_settings(settings::DEFAULT);
    let mut w = World::boot();
    let d = w.add_unpaired(Side::Right);
    let t = w.now();
    w.devs[d].enter_pairing(t);
    let _ = cmd(&[0x10, 0, 0, 0, 0, 0, 0, 0, 0, 1]); // DM_START any
    let events = run_events(&mut w, 300);
    let link = events.iter().find(|f| f[0] == 0x90);
    r.check(link.is_some_and(|f| f[1] == 1 && f[2..6] == w.devs[d].id0.to_le_bytes()), "EV_DM_LINK: connected to the advertising controller".into());
    let sent = cmd(&[0x11, (0x03 << 1) | 1]);
    r.check(sent == [0x93, 1], "DM_SEND read serial accepted".into());
    let events = run_events(&mut w, 50);
    let resp = events.iter().find(|f| f[0] == 0x91);
    r.check(resp.is_some_and(|f| f.len() >= 3 + 14 && &f[3..17] == b"1WMHH000000000"), "EV_DM_RESP: the controller's serial number".into());
    let mut rec = vec![0x11, 0x14 << 1];
    rec.extend_from_slice(&0x1234_5678u32.to_le_bytes());
    rec.extend_from_slice(&KEY);
    let _ = cmd(&rec);
    run(&mut w, 50);
    r.check(w.devs[d].pairing == Some((0x1234_5678, KEY)), "DM_SEND 0x14: the controller took the pairing record".into());
}
