//! Real-time mode: the whole dongle (world.rs) with two virtual controllers on the air, steered
//! over UDP (host/vrsim_view.py), plus the dongle's serial port over TCP for the host tools
//! (`QUESTPAD_PORT=socket://127.0.0.1:26772 python host/gui.py`, dsu.py, regs.py, link.py ...).
//!
//! Out (to `--view`, default 127.0.0.1:26770), one datagram each:
//!   - the 64-byte VR state report, byte for byte as on USB (report type 0x01), so a runtime driver
//!     can read it from a socket instead of the USB endpoint;
//!   - ground truth for viewers, 19 bytes: 0xF0, side (0 left, 1 right), true orientation f32
//!     x y z w, the controller's motor amplitude u8. Report readers skip it (type is not 0x01).
//! In (on `--listen`, default 127.0.0.1:26771), ASCII, one controller per datagram:
//!   `L|R [q=x,y,z,w] [b=buttons] [t=touch] [sx=..] [sy=..] [tr=0..4095] [gr=0..4095] [bat=%] [c=0|1]`
//!   `L|R cmd=<hex bytes>`  e.g. `R cmd=10c86400` (an OUT command: haptic 200 for 100 ms)
//! Serial (on `--serial`, default 127.0.0.1:26772): the CDC byte stream (COBS frames).
//!
//! The emulated flash is kept in `--flash` (default: live-flash.bin next to the executable): the
//! first run provisions a pairing for the two virtual controllers and VR mode. A system reset (mode
//! change, firmware update, forget) saves the flash and restarts the simulator, as the dongle would.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, UdpSocket};
use std::time::{Duration, Instant};

use crate::gamepad::Side;
use crate::math::{self, Quat};
use crate::report::Report;
use crate::world::World;
use crate::{cobs, hw, radio, serial, settings, stack, vr};

struct Hand {
    target: Quat,
    steered: bool,
    stop: Option<embassy_time::Instant>,
    reports: u32,
    last: Option<[u8; 64]>,
}

pub fn run(args: &[String]) -> i32 {
    let mut auto = false;
    let mut view = "127.0.0.1:26770".to_string();
    let mut listen = "127.0.0.1:26771".to_string();
    let mut serial_addr = "127.0.0.1:26772".to_string();
    let mut flash_path = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.join("live-flash.bin")))
        .unwrap_or_else(|| "live-flash.bin".into());
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--auto" => auto = true,
            "--view" => view = it.next().cloned().unwrap_or(view),
            "--listen" => listen = it.next().cloned().unwrap_or(listen),
            "--serial" => serial_addr = it.next().cloned().unwrap_or(serial_addr),
            "--flash" => flash_path = it.next().map(Into::into).unwrap_or(flash_path),
            other => {
                eprintln!("unknown option {other}");
                return 2;
            }
        }
    }
    let udp = match UdpSocket::bind(&listen) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("can't listen on {listen}: {e}");
            return 1;
        }
    };
    udp.set_nonblocking(true).expect("nonblocking socket");
    let tcp = match TcpListener::bind(&serial_addr) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("can't listen on {serial_addr}: {e}");
            return 1;
        }
    };
    tcp.set_nonblocking(true).expect("nonblocking socket");

    // Flash: from the last run, else a fresh dongle paired with the two virtual controllers.
    match std::fs::read(&flash_path) {
        Ok(img) if img.len() == hw::with(|c| c.flash.len()) => hw::with(|c| c.flash = img),
        _ => stack::provision(settings::Mode::Vr),
    }
    let mut w = World::boot();
    let mode = settings::get().mode;
    for side in [Side::Left, Side::Right] {
        let d = w.add_paired(side, stack::NET, stack::KEY);
        w.devs[d].c.bias_dps = if side == Side::Left { [0.6, -1.1, 0.3] } else { [1.2, -0.8, 0.5] };
        let t = w.now();
        w.devs[d].wake(t);
    }
    println!(
        "questpad-sim live: firmware in {} mode | VR reports -> {view} | control <- {listen} | serial on {serial_addr}{}",
        ["HID", "DS4", "XInput", "VR"][mode as usize],
        if auto { " | auto motion" } else { "" }
    );
    println!("  3D view:        python host/vrsim_view.py");
    println!("  control panel:  QUESTPAD_PORT=socket://{serial_addr} python host/gui.py   (also dsu.py, regs.py, link.py)");
    if mode != settings::Mode::Vr {
        println!("  (not in VR mode: no VR reports; switch with the panel, or delete {})", flash_path.display());
    }

    let mut hands: Vec<Hand> =
        (0..2).map(|_| Hand { target: math::IDENTITY, steered: false, stop: None, reports: 0, last: None }).collect();
    let mut client: Option<TcpStream> = None;
    let mut rx_acc: Vec<u8> = Vec::new();
    let start = Instant::now();
    let t0 = w.now();
    let mut last_status = Instant::now();
    let mut since = [0u32; 2];
    let mut buf = [0u8; 2048];
    loop {
        while let Ok((n, _)) = udp.recv_from(&mut buf) {
            control(&mut w, &mut hands, &String::from_utf8_lossy(&buf[..n]));
        }
        // Serial: one client at a time (like the CDC port).
        if client.is_none() {
            if let Ok((s, _)) = tcp.accept() {
                let _ = s.set_nonblocking(true);
                let _ = s.set_nodelay(true);
                client = Some(s);
                rx_acc.clear();
            }
        }
        if let Some(s) = client.as_mut() {
            match s.read(&mut buf) {
                Ok(0) => client = None,
                Ok(n) => rx_acc.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(_) => client = None,
            }
        }
        while let Some(pos) = rx_acc.iter().position(|&b| b == 0) {
            let frame: Vec<u8> = rx_acc.drain(..=pos).collect();
            let mut dec = [0u8; 300];
            if let Some(len) = cobs::decode(&frame[..frame.len() - 1], &mut dec) {
                let mut out = [0u8; 300];
                let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| serial::handle(&dec[..len], &mut out)));
                match res {
                    Ok(n) if n > 0 => send_frame(&mut client, &out[..n]),
                    Ok(_) => {}
                    Err(e) if e.downcast_ref::<cortex_m::SysReset>().is_some() => w.reset = true,
                    Err(e) => std::panic::resume_unwind(e),
                }
            }
        }

        // Simulated time follows the wall clock, in 1 ms steps.
        let target = t0 + start.elapsed().as_micros() as u64;
        let mut steps = 0;
        while w.now() < target && steps < 100 && !w.reset {
            steps += 1;
            let t = w.now() as f64 * 1e-6;
            for (i, h) in hands.iter().enumerate() {
                let d = &mut w.devs[i];
                d.w = if auto && !h.steered {
                    let k = if i == 0 { 1.0 } else { -1.3 };
                    [0.9 * (0.7 * t * k).sin(), 1.4 * (0.4 * t).sin() * k, 0.6 * (1.1 * t + i as f64).sin()]
                } else {
                    clamp(math::rate_between(d.c.q, h.target, 0.04), 15.0)
                };
            }
            let next = (w.now() + 1000).min(target);
            w.run_until(next);
            let now = embassy_time::Instant::now();
            for (i, h) in hands.iter_mut().enumerate() {
                if h.stop.is_some_and(|s| now >= s) {
                    h.stop = None;
                    crate::link::rumble_side(i == 1, 0);
                }
                since[i] += 1;
                if let Some(r) = vr::REPORTS[i].try_take() {
                    h.last = Some(r);
                    h.reports += 1;
                    let _ = udp.send_to(&r, &view);
                    since[i] = 0;
                } else if since[i] >= 100 {
                    // Like the USB task: the last report again every 100 ms.
                    if let Some(r) = h.last {
                        let _ = udp.send_to(&r, &view);
                    }
                    since[i] = 0;
                }
                if w.now() / 1000 % 16 == 0 {
                    let q = w.devs[i].c.q;
                    let mut p = vec![0xF0, i as u8];
                    for v in [q[1], q[2], q[3], q[0]] {
                        p.extend_from_slice(&(v as f32).to_le_bytes());
                    }
                    p.push(w.devs[i].motor);
                    let _ = udp.send_to(&p, &view);
                }
            }
            while let Ok(ev) = radio::EVENTS.try_receive() {
                let mut out = [0u8; 300];
                let n = serial::encode_event(ev, &mut out);
                send_frame(&mut client, &out[..n]);
            }
        }
        if w.reset {
            save_flash(&flash_path);
            println!("dongle restarted (mode change, firmware update or forget): restarting the simulator");
            drop(client);
            drop(tcp);
            drop(udp);
            let exe = std::env::current_exe().expect("own path");
            let mut a: Vec<String> = vec!["live".into()];
            a.extend(args.iter().cloned());
            if !args.iter().any(|x| x == "--flash") {
                a.push("--flash".into());
                a.push(flash_path.display().to_string());
            }
            let status = std::process::Command::new(exe).args(a).status();
            return status.map(|s| s.code().unwrap_or(0)).unwrap_or(1);
        }
        if last_status.elapsed() >= Duration::from_secs(2) {
            let secs = last_status.elapsed().as_secs_f64();
            last_status = Instant::now();
            save_flash(&flash_path);
            let line: Vec<String> = hands
                .iter_mut()
                .enumerate()
                .map(|(i, h)| {
                    let hz = h.reports as f64 / secs;
                    h.reports = 0;
                    let d = &w.devs[i];
                    let est = h.last.and_then(|r| Report::parse(&r)).map_or(math::IDENTITY, |r| r.q);
                    let [y, p, r] = math::ypr(est);
                    format!(
                        "{}: {:>3.0} Hz yaw {:6.1} pitch {:5.1} roll {:6.1} err {:4.1}° {:?}",
                        if i == 0 { "L" } else { "R" },
                        hz,
                        y,
                        p,
                        r,
                        math::angle_deg(est, d.c.q),
                        d.state
                    )
                })
                .collect();
            println!("{}{}", line.join("  |  "), if client.is_some() { "  | serial client" } else { "" });
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn save_flash(path: &std::path::Path) {
    let img = hw::with(|c| c.flash.clone());
    let _ = std::fs::write(path, img);
}

fn send_frame(client: &mut Option<TcpStream>, raw: &[u8]) {
    let Some(s) = client.as_mut() else { return };
    let mut enc = [0u8; 310];
    let n = cobs::encode(raw, &mut enc);
    enc[n] = 0;
    if s.write_all(&enc[..n + 1]).is_err() {
        *client = None;
    }
}

fn clamp(w: [f64; 3], max: f64) -> [f64; 3] {
    let n = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
    if n > max { w.map(|x| x * max / n) } else { w }
}

fn control(w: &mut World, hands: &mut [Hand], msg: &str) {
    let mut words = msg.split_whitespace();
    let i = match words.next() {
        Some("L") => 0,
        Some("R") => 1,
        _ => return,
    };
    let side = if i == 0 { Side::Left } else { Side::Right };
    let now = w.now();
    let h = &mut hands[i];
    let d = &mut w.devs[i];
    for word in words {
        let Some((k, v)) = word.split_once('=') else { continue };
        let num = |v: &str| v.parse::<i64>().ok();
        match k {
            "q" => {
                let f: Vec<f64> = v.split(',').filter_map(|x| x.parse().ok()).collect();
                if f.len() == 4 {
                    h.target = math::normalize([f[3], f[0], f[1], f[2]]);
                    h.steered = true;
                }
            }
            "b" => d.c.buttons = num(v).unwrap_or(0) as u8 & 0x0F,
            "t" => d.c.touch = num(v).unwrap_or(0) as u8,
            "sx" => d.c.stick.0 = num(v).unwrap_or(0).clamp(-32000, 32000) as i16,
            "sy" => d.c.stick.1 = num(v).unwrap_or(0).clamp(-32000, 32000) as i16,
            "tr" => d.c.trigger = num(v).unwrap_or(0).clamp(0, 4095) as u16,
            "gr" => d.c.grip = num(v).unwrap_or(0).clamp(0, 4095) as u16,
            "bat" => d.c.battery = num(v).unwrap_or(80).clamp(0, 100) as u8,
            "c" => match (v, d.state) {
                ("0", s) if s != crate::air::State::Off => d.power_off(),
                ("1", crate::air::State::Off) => d.wake(now),
                _ => {}
            },
            "cmd" => {
                let bytes: Vec<u8> =
                    (0..v.len() / 2).filter_map(|j| u8::from_str_radix(&v[2 * j..2 * j + 2], 16).ok()).collect();
                if let Some(stop) = vr::command(side, &bytes) {
                    h.stop = stop;
                }
            }
            _ => {}
        }
    }
}
