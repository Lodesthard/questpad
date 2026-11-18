#![no_std]
#![no_main]

mod board;
mod cobs;
mod ds4;
mod gamepad;
mod hw;
mod link;
mod pairing;
mod radio;
#[cfg(not(feature = "nicenano"))]
mod selfupdate;
mod serial;
mod settings;
mod ui;
mod vr;
mod wdt;
mod xinput;

use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::gpio::{Input, Level, Pull};
use embassy_nrf::pwm::{Prescaler, SimpleConfig, SimplePwm};
use embassy_nrf::{bind_interrupts, peripherals, usb};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::class::hid::{self, HidReader, HidReaderWriter, HidWriter};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, UsbDevice};
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

type UsbDriver = Driver<'static, HardwareVbusDetect>;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

#[embassy_executor::task]
async fn hid_task(mut writer: HidWriter<'static, UsbDriver, { gamepad::REPORT_LEN }>) -> ! {
    writer.ready().await;
    let _ = writer.write(&gamepad::NEUTRAL).await;
    loop {
        let report = match select(gamepad::PAD.wait(), gamepad::OVERRIDE.wait()).await {
            Either::First(pad) => gamepad::hid_report(&pad),
            Either::Second(raw) => raw,
        };
        let _ = writer.write(&report).await;
    }
}

/// USB bus events: when the PC suspends (sleeps), power the controllers off.
struct UsbEvents;

static SUSPENDED: embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ()> =
    embassy_sync::signal::Signal::new();

impl embassy_usb::Handler for UsbEvents {
    fn suspended(&mut self, suspended: bool) {
        if suspended {
            SUSPENDED.signal(());
        }
    }
}

#[embassy_executor::task]
async fn suspend_task() -> ! {
    loop {
        SUSPENDED.wait().await;
        if settings::get().off_on_suspend {
            link::power_off_all();
        }
    }
}

/// DS4 mode: stream input reports (at least every 8 ms, like a real pad; usually 2 ms from the IMU).
#[embassy_executor::task]
async fn ds4_task(mut writer: HidWriter<'static, UsbDriver, { ds4::IN_LEN }>) -> ! {
    writer.ready().await;
    let mut pad = gamepad::Pad::default();
    loop {
        if let Ok(p) = embassy_time::with_timeout(Duration::from_millis(8), gamepad::PAD.wait()).await {
            pad = p;
        }
        let _ = writer.write(&ds4::report(&pad)).await;
    }
}

#[embassy_executor::task]
async fn ds4_out_task(reader: HidReader<'static, UsbDriver, { ds4::OUT_LEN }>) -> ! {
    reader.run(true, &mut ds4::Handler).await
}

#[embassy_executor::task]
async fn xinput_in_task(ep: usb::Endpoint<'static, usb::In>) -> ! {
    xinput::run_in(ep).await
}

#[embassy_executor::task]
async fn xinput_out_task(ep: usb::Endpoint<'static, usb::Out>) -> ! {
    xinput::run_out(ep).await
}

/// VR mode: one state stream and one command endpoint per controller.
#[embassy_executor::task(pool_size = 2)]
async fn vr_in_task(side: gamepad::Side, ep: usb::Endpoint<'static, usb::In>) -> ! {
    vr::run_in(side, ep).await
}

#[embassy_executor::task(pool_size = 2)]
async fn vr_out_task(side: gamepad::Side, ep: usb::Endpoint<'static, usb::Out>) -> ! {
    vr::run_out(side, ep).await
}

#[embassy_executor::task]
async fn rumble_task(reader: HidReader<'static, UsbDriver, { gamepad::OUT_REPORT_LEN }>) -> ! {
    reader.run(false, &mut gamepad::Rumble).await
}

#[embassy_executor::task]
async fn usb_task(mut dev: UsbDevice<'static, UsbDriver>) -> ! {
    dev.run().await
}

async fn send_frame(tx: &mut Sender<'static, UsbDriver>, raw: &[u8]) -> Result<(), EndpointError> {
    let mut enc = [0u8; 300];
    let n = cobs::encode(raw, &mut enc);
    enc[n] = 0;
    let total = n + 1;
    for chunk in enc[..total].chunks(64) {
        tx.write_packet(chunk).await?;
    }
    if total % 64 == 0 {
        tx.write_packet(&[]).await?;
    }
    Ok(())
}

async fn session(tx: &mut Sender<'static, UsbDriver>, rx: &mut Receiver<'static, UsbDriver>) -> Result<(), EndpointError> {
    let mut pkt = [0u8; 64];
    let mut acc = [0u8; 300];
    let mut acc_len = 0usize;
    let mut out = [0u8; 300];
    loop {
        match select(rx.read_packet(&mut pkt), radio::EVENTS.receive()).await {
            Either::First(n) => {
                for &b in &pkt[..n?] {
                    if b != 0 {
                        if acc_len < acc.len() {
                            acc[acc_len] = b;
                            acc_len += 1;
                        }
                        continue;
                    }
                    let mut dec = [0u8; 300];
                    let len = cobs::decode(&acc[..acc_len], &mut dec);
                    acc_len = 0;
                    if let Some(len) = len {
                        let n = serial::handle(&dec[..len], &mut out);
                        if n > 0 {
                            send_frame(tx, &out[..n]).await?;
                        }
                    }
                }
            }
            Either::Second(ev) => {
                let n = serial::encode_event(ev, &mut out);
                send_frame(tx, &out[..n]).await?;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.lfclk_source = embassy_nrf::config::LfclkSource::Synthesized;
    let p = embassy_nrf::init(config);

    wdt::start();
    let mode = settings::load().mode;
    vr::ENABLED.store(mode == settings::Mode::Vr, core::sync::atomic::Ordering::Relaxed);
    radio::init();
    serial::start_stored_link();

    // Front panel: the status LED on PWM0 (idle = off), and the button.
    let mut pwm_cfg = SimpleConfig::default();
    pwm_cfg.max_duty = 1000;
    pwm_cfg.prescaler = Prescaler::Div16;
    let off = if board::LED_ACTIVE_LOW { Level::High } else { Level::Low };
    pwm_cfg.ch0_idle_level = off;
    pwm_cfg.ch1_idle_level = off;
    pwm_cfg.ch2_idle_level = off;
    // Dongle LED2 as channels R, G, B. Green / blue pins verified on hardware: P0.12 lights green,
    // P1.09 blue.
    #[cfg(not(feature = "nicenano"))]
    let pwm = SimplePwm::new_3ch(p.PWM0, p.P0_08, p.P0_12, p.P1_09, &pwm_cfg);
    #[cfg(feature = "nicenano")]
    let pwm = SimplePwm::new_1ch(p.PWM0, p.P0_15, &pwm_cfg);
    spawner.spawn(ui::led_task(pwm).unwrap());
    // Dongle SW1; on the nice!nano a header pin, for an optional button to GND.
    spawner.spawn(ui::button_task(Input::new(p.P1_06, Pull::Up)).unwrap());
    spawner.spawn(ui::pair_task().unwrap());
    spawner.spawn(ui::mode_task().unwrap());
    spawner.spawn(wdt::task().unwrap());

    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    // DS4 mode copies a real DS4's strings: some games (e.g. InControl's Unity profiles) match on the name.
    let (vid, pid, manufacturer, product) = match mode {
        settings::Mode::Ds4 => (ds4::VID, ds4::PID, "Sony Interactive Entertainment", "Wireless Controller"),
        settings::Mode::XInput => (xinput::VID, xinput::PID, "QuestPad", "QuestPad (XInput mode)"),
        settings::Mode::Hid => (0x1209, 0x5151, "QuestPad", "QuestPad"),
        // Same ids as HID mode (bcdDevice 3.00 and the interface classes tell them apart).
        settings::Mode::Vr => (0x1209, 0x5151, "QuestPad", "QuestPad VR"),
    };
    let mut ucfg = embassy_usb::Config::new(vid, pid);
    if mode == settings::Mode::XInput {
        xinput::configure(&mut ucfg);
    }
    if mode == settings::Mode::Vr {
        vr::configure(&mut ucfg);
    }
    ucfg.manufacturer = Some(manufacturer);
    ucfg.product = Some(product);
    ucfg.serial_number = Some("QCD-0001");
    ucfg.max_power = 100;
    ucfg.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 512]> = StaticCell::new(); // VR mode: WinUSB descriptors for 2 functions
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();
    static HID_STATE: StaticCell<hid::State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        ucfg,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 512]),
        CONTROL_BUF.init([0; 128]),
    );
    static USB_EVENTS: StaticCell<UsbEvents> = StaticCell::new();
    builder.handler(USB_EVENTS.init(UsbEvents));
    let class = match mode {
        settings::Mode::XInput => {
            let eps = xinput::build(&mut builder);
            spawner.spawn(xinput_in_task(eps.ep_in).unwrap());
            spawner.spawn(xinput_out_task(eps.ep_out).unwrap());
            None
        }
        settings::Mode::Ds4 => {
            // HID first (interface 0, like a real pad), then the serial port for the GUI / tools.
            static DS4_HANDLER: StaticCell<ds4::Handler> = StaticCell::new();
            let hid_cfg = hid::Config {
                report_descriptor: ds4::REPORT_DESCRIPTOR,
                request_handler: Some(DS4_HANDLER.init(ds4::Handler)),
                poll_ms: 1,
                max_packet_size: 64,
                hid_subclass: hid::HidSubclass::No,
                hid_boot_protocol: hid::HidBootProtocol::None,
            };
            let pad = HidReaderWriter::<_, { ds4::OUT_LEN }, { ds4::IN_LEN }>::new(
                &mut builder,
                HID_STATE.init(hid::State::new()),
                hid_cfg,
            );
            let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
            let (pad_rx, pad_tx) = pad.split();
            spawner.spawn(ds4_task(pad_tx).unwrap());
            spawner.spawn(ds4_out_task(pad_rx).unwrap());
            Some(class)
        }
        settings::Mode::Vr => {
            // The two controller interfaces first (0 left, 1 right), then the serial port.
            let eps = vr::build(&mut builder);
            spawner.spawn(vr_in_task(gamepad::Side::Left, eps.left_in).unwrap());
            spawner.spawn(vr_out_task(gamepad::Side::Left, eps.left_out).unwrap());
            spawner.spawn(vr_in_task(gamepad::Side::Right, eps.right_in).unwrap());
            spawner.spawn(vr_out_task(gamepad::Side::Right, eps.right_out).unwrap());
            Some(CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64))
        }
        _ => {
            let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), 64);
            static RUMBLE: StaticCell<gamepad::Rumble> = StaticCell::new();
            let hid_cfg = hid::Config {
                report_descriptor: gamepad::REPORT_DESCRIPTOR,
                request_handler: Some(RUMBLE.init(gamepad::Rumble)),
                poll_ms: 1,
                max_packet_size: 64,
                hid_subclass: hid::HidSubclass::No,
                hid_boot_protocol: hid::HidBootProtocol::None,
            };
            let pad = HidReaderWriter::<_, { gamepad::OUT_REPORT_LEN }, { gamepad::REPORT_LEN }>::new(
                &mut builder,
                HID_STATE.init(hid::State::new()),
                hid_cfg,
            );
            let (pad_rx, pad_tx) = pad.split();
            spawner.spawn(hid_task(pad_tx).unwrap());
            spawner.spawn(rumble_task(pad_rx).unwrap());
            Some(class)
        }
    };
    let dev = builder.build();
    spawner.spawn(usb_task(dev).unwrap());
    spawner.spawn(suspend_task().unwrap());

    let Some(class) = class else {
        // XInput mode: no serial port; the other tasks do all the work.
        core::future::pending::<()>().await;
        unreachable!()
    };
    let (mut tx, mut rx) = class.split();
    loop {
        rx.wait_connection().await;
        let _ = session(&mut tx, &mut rx).await;
        // A host tool exited: drop any bridge-driven mode, but keep (or resume) the standalone link
        // (and leave dongle-driven pairing alone).
        let pairing = matches!(ui::pair_state(), ui::PairState::Waiting | ui::PairState::Writing);
        if !link::running() && !pairing {
            radio::stop();
            serial::start_stored_link();
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
