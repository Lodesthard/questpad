//! Simulator stand-in for embassy-nrf: only what radio.rs, link.rs and ui.rs touch. Interrupts are
//! dispatched by sim/src/hw.rs, the button is a flag the scenarios set, the LED PWM duties are
//! recorded for the scenarios to read.

pub mod interrupt {
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub enum Priority {
        P0,
        P1,
        P2,
        P3,
        P4,
        P5,
        P6,
        P7,
    }

    #[derive(Clone, Copy)]
    pub struct Irq(pub &'static str);

    pub const RADIO: Irq = Irq("RADIO");
    pub const TIMER0: Irq = Irq("TIMER0");
    pub const AAR_CCM: Irq = Irq("AAR_CCM");

    pub trait InterruptExt {
        fn set_priority(self, prio: Priority);
        fn unpend(self);
        /// # Safety
        /// As in embassy-nrf (here: nothing to uphold).
        unsafe fn enable(self);
    }

    impl InterruptExt for Irq {
        fn set_priority(self, _prio: Priority) {}
        fn unpend(self) {}
        unsafe fn enable(self) {}
    }
}

pub mod gpio {
    use core::marker::PhantomData;
    use core::sync::atomic::{AtomicBool, Ordering};

    /// The dongle button (SW1, active low): true = pressed.
    pub static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);

    pub struct Input<'d> {
        _p: PhantomData<&'d ()>,
    }

    impl Input<'static> {
        /// The simulated button.
        pub fn sim_button() -> Self {
            Input { _p: PhantomData }
        }
    }

    impl<'d> Input<'d> {
        pub fn is_low(&self) -> bool {
            BUTTON_PRESSED.load(Ordering::Relaxed)
        }

        pub fn is_high(&self) -> bool {
            !self.is_low()
        }

        pub async fn wait_for_low(&mut self) {
            while !self.is_low() {
                embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
            }
        }

        pub async fn wait_for_high(&mut self) {
            while self.is_low() {
                embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
            }
        }
    }
}

pub mod pwm {
    use core::marker::PhantomData;
    use core::sync::atomic::{AtomicU16, Ordering};

    /// Last duty per channel, as set (0..=max), and whether it was `inverted`.
    pub static DUTY: [AtomicU16; 4] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];
    pub static INVERTED: [AtomicU16; 4] = [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)];

    #[derive(Clone, Copy)]
    pub struct DutyCycle {
        value: u16,
        inverted: bool,
    }

    impl DutyCycle {
        pub const fn normal(value: u16) -> Self {
            DutyCycle { value, inverted: false }
        }
        pub const fn inverted(value: u16) -> Self {
            DutyCycle { value, inverted: true }
        }
    }

    pub struct SimplePwm<'d> {
        _p: PhantomData<&'d ()>,
    }

    impl SimplePwm<'static> {
        pub fn sim() -> Self {
            SimplePwm { _p: PhantomData }
        }
    }

    impl<'d> SimplePwm<'d> {
        pub fn set_duty(&mut self, channel: usize, duty: DutyCycle) {
            DUTY[channel].store(duty.value, Ordering::Relaxed);
            INVERTED[channel].store(duty.inverted as u16, Ordering::Relaxed);
        }
    }
}
