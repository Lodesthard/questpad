//! Simulator stand-in for cortex-m. A system reset unwinds with `SysReset`, which the simulator
//! catches: the scenario then inspects what the firmware left behind (flash, settings).

/// Panic payload of `SCB::sys_reset`.
#[derive(Debug)]
pub struct SysReset;

pub mod peripheral {
    pub struct SCB;

    impl SCB {
        pub fn sys_reset() -> ! {
            std::panic::panic_any(super::SysReset)
        }
    }
}

pub mod interrupt {
    pub fn disable() {}
    /// # Safety
    /// Nothing to uphold in the simulator.
    pub unsafe fn enable() {}
}
