//! The one place that touches the chip's memory-mapped registers and flash. Everything else goes
//! through `read` / `write`, so the PC simulator (sim/) can stand in for the peripherals and run
//! the rest of the firmware unchanged. On the dongle these are plain volatile accesses.

#[inline(always)]
pub fn read(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
pub fn write(addr: u32, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}
