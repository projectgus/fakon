#![no_main]
#![no_std]
#![feature(never_type)]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

use stm32g4xx_hal as _; // memory layout

pub mod can_queue;
pub mod can_utils;
pub mod hardware;
pub mod periodic;
pub mod car;
pub mod fresh;
pub mod ieb;
pub mod igpm;
pub mod airbag_control;
pub mod dbc;
pub mod unknown;

// Make some common type aliases for fugit Duration, Instance and Rate
// based on our firmware's 1ms tick period
type Duration = fugit::Duration<u32, 1, 1000>;
type Instant = fugit::Instant<u32, 1, 1000>;
type Rate = fugit::Rate<u32, 1, 1000>;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
