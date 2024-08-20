#![no_main]
#![no_std]
#![feature(never_type)]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

use stm32g4xx_hal as _; // memory layout

pub mod can_periodic;
pub mod can_queue;
pub mod hardware;
pub mod car;
pub mod ieb;
pub mod igpm;
pub mod srscm;


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
