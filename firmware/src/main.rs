#![no_main]
#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _; // global logger

use panic_probe as _;

use stm32g4xx_hal as _; // memory layout

pub mod airbag_control;
pub mod can_queue;
pub mod can_utils;
pub mod car;
pub mod dbc;
pub mod fresh;
pub mod hardware;
pub mod ieb;
pub mod igpm;
pub mod repeater;

// Make some common type aliases for fugit Duration, Instance and Rate
// based on our firmware's 1ms tick period
type Duration = fugit::Duration<u32, 1, 1000>;
type Instant = fugit::Instant<u32, 1, 1000>;
type Rate = fugit::Rate<u32, 1, 1000>;

#[rtic::app(
    device = stm32g4xx_hal::stm32,
    dispatchers = [USBWAKEUP, COMP1_2_3, COMP4_5_6, COMP7, SAI, I2C4_EV, I2C4_ER]
)]
mod app {
    use crate::can_queue;
    use crate::car;
    use crate::car::Ignition;
    use crate::dbc::pcan;
    use crate::hardware;
    use crate::hardware::Mono;
    use debouncr::debounce_stateful_3;
    use debouncr::debounce_stateful_5;
    use debouncr::Edge::Falling;
    use debouncr::Edge::Rising;
    use defmt::Debug2Format;
    use embedded_can::Frame;
    use embedded_can::Id;
    use fugit::ExtU32;
    use rtic_monotonics::Monotonic;
    use stm32g4xx_hal::hal::digital::v2::OutputPin;
    use stm32g4xx_hal::prelude::InputPin;

    // Task functions
    use crate::airbag_control::task_airbag_control;
    use crate::ieb::task_ieb;
    use crate::igpm::task_igpm;

    #[shared]
    struct Shared {
        pcan_tx: can_queue::Tx<hardware::PCAN>,
        car: car::CarState,
    }

    #[local]
    struct Local {
        pcan_control: can_queue::Control<hardware::PCAN>,
        pcan_rx: can_queue::Rx,
        brake_input: hardware::BrakeInput,
        ig1_on_input: hardware::IG1OnInput,
        relay_ig3: hardware::RelayIG3Output,
        led_ignition: hardware::LEDIgnitionOutput,
        srs_crash_out: hardware::AcuCrashOutput,
        ev_ready: hardware::EVReadyInput,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let hardware::Board {
            pcan_config,
            srs_crash_out,
            can_timing_500kbps,
            brake_input,
            ig1_on_input,
            led_ignition,
            relay_ig3,
            ev_ready,
        } = hardware::init(cx.core, cx.device);

        let (pcan_control, pcan_rx, pcan_tx) =
            can_queue::Control::init(pcan_config, &can_timing_500kbps);

        let car = car::CarState::new();

        pcan_rx::spawn().unwrap();
        poll_slow_inputs::spawn().unwrap();
        task_airbag_control::spawn().unwrap();
        task_ieb::spawn().unwrap();
        task_igpm::spawn().unwrap();
        log_info::spawn().unwrap();

        (
            Shared { pcan_tx, car },
            Local {
                pcan_control,
                pcan_rx,
                brake_input,
                srs_crash_out,
                ig1_on_input,
                led_ignition,
                relay_ig3,
                ev_ready,
            },
        )
    }

    fn is_diagnostic(id: Id) -> bool {
        match id {
            Id::Standard(std) => std.as_raw() >= 0x700,
            Id::Extended(_) => false,
        }
    }

    // TODO: consider bumping this to priority 4, above the sending of messages,
    // so that the vehicle state is always up to date
    #[task(local = [pcan_rx], shared = [car], priority = 2)]
    async fn pcan_rx(cx: pcan_rx::Context) {
        let pcan_rx = cx.local.pcan_rx;
        let mut car = cx.shared.car;

        loop {
            let frame = pcan_rx.recv().await.unwrap();
            if is_diagnostic(frame.id()) {
                // Skip diagnostic IDs for now (can change to be a hw filter, or
                // possibly will add diagnostic handlers elsewhere
                continue;
            }
            let msg = pcan::Messages::from_can_message(frame.id(), frame.data());
            match msg {
                Err(_) => {
                    defmt::error!(
                        "Failed to parse CAN message ID {:?} data {:?}",
                        Debug2Format(&frame.id()),
                        frame.data(),
                    );
                }
                Ok(msg) => {
                    defmt::debug!("PCAN RX {:?}", msg); // +25KB of code(!)
                    defmt::trace!("RAW RX {:?}", frame);

                    car.lock(|car| car.update_state(&msg));
                }
            }
        }
    }

    // Task declarations all extern-ed to split the firmware up into modules
    extern "Rust" {
        #[task(shared = [pcan_tx, car], local = [srs_crash_out], priority = 3)]
        async fn task_airbag_control(cx: task_airbag_control::Context);

        #[task(shared = [pcan_tx, car], priority = 3)]
        async fn task_ieb(cx: task_ieb::Context);

        #[task(shared = [pcan_tx, car], priority = 3)]
        async fn task_igpm(cx: task_igpm::Context);
    }

    // FDCAN_INTR0_IT and FDCAN_INTR1_IT are swapped, until stm32g4 crate
    // updates to include https://github.com/stm32-rs/stm32-rs/pull/996
    #[task(binds = FDCAN1_INTR1_IT, shared = [pcan_tx], local=[pcan_control], priority = 6)]
    fn pcan_irq(cx: pcan_irq::Context) {
        cx.local.pcan_control.on_irq(cx.shared.pcan_tx);
    }

    // Power state changes are slow, so poll them in a timed loop with some debounce logic
    #[task(shared = [car], local = [brake_input, ig1_on_input, led_ignition, relay_ig3, ev_ready], priority = 5)]
    async fn poll_slow_inputs(mut cx: poll_slow_inputs::Context) {
        // Time base for the debouncing delays
        let PERIOD = 10.millis();
        // Debouncers. Each debounce period is (_N * PERIOD)
        let mut ig1_on = debounce_stateful_5(false);
        let mut brakes_on = debounce_stateful_3(false);
        let mut ev_ready = debounce_stateful_5(false);

        let mut next = Mono::now() + PERIOD;
        loop {
            Mono::delay_until(next).await;
            next += PERIOD;

            let ig1_edge = ig1_on.update(cx.local.ig1_on_input.is_high().unwrap());
            let brakes_edge = brakes_on.update(cx.local.brake_input.is_high().unwrap());
            let ready_edge = ev_ready.update(cx.local.ev_ready.is_high().unwrap());

            if [ig1_edge, brakes_edge, ready_edge]
                .into_iter()
                .any(|o| o.is_some())
            {
                cx.shared.car.lock(|car| {
                    // TODO: also read external IG3 state and update accordingly

                    if let Some(edge) = ig1_edge {
                        car.set_ignition(match edge {
                            Rising => Ignition::On,
                            Falling => Ignition::Off,
                        });
                    };
                    if let Some(edge) = brakes_edge {
                        car.set_is_braking(edge == Rising);
                    }
                    if let Some(edge) = ready_edge {
                        car.set_ev_ready_input(edge == Rising);
                    }
                });
            }

            if let Some(edge) = ig1_edge {
                // FIXME: ignition sequence should be its own task
                match edge {
                    Rising => {
                        cx.local.relay_ig3.set_high().unwrap();
                        cx.local.led_ignition.set_high().unwrap();
                    }
                    Falling => {
                        cx.local.relay_ig3.set_low().unwrap();
                        cx.local.led_ignition.set_low().unwrap();
                    }
                }
            }
        }
    }

    #[task(shared = [car], priority = 0)]
    async fn log_info(mut cx: log_info::Context) {
        loop {
            Mono::delay(2.secs()).await;

            cx.shared.car.lock(|car| {
                defmt::info!(
                    "Ign: {:?} Con: {:?} Batt: {:05}% Inv: {:?}V RPM: {:?}",
                    car.ignition(),
                    car.contactor(),
                    car.soc_batt(),
                    car.v_inverter(),
                    car.motor_rpm(),
                );
            });
        }
    }
}

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