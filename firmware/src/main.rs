#![no_main]
#![no_std]

use defmt_brtt as _; // global logger

use hardware::Mono;
use panic_probe as _;

use stm32g4xx_hal as _; // memory layout

use rtic_monotonics::Monotonic;

mod airbag_control;
mod can_queue;
mod car;
mod dbc;
mod fresh;
mod hardware;
mod ieb;
mod igpm;
mod repeater;
mod shift_control;

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
    use crate::shift_control;
    use car::ChargeLock;
    use debouncr::debounce_stateful_12;
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
    use crate::igpm::{self, task_igpm, task_lock_charge_port};
    use crate::shift_control::task_scu_can_tx;
    use crate::shift_control::task_scu_pwm_rx;
    use crate::shift_control::task_scu_pwm_tx;

    #[shared]
    struct Shared {
        pcan_tx: can_queue::Tx<hardware::PCAN>,
        car: car::CarState,
        park_actuator: shift_control::ActuatorState,
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
        scu_park_tx: hardware::ScuParkTx,
        scu_park_rx: hardware::ScuParkRx,
        charge_lock_drive: hardware::ChargeLockDriveOutput,
        charge_lock_dir: hardware::ChargeLockDirOutput,
        charge_lock_sensor: hardware::ChargeLockSensorInput,
        standby: hardware::Standby,
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
            scu_park_tx,
            scu_park_rx,
            charge_lock_drive,
            charge_lock_dir,
            charge_lock_sensor,
            standby,
        } = hardware::init(cx.core, cx.device);

        let (pcan_control, pcan_rx, pcan_tx) =
            can_queue::Control::init(pcan_config, &can_timing_500kbps);

        let car = car::CarState::new();

        let park_actuator = shift_control::ActuatorState::default();

        pcan_rx::spawn().unwrap();
        poll_slow_inputs::spawn().unwrap();
        task_airbag_control::spawn().unwrap();
        task_ieb::spawn().unwrap();
        task_igpm::spawn().unwrap();
        task_scu_can_tx::spawn().unwrap();
        task_scu_pwm_tx::spawn().unwrap();
        log_info::spawn().unwrap();
        ignition_sequence::spawn().unwrap();

        (
            Shared {
                pcan_tx,
                car,
                park_actuator,
            },
            Local {
                pcan_control,
                pcan_rx,
                brake_input,
                srs_crash_out,
                ig1_on_input,
                led_ignition,
                relay_ig3,
                ev_ready,
                scu_park_tx,
                scu_park_rx,
                charge_lock_drive,
                charge_lock_dir,
                charge_lock_sensor,
                standby,
            },
        )
    }

    fn is_diagnostic(id: Id) -> bool {
        match id {
            Id::Standard(std) => std.as_raw() >= 0x700,
            Id::Extended(_) => false,
        }
    }

    #[task(local = [pcan_rx], shared = [car, park_actuator], priority = 4)]
    async fn pcan_rx(cx: pcan_rx::Context) {
        let pcan_rx = cx.local.pcan_rx;
        let mut car = cx.shared.car;
        let mut park_actuator = cx.shared.park_actuator;

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
                    // msg implements Format but reporting it here results in RX overruns
                    defmt::trace!("PCAN RX {:?}", frame);

                    car.lock(|car| car.update_state(&msg));

                    shift_control::on_can_rx(&msg, &mut park_actuator);

                    igpm::on_can_rx(&msg);
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

        #[task(shared = [car], local=[charge_lock_drive, charge_lock_dir], priority = 2)]
        async fn task_lock_charge_port(cx: task_lock_charge_port::Context, direction: ChargeLock);

        #[task(shared = [pcan_tx, car, park_actuator], priority = 3)]
        async fn task_scu_can_tx(cx: task_scu_can_tx::Context);

        #[task(shared = [car, park_actuator], local = [scu_park_tx], priority = 6)]
        async fn task_scu_pwm_tx(cx: task_scu_pwm_tx::Context);

        #[task(binds = EXTI2, shared = [car, park_actuator], local = [scu_park_rx], priority = 6)]
        fn task_scu_pwm_rx(cx: task_scu_pwm_rx::Context);
    }

    // FDCAN_INTR0_IT and FDCAN_INTR1_IT are swapped, until stm32g4 crate
    // updates to include https://github.com/stm32-rs/stm32-rs/pull/996
    #[task(binds = FDCAN1_INTR1_IT, shared = [car, pcan_tx], local=[pcan_control], priority = 6)]
    fn pcan_irq(mut cx: pcan_irq::Context) {
        let set_bus_off = || { cx.shared.car.lock(|car| car.set_pcan_bus_off()) };
        cx.local.pcan_control.on_irq(cx.shared.pcan_tx, set_bus_off);
    }

    // Power state changes are slow, so poll them in a timed loop with some debounce logic
    #[task(shared = [car], local = [brake_input, ev_ready, charge_lock_sensor], priority = 5)]
    async fn poll_slow_inputs(mut cx: poll_slow_inputs::Context) {
        // Time base for the debouncing delays
        let PERIOD = 10.millis();
        // Debouncers. Each debounce period is (_N * PERIOD)
        let mut brakes_on = debounce_stateful_3(false);
        let mut ev_ready = debounce_stateful_5(false);
        // Note: we track the charge port lock state here not from task_lock_charge_port as it
        // can also be manually unlocked at any time
        let mut charge_lock = debounce_stateful_12(false);

        let mut next = Mono::now() + PERIOD;
        loop {
            Mono::delay_until(next).await;
            next += PERIOD;

            let brakes_edge = brakes_on.update(cx.local.brake_input.is_high().unwrap());
            let ready_edge = ev_ready.update(cx.local.ev_ready.is_high().unwrap());
            let charge_lock_edge = charge_lock.update(cx.local.charge_lock_sensor.is_high().unwrap());

            if [brakes_edge, ready_edge, charge_lock_edge]
                .into_iter()
                .any(|o| o.is_some())
            {
                cx.shared.car.lock(|car| {
                    if let Some(edge) = brakes_edge {
                        car.set_is_braking(edge == Rising);
                    }
                    if let Some(edge) = ready_edge {
                        car.set_ev_ready_input(edge == Rising);
                    }
                    if let Some(edge) = charge_lock_edge {
                        car.set_charge_port(match edge {
                            Rising => ChargeLock::Unlocked,
                            Falling => ChargeLock::Locked,
                        });
                    }
                });
            }
        }
    }

    /// Tracks the ignition state of the vehicle by monitoring:
    /// - IG1 power on input
    /// - CAN messages sent when IG3 powered on (may change to hard-wired in future)
    /// - Any CAN messages received at all
    ///
    /// Will go to Standby (pending a reset) if rest of vehicle is asleep.
    #[task(shared = [car], local = [ig1_on_input, led_ignition, relay_ig3, standby], priority = 4)]
    async fn ignition_sequence(cx: ignition_sequence::Context) {
        let mut car = cx.shared.car;
        let mut ig1_on = debounce_stateful_5(false);
        let mut idle_count = 0;

        loop {
            let ignition = car.lock(|car| car.ignition());

            let ig1_edge = ig1_on.update(cx.local.ig1_on_input.is_high().unwrap());

            let ig3_alive = car.lock(|car| car.ig3_appears_powered());

            let pcan_bus_off = car.lock(|car| car.pcan_bus_off());

            // PCAN Bus should be functioning *unless* ignition is off
            // and we're about to go to standby anyway
            //
            // TODO: handle this in a better way than panicking
            assert!(!pcan_bus_off || ignition == Ignition::Off);

            let next_ignition = match ignition {
                Ignition::Off => {
                    if ig1_edge == Some(Rising) {
                        Some(Ignition::On)
                    }
                    else if ig3_alive {
                        Some(Ignition::IG3)
                    }
                    else if pcan_bus_off || !car.lock(|car| car.pcan_receiving()) {
                        // FIXME: Hacky check while we don't have wakeup sources
                        // to automatically get back out of standby mode: wait
                        // at least 10 more seconds before going into standby to
                        // give the rest of the vehicle a chance to start up if
                        // we just powered on
                        idle_count += 1;
                        if idle_count > 500 {
                            cx.local.standby.enter_standby_mode().await
                        } else {
                            None
                        }
                    }
                    else {
                        idle_count = 0;
                        None // No change
                    }
                },
                Ignition::IG3 => {
                    if ig1_edge == Some(Rising) {
                        Some(Ignition::On)
                    }
                    else if !ig3_alive {
                        Some(Ignition::Off)
                    }
                    else {
                        None // No change
                    }

                },
                Ignition::On => {
                    if ig1_edge == Some(Falling) {
                        if ig3_alive {
                            Some(Ignition::IG3)
                        } else {
                            // This starts a shutdown sequence that last
                            // slightly over 3 minutes before the VCU and other
                            // modules stop sending messages (if ignition stays off).
                            //
                            // Can possibly make this time out faster by telling it the
                            // doors are locked, or stopping some of our own IGPM messages.
                            Some(Ignition::Off)
                        }
                    }
                    else {
                        None // No change
                    }
                },
            };

            if let Some(next) = next_ignition {
                car.lock(|car| car.set_ignition(next));

                // Update the IG3 external power on relay, if needed
                if next == Ignition::On {
                    cx.local.relay_ig3.set_high().unwrap();
                    cx.local.led_ignition.set_high().unwrap();
                } else {
                    cx.local.relay_ig3.set_low().unwrap();
                    cx.local.led_ignition.set_low().unwrap();
                }
            }

            Mono::delay(20.millis()).await;
        }
    }

    #[task(shared = [car], priority = 0)]
    async fn log_info(mut cx: log_info::Context) {
        loop {
            Mono::delay(2.secs()).await;

            cx.shared.car.lock(|car| {
                defmt::info!(
                    "Ign: {:?} Gear: {:?} Con: {:?} Batt: {:05}% Inv: {:?}V RPM: {:?}",
                    car.ignition(),
                    car.gear(),
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

defmt::timestamp!("{=u32}", { Mono::now().ticks() });

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
