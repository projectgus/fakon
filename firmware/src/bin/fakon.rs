#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use fakon as _;

#[rtic::app(
    device = stm32g4xx_hal::stm32,
    dispatchers = [USBWAKEUP, COMP1_2_3, COMP4_5_6, COMP7, SAI, I2C4_EV, I2C4_ER]
)]
mod app {
    use defmt::Debug2Format;
    use embedded_can::Frame;
    use embedded_can::Id;
    use fakon;
    use fakon::can_queue;
    use fakon::car;
    use fakon::car::Ignition;
    use fakon::dbc::pcan;
    use fakon::hardware;
    use fakon::hardware::Mono;
    use fugit::ExtU32;
    use rtic_monotonics::Monotonic;
    use stm32g4xx_hal::hal::digital::v2::OutputPin;
    use stm32g4xx_hal::prelude::InputPin;

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
        } = hardware::init(cx.core, cx.device);

        let (pcan_control, pcan_rx, pcan_tx) =
            can_queue::Control::init(pcan_config, &can_timing_500kbps);

        let car = car::CarState::new();

        pcan_rx::spawn().unwrap();
        poll_inputs::spawn().unwrap();
        airbag_control::spawn().unwrap();
        ieb::spawn().unwrap();
        igpm::spawn().unwrap();

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
            },
        )
    }

    fn is_diagnostic(id: Id) -> bool {
        match id {
            Id::Standard(std) => std.as_raw() >= 0x700,
            Id::Extended(_) => false,
        }
    }

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
                    defmt::warn!("Failed to parse CAN message ID {:?} data {:?}",
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

    #[task(shared = [pcan_tx, car], local = [srs_crash_out], priority = 3)]
    async fn airbag_control(cx: airbag_control::Context) {
        fakon::airbag_control::task(cx.shared.pcan_tx, cx.shared.car, cx.local.srs_crash_out).await;
    }

    #[task(shared = [pcan_tx, car], priority = 3)]
    async fn ieb(cx: ieb::Context) {
        fakon::ieb::task_ieb(cx.shared.pcan_tx, cx.shared.car).await;
    }

    #[task(shared = [pcan_tx, car], priority = 3)]
    async fn igpm(cx: igpm::Context) {
        fakon::igpm::task_igpm(cx.shared.pcan_tx, cx.shared.car).await;
    }

    // FDCAN_INTR0_IT and FDCAN_INTR1_IT are swapped, until stm32g4 crate
    // updates to include https://github.com/stm32-rs/stm32-rs/pull/996
    #[task(binds = FDCAN1_INTR1_IT, shared = [pcan_tx], local=[pcan_control], priority = 5)]
    fn pcan_irq(cx: pcan_irq::Context) {
        cx.local.pcan_control.on_irq(cx.shared.pcan_tx);
    }

    // Why debounce interrupts when you can poll GPIOs in a loop?!?
    #[task(shared = [car], local = [brake_input, ig1_on_input, led_ignition, relay_ig3], priority = 6)]
    async fn poll_inputs(mut cx: poll_inputs::Context) {
        loop {
            Mono::delay(10.millis()).await;

            let ig1_on = cx.local.ig1_on_input.is_high().unwrap();
            let brakes_on = cx.local.brake_input.is_high().unwrap();

            cx.shared.car.lock(|car| {
                // TODO: also read external IG3 state and update accordingly
                car.set_ignition(if ig1_on { Ignition::On } else { Ignition::Off });
                car.set_is_braking(brakes_on);
            });

            // FIXME: ignition sequence should be its own task
            match ig1_on {
                true => {
                    cx.local.relay_ig3.set_high().unwrap();
                    cx.local.led_ignition.set_high().unwrap();
                }
                false => {
                    cx.local.relay_ig3.set_low().unwrap();
                    cx.local.led_ignition.set_low().unwrap();
                }
            }
        }
    }

    #[task(shared = [car], priority = 2)]
    async fn log_info(mut cx: log_info::Context) {
        loop {
            Mono::delay(2.secs()).await;

            cx.shared.car.lock(|car| {
                defmt::info!("Ign: {:?} Con: {:?} Batt: {:05}% Inv: {:?}V RPM: {:?}",
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
