#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use fakon as _;

#[rtic::app(
    device = stm32g4xx_hal::stm32,
    dispatchers = [USBWAKEUP, COMP1_2_3, COMP4_5_6, COMP7, SAI, I2C4_EV, I2C4_ER]
)]
mod app {
    use fakon;
    use fakon::can_queue;
    use fakon::car;
    use fakon::dbc::pcan;
    use fakon::hardware;
    use fakon::hardware::Mono;
    use fugit::ExtU32;
    use rtic_monotonics::Monotonic;
    use stm32g4xx_hal::hal::digital::v2::OutputPin;
    use stm32g4xx_hal::prelude::InputPin;
    use embedded_can::Frame;

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

    #[task(local = [pcan_rx], priority = 2)]
    async fn pcan_rx(cx: pcan_rx::Context) {
        let pcan_rx = cx.local.pcan_rx;
        loop {
            let frame = pcan_rx.recv().await.unwrap();
            let msg = pcan::Messages::from_can_message(frame.id(), frame.data());
            // TODO: actually handle received CAN messages!

            defmt::debug!("PCAN RX {:?}", defmt::Debug2Format(&msg)); // +60KB of code(!)
            defmt::trace!("RAW RX {:?}", frame);
        }
    }

    #[task(shared = [pcan_tx], local = [srs_crash_out], priority = 3)]
    async fn airbag_control(cx: airbag_control::Context) {
        fakon::airbag_control::task(cx.shared.pcan_tx, cx.local.srs_crash_out).await;
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

            cx.shared.car.lock(|car| {
                car.set_ignition_on(ig1_on);
                car.set_is_braking(cx.local.brake_input.is_high().unwrap());
            });

            // FIXME: ignition sequence should be its own task
            match ig1_on {
                true => {
                    cx.local.relay_ig3.set_high().unwrap();
                    cx.local.led_ignition.set_high().unwrap();
                },
                false => {
                    cx.local.relay_ig3.set_low().unwrap();
                    cx.local.led_ignition.set_low().unwrap();
                }
            }

        }
    }
}
