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
    use fakon::hardware;

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
        srs_crash_out: hardware::PwmSrsCrashOutput,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let hardware::Board {
            pcan_config,
            srs_crash_out,
            can_timing_500kbps,
            brake_input,
        } = hardware::init(cx.core, cx.device);

        let (pcan_control, pcan_rx, pcan_tx) =
            can_queue::Control::init(pcan_config, &can_timing_500kbps);

        let car = car::CarState {
            charge_port_locked: false,
            ignition_on: false,
        };

        pcan_rx::spawn().unwrap();
        srscm::spawn().unwrap();
        ieb::spawn().unwrap();
        igpm::spawn().unwrap();

        (
            Shared { pcan_tx, car },
            Local {
                pcan_control,
                pcan_rx,
                brake_input,
                srs_crash_out,
            },
        )
    }

    #[task(local = [pcan_rx], priority = 2)]
    async fn pcan_rx(cx: pcan_rx::Context) {
        let pcan_rx = cx.local.pcan_rx;
        loop {
            let frame = pcan_rx.recv().await.unwrap();
            // TODO: actually handle received CAN messages!
            defmt::debug!("PCAN RX {:?}", frame);
        }
    }

    #[task(shared = [pcan_tx], local = [srs_crash_out], priority = 3)]
    async fn srscm(cx: srscm::Context) {
        fakon::srscm::task(cx.shared.pcan_tx, cx.local.srs_crash_out).await;
    }

    #[task(shared = [pcan_tx], local = [brake_input], priority = 3)]
    async fn ieb(cx: ieb::Context) {
        fakon::ieb::task_ieb(cx.shared.pcan_tx, cx.local.brake_input).await;
    }

    #[task(shared = [pcan_tx, car], priority = 3)]
    async fn igpm(cx: igpm::Context) {
        fakon::igpm::task_igpm(cx.shared.pcan_tx, cx.shared.car).await;
    }

    #[task(binds = FDCAN1_INTR0_IT, shared = [pcan_tx], local=[pcan_control], priority = 5)]
    fn pcan_irq(cx: pcan_irq::Context) {
        cx.local.pcan_control.on_irq(cx.shared.pcan_tx);
    }
}
