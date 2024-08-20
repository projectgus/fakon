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
    use fakon::car;
    use fakon::can_queue;
    use fakon::hardware;

    // Shared resources go here
    #[shared]
    struct Shared {
        pcan_tx: can_queue::TxQueue<hardware::PCAN>,
        car: fakon::car::CarState,
    }

    // Local resources go here
    #[local]
    struct Local {
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

        let (_pcan_rx, _pcan_receive, pcan_tx) = can_queue::init(
            pcan_config,
            &can_timing_500kbps);

        let car = car::CarState {
            charge_port_locked: false,
            ignition_on: false,
        };

        srscm::spawn().ok();
        ieb_100_hz::spawn().ok();
        ieb_50_hz::spawn().ok();
        ieb_10_hz::spawn().ok();
        igpm_10_hz::spawn().ok();
        igpm_5_hz::spawn().ok();
        igpm_1_hz::spawn().ok();

        (
            Shared { pcan_tx, car },
            Local {
                brake_input,
                srs_crash_out
            },
        )
    }

    #[task(shared = [pcan_tx], local = [srs_crash_out], priority = 3)]
    async fn srscm(cx: srscm::Context) {
        fakon::srscm::task(cx.shared.pcan_tx, cx.local.srs_crash_out).await;
    }

    #[task(shared = [pcan_tx], local = [brake_input], priority = 3)]
    async fn ieb_100_hz(cx: ieb_100_hz::Context) {
        fakon::ieb::task_ieb_100_hz(cx.shared.pcan_tx, cx.local.brake_input).await;
    }

    #[task(shared = [pcan_tx], priority = 3)]
    async fn ieb_50_hz(cx: ieb_50_hz::Context) {
        fakon::ieb::task_ieb_50_hz(cx.shared.pcan_tx).await;
    }

    #[task(shared = [pcan_tx], priority = 3)]
    async fn ieb_10_hz(cx: ieb_10_hz::Context) {
        fakon::ieb::task_ieb_10_hz(cx.shared.pcan_tx).await;
    }

    #[task(shared = [pcan_tx, car], priority = 3)]
    async fn igpm_10_hz(cx: igpm_10_hz::Context) {
        fakon::igpm::task_igpm_10_hz(cx.shared.pcan_tx, cx.shared.car).await;
    }

    #[task(shared = [pcan_tx], priority = 3)]
    async fn igpm_5_hz(cx: igpm_5_hz::Context) {
        fakon::igpm::task_igpm_5_hz(cx.shared.pcan_tx).await;
    }

    #[task(shared = [pcan_tx], priority = 3)]
    async fn igpm_1_hz(cx: igpm_1_hz::Context) {
        fakon::igpm::task_igpm_1_hz(cx.shared.pcan_tx).await;
    }

    // FIXME: Enable and process FDCAN interrupts

    // #[task(binds = USB_HP_CAN_TX, shared = [can_tx], priority = 5)]
    // fn can1_tx(mut cx: can1_tx::Context) {
    //     cx.shared.can_tx.lock(|can_tx| can_tx.on_tx_irq());
    // }

    // #[task(binds = USB_LP_CAN_RX0, local = [can_rx], priority = 5)]
    // fn can1_rx(cx: can1_rx::Context) {
    //     cx.local.can_rx.on_rx_irq();
    // }

}
