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
        pcan_control: fdcan::FdCanControl<hardware::PCAN, fdcan::NormalOperationMode>,
        pcan_rx: can_queue::Rx<hardware::PCAN>,
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

        let (pcan_control, pcan_rx, _pcan_receive, pcan_tx) = can_queue::init(
            pcan_config,
            &can_timing_500kbps);

        let car = car::CarState {
            charge_port_locked: false,
            ignition_on: false,
        };

        srscm::spawn().ok();
        ieb::spawn().ok();
        igpm::spawn().ok();

        (
            Shared { pcan_tx, car },
            Local {
                pcan_control,
                pcan_rx,
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
    async fn ieb(cx: ieb::Context) {
        fakon::ieb::task_ieb(cx.shared.pcan_tx, cx.local.brake_input).await;
    }

    #[task(shared = [pcan_tx, car], priority = 3)]
    async fn igpm(cx: igpm::Context) {
        fakon::igpm::task_igpm(cx.shared.pcan_tx, cx.shared.car).await;
    }

    // FIXME: Enable and process FDCAN interrupts

    #[task(binds = FDCAN1_INTR0_IT, shared = [pcan_tx], local=[pcan_control, pcan_rx], priority = 5)]
    fn pcan_int(mut cx: pcan_int::Context) {
        let control = &mut cx.local.pcan_control;
        if control.has_interrupt(fdcan::interrupt::Interrupt::TxComplete) {
            cx.shared.pcan_tx.lock(|pcan_tx| { pcan_tx.on_tx_irq(); });
        } else if control.has_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg) {
            cx.local.pcan_rx.on_rx_irq();
        } else {
            // TODO: handle error interrupts
        }
        control.clear_interrupts(fdcan::interrupt::Interrupts::all);
    }

    // #[task(binds = USB_LP_CAN_RX0, local = [can_rx], priority = 5)]
    // fn can1_rx(cx: can1_rx::Context) {
    //     cx.local.can_rx.on_rx_irq();
    // }

}
