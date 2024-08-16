#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use fakon as _;

#[rtic::app(
    device = stm32g4xx_hal::stm32,
    dispatchers = [USBWAKEUP, COMP1_2_3, COMP4_5_6, COMP7, SAI, I2C4_EV, I2C4_ER]
)]
mod app {
    use fakon::can_queue;
    use fakon::can_queue::Tx;
    use fakon::hardware;
    use fugit::ExtU32;
    use hex_literal::hex;
    use rtic_monotonics;
    use rtic_monotonics::Monotonic;
    use stm32g4xx_hal::prelude::OutputPin;

    pub const MONOTONIC_FREQUENCY: u32 = 1_000;
    rtic_monotonics::systick_monotonic!(Systick, MONOTONIC_FREQUENCY);

    // Shared resources go here
    #[shared]
    struct Shared {
        pcan_tx: can_queue::TxQueue<hardware::PCAN>,
    }

    // Local resources go here
    #[local]
    struct Local {
        srs_crash_out: hardware::PwmSrsCrashOutput,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // TODO: deconstruct board here
        let board = hardware::init(cx.device);

        let (_pcan_rx, _pcan_receive, pcan_tx) = can_queue::init(
            board.pcan_config,
            &board.can_timing_500kbps);

        task1::spawn().ok();

        (
            Shared { pcan_tx },
            Local {
                srs_crash_out: board.srs_crash_out,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1)]
    async fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }

    #[task(shared = [pcan_tx], local = [srs_crash_out], priority = 3)]
    async fn srscm(mut cx: srscm::Context) {
        // SRS/Airbag control system task. Currently very simple:
        // 1Hz CAN message (constant)
        // 50Hz PWM output, 80% high for "not crashed"
        let can_1hz = can_queue::QueuedFrame::new_std(0x5a0, &hex!("000000C025029101"));
        let duty_pct = 80;
        // TODO: calculate this from a Rate in a less icky way
        //let cycle_time = 50.Hz().try_into_duration().unwrap();
        //let time_high = Systick::delay(cycle_time * duty_pct / 100);
        //let time_low = Systick::delay(cycle_time * (100 - duty_pct) / 100);
        let time_high = (20_u32 * duty_pct / 100).millis();
        let time_low = (20_u32 * duty_pct / 100).millis();

        let crash_out = &mut cx.local.srs_crash_out;

        loop {
            // Every 1Hz
            cx.shared.pcan_tx.lock(|tx| tx.transmit(&can_1hz));

            for _ in 0..50 {
                crash_out.set_low().unwrap();
                Systick::delay(time_low).await;
                crash_out.set_high().unwrap();
                Systick::delay(time_high).await;
            }
        }
    }

    // #[task(binds = USB_HP_CAN_TX, shared = [can_tx], priority = 5)]
    // fn can1_tx(mut cx: can1_tx::Context) {
    //     cx.shared.can_tx.lock(|can_tx| can_tx.on_tx_irq());
    // }

    // #[task(binds = USB_LP_CAN_RX0, local = [can_rx], priority = 5)]
    // fn can1_rx(cx: can1_rx::Context) {
    //     cx.local.can_rx.on_rx_irq();
    // }

}
