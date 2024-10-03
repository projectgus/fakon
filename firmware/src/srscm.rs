use crate::can_queue;
use crate::hardware;
use crate::hardware::Mono;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;
use stm32g4xx_hal::prelude::OutputPin;

pub async fn task<M>(mut pcan_tx: M, crash_out: &mut hardware::PwmSrsCrashOutput) -> !
where
    M: Mutex<T = can_queue::Tx<hardware::PCAN>>,
{
    // SRS/Airbag control system task. Currently very simple:
    // 1Hz CAN message (constant)
    // 50Hz soft PWM output, 80% high for "not crashed", 20% high for "crashed"
    let can_1hz = can_queue::QueuedFrame::new_std(0x5a0, &hex!("000000C025029101"));
    let duty_pct = 80;

    let cycle_time = 50.Hz::<1, 1000>().into_duration();
    let time_high = cycle_time * duty_pct / 100;

    let mut next_cycle = Mono::now();

    loop {
        // Every 1Hz
        pcan_tx.lock(|tx| tx.transmit(&can_1hz));

        for _ in 0..50 {
            crash_out.set_low().unwrap(); // Note: actual output inverted by N-FET
            Mono::delay(time_high).await;
            crash_out.set_high().unwrap();
            next_cycle += cycle_time;
            Mono::delay_until(next_cycle).await;
        }
    }
}
