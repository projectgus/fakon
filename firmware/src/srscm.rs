use crate::can_queue;
use crate::can_queue::Tx;
use crate::hardware;
use crate::hardware::Mono;
use fugit::ExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;
use stm32g4xx_hal::prelude::OutputPin;

pub async fn task<M>(mut pcan_tx: M, crash_out: &mut hardware::PwmSrsCrashOutput) -> !
where
    M: Mutex<T = can_queue::TxQueue<hardware::PCAN>>,
{
    // SRS/Airbag control system task. Currently very simple:
    // 1Hz CAN message (constant)
    // 50Hz soft PWM output, 80% high for "not crashed"
    let can_1hz = can_queue::QueuedFrame::new_std(0x5a0, &hex!("000000C025029101"));
    let duty_pct = 80;

    // TODO: calculate this from a Rate in a less icky way
    //let cycle_time = 50.Hz().try_into_duration().unwrap();
    //let time_high = Systick::delay(cycle_time * duty_pct / 100);
    //let time_low = Systick::delay(cycle_time * (100 - duty_pct) / 100);
    let time_high = (20_u32 * duty_pct / 100).millis();
    let time_low = (20_u32 * duty_pct / 100).millis();

    loop {
        // Every 1Hz
        pcan_tx.lock(|tx| tx.transmit(&can_1hz));

        for _ in 0..50 {
            crash_out.set_low().unwrap();
            Mono::delay(time_low).await;
            crash_out.set_high().unwrap();
            Mono::delay(time_high).await;
        }
    }
}
