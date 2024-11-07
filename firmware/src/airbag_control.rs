//! ACU = Airbag Control Unit, i.e. SRS control system.
//!
//! The role in Fakon is pretty small - provided a "crashed" / "not crashed"
//! PWM signal to the BMS. Is optional: BMS shows fault P1A6F00 if missing but works.
//!
//! However having it allows clearing all faults, and allows us to extend later to send
//! a "crashed" and open contactors in an emergency.
use crate::can_queue;
use crate::car;
use crate::car::Ignition;
use crate::dbc::pcan;
use crate::hardware;
use crate::hardware::Mono;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;
use stm32g4xx_hal::prelude::OutputPin;

// Task does two things:
// - 1Hz Send CAN message (constant contents)
// - 50Hz soft PWM output, 80% high duty for "not crashed", 20% for "crashed"
pub async fn task<M, MCAR>(
    mut pcan_tx: M,
    mut car: MCAR,
    crash_out: &mut hardware::AcuCrashOutput,
) -> !
where
    M: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = car::CarState>,
{
    let airbag_status = pcan::AirbagStatus::try_from(hex!("000000C025029101").as_slice()).unwrap();
    let duty_pct = 80;

    let cycle_time = 50.Hz::<1, 1000>().into_duration();
    let time_high = cycle_time * duty_pct / 100;

    let mut next_cycle = Mono::now();

    loop {
        // Every 1Hz
        if car.lock(|car| car.ignition() == Ignition::On) {
            pcan_tx.lock(|tx| tx.transmit(&airbag_status));
        }

        for _ in 0..50 {
            crash_out.set_high().unwrap();
            Mono::delay(time_high).await;
            crash_out.set_low().unwrap();
            next_cycle += cycle_time;
            Mono::delay_until(next_cycle).await;
        }
    }
}
