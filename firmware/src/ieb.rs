use crate::can_periodic::{byte_checksum_simple, counter_update_skip, task_periodic_callback};
use crate::can_queue::{self, QueuedFrame, Tx};
use crate::hardware;
use stm32g4xx_hal::prelude::InputPin;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;

// TODO: Add the other brake pedal signal gpio
pub async fn task_ieb_100_hz<M>(mut pcan_tx: M, brake_input: &mut hardware::BrakeInput) -> !
where
    M: Mutex<T = can_queue::TxQueue<hardware::PCAN>>,
{
    let mut tcs_153 = QueuedFrame::new_std(0x153, &hex!("208010FF00FF0000"));
    let mut ieb_2a2 = QueuedFrame::new_std(0x2a2, &hex!("0500001C1000005E"));

    task_periodic_callback(100.Hz(), || -> () {
        // TCS 153
        {
            counter_update_skip::<0xF0>(&mut tcs_153.data[6]);
            tcs_153.data[7] = byte_checksum_simple(&tcs_153.data[..6]);
        }

        // 2A2 Brake pedal data. Includes pedal force field and other brake-proportional field.
        {
            ieb_2a2.data[0] ^= 0x01; // LSB of byte 0 is a heartbit bit TODO:confirm LSB not MSB

            if brake_input.is_high().unwrap() {
                ieb_2a2.data[3] = 0x1C;  // 3,4 BrakePedalForce 16-bit
                ieb_2a2.data[4] = 0x10;
                ieb_2a2.data[7] = 0x5e;  // 7 BrakeUnknown, roughly correlates with BrakePedalForce
            } else {
                ieb_2a2.data[3] = 0x00;
                ieb_2a2.data[4] = 0x00;
                ieb_2a2.data[7] = 0x00;
            }
        }

        pcan_tx.lock(|tx| {
            for m in [&tcs_153, &ieb_2a2] {
                tx.transmit(m)
            }
        });
    })
    .await
}
