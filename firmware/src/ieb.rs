use crate::can_periodic::{
    byte_checksum_simple, counter_update, counter_update_skip, task_periodic_callback,
};
use crate::can_queue::{self, QueuedFrame, Tx};
use crate::hardware;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;
use stm32g4xx_hal::prelude::InputPin;

// TODO: Add the other brake pedal signal gpio
pub async fn task_ieb_100_hz<M>(mut pcan_tx: M, brake_input: &mut hardware::BrakeInput) -> !
where
    M: Mutex<T = can_queue::TxQueue<hardware::PCAN>>,
{
    let mut tcs_153 = QueuedFrame::new_std(0x153, &hex!("208010FF00FF0000"));
    let mut ieb_2a2 = QueuedFrame::new_std(0x2a2, &hex!("0500001C1000005E"));
    let mut ieb_331 = QueuedFrame::new_std(0x331, &hex!("F000000000000000"));

    task_periodic_callback(100.Hz(), || -> () {
        let braking = brake_input.is_high().unwrap();

        // TCS 153
        {
            counter_update_skip::<0xF0, 0xF0>(&mut tcs_153.data[6]);
            tcs_153.data[7] = byte_checksum_simple(&tcs_153.data[..6]);
        }

        // 2A2 Brake pedal data. Includes pedal force field and other brake-proportional field.
        {
            ieb_2a2.data[0] ^= 0x01; // LSB of byte 0 is a heartbit bit TODO:confirm LSB not MSB

            if braking {
                ieb_2a2.data[3] = 0x1C; // 3,4 BrakePedalForce 16-bit
                ieb_2a2.data[4] = 0x10;
                ieb_2a2.data[7] = 0x5e; // 7 BrakeUnknown, roughly correlates with BrakePedalForce
            } else {
                ieb_2a2.data[3] = 0x00;
                ieb_2a2.data[4] = 0x00;
                ieb_2a2.data[7] = 0x00;
            }
        }

        // IEB 331, unknown message includes wheel speed data
        {
            // Seems to be approx 2x "BrakeUnknown" in ieb_2a2
            ieb_331.data[0] = if braking { 0xEB } else { 0x00 };
        }

        // IEB 386, wheel speed data

        pcan_tx.lock(|tx| {
            for m in [&tcs_153, &ieb_2a2, &ieb_331] {
                tx.transmit(m)
            }
        });
    })
    .await
}

// TODO: Add the other brake pedal signal gpio
pub async fn task_ieb_50_hz<M>(mut pcan_tx: M) -> !
where
    M: Mutex<T = can_queue::TxQueue<hardware::PCAN>>,
{
    let mut ieb_386 = QueuedFrame::new_std(0x386, &hex!("0000000000400080"));
    let mut ieb_387 = QueuedFrame::new_std(0x387, &hex!("0A0D000000210A00"));

    task_periodic_callback(50.Hz(), || -> () {
        // IEB 386, wheel speed data
        {
            // Live counters in the top 2 bits of each 16-bit wheel speed value
            counter_update::<0xC0>(&mut ieb_386.data[1]);
            counter_update::<0xC0>(&mut ieb_386.data[3]);
        }

        // IEB 387 Wheel pulse counts
        {
            counter_update::<0x0F>(&mut ieb_387.data[7]);

            // 4 bit alive counter in the lower nibble of bytes 6
            counter_update_skip::<0x0F, 0x0E>(&mut ieb_387.data[6]);

            // Byte 5 is a checksum that weirdly includes byte 6 after it, maybe also 7
            ieb_387.data[5] = 0;
            ieb_387.data[5] = byte_checksum_simple(&ieb_387.data);
        }

        pcan_tx.lock(|tx| {
            for m in [&ieb_386, &ieb_387] {
                tx.transmit(m)
            }
        });
    })
    .await
}

pub async fn task_ieb_10_hz<M>(mut pcan_tx: M) -> !
where
    M: Mutex<T = can_queue::TxQueue<hardware::PCAN>>,
{
    let ieb_507 = QueuedFrame::new_std(0x507, &hex!("00000001"));

    task_periodic_callback(10.Hz(), || -> () {
        pcan_tx.lock(|tx| {
            tx.transmit(&ieb_507);
        });
    })
    .await
}
