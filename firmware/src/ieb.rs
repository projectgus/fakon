//! IEB - Integrated Electronic Brake module
//!
//! Also manages traction control and vehicle stability control messages. Most of this
//! is spoofed, the VCU's perspective should be that it's forever driving in a straight
//! line down a road with perfect traction...
use crate::app;
use crate::can_utils::byte_checksum_simple;
use crate::car::{CarState, Ignition};
use crate::dbc::pcan::{
    Ieb2a2, Ieb331, Ieb386Wheel, Ieb387Wheel, Ieb507Tcs, ParkingBrake, StabilityControl,
    TractionControlFast, TractionControlMed,
};
use crate::hardware::Mono;
use crate::repeater::{Period, Repeater};
use fugit::ExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

pub async fn task_ieb(cx: app::task_ieb::Context<'_>) {
    let mut car = cx.shared.car;
    let mut pcan_tx = cx.shared.pcan_tx;

    // Initialise all the raw CAN messages

    let ieb507 = Ieb507Tcs::try_from(hex!("00000001").as_slice()).unwrap();

    // Parking brake has a lot of fields but for now send a constant "parking brake off" message
    let parking_brake = ParkingBrake::try_from(hex!("0000082100000000").as_slice()).unwrap();

    loop {
        while car.lock(|car| car.ignition() != Ignition::On) {
            // IEB only runs while ignition is on. Until we have some
            // event trigger for this, poll for it in a loop...
            Mono::delay(20.millis()).await;
        }

        // Restart all the periodic counters each time ignition comes on
        let mut repeater = Repeater::new();

        // Counters
        let mut tf_counter1 = 0u8;
        let mut tf_counter2 = 0u8;
        let mut tm_counter = 0u8;
        let mut ieb_counter = false;
        let mut wheel_counter1 = 0u8;
        let mut wheel_counter2 = 0u8;
        let mut wheel_counter3 = 0u8;
        let mut stability_counter = 0u8;

        loop {
            if car.lock(|car| car.ignition() != Ignition::On) {
                break; // Go back around to the outer loop
            }

            for next in repeater.tick().await {
                match next {
                    Period::Hz10 => {
                        pcan_tx.lock(|tx| {
                            tx.transmit(&ieb507);
                        });
                    }
                    Period::Hz20 => {
                        pcan_tx.lock(|tx| {
                            tx.transmit(&parking_brake);
                        });
                    }
                    Period::Hz50 => {
                        let (tcs_med, ieb386, ieb387) = car.lock(|car| {
                            (
                                TractionControlMed::latest(car, &mut tm_counter),
                                Ieb386Wheel::latest(car, &mut wheel_counter1, &mut wheel_counter2),
                                Ieb387Wheel::latest(car, &mut wheel_counter3),
                            )
                        });
                        pcan_tx.lock(|tx| {
                            tx.transmit(&tcs_med);
                            tx.transmit(&ieb386);
                            tx.transmit(&ieb387);
                        });
                    }
                    Period::Hz100 => {
                        let (ieb2a2, ieb331, stability) = car.lock(|car| {
                            (
                                Ieb2a2::latest(car, &mut ieb_counter),
                                Ieb331::latest(car),
                                StabilityControl::latest(&mut stability_counter),
                            )
                        });
                        pcan_tx.lock(|tx| {
                            tx.transmit(&TractionControlFast::latest(
                                &mut tf_counter1,
                                &mut tf_counter2,
                            ));
                            tx.transmit(&ieb2a2);
                            tx.transmit(&ieb331);
                            tx.transmit(&stability);
                        });
                    }
                    _ => (),
                }
            }
        }
    }
}

impl TractionControlFast {
    fn latest(counter1: &mut u8, counter2: &mut u8) -> Self {
        let mut res = Self::try_from(hex!("208010FF00FF40EE").as_slice()).unwrap();

        // Update counters
        {
            *counter1 = match *counter1 {
                Self::COUNTER1_MAX.. => Self::COUNTER1_MIN,
                c => c + 1,
            };

            *counter2 = match *counter2 {
                Self::COUNTER2_MAX.. => Self::COUNTER2_MIN,
                0x8 => 0xA, // counter2 skips 0x9 rather than 0xF
                c => c + 1,
            };

            res.set_counter1(*counter1).unwrap();
            res.set_counter2(*counter2).unwrap();
        }

        res
    }
}

impl TractionControlMed {
    fn latest(car: &CarState, counter: &mut u8) -> Self {
        let mut res = Self::try_from(hex!("00E00000FF43B298").as_slice()).unwrap();

        // Update brake pedal state
        res.set_driver_braking(car.is_braking()).unwrap();

        // Increment counter
        {
            *counter = match *counter {
                Self::COUNTER_MAX.. => Self::COUNTER_MIN,
                n => n + 1,
            };
            res.set_counter(*counter).unwrap();
        }

        // Update checksum
        {
            res.set_checksum(0).unwrap(); // TODO: needed?

            // Checksum is the sum of the nibbles of all bytes except the last
            let new_sum = res.raw()[..7]
                .iter()
                .fold(0u8, |n, e| (n + (e >> 4) + (e & 0xF)) & 0xF);

            // Then take binary complement, add one, and mask modulo 16
            res.set_checksum(((new_sum ^ 0x0F) + 1) & 0x0F).unwrap();
        }

        res
    }
}

impl Ieb2a2 {
    // Brake pedal data. Includes pedal force field and other brake-proportional field.
    fn latest(car: &CarState, counter: &mut bool) -> Self {
        let mut res = Self::try_from(hex!("0500001C1000005E").as_slice()).unwrap();

        res.set_heart_beat(*counter).unwrap();
        *counter = !(*counter);

        if car.is_braking() {
            res.set_brake_pedal_force(0x101C).unwrap(); // Arbitrary value
            res.set_brake_unknown(0x5e).unwrap(); // BrakeUnknown, roughly correlates with BrakePedalForce
        } else {
            res.set_brake_pedal_force(0).unwrap();
            res.set_brake_unknown(0).unwrap();
        }

        res
    }
}

impl Ieb331 {
    fn latest(car: &CarState) -> Self {
        let mut res = Self::try_from(hex!("F000000000000000").as_slice()).unwrap();

        // Seems to be approx 2x "BrakeUnknown" in Ieb2a2
        res.set_brake_unknown(if car.is_braking() { 0xEB } else { 0x00 })
            .unwrap();

        res
    }
}

impl Ieb386Wheel {
    // Wheel speed data
    fn latest(_car: &CarState, counter1: &mut u8, counter2: &mut u8) -> Self {
        let mut res = Self::try_from(hex!("0000000000400080").as_slice()).unwrap();

        // Live counters in the top 2 bits of each 16-bit wheel speed value
        *counter1 = match *counter1 {
            Self::WHL_SPD_ALIVE_COUNTER_LSB_MAX.. => Self::WHL_SPD_ALIVE_COUNTER_LSB_MIN,
            lsb => lsb + 1,
        };

        if *counter1 == Self::WHL_SPD_ALIVE_COUNTER_LSB_MIN {
            // When LSB wraps, increment the MSB
            *counter2 = match *counter2 {
                Self::WHL_SPD_ALIVE_COUNTER_MSB_MAX.. => Self::WHL_SPD_ALIVE_COUNTER_MSB_MIN,
                msb => msb + 1,
            };
        }

        res.set_whl_spd_alive_counter_lsb(*counter1).unwrap();
        res.set_whl_spd_alive_counter_lsb(*counter2).unwrap();

        // Note: the rear wheel speeds have 2-bit checksum fields (rather than live counters), but
        // if the rear wheel speeds stay at 0 then these checksums don't change either (are set in the
        // hex raw constructor value.)

        res
    }
}

impl Ieb387Wheel {
    fn latest(_car: &CarState, counter: &mut u8) -> Self {
        let mut res = Self::try_from(hex!("0A0D000000000A00").as_slice()).unwrap();

        // Update counter
        {
            *counter = match *counter {
                Self::ALIVE_COUNTER_WHL_PUL_MAX.. => Self::ALIVE_COUNTER_WHL_PUL_MIN,
                n => n + 1,
            };
            res.set_alive_counter_whl_pul(*counter).unwrap();
        }

        // Update checksum
        {
            // Byte 5 is a checksum that weirdly includes byte 6 after it, maybe also 7
            res.set_whl_pul_chksum(byte_checksum_simple(res.raw()))
                .unwrap();
        }

        res
    }
}

impl StabilityControl {
    fn latest(counter: &mut u8) -> Self {
        let mut res = StabilityControl::new(
            0.0, false, false, // Lat accel
            0.0, false, false, // Long accel
            0.0, false, false, // Cyl pressure
            0.0, false, false, // Yaw rate
            0, 0,
        )
        .unwrap();

        // Increment counter
        {
            *counter = match *counter {
                Self::COUNTER_MAX.. => Self::COUNTER_MIN,
                n => n + 1,
            };
            res.set_counter(*counter).unwrap();
        }

        // Update checksum
        {
            let new_sum = res.raw().iter().fold(0u8, |n, e| n.wrapping_add(*e));
            res.set_checksum((new_sum ^ 0x9) & 0xF).unwrap();
        }

        res
    }
}
