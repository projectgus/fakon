///! IEB - Integrated Electronic Brake module
///!
///! Also manages traction control and vehicle stability control messages. Most of this
///! is spoofed, the VCU's perspective should be that it's forever driving in a straight
///! line down a road with perfect traction...
use crate::can_queue::{self, QueuedFrame};
use crate::can_utils::byte_checksum_simple;
use crate::car::{CarState, Ignition};
use crate::dbc::pcan::{
    Ieb2a2, Ieb331, Ieb386Wheel, Ieb387Wheel, Ieb507Tcs, ParkingBrake, StabilityControl,
    TractionControlFast, TractionControlMed,
};
use crate::periodic::{send_periodic, PeriodicMessage};
use crate::{hardware, Rate};
use embedded_can::Frame;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;

pub async fn task_ieb<MPCAN, MCAR>(mut pcan_tx: MPCAN, mut car: MCAR) -> !
where
    MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = CarState>,
{
    // Initialise all the raw CAN messages

    let mut traction_fast =
        TractionControlFast::try_from(hex!("208010FF00FF40EE").as_slice()).unwrap();
    let mut traction_med =
        TractionControlMed::try_from(hex!("00E00000FF43B298").as_slice()).unwrap();
    let mut brake_pedal = Ieb2a2::try_from(hex!("0500001C1000005E").as_slice()).unwrap();
    let mut ieb_331 = Ieb331::try_from(hex!("F000000000000000").as_slice()).unwrap();
    let mut ieb_386 = Ieb386Wheel::try_from(hex!("0000000000400080").as_slice()).unwrap();
    let mut ieb_387 = Ieb387Wheel::try_from(hex!("0A0D000000210A00").as_slice()).unwrap();
    let mut ieb_507 = Ieb507Tcs::try_from(hex!("00000001").as_slice()).unwrap();
    // Parking brake has a lot of fields but can send a static "parking brake off" message
    let mut parking_brake = ParkingBrake::try_from(hex!("0000082100000000").as_slice()).unwrap();
    let mut stability = StabilityControl::new(
        0.0, false, false, // Lat accel
        0.0, false, false, // Long accel
        0.0, false, false, // Cyl pressure
        0.0, false, false, // Yaw rate
        0, 0,
    )
    .unwrap();

    send_periodic(
        &mut [
            &mut traction_fast,
            &mut traction_med,
            &mut stability,
            &mut brake_pedal,
            &mut ieb_331,
            &mut parking_brake,
            &mut ieb_386,
            &mut ieb_387,
            &mut ieb_507,
        ],
        &mut pcan_tx,
        &mut car,
    )
    .await;
}

impl PeriodicMessage for TractionControlFast {
    fn rate(&self) -> Rate {
        100.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None; // IEB is only on when car is on
        }

        // Update counters
        {
            self.set_counter1(match self.counter1() {
                Self::COUNTER1_MAX.. => Self::COUNTER1_MIN,
                c => c + 1,
            })
            .unwrap();

            self.set_counter2(match self.counter2() {
                Self::COUNTER2_MAX.. => Self::COUNTER2_MIN,
                0x8 => 0xA, // counter2 skips 0x9 rather than 0xF
                c => c + 1,
            })
            .unwrap();
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for TractionControlMed {
    fn rate(&self) -> Rate {
        50.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        // Update brake pedal state
        self.set_driver_braking(car.is_braking()).unwrap();

        // Increment counter
        {
            self.set_counter(match self.counter() {
                Self::COUNTER_MAX.. => Self::COUNTER_MIN,
                n => n + 1,
            })
            .unwrap();
        }

        // Update checksum
        {
            self.set_checksum(0).unwrap();

            // Checksum is the sum of the nibbles of all bytes except the last
            let new_sum = self.raw()[..7]
                .iter()
                .fold(0u8, |n, e| (n + (e >> 4) + (e & 0xF)) & 0xF);

            // Then take binary complement, add one, and mask modulo 16
            self.set_checksum(((new_sum ^ 0x0F) + 1) & 0x0F).unwrap();
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

// Brake pedal data. Includes pedal force field and other brake-proportional field.
impl PeriodicMessage for Ieb2a2 {
    fn rate(&self) -> Rate {
        100.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        {
            if car.ignition() != Ignition::On {
                return None;  // IEB is only on when car is on
            }

            self.set_heart_beat(!self.heart_beat()).unwrap();

            if car.is_braking() {
                self.set_brake_pedal_force(0x101C).unwrap(); // Arbitrary value
                self.set_brake_unknown(0x5e).unwrap(); // BrakeUnknown, roughly correlates with BrakePedalForce
            } else {
                self.set_brake_pedal_force(0).unwrap();
                self.set_brake_unknown(0).unwrap();
            }
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Ieb331 {
    fn rate(&self) -> Rate {
        100.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        // Seems to be approx 2x "BrakeUnknown" in Ieb2a2
        self.set_brake_unknown(if car.is_braking() { 0xEB } else { 0x00 })
            .unwrap();

        QueuedFrame::new(self.id(), self.raw())
    }
}

// IEB 386, wheel speed data
impl PeriodicMessage for Ieb386Wheel {
    fn rate(&self) -> Rate {
        50.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        // Live counters in the top 2 bits of each 16-bit wheel speed value
        self.set_whl_spd_alive_counter_lsb(match self.whl_spd_alive_counter_lsb() {
            Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MAX.. => {
                Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MIN
            }
            lsb => lsb + 1,
        })
        .unwrap();

        if self.whl_spd_alive_counter_lsb() == Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MIN {
            // When LSB wraps, increment the MSB
            self.set_whl_spd_alive_counter_msb(match self.whl_spd_alive_counter_msb() {
                Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_MSB_MAX.. => {
                    Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_MSB_MIN
                }
                msb => msb + 1,
            })
            .unwrap();
        }

        // Note: the rear wheel speeds have 2-bit checksum fields (rather than live counters), but
        // if the rear wheel speeds stay at 0 then these checksums don't change either (are set in the
        // hex string for the constructor.)

        QueuedFrame::new(self.id(), self.raw())
    }
}

// IEB 387 Wheel pulse counts
impl PeriodicMessage for Ieb387Wheel {
    fn rate(&self) -> Rate {
        50.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        // Update counter
        {
            self.set_alive_counter_whl_pul(match self.alive_counter_whl_pul() {
                Ieb387Wheel::ALIVE_COUNTER_WHL_PUL_MAX.. => Ieb387Wheel::ALIVE_COUNTER_WHL_PUL_MIN,
                n => n + 1,
            })
            .unwrap();
        }

        // Update checksum
        {
            // Byte 5 is a checksum that weirdly includes byte 6 after it, maybe also 7
            self.set_whl_pul_chksum(0).unwrap();
            self.set_whl_pul_chksum(byte_checksum_simple(self.raw()))
                .unwrap();
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Ieb507Tcs {
    fn rate(&self) -> Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for ParkingBrake {
    fn rate(&self) -> Rate {
        20.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for StabilityControl {
    fn rate(&self) -> Rate {
        100.Hz()
    }

    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame> {
        if car.ignition() != Ignition::On {
            return None;  // IEB is only on when car is on
        }

        // Increment counter
        {
            self.set_counter(match self.counter() {
                Self::COUNTER_MAX.. => Self::COUNTER_MIN,
                n => n + 1,
            })
            .unwrap();
        }

        // Update checksum
        {
            self.set_checksum(0).unwrap();
            let new_sum = self.raw().iter().fold(0u8, |n, e| n.wrapping_add(*e));
            self.set_checksum((new_sum ^ 0x9) & 0xF).unwrap();
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}
