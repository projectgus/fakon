use crate::can_queue;
use crate::can_utils::byte_checksum_simple;
use crate::car::{self, Ignition};
use crate::dbc::pcan::{Ieb153Tcs, Ieb2a2, Ieb331, Ieb386Wheel, Ieb387Wheel, Ieb507Tcs, StabilityControl};
use crate::hardware;
use crate::periodic::PeriodicGroup;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;

// TODO: Add the other brake pedal signal gpio
pub async fn task_ieb<MPCAN, MCAR>(mut pcan_tx: MPCAN, mut car: MCAR) -> !
where
    MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = car::CarState>,
{
    let mut tcs_153 = Ieb153Tcs::try_from(hex!("208010FF00FF0000").as_slice()).unwrap();
    let mut brake_pedal = Ieb2a2::try_from(hex!("0500001C1000005E").as_slice()).unwrap();
    let mut ieb_331 = Ieb331::try_from(hex!("F000000000000000").as_slice()).unwrap();
    let mut ieb_386 = Ieb386Wheel::try_from(hex!("0000000000400080").as_slice()).unwrap();
    let mut ieb_387 = Ieb387Wheel::try_from(hex!("0A0D000000210A00").as_slice()).unwrap();
    let ieb_507 = Ieb507Tcs::try_from(hex!("00000001").as_slice()).unwrap();
    // let mut stability = StabilityControl::new(0.0, false, false, // Lat accel
    //                                       0.0, false, false, // Long accel
    //                                       0.0, false, false, // Cyl pressure
    //                                       0.0, false, false, // Yaw rate
    //                                       0, 0).unwrap();

    // The stability control message has a non-trivial 4-bit checksum, so for now
    // replay stationary messages for all 4-bit counter values, on a loop...
    let stability_values = [
        hex!("CFA388A003039009"),
        hex!("D08388A003FC8F10"),
        hex!("D183889C03FE8F21"),
        hex!("D0C388A003FF8F35"),
        hex!("CFC3889C03F98F48"),
        hex!("CFC3889C03FB8F5A"),
        hex!("CE03899C03049064"),
        hex!("CFE388A003049078"),
        hex!("D1C388A00304908A"),
        hex!("D1A388A003029098"),
        hex!("D0A388A0030290A9"),
        hex!("D0A388A0030190B6"),
        hex!("D3A388A003FB8FC2"),
        hex!("D3E388A003FD8FD4"),
        hex!("D0C388A0030190E6"),
        hex!("CFA388A0030190F7"),
    ].map(|h| StabilityControl::try_from(h.as_ref()).unwrap());
    let mut stability_cycle = stability_values.iter().cycle();

    let mut group = PeriodicGroup::new(100.Hz());
    let mut every_100hz = group.new_period(100.Hz());
    let mut every_50hz = group.new_period(50.Hz());
    let mut every_10hz = group.new_period(10.Hz());

    loop {
        group.next_poll().await;

        if !car.lock(|car| car.ignition() == Ignition::On) {
            // IEB only starts sending messages when car ignition comes on.
            //
            // NOTE: It keeps sending them for a time after ignition goes off, until the
            // thing goes to sleep. Unsure if need to model that here or not.
            continue;
        }

        // Read once per loop for consistency
        let braking = car.lock(|car| car.is_braking());

        if every_100hz.due(&group) {
            // TCS 153
            {
                tcs_153
                    .set_alive_counter_tcs1(match tcs_153.alive_counter_tcs1() {
                        Ieb153Tcs::ALIVE_COUNTER_TCS1_MAX => Ieb153Tcs::ALIVE_COUNTER_TCS1_MIN,
                        c => c + 1,
                    })
                    .unwrap();

                // TODO: this message has a checksum!
            }

            // Brake pedal data. Includes pedal force field and other brake-proportional field.
            {
                brake_pedal
                    .set_heart_beat(!brake_pedal.heart_beat())
                    .unwrap();

                if braking {
                    brake_pedal.set_brake_pedal_force(0x101C).unwrap(); // Arbitrary value
                    brake_pedal.set_brake_unknown(0x5e).unwrap(); // BrakeUnknown, roughly correlates with BrakePedalForce
                } else {
                    brake_pedal.set_brake_pedal_force(0).unwrap();
                    brake_pedal.set_brake_unknown(0).unwrap();
                }
            }

            // IEB 331, unknown message includes wheel speed data
            {
                // Seems to be approx 2x "BrakeUnknown" in brake_pedal
                ieb_331
                    .set_brake_unknown(if braking { 0xEB } else { 0x00 })
                    .unwrap();
            }

            // ESP status
            {
                //stability.set_counter(match stability.counter() {
                //    StabilityControl::COUNTER_MAX => StabilityControl::COUNTER_MIN,
                //    n => n + 1,
                //}).unwrap();
            }
            let stability = stability_cycle.next().unwrap();

            pcan_tx.lock(|tx| {
                // TODO: can be a loop or a map, maybe?
                tx.transmit(&tcs_153);
                tx.transmit(&brake_pedal);
                tx.transmit(&ieb_331);
                tx.transmit(stability);
            });
        }

        if every_50hz.due(&group) {
            // IEB 386, wheel speed data
            {
                // Live counters in the top 2 bits of each 16-bit wheel speed value
                ieb_386
                    .set_whl_spd_alive_counter_lsb(match ieb_386.whl_spd_alive_counter_lsb() {
                        Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MAX => {
                            Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MIN
                        }
                        lsb => lsb + 1,
                    })
                    .unwrap();

                if ieb_386.whl_spd_alive_counter_lsb() == Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MIN
                {
                    // When LSB wraps, increment the MSB
                    ieb_386
                        .set_whl_spd_alive_counter_msb(match ieb_386.whl_spd_alive_counter_msb() {
                            Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_MSB_MAX => {
                                Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_MSB_MIN
                            }
                            msb => msb + 1,
                        })
                        .unwrap();
                }

                // TODO: this message has 2-bit checksums
            }

            // IEB 387 Wheel pulse counts
            {
                ieb_387
                    .set_alive_counter_whl_pul(match ieb_387.alive_counter_whl_pul() {
                        Ieb387Wheel::ALIVE_COUNTER_WHL_PUL_MAX => {
                            Ieb387Wheel::ALIVE_COUNTER_WHL_PUL_MIN
                        }
                        n => n + 1,
                    })
                    .unwrap();

                // Byte 5 is a checksum that weirdly includes byte 6 after it, maybe also 7
                ieb_387.set_whl_pul_chksum(0).unwrap();
                ieb_387
                    .set_whl_pul_chksum(byte_checksum_simple(ieb_387.raw()))
                    .unwrap();
            }

            pcan_tx.lock(|tx| {
                tx.transmit(&ieb_386);
                tx.transmit(&ieb_387);
            });
        }

        if every_10hz.due(&group) {
            pcan_tx.lock(|tx| {
                tx.transmit(&ieb_507);
            });
        }
    }
}
