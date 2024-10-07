use crate::can_utils::byte_checksum_simple;
use crate::hardware;
use crate::car;
use crate::dbc::pcan::{Ieb153Tcs, Ieb2a2, Ieb331, Ieb386Wheel, Ieb387Wheel, Ieb507Tcs};
use crate::periodic::PeriodicGroup;
use crate::can_queue;
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

    let mut group = PeriodicGroup::new(100.Hz());
    let mut every_100hz = group.new_period(100.Hz());
    let mut every_50hz = group.new_period(50.Hz());
    let mut every_10hz = group.new_period(10.Hz());

    loop {
        group.next_poll().await;

        // Read once per loop for consistency
        let braking = car.lock(|car| car.is_braking() );

        if every_100hz.due(&group) {
            // TCS 153
            {
                tcs_153.set_alive_counter_tcs1(
                    match tcs_153.alive_counter_tcs1() {
                        Ieb153Tcs::ALIVE_COUNTER_TCS1_MAX => Ieb153Tcs::ALIVE_COUNTER_TCS1_MIN,
                        c => c + 1,
                    }).unwrap();
            }

            // Brake pedal data. Includes pedal force field and other brake-proportional field.
            {
                brake_pedal.set_heart_beat(!brake_pedal.heart_beat()).unwrap();

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
                ieb_331.set_brake_unknown(if braking { 0xEB } else { 0x00 }).unwrap();
            }

            pcan_tx.lock(|tx| {
                // TODO: can be a loop or a map, maybe?
                tx.transmit(&tcs_153);
                tx.transmit(&brake_pedal);
                tx.transmit(&ieb_331);
            });
        }

        if every_50hz.due(&group) {
            // IEB 386, wheel speed data
            {
                // Live counters in the top 2 bits of each 16-bit wheel speed value
                ieb_386.set_whl_spd_alive_counter_lsb(
                    match ieb_386.whl_spd_alive_counter_lsb() {
                        Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MAX => Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MIN,
                        lsb => lsb + 1
                    }).unwrap();

                if ieb_386.whl_spd_alive_counter_lsb() == Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_LSB_MIN {
                    // When LSB wraps, increment the MSB
                    ieb_386.set_whl_spd_alive_counter_msb(
                        match ieb_386.whl_spd_alive_counter_msb() {
                            Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_MSB_MAX => Ieb386Wheel::WHL_SPD_ALIVE_COUNTER_MSB_MIN,
                            msb => msb + 1,
                        }).unwrap();
                }
            }

            // IEB 387 Wheel pulse counts
            {
                ieb_387.set_alive_counter_whl_pul(
                    match ieb_387.alive_counter_whl_pul() {
                        Ieb387Wheel::ALIVE_COUNTER_WHL_PUL_MAX => Ieb387Wheel::ALIVE_COUNTER_WHL_PUL_MIN,
                        n => n + 1,
                    }).unwrap();

                // Byte 5 is a checksum that weirdly includes byte 6 after it, maybe also 7
                ieb_387.set_whl_pul_chksum(0).unwrap();
                ieb_387.set_whl_pul_chksum(byte_checksum_simple(ieb_387.raw())).unwrap();
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
