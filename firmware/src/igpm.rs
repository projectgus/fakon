use crate::car::Contactor;
use crate::car::Ignition;
use crate::dbc::pcan::ChargeSettingsAcChargingCurrent;
// "CAN Gateway" messages ("Integrated Gateway Power Module").
// Some of these may originate from modules on other buses and
// be relayed via the IGPM, some generated by IGPM.
use crate::can_queue;
use crate::car;
use crate::dbc::pcan::Steering;
use crate::dbc::pcan::{
    BodyState, BodyStateDrvDoorSw, BodyStateDrvSeatBeltSw, BodyStateIgnitionSw,
    BodyStatePassDoorSw, BodyWarnings, Cgw5df, ChargePort, ChargeSettings, Clock,
    Odometer,
};
use crate::hardware::{self, Mono};
use crate::periodic::PeriodicGroup;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

pub async fn task_igpm<MPCAN, MCAR>(mut pcan_tx: MPCAN, mut car: MCAR) -> !
where
    MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = car::CarState>,
{
    // Note this message is only used by 2019 model Kona, not 2021 model which uses a different one
    let charge_settings =
        ChargeSettings::new(ChargeSettingsAcChargingCurrent::Maximum.into()).unwrap();

    // BodyState constructor has 43 args, so start from all zeroes and then set some bits!
    let mut body_state = BodyState::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // Driver and passenger door closed, drivers seatbelt on - necessary to allow Drive
    body_state
        .set_drv_door_sw(BodyStateDrvDoorSw::Closed.into())
        .unwrap();
    body_state
        .set_pass_door_sw(BodyStatePassDoorSw::Closed.into())
        .unwrap();
    body_state
        .set_drv_seat_belt_sw(BodyStateDrvSeatBeltSw::Present.into())
        .unwrap();

    // Unclear what this signal does or if it needs to be set. TODO: check if needed
    body_state.set_cf_gway_drive_type_option(true).unwrap();

    // First arg is "unknown" field as seen when vehicle is off, details are unknown...
    let mut clock = Clock::new(0x8D, 0, 0, 0, 0).unwrap();

    let mut charge_port = ChargePort::new(false, false, false, false).unwrap();

    // Unknown message. The message contents changes sometimes in logs, but very irregularly.
    let igpm_5df = Cgw5df::try_from(hex!("C5FFFF0100000000").as_ref()).unwrap();

    // Another type whose constructor has too many args
    let body_warnings = {
        let mut bw = BodyWarnings::try_from(hex!("0000000000000000").as_ref()).unwrap();
        // These two bits seem to be set during "normal" operation
        bw.set_cf_gway_gway_diag_state(true).unwrap();
        bw.set_cf_gway_sjb_delivery_mode(true).unwrap();
        bw
    };

    // Odometer reading
    // Currently using the logged reading from the 2022 car. TODO: check if needs to exceed value set in the 2019 VCU?
    let odometer = Odometer::new(14452.5).unwrap();

    // Steering angle sensor
    let mut steering = Steering::new(9.2, 0, 0x7, 0, 0).unwrap();

    let mut group = PeriodicGroup::new(100.Hz());
    let mut period_100hz = group.new_period(100.Hz());
    let mut period_10hz = group.new_period(10.Hz());
    let mut period_5hz = group.new_period(5.Hz());
    let mut period_1hz = group.new_period(1.Hz());

    loop {
        group.next_poll().await;

        // Car state is a reasonably small struct, cloning it means whole iteration
        // sees consistent state without holding the lock
        let car_state = car.lock(|c| c.clone());

        if period_100hz.due(&group) && car_state.ignition().ig3_on() {
            // Steering angle            {
            steering
                .set_counter(match steering.counter() {
                    Steering::COUNTER_MAX.. => Steering::COUNTER_MIN,
                    n => n + 1,
                })
                .unwrap();

            // 4-bit checksum is XOR of all other nibbles in the message
            steering.set_checksum(0).unwrap();
            let mut checksum = steering.raw().iter().fold(0, |n, a| n ^ a);
            checksum = (checksum >> 4) ^ (checksum & 0x0f);
            steering.set_checksum(checksum).unwrap();

            pcan_tx.lock(|tx| {
                tx.transmit(&steering);
            });
        }

        if period_10hz.due(&group) {
            // Body State
            {
                if car_state.ignition() == Ignition::On {
                    let ignition_sw = if car_state.contactor().get() == Some(Contactor::PreCharging)
                    {
                        // Note: currently the precharging relay is closed much shorter
                        // time than the period of "Starting" shown in the logs.
                        //
                        // This might be timed by other messages, unsure which.
                        BodyStateIgnitionSw::Starting
                    } else {
                        BodyStateIgnitionSw::On
                    };
                    body_state.set_ignition_sw(ignition_sw.into()).unwrap();
                    // In the logs it actually looks like IGN1 & 2 go high at slightly
                    // different times. Not sure what these actually signify...
                    body_state.set_ign1(true).unwrap();
                    body_state.set_ign2(true).unwrap();
                } else {
                    body_state
                        .set_ignition_sw(BodyStateIgnitionSw::Off.into())
                        .unwrap();
                    body_state.set_ign1(false).unwrap();
                    body_state.set_ign2(false).unwrap();
                }
            }

            // Clock
            // Note: technically the clock normally goes "valid" when ignition comes on,
            // placeholder-looking values are sent until then. Hoping can ignore this fact.
            let secs = Mono::now().duration_since_epoch().to_secs();
            clock.set_unknown(0x02).unwrap();
            clock.set_second((secs % 60) as u8).unwrap(); // Seconds
            clock.set_minute(((secs / 60) % 60) as u8).unwrap(); // Minutes
            clock.set_hour(((secs / 60 / 60) % 24) as u8).unwrap(); // Hours
            clock.set_valid(1).unwrap(); // "Valid" flag

            // IGPM 5EC
            {
                // Is charge port locked?
                charge_port
                    .set_charge_port_locked(car_state.charge_port_locked())
                    .unwrap();
            }

            pcan_tx.lock(|tx| {
                tx.transmit(&body_state);
                tx.transmit(&clock);
                tx.transmit(&charge_port);
            });
        }

        if period_5hz.due(&group) {
            pcan_tx.lock(|tx| {
                tx.transmit(&charge_settings); // Note: this message not being sent when it was gated on IG3?
                tx.transmit(&body_warnings);
                tx.transmit(&igpm_5df);
            });
        }

        if period_1hz.due(&group) {
            pcan_tx.lock(|tx| {
                tx.transmit(&odometer);
            });
        }
    }
}
