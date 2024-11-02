///! IGPM (Integrated Gateway Power Module) is the CAN Gateway and main power
///! distribution unit in the Kona.
///!
///! Some of these messages may originate from other modules in the car, and be
///! forwarded onto the PCAN bus by the IGPM. Others originate from the IGPM.
use crate::car::Contactor;
use crate::car::Ignition;
use crate::dbc::pcan::ChargeSettingsAcChargingCurrent;
use crate::can_queue;
use crate::car;
use crate::dbc::pcan::Steering;
use crate::dbc::pcan::{
    self, BodyState, BodyStateDrvDoorSw, BodyStateDrvSeatBeltSw, BodyStateIgnitionSw,
    BodyStatePassDoorSw, BodyWarnings, Cgw5b3PowerState, Cgw5b3UnkPowerRelated, Cgw5df, ChargePort,
    ChargeSettings, Clock, Odometer,
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

    // this messages is mostly all zeroes but in some logs it's FFFFFFFFFFFFFF0F instead...?
    let zeroes45d = pcan::Cgw45d::try_from(hex!("0000000000000000").as_ref()).unwrap();
    // this one is either all zeroes or FFFFFFFFFF000000
    let zeroes45e = pcan::Cgw45e::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // speed and maybe cruise control buttons?
    let unk450 = pcan::Cgw450::new(0, 0x1804).unwrap();

    // On 2019 this is mostly constant except for one signal, but even that
    // signal only seems to count sometimes while moving... *shrug*
    let unk462 = pcan::Cgw462::try_from(hex!("FE3FFF1FF01F0000").as_ref()).unwrap();

    let unk4fe = pcan::Cgw4fe::try_from(hex!("FFFF7FFFFF00FFFF").as_ref()).unwrap();

    // This message is always zeroes in the 2019 logs, although changes to another
    // pattern on the 2021
    let unk55f = pcan::Cgw55f::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // First two bytes here are unique to my 2019 Kona, unsure if that matters
    let unk55c = pcan::Cgw55c::try_from(hex!("071F14FF01000000").as_ref()).unwrap();

    // Another constant-looking message, although is sometimes all zeroes in logs (briefly)
    let unk561 = pcan::Cgw561::try_from(hex!("0560000780000F00").as_ref()).unwrap();

    let unk578 = pcan::Cgw578::try_from(hex!("000000000000").as_ref()).unwrap();

    let mut unk588 = pcan::Cgw588::new(0x00, 0x00, 0x00).unwrap();

    let mut unk5b3 = pcan::Cgw5b3::new(0xF, 0xFF, 0xFF, 0xF).unwrap();

    let mut group = PeriodicGroup::new(100.Hz());
    let mut period_100hz = group.new_period(100.Hz());
    let mut period_50hz = group.new_period(50.Hz());
    let mut period_10hz = group.new_period(10.Hz());
    let mut period_5hz = group.new_period(5.Hz());
    let mut period_1hz = group.new_period(1.Hz());

    loop {
        group.next_poll().await;

        // Car state is a reasonably small struct, cloning it means whole iteration
        // sees consistent state without holding the lock
        let car_state = car.lock(|c| c.clone());

        let ignition = car_state.ignition();

        if period_100hz.due(&group) && ignition.ig3_on() {
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

        if period_50hz.due(&group) {
            pcan_tx.lock(|tx| {
                tx.transmit(&unk450);
                tx.transmit(&unk462);
            });
        }

        if period_10hz.due(&group) {
            // Body State
            {
                if ignition == Ignition::On {
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
            {
                // Note: technically the clock normally goes "valid" when ignition comes on,
                // placeholder-looking values are sent until then. Hoping can ignore this fact.
                let secs = Mono::now().duration_since_epoch().to_secs();
                clock.set_unknown(0x02).unwrap();
                clock.set_second((secs % 60) as u8).unwrap(); // Seconds
                clock.set_minute(((secs / 60) % 60) as u8).unwrap(); // Minutes
                clock.set_hour(((secs / 60 / 60) % 24) as u8).unwrap(); // Hours
                clock.set_valid(1).unwrap(); // "Valid" flag
            }

            // Charge Port status
            {
                charge_port
                    .set_charge_port_locked(car_state.charge_port_locked())
                    .unwrap();
            }

            // Unk588 reflects vehicle status in some way, not exactly
            // clear how
            {
                let (b0, b1, b2) = match ignition {
                    car::Ignition::Off => (0x00, 0x00, 0x00),
                    car::Ignition::IG3 => (0x58, 0x1c, 0x00),
                    car::Ignition::On => (0xfc, 0xff, 0x03),
                };
                unk588.set_unk0(b0).unwrap();
                unk588.set_unk1(b1).unwrap();
                unk588.set_unk2(b2).unwrap();
            }

            pcan_tx.lock(|tx| {
                tx.transmit(&body_state);
                tx.transmit(&clock);
                tx.transmit(&charge_port);
                tx.transmit(&unk55c);
                tx.transmit(&unk55f);
                tx.transmit(&unk561);
                tx.transmit(&unk578);
                tx.transmit(&unk588);
            });
        }

        if period_5hz.due(&group) {
            // Update 5b3 from ignition
            {
                unk5b3
                    .set_unk_power_related(
                        if ignition.ig3_on() {
                            Cgw5b3UnkPowerRelated::PowerOn
                        } else {
                            Cgw5b3UnkPowerRelated::PoweredOff
                        }
                        .into(),
                    )
                    .unwrap();
                if ignition != car::Ignition::Off {
                    unk5b3.set_unknown2(0x10).unwrap();
                }
                unk5b3
                    .set_power_state(
                        match (ignition, unk5b3.power_state()) {
                            (car::Ignition::Off, Cgw5b3PowerState::Off) => Cgw5b3PowerState::Off,
                            (car::Ignition::Off, _) => Cgw5b3PowerState::GoingToSleep,
                            (car::Ignition::IG3, _) => Cgw5b3PowerState::On,
                            (car::Ignition::On, _) => Cgw5b3PowerState::On,
                        }
                        .into(),
                    )
                    .unwrap();
            }

            pcan_tx.lock(|tx| {
                tx.transmit(&charge_settings); // Note: this message not being sent when it was gated on IG3?
                tx.transmit(&body_warnings);
                tx.transmit(&igpm_5df);
                tx.transmit(&zeroes45d);
                tx.transmit(&zeroes45e);
                tx.transmit(&unk4fe);
                tx.transmit(&unk5b3);
            });
        }

        if period_1hz.due(&group) {
            pcan_tx.lock(|tx| {
                tx.transmit(&odometer);
            });
        }
    }
}
