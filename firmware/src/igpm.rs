//! IGPM (Integrated Gateway Power Module) is the CAN Gateway and main power
//! distribution unit in the Kona.
//!
//! Some of these messages may originate from other modules in the car, and be
//! forwarded onto the PCAN bus by the IGPM. Others originate from the IGPM.
use crate::car::{self, CarState, ChargeLock, Contactor, Ignition};
use crate::dbc::pcan::{
    BodyState, BodyStateDrvDoorSw, BodyStateDrvSeatBeltSw, BodyStateIgnitionSw,
    BodyStatePassDoorSw, BodyWarnings, Cgw450, Cgw45d, Cgw45e, Cgw462, Cgw4fe, Cgw55c, Cgw55f,
    Cgw561, Cgw578, Cgw588, Cgw5b3, Cgw5b3PowerState, Cgw5b3UnkPowerRelated, Cgw5df, ChargePort,
    ChargeSettings, ChargeSettingsAcChargingCurrent, Clock, Messages, Odometer, Steering,
};
use crate::fresh::IsFresh;
use crate::hardware::Mono;
use crate::repeater::{Period, Repeater};
use crate::{app, Duration};
use fugit::ExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;
use stm32g4xx_hal::prelude::{OutputPin, PinState};

pub async fn task_igpm(cx: app::task_igpm::Context<'_>) {
    let mut pcan_tx = cx.shared.pcan_tx;
    let mut car = cx.shared.car;

    let charge_settings =
        ChargeSettings::new(ChargeSettingsAcChargingCurrent::Maximum.into()).unwrap();

    // Unknown message. The message contents changes sometimes in logs, but very irregularly.
    let igpm_5df = Cgw5df::try_from(hex!("C5FFFF0100000000").as_ref()).unwrap();

    let body_warnings = {
        // This constructor has too many args, so emulate a "builder" pattern this way...
        let mut bw = BodyWarnings::try_from(hex!("0000000000000000").as_ref()).unwrap();
        // These two bits seem to be set during "normal" operation
        bw.set_cf_gway_gway_diag_state(true).unwrap();
        bw.set_cf_gway_sjb_delivery_mode(true).unwrap();
        bw
    };

    // Odometer reading
    // Currently using the logged reading from the 2022 car. TODO: check if needs to exceed value set in the 2019 VCU?
    let odometer = Odometer::new(14452.5).unwrap();

    // this messages is mostly all zeroes but in some logs it's FFFFFFFFFFFFFF0F instead...?
    let zeroes45d = Cgw45d::try_from(hex!("0000000000000000").as_ref()).unwrap();
    // this one is either all zeroes or FFFFFFFFFF000000
    let zeroes45e = Cgw45e::try_from(hex!("0000000000000000").as_ref()).unwrap();

    let unk4fe = Cgw4fe::try_from(hex!("FFFF7FFFFF00FFFF").as_ref()).unwrap();

    // speed and maybe cruise control buttons?
    let unk450 = Cgw450::new(0, 0x1804).unwrap();

    // On 2019 this is mostly constant except for one signal, but even that
    // signal only seems to count sometimes while moving... *shrug*
    let unk462 = Cgw462::try_from(hex!("FE3FFF1FF01F0000").as_ref()).unwrap();

    // This message is always zeroes in the 2019 logs, although changes to another
    // pattern on the 2021
    let unk55f = Cgw55f::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // First two bytes here are unique to my 2019 Kona, unsure if that matters
    let unk55c = Cgw55c::try_from(hex!("071F14FF01000000").as_ref()).unwrap();

    // Another constant-looking message, although is sometimes all zeroes in logs (briefly)
    let unk561 = Cgw561::try_from(hex!("0560000780000F00").as_ref()).unwrap();

    let unk578 = Cgw578::try_from(hex!("000000000000").as_ref()).unwrap();

    let mut steering_counter = 0u8;

    let mut repeater = Repeater::new();

    loop {
        for next in repeater.tick().await {
            match next {
                Period::Hz1 => {
                    pcan_tx.lock(|tx| tx.transmit(&odometer));
                }
                Period::Hz5 => {
                    let cgw5b3 = car.lock(|car| Cgw5b3::latest(car));
                    pcan_tx.lock(|tx| {
                        tx.transmit(&charge_settings);
                        tx.transmit(&igpm_5df);
                        tx.transmit(&body_warnings);
                        tx.transmit(&zeroes45d);
                        tx.transmit(&zeroes45e);
                        tx.transmit(&unk4fe);
                        tx.transmit(&cgw5b3);
                    });
                }
                Period::Hz10 => {
                    let (body_state, clock, charge_port, cgw588) = car.lock(|car| {
                        (
                            BodyState::latest(car),
                            Clock::latest(car),
                            ChargePort::latest(car),
                            Cgw588::latest(car),
                        )
                    });
                    pcan_tx.lock(|tx| {
                        tx.transmit(&body_state);
                        tx.transmit(&charge_port);
                        tx.transmit(&clock);
                        tx.transmit(&cgw588);
                        tx.transmit(&unk55f);
                        tx.transmit(&unk55c);
                        tx.transmit(&unk561);
                        tx.transmit(&unk578);
                    });
                }
                Period::Hz50 => {
                    pcan_tx.lock(|tx| {
                        tx.transmit(&unk450);
                        tx.transmit(&unk462);
                    });
                }
                Period::Hz100 => {
                    let ignition = car.lock(|car| car.ignition());
                    if ignition.ig3_on() {
                        pcan_tx.lock(|tx| tx.transmit(&Steering::latest(&mut steering_counter)))
                    }
                }
                _ => (),
            }
        }
    }
}

pub fn on_can_rx(outer_msg: &Messages) {
    if let Messages::Obc58e(msg) = outer_msg {
        let unlock = msg.port_unlock_req();
        let lock = msg.port_lock_req();
        if lock && unlock {
            defmt::error!("Invalid OBC lock and unlock requested simultaneously");
        } else if lock || unlock {
            let direction = if lock {
                ChargeLock::Locked
            } else {
                ChargeLock::Unlocked
            };
            // Result: Ignoring result because an existing lock/unlock may be in progress
            let _ = app::task_lock_charge_port::spawn(direction);
        }
    }
}

/// Task which is spawned to lock or unlock the charge port.
///
/// Is one-shot, so it runs to completion and prevents another lock or unlock
/// from starting while it is executing.
///
/// This is in the IGPM module as in the original Kona this function is
/// managed by the IGPM, although it makes a bit less sense here in Fakon.
pub async fn task_lock_charge_port(
    cx: app::task_lock_charge_port::Context<'_>,
    direction: ChargeLock,
) {
    let mut car = cx.shared.car;

    defmt::info!("Moving charge port to {}", direction);

    // Charge port actuator is "Kusler 04S" also sold as EV-T2M3S-E-LOCK12V (datasheet online)
    // Datasheet says:
    let drive_time: Duration = 600.millis(); // "Recommended adaptation time 600 ms"
    let pause_time: Duration = 3.secs(); // "Pause time after entry or exit path 3 s"

    let drive = cx.local.charge_lock_drive;
    let dir = cx.local.charge_lock_dir;

    dir.set_state(match direction {
        ChargeLock::Unlocked => PinState::Low,
        ChargeLock::Locked => PinState::High,
    })
    .unwrap();

    drive.set_high().unwrap(); // Start actuator

    Mono::delay(drive_time).await;

    drive.set_low().unwrap(); // Stop actuator

    if car.lock(|car| car.charge_port() != direction) {
        defmt::error!("Charge port lock actuator failed");
    }

    // Pausing here prevents another lock/unlock request
    // starting early
    //
    // 600ms/3s off in a loop will still hammer the actuator a bit if it's
    // failing, but the OBC appears to give up and go into a fault state after
    // about ~30s if it doesn't see the expected result - relying on that to
    // avoid wearing the motor out.
    Mono::delay(pause_time).await;
}

impl BodyState {
    fn latest(car: &CarState) -> Self {
        // BodyState constructor has 43 args, so start from all zeroes and then set some bits!
        let mut result = Self::try_from(hex!("0000000000000000").as_ref()).unwrap();

        // Driver and passenger door closed, drivers seatbelt on - necessary to allow Drive
        result
            .set_drv_door_sw(BodyStateDrvDoorSw::Closed.into())
            .unwrap();
        result
            .set_pass_door_sw(BodyStatePassDoorSw::Closed.into())
            .unwrap();
        result
            .set_drv_seat_belt_sw(BodyStateDrvSeatBeltSw::Present.into())
            .unwrap();

        // Unclear what this signal does or if it needs to be set. TODO: check if needed
        result.set_cf_gway_drive_type_option(true).unwrap();

        if car.ignition() == Ignition::On {
            let ignition_sw = if car.contactor().get() == Some(Contactor::PreCharging) {
                // Note: currently the precharging relay is closed much shorter
                // time than the period of "Starting" shown in the logs.
                //
                // This might be timed by other messages, unsure which.
                BodyStateIgnitionSw::Starting
            } else {
                BodyStateIgnitionSw::On
            };
            result.set_ignition_sw(ignition_sw.into()).unwrap();
            // In the logs it actually looks like IGN1 & 2 go high at slightly
            // different times. Not sure what these actually signify...
            result.set_ign1(true).unwrap();
            result.set_ign2(true).unwrap();
        } else {
            result
                .set_ignition_sw(BodyStateIgnitionSw::Off.into())
                .unwrap();
            result.set_ign1(false).unwrap();
            result.set_ign2(false).unwrap();
        }

        result
    }
}

impl Clock {
    fn latest(car: &CarState) -> Self {
        if !car.most_on().ig3_on() {
            // First arg is "unknown" field as seen when vehicle is off, details are unknown...
            Self::new(0x8D, 0, 0, 0, 0).unwrap()
        } else {
            let total_secs = Mono::now().duration_since_epoch().to_secs();

            let second = total_secs % 60;
            let minute = (total_secs / 60) % 60;
            let hour = (total_secs / 60 / 60) % 24;
            let valid = 1;

            Self::new(0x02, hour as u8, minute as u8, second as u8, valid).unwrap()
        }
    }
}

impl ChargePort {
    fn latest(car: &CarState) -> Self {
        Self::new(car.charge_port().is_locked(), false, false, false).unwrap()
    }
}

impl Steering {
    fn latest(counter: &mut u8) -> Self {
        *counter = match *counter {
            Self::COUNTER_MAX.. => Self::COUNTER_MIN,
            n => n + 1,
        };

        let mut steering = Self::new(9.2, 0, 0x7, *counter, 0).unwrap();

        // 4-bit checksum is XOR of all other nibbles in the message
        {
            let mut checksum = steering.raw().iter().fold(0, |n, a| n ^ a);
            checksum = (checksum >> 4) ^ (checksum & 0x0f);
            steering.set_checksum(checksum).unwrap();
        }

        steering
    }
}

impl Cgw588 {
    fn latest(car: &CarState) -> Self {
        // Cgw588 reflects vehicle status in some way, not exactly
        // clear how
        let (b0, b1, b2) = match car.ignition() {
            car::Ignition::Off => (0x00, 0x00, 0x00),
            car::Ignition::IG3 => (0x58, 0x1c, 0x00),
            car::Ignition::On => (0xfc, 0xff, 0x03),
        };

        Self::new(b0, b1, b2).unwrap()
    }
}

impl Cgw5b3 {
    fn latest(car: &CarState) -> Self {
        let ignition = car.ignition();

        let unk_power_related = if ignition.ig3_on() {
            Cgw5b3UnkPowerRelated::PowerOn
        } else {
            Cgw5b3UnkPowerRelated::PoweredOff
        };

        let unknown2 = if ignition != car::Ignition::Off {
            0x10
        } else {
            // Note: in real car this seems to get set once on start and stay set
            0x00
        };

        let power_state = match (ignition, car.most_on().ig3_on()) {
            (car::Ignition::Off, false) => Cgw5b3PowerState::Off,
            (car::Ignition::Off, true) => Cgw5b3PowerState::GoingToSleep,
            (car::Ignition::IG3, _) => Cgw5b3PowerState::On,
            (car::Ignition::On, _) => Cgw5b3PowerState::On,
        };

        Self::new(unk_power_related.into(), unknown2, 0xFF, power_state.into()).unwrap()
    }
}
