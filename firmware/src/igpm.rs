//! IGPM (Integrated Gateway Power Module) is the CAN Gateway and main power
//! distribution unit in the Kona.
//!
//! Some of these messages may originate from other modules in the car, and be
//! forwarded onto the PCAN bus by the IGPM. Others originate from the IGPM.
use crate::can_queue::{QueuedFrame, Tx};
use crate::car::{self, Contactor, Ignition};
use crate::dbc::pcan::{
    BodyState, BodyStateDrvDoorSw, BodyStateDrvSeatBeltSw, BodyStateIgnitionSw,
    BodyStatePassDoorSw, BodyWarnings, Cgw450, Cgw45d, Cgw45e, Cgw462, Cgw4fe, Cgw55c, Cgw55f,
    Cgw561, Cgw578, Cgw588, Cgw5b3, Cgw5b3PowerState, Cgw5b3UnkPowerRelated, Cgw5df, ChargePort,
    ChargeSettings, ChargeSettingsAcChargingCurrent, Clock, Odometer, Steering,
};
use crate::hardware::{self, Mono};
use crate::periodic::{send_periodic, PeriodicMessage};
use embedded_can::Frame as _;
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

pub async fn task_igpm<MPCAN, MCAR>(mut pcan_tx: MPCAN, mut car: MCAR) -> !
where
    MPCAN: Mutex<T = Tx<hardware::PCAN>>,
    MCAR: Mutex<T = car::CarState>,
{
    let mut charge_settings =
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
    let mut igpm_5df = Cgw5df::try_from(hex!("C5FFFF0100000000").as_ref()).unwrap();

    // Another type whose constructor has too many args
    let mut body_warnings = BodyWarnings::try_from(hex!("0000000000000000").as_ref()).unwrap();
    // These two bits seem to be set during "normal" operation
    body_warnings.set_cf_gway_gway_diag_state(true).unwrap();
    body_warnings.set_cf_gway_sjb_delivery_mode(true).unwrap();

    // Odometer reading
    // Currently using the logged reading from the 2022 car. TODO: check if needs to exceed value set in the 2019 VCU?
    let mut odometer = Odometer::new(14452.5).unwrap();

    // Steering angle sensor
    let mut steering = Steering::new(9.2, 0, 0x7, 0, 0).unwrap();

    // this messages is mostly all zeroes but in some logs it's FFFFFFFFFFFFFF0F instead...?
    let mut zeroes45d = Cgw45d::try_from(hex!("0000000000000000").as_ref()).unwrap();
    // this one is either all zeroes or FFFFFFFFFF000000
    let mut zeroes45e = Cgw45e::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // speed and maybe cruise control buttons?
    let mut unk450 = Cgw450::new(0, 0x1804).unwrap();

    // On 2019 this is mostly constant except for one signal, but even that
    // signal only seems to count sometimes while moving... *shrug*
    let mut unk462 = Cgw462::try_from(hex!("FE3FFF1FF01F0000").as_ref()).unwrap();

    let mut unk4fe = Cgw4fe::try_from(hex!("FFFF7FFFFF00FFFF").as_ref()).unwrap();

    // This message is always zeroes in the 2019 logs, although changes to another
    // pattern on the 2021
    let mut unk55f = Cgw55f::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // First two bytes here are unique to my 2019 Kona, unsure if that matters
    let mut unk55c = Cgw55c::try_from(hex!("071F14FF01000000").as_ref()).unwrap();

    // Another constant-looking message, although is sometimes all zeroes in logs (briefly)
    let mut unk561 = Cgw561::try_from(hex!("0560000780000F00").as_ref()).unwrap();

    let mut unk578 = Cgw578::try_from(hex!("000000000000").as_ref()).unwrap();

    let mut unk588 = Cgw588::new(0x00, 0x00, 0x00).unwrap();

    let mut unk5b3 = Cgw5b3::new(0xF, 0xFF, 0xFF, 0xF).unwrap();

    send_periodic(
        &mut [
            &mut charge_settings,
            &mut body_state,
            &mut clock,
            &mut charge_port,
            &mut igpm_5df,
            &mut body_warnings,
            &mut odometer,
            &mut steering,
            &mut zeroes45d,
            &mut zeroes45e,
            &mut unk450,
            &mut unk561,
            &mut unk462,
            &mut unk4fe,
            &mut unk55f,
            &mut unk55c,
            &mut unk578,
            &mut unk588,
            &mut unk5b3,
        ],
        &mut pcan_tx,
        &mut car,
    )
    .await;
}

impl PeriodicMessage for ChargeSettings {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        // Note: possibly this message should be gated behind ig3_on()?

        // Also, this message is only used by 2019 model Kona, not 2021 model which uses a different message. So might need to not send on later models...

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for BodyState {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, car: &car::CarState) -> Option<QueuedFrame> {
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
            self.set_ignition_sw(ignition_sw.into()).unwrap();
            // In the logs it actually looks like IGN1 & 2 go high at slightly
            // different times. Not sure what these actually signify...
            self.set_ign1(true).unwrap();
            self.set_ign2(true).unwrap();
        } else {
            self.set_ignition_sw(BodyStateIgnitionSw::Off.into())
                .unwrap();
            self.set_ign1(false).unwrap();
            self.set_ign2(false).unwrap();
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Clock {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        // Note: technically the clock normally goes "valid" when ignition comes on,
        // placeholder-looking values are sent until then. Hoping can ignore this fact.
        let secs = Mono::now().duration_since_epoch().to_secs();
        self.set_unknown(0x02).unwrap();
        self.set_second((secs % 60) as u8).unwrap(); // Seconds
        self.set_minute(((secs / 60) % 60) as u8).unwrap(); // Minutes
        self.set_hour(((secs / 60 / 60) % 24) as u8).unwrap(); // Hours
        self.set_valid(1).unwrap(); // "Valid" flag

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for ChargePort {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, car: &car::CarState) -> Option<QueuedFrame> {
        self.set_charge_port_locked(car.charge_port_locked())
            .unwrap();
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw5df {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for BodyWarnings {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Odometer {
    fn rate(&self) -> crate::Rate {
        1.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Steering {
    fn rate(&self) -> crate::Rate {
        100.Hz()
    }

    fn update_for_transmit(&mut self, car: &car::CarState) -> Option<QueuedFrame> {
        if !car.ignition().ig3_on() {
            return None;
        }

        // Counter
        {
            self.set_counter(match self.counter() {
                Self::COUNTER_MAX.. => Self::COUNTER_MIN,
                n => n + 1,
            })
            .unwrap();
        }

        // 4-bit checksum is XOR of all other nibbles in the message
        {
            self.set_checksum(0).unwrap();
            let mut checksum = self.raw().iter().fold(0, |n, a| n ^ a);
            checksum = (checksum >> 4) ^ (checksum & 0x0f);
            self.set_checksum(checksum).unwrap();
        }

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw45d {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw45e {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw450 {
    fn rate(&self) -> crate::Rate {
        50.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw462 {
    fn rate(&self) -> crate::Rate {
        50.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw4fe {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw55f {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw55c {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw561 {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw578 {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, _car: &car::CarState) -> Option<QueuedFrame> {
        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw588 {
    fn rate(&self) -> crate::Rate {
        10.Hz()
    }

    fn update_for_transmit(&mut self, car: &car::CarState) -> Option<QueuedFrame> {
        // Unk588 reflects vehicle status in some way, not exactly
        // clear how
        let (b0, b1, b2) = match car.ignition() {
            car::Ignition::Off => (0x00, 0x00, 0x00),
            car::Ignition::IG3 => (0x58, 0x1c, 0x00),
            car::Ignition::On => (0xfc, 0xff, 0x03),
        };
        self.set_unk0(b0).unwrap();
        self.set_unk1(b1).unwrap();
        self.set_unk2(b2).unwrap();

        QueuedFrame::new(self.id(), self.raw())
    }
}

impl PeriodicMessage for Cgw5b3 {
    fn rate(&self) -> crate::Rate {
        5.Hz()
    }

    fn update_for_transmit(&mut self, car: &car::CarState) -> Option<QueuedFrame> {
        let ignition = car.ignition();

        self.set_unk_power_related(
            if ignition.ig3_on() {
                Cgw5b3UnkPowerRelated::PowerOn
            } else {
                Cgw5b3UnkPowerRelated::PoweredOff
            }
            .into(),
        )
        .unwrap();

        if ignition != car::Ignition::Off {
            // This seems to get once on start and stay set
            self.set_unknown2(0x10).unwrap();
        }

        self.set_power_state(
            match (ignition, self.power_state()) {
                (car::Ignition::Off, Cgw5b3PowerState::Off) => Cgw5b3PowerState::Off,
                (car::Ignition::Off, _) => Cgw5b3PowerState::GoingToSleep,
                (car::Ignition::IG3, _) => Cgw5b3PowerState::On,
                (car::Ignition::On, _) => Cgw5b3PowerState::On,
            }
            .into(),
        )
        .unwrap();

        QueuedFrame::new(self.id(), self.raw())
    }
}
