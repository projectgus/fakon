//! SCU - Shift Control Unit
//!
//! In the Kona this is responsible for driving the parking actuator.
//!
//! Sends one CAN message, and has bidirectional PWM link to the EPCU
use crate::car::Gear;
use crate::{app, Rate};
use atomic_enum::atomic_enum;
use defmt::Format;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;
use rtic_sync::portable_atomic::Ordering::Relaxed;
use stm32g4xx_hal::prelude::OutputPin;
use crate::Duration;
use crate::hardware::Mono;
use crate::dbc::pcan::{Scu10c, Scu10cParkingActuator};
use fugit::ExtU32;

const PWM_PERIOD: Duration = Duration::millis(100);

/// Tracking the internal state of the park actuator
#[derive(Format, PartialEq)]
#[atomic_enum]
pub enum ParkActuator {
    ModuleOff,
    Unlocked,
    Moving,
    Locked,
    Fault,
}

static ACTUATOR: AtomicParkActuator = AtomicParkActuator::new(ParkActuator::ModuleOff);

impl From<ParkActuator> for Scu10cParkingActuator {
    fn from(value: ParkActuator) -> Self {
        match value {
            ParkActuator::ModuleOff => Self::Initialising,
            ParkActuator::Unlocked => Self::Unlocked,
            ParkActuator::Moving => Self::Moving,
            ParkActuator::Locked => Self::Locked,
            ParkActuator::Fault => Self::Initialising, // ?
        }
    }
}

impl ParkActuator {
    /// SCU PWM transmit low period for parking state
    fn tx_low_period(&self) -> Duration {
        Duration::millis(match self {
            ParkActuator::ModuleOff => 100, // FIXME: Not sure if should be 100% or or high?
            ParkActuator::Unlocked => 36,
            ParkActuator::Moving => 50, // FIXME: Need to figure this value out
            ParkActuator::Locked => 76, // Maybe 80ms?
            ParkActuator::Fault => 100, // Maybe 40ms?
        })
    }

    /// SCU PWM transmit high period for parking state
    fn tx_high_period(&self) -> Duration {
        PWM_PERIOD - self.tx_low_period()
    }
}

pub async fn task_scu_main(cx: app::task_scu_main::Context<'_>)
{
    let mut car = cx.shared.car;

    loop {
        ACTUATOR.store(ParkActuator::ModuleOff, Relaxed);

        while !car.lock(|car| car.ignition().ig3_on()) {
            // SCU only runs while IG3 is on. Until we have some
            // event trigger for this, poll for it in a loop...
            Mono::delay(20.millis()).await;
        }

        let cur_gear = car.lock(|car| car.gear());

        while car.lock(|car| car.ignition().ig3_on()) {

            let actuator = match cur_gear.get() {
                None => ParkActuator::Locked,
                Some(Gear::Park) => ParkActuator::Locked,
                _ => {
                    defmt::warn!("Started with unlocked park actuator...");
                    ParkActuator::Unlocked
                },
            };

            ACTUATOR.store(actuator, Relaxed);

        }
    }
}

pub async fn task_scu_can(cx: app::task_scu_can::Context<'_>)
{
    let mut car = cx.shared.car;
    let mut pcan_tx = cx.shared.pcan_tx;
    let mut counter = 0u8;

    loop {
        while !car.lock(|car| car.ignition().ig3_on()) {
            // SCU only runs while IG3 is on. Until we have some
            // event trigger for this, poll for it in a loop...
            Mono::delay(20.millis()).await;
        }

        Mono::delay(Rate::Hz(100).into_duration()).await;
        let scu10c = Scu10c::latest(ACTUATOR.load(Relaxed), &mut counter);
        pcan_tx.lock(|tx| tx.transmit(&scu10c));
    }
}

/// Soft PWM task for SCU backup TX pin
pub async fn task_scu_pwm(mut cx: app::task_scu_pwm::Context<'_>)
{
    let mut car = cx.shared.car;
    let scu_park_tx = &mut cx.local.scu_park_tx;

    // TODO: check the level of this signal when vehicle is off
    scu_park_tx.set_low().unwrap();

    loop {
        while !car.lock(|car| car.ignition().ig3_on()) {
            // SCU only runs while IG3 is on. Until we have some
            // event trigger for this, poll for it in a loop...
            Mono::delay(Rate::Hz(10).into_duration()).await;
        }

        let state = ACTUATOR.load(Relaxed);

        scu_park_tx.set_high().unwrap();

        Mono::delay(state.tx_high_period()).await;

        scu_park_tx.set_low().unwrap();

        Mono::delay(state.tx_low_period()).await;
    }
}

impl Scu10c {
    fn latest(state: ParkActuator, counter: &mut u8) -> Self {
        // Another struct with two many args for constructor
        let mut scu = Scu10c::try_from(hex!("0100555415400100").as_slice()).unwrap();

        let msg_val: Scu10cParkingActuator = state.into();
        scu.set_parking_actuator(msg_val.into()).unwrap(); // !

        // Update counter
        {
            *counter = match *counter {
                Self::COUNTER_MAX.. => Self::COUNTER_MIN,
                n => n + 1,
            };
            scu.set_counter(*counter).unwrap();
        }

        // Calculate checksum
        {
            let csum = scu.raw().iter()
                .fold(0u8, |n, e| (n + (e >> 4) + (e & 0xF)) & 0xF);
            scu.set_checksum(csum).unwrap();
        }

        scu
    }
}
