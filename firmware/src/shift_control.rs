//! SCU - Shift Control Unit
//!
//! In the Kona this is responsible for driving the parking actuator.
//!
//! VCU sends requests for Lock/Unlock to the SCU, which then moves the actuator.
//! SCU sends the current position of the actuator to the VCU.
//!
//! Two redundant communication links for this:
//! - CAN messages (SCU sends Scu10c message and listens for Vcu109 message, both at 100Hz).
//! - 10Hz PWM RX/TX connection with different duty cycles corresponding to the same data.
//!
//! We emulate both links, but don't emulate a real actuator: the emulated SCU
//! immediately updates the parking actuator state to whatever the VCU most
//! recently asked for.
use crate::dbc::pcan::{Messages, Scu10c, Scu10cParkingActuator, Vcu109ParkActuatorRequest};
use crate::hardware::Mono;
use crate::Duration;
use crate::{app, Instant, Rate};
use defmt::Format;
use fugit::ExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;
use stm32g4xx_hal::prelude::{InputPin, OutputPin};
use stm32g4xx_hal::gpio::ExtiPin;

const PWM_PERIOD: Duration = Duration::millis(100);

/// Struct to track the overall park actuator state
///
/// (This struct bundles two pieces of mostly unrelated state together: the current
/// position and the PWM RX timestamps, in order to only keep the top-level Shared structure
/// simple.)
#[derive(Copy, Clone, Format)]
pub struct ActuatorState {
    /// Current position of the emulated park actuator
    position: ActuatorPosition,

    /// Last PWM RX rising edge
    rising: Option<Instant>,

    /// Last PWM RX falling edge
    falling: Option<Instant>,
}

/// Tracking the emulated position of the park actuator
#[derive(Clone, Copy, Format, PartialEq)]
enum ActuatorPosition {
    Unknown,
    Unlocked,
    //Moving, // Not emulated
    Locked,
}

impl From<ActuatorPosition> for Scu10cParkingActuator {
    fn from(value: ActuatorPosition) -> Self {
        match value {
            ActuatorPosition::Unknown => Self::Unknown,
            ActuatorPosition::Unlocked => Self::Unlocked,
            //ActuatorPosition::Moving => Self::Moving,
            ActuatorPosition::Locked => Self::Locked,
        }
    }
}

impl ActuatorPosition {
    /// SCU PWM TX duty cycle (high period) for actuator position
    fn pwm_tx_duty_percent(&self) -> u32 {
        match self {
            ActuatorPosition::Unknown => 84,
            ActuatorPosition::Unlocked => 64,
            //ActuatorPosition::Moving => 44,
            ActuatorPosition::Locked => 24,
        }
    }

    /// Decode SCU PWM RX duty cycle (high period) for a possible
    /// actuator position request.
    fn from_pwm_rx_duty_percent(pct: u32) -> Option<Self> {
        const IDLE_PCT: i32 = 55;
        const REQ_UNLOCK_PCT: i32 = 25;
        const REQ_LOCK_PCT: i32 = 85;
        const TOLERANCE_PCT: i32 = 5; // +/- tolerance
        let pct = pct as i32;

        if (pct - REQ_UNLOCK_PCT).abs() < TOLERANCE_PCT {
            return Some(Self::Unlocked);
        }
        if (pct - REQ_LOCK_PCT).abs() < TOLERANCE_PCT {
            return Some(Self::Locked);
        }
        if (pct - IDLE_PCT).abs() > TOLERANCE_PCT {
            defmt::warn!("SCU PWM RX unexpected duty {}%", pct);
        }
        None
    }
}

/// Soft PWM task for SCU backup TX pin
pub async fn task_scu_pwm_tx(mut cx: app::task_scu_pwm_tx::Context<'_>) {
    let mut car = cx.shared.car;
    let mut actuator = cx.shared.park_actuator;
    let scu_park_tx = &mut cx.local.scu_park_tx;

    // TODO: check the level of this signal when vehicle is off
    scu_park_tx.set_low().unwrap();

    while !car.lock(|car| car.ignition().ig3_on()) {
        // SCU only runs after IG3 is on. Until we have some
        // event trigger for this, poll for it in a loop...
        Mono::delay(Rate::Hz(10).into_duration()).await;
    }

    loop {
        let position = actuator.lock(|actuator| actuator.position);
        let pwm_duty_pct = position.pwm_tx_duty_percent();
        let high_time = PWM_PERIOD * pwm_duty_pct / 100;
        let low_time = PWM_PERIOD - high_time;

        scu_park_tx.set_low().unwrap();

        Mono::delay(low_time).await;

        scu_park_tx.set_high().unwrap();

        Mono::delay(high_time).await;
    }
}

impl Default for ActuatorState {
    fn default() -> Self {
        Self {
            position: ActuatorPosition::Unknown,
            rising: None,
            falling: None,
        }
    }
}

impl ActuatorState {
    /// If this new edge timestamp indicates a PWM actuator request
    /// then return the new position that's being requested.
    fn is_pwm_request(&self, rising: bool, ts: Instant) -> Option<ActuatorPosition> {
        // When the duty changes it seems to change starting from a falling edge,
        // so measure each cycle falling edge to falling edge.
        if rising {
            return None;
        }

        if let (Some(falling), Some(rising)) = (self.falling, self.rising) {
            let cycle_time = ts - falling; // Time since first falling edge
            let tolerance = PWM_PERIOD / 10;

            if PWM_PERIOD - tolerance < cycle_time && cycle_time < PWM_PERIOD + tolerance {
                let high_time = ts - rising;
                let duty_pct = (100 * high_time) / cycle_time;
                defmt::trace!("SCU PWM RX duty {}% (high time {} cycle time {}", duty_pct, high_time, cycle_time);
                return ActuatorPosition::from_pwm_rx_duty_percent(duty_pct);
            } else {
                // This happens as a one-off in normal operation when VCU powers
                // on, and sometimes shortly after that
                defmt::debug!("SCU PWM RX unexpected cycle time {}", ts - falling);
            }
        }
        Some(ActuatorPosition::Unknown)
    }

    /// Update the PWM state with this latest edge
    fn update_pwm_edge(&mut self, rising: bool, now: Instant) {
        if rising {
            self.rising = Some(now);
        } else {
            self.falling = Some(now);
        }
    }
}

// Pin interrupt for edge transitions of SCU RX PWM signal
pub fn task_scu_pwm_rx(mut cx: app::task_scu_pwm_rx::Context) {
    let now = Mono::now();
    let rising = cx.local.scu_park_rx.is_high().unwrap();
    let car = &mut cx.shared.car;
    let park_actuator = &mut cx.shared.park_actuator;

    if !car.lock(|car| car.ignition().ig3_on()) {
        // Vehicle is off, so reset the emulated actuator state and ignore VCU PWM edge
        // transitions until it comes back on
        park_actuator.lock(|state| *state = ActuatorState::default());
    } else {
        park_actuator.lock(|state| {
            // Update the position if it changed
            if let Some(new_pos) = state.is_pwm_request(rising, now) {
                if new_pos != state.position {
                    defmt::info!("Emulating Parking Actuator => {} (PWM)", new_pos);
                    state.position = new_pos;
                }
            }

            // Update latest rising or falling edge
            state.update_pwm_edge(rising, now);
        });
    }

    cx.local.scu_park_rx.clear_interrupt_pending_bit();
}

/// Sender task for SCU message
pub async fn task_scu_can_tx(cx: app::task_scu_can_tx::Context<'_>) {
    let mut car = cx.shared.car;
    let mut actuator = cx.shared.park_actuator;
    let mut pcan_tx = cx.shared.pcan_tx;
    let mut counter = 0u8;

    while !car.lock(|car| car.ignition().ig3_on()) {
        // SCU doesn't start until IG3 is on. Until we have some
        // event trigger for this, poll for it in a loop...
        Mono::delay(20.millis()).await;
    }

    loop {
        let scu10c = actuator.lock(|actuator| Scu10c::latest(actuator.position, &mut counter));
        pcan_tx.lock(|tx| tx.transmit(&scu10c));

        Mono::delay(Rate::Hz(100).into_duration()).await;
    }
}

/// Handler function for received CAN messages.
///
/// Extracts any park actuator request and updates the emulated position.
pub fn on_can_rx<MPARK>(msg: &Messages, state: &mut MPARK)
where
    MPARK: Mutex<T = ActuatorState>,
{
    if let Messages::Vcu109(msg) = msg {
        let maybe_new_pos = match msg.park_actuator_request() {
            Vcu109ParkActuatorRequest::RequestLock => Some(ActuatorPosition::Locked),
            Vcu109ParkActuatorRequest::RequestUnlock => Some(ActuatorPosition::Unlocked),
            _ => None,
        };

        if let Some(new_pos) = maybe_new_pos {
            state.lock(|state| {
                if state.position != new_pos {
                    defmt::info!("Emulating Parking Actuator => {} (CAN)", new_pos);
                    state.position = new_pos;
                }
            });
        }
    }
}

impl Scu10c {
    fn latest(state: ActuatorPosition, counter: &mut u8) -> Self {
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
            let csum = scu
                .raw()
                .iter()
                .fold(0u8, |n, e| (n + (e >> 4) + (e & 0xF)) & 0xF);
            scu.set_checksum(csum).unwrap();
        }

        scu
    }
}
