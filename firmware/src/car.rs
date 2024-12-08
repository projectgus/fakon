//! Common state of the entire "car" as presented to the Kona
//! components.
use crate::dbc::pcan::{BattHvStatusPrechargeRelay, Messages, Vcu200CurrentGear};
use crate::fresh::{Fresh, IsFresh};
use crate::hardware::Mono;
use crate::Instant;
use defmt::Format;
use rtic_monotonics::Monotonic;

#[derive(Clone, Format)]
pub struct CarState {
    /// Main ignition power state. Updated from hard wired inputs.
    ignition: Ignition,

    /// The "most on" that the car has been since reset
    most_on: Ignition,

    /// Main high voltage contactor state. Updated from BMS whenever IG1 or IG3 is on.
    contactor: Fresh<Contactor, 3>,

    /// Debounced and de-inverted level of EV Ready input
    ev_ready_input: bool,

    charge_port: ChargeLock,
    is_braking: bool,

    gear: Fresh<Gear, 3>,

    soc_batt: f32,
    v_batt: f32,
    i_batt: f32,
    v_inverter: Fresh<u16, 3>,
    motor_rpm: Fresh<u16, 1>,

    // Internal state of pre-charge relay. Used to update 'contactor' field. Updated from BMS.
    last_precharge: Fresh<bool, 3>,

    /// Connected EVSE detected by OBC. Doubles as tracker for OBC powered on
    evse_detected: Fresh<bool, 3>,

    /// Timestamp of last time a valid CAN message was received via PCAN
    last_pcan_rx: Option<Instant>,

    /// Once CAN bus goes Bus Off, can't really recover
    pcan_bus_off: bool,
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum Ignition {
    /// Car is off, and will probably transition to sleep soon
    Off,
    /// Car is partially on for charging (IG3 relay), but not to drive
    IG3,
    /// Car is fully on with key in ignition (both IG1 and IG3 relays)
    On,
}

/// Map the state of the charge port lock actuator
#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum ChargeLock {
    Unlocked,
    Locked,
}

impl Ignition {
    /// Return true if IG3 is on (with or without IG1)
    pub fn ig3_on(&self) -> bool {
        !matches!(self, Ignition::Off)
    }
}

/// High Voltage Contactor state
#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum Contactor {
    /// Contactors are open (i.e. high voltage isolated)
    Open,
    /// Contactors are pre-charging the inverter
    PreCharging,
    /// Contactors are fully closed and HV is fully on
    Closed,
}

#[derive(Clone, Copy, Format, PartialEq)]
pub enum Gear {
    Park,
    Neutral,
    Drive,
    Reverse,
}

impl TryFrom<Vcu200CurrentGear> for Gear {
    type Error = ();

    fn try_from(value: Vcu200CurrentGear) -> Result<Self, Self::Error> {
        match value {
            Vcu200CurrentGear::P => Ok(Self::Park),
            Vcu200CurrentGear::D => Ok(Self::Drive),
            Vcu200CurrentGear::N => Ok(Self::Neutral),
            Vcu200CurrentGear::R => Ok(Self::Reverse),
            Vcu200CurrentGear::_Other(_) => Err(()),
        }
    }
}

impl CarState {
    pub fn new() -> Self {
        // Note this is where all of the stale timeouts for the Fresh values are set
        Self {
            ignition: Ignition::Off,
            most_on: Ignition::On,
            contactor: Fresh::new(),
            ev_ready_input: false,
            charge_port: ChargeLock::Unlocked,
            is_braking: false,

            gear: Fresh::new(),

            soc_batt: 0.0,
            v_batt: 0.0,
            i_batt: 0.0,
            v_inverter: Fresh::new(),
            motor_rpm: Fresh::new(),

            last_precharge: Fresh::new(),
            evse_detected: Fresh::new(),
            last_pcan_rx: None,
            pcan_bus_off: false,
        }
    }

    #[inline]
    pub fn ignition(&self) -> Ignition {
        self.ignition
    }

    #[inline]
    /// Return the contactor state.
    pub fn contactor(&self) -> impl IsFresh<Contactor> {
        self.contactor
    }

    #[inline]
    pub fn v_inverter(&self) -> impl IsFresh<u16> {
        self.v_inverter
    }

    #[inline]
    pub fn motor_rpm(&self) -> impl IsFresh<u16> {
        self.motor_rpm
    }

    #[inline]
    pub fn soc_batt(&self) -> f32 {
        self.soc_batt
    }

    #[inline]
    pub fn gear(&self) -> impl IsFresh<Gear> {
        self.gear
    }

    #[inline]
    pub fn evse_detected(&self) -> impl IsFresh<bool> {
        self.evse_detected
    }

    pub fn set_ignition(&mut self, value: Ignition) {
        if value != self.ignition {
            defmt::info!("Ignition => {}", value);
            self.ignition = value;
        }
        // Update the lifetime "most on" value
        self.most_on = match (self.most_on, value) {
            (Ignition::Off, Ignition::IG3) => Ignition::IG3,
            (_, Ignition::On) => Ignition::On,
            (existing, _) => existing,
        };
    }

    #[inline]
    pub fn is_braking(&self) -> bool {
        self.is_braking
    }

    #[inline]
    pub fn set_is_braking(&mut self, value: bool) {
        if value != self.is_braking {
            defmt::info!("Braking => {}", value);
            self.is_braking = value;
        }
    }

    #[inline]
    pub fn ev_ready(&self) -> bool {
        // As this is an active low input, it's also low when VCU is off
        self.ev_ready_input && self.v_inverter.is_fresh()
    }

    #[inline]
    pub fn set_ev_ready_input(&mut self, value: bool) {
        if value != self.ev_ready_input {
            defmt::info!("EV Ready => {}", value);
            self.ev_ready_input = value;
        }
    }

    #[inline]
    pub fn charge_port(&self) -> ChargeLock {
        self.charge_port
    }

    #[inline]
    pub fn set_charge_port(&mut self, value: ChargeLock) {
        if value != self.charge_port {
            defmt::info!("Charge Port => {}", value);
            self.charge_port = value;
        }
    }

    // Return the "most on" that the ignition has been since reset
    #[inline]
    pub(crate) fn most_on(&self) -> Ignition {
        self.most_on
    }

    pub fn set_evse_detected(&mut self, value: bool) {
        self.evse_detected.set(value);
    }

    /// Update the timestamp of the last "IG3 on" message received from VCU
    pub fn set_last_pcan_rx(&mut self) {
        self.last_pcan_rx = Some(Mono::now());
    }

    /// Until we have IG3 direct monitoring, use "OBC is sending messages" as a proxy
    pub fn ig3_appears_powered(&self) -> bool {
        // evse_detected is for OBC
        self.evse_detected.is_fresh()
    }

    pub fn pcan_receiving(&self) -> bool {
        self.last_pcan_rx
            .is_some_and(|val| (Mono::now() - val).to_secs() < 2)
    }

    /// Set the flag that PCAN Bus has gone Bus Off
    pub fn set_pcan_bus_off(&mut self) {
        self.pcan_bus_off = true;
    }

    pub fn pcan_bus_off(&self) -> bool {
        self.pcan_bus_off
    }

    // Contactor state only updates in response to BMS CAN messages
    fn set_contactor(&mut self, new_state: Contactor) {
        if self.contactor.get() != Some(new_state) {
            // Log contactor transitions, including warnings for unexpected
            // ones
            if matches!(
                (self.contactor.get_unchecked(), new_state),
                (None, Contactor::Closed) // Firmware reset w/ contactors closed
                    | (None, Contactor::PreCharging) // Firmware reset in precharge state
                    | (Some(Contactor::Open), Contactor::Closed) // Skipped precharge
                    | (Some(Contactor::PreCharging), Contactor::Open) // Precharge failed
                    | (Some(Contactor::Closed), Contactor::PreCharging) // ???
            ) {
                defmt::warn!(
                    "Unexpected main contactors {:?} => {:?} inverter {:?} V",
                    self.contactor,
                    new_state,
                    self.v_inverter
                );
            } else {
                defmt::info!(
                    "Main contactors {:?} => {:?} inverter {:?} V",
                    self.contactor,
                    new_state,
                    self.v_inverter
                );
            }
        }
        // Always call set here to mark freshness of the value
        self.contactor.set(new_state);
    }

    pub fn update_state(&mut self, pcan_msg: &Messages) {
        match pcan_msg {
            Messages::Bms5a3(msg) => {
                if self.contactor.is_fresh() && self.last_precharge.is_stale() {
                    // We've received more than one of this message recently, but none of
                    // the higher priority BmsHvMonitor message with the precharge state
                    defmt::error!("BMS is sending contactor state without precharge state");
                } else {
                    let contactor_closed = msg.contactor_closed();
                    let precharging = self.last_precharge.get().unwrap_or(false);
                    self.set_contactor(match (contactor_closed, precharging) {
                        (true, _) => Contactor::Closed,
                        (false, true) => Contactor::PreCharging,
                        (false, false) => Contactor::Open,
                    });
                }
            }
            Messages::BattHvStatus(msg) => {
                // Pre-charge relay state
                {
                    // Don't update the contactor state here, wait for the next Bms5a3 message
                    // and switch it there (this is to avoid races when switching in and out of
                    // pre-charge
                    self.last_precharge
                        .set(msg.precharge_relay() == BattHvStatusPrechargeRelay::Closed);
                }

                // Raw battery stats
                {
                    self.v_batt = msg.v_batt();
                    self.i_batt = msg.i_batt();
                }
            }
            Messages::Bms542(msg) => {
                // Recording the "display SoC" not the "real SoC" i.e. as shown on dash
                self.soc_batt = msg.soc_disp()
            }
            Messages::Obc58e(msg) => {
                self.set_evse_detected(msg.evse_detected());
            }
            Messages::InverterStatus(msg) => {
                // as these two have the same "freshness" they could conceivably be merged somehow
                self.v_inverter.set(msg.v_inverter());
                self.motor_rpm.set(msg.speed_abs() as u16);
            }
            Messages::Vcu200(msg) => {
                if let Ok(gear) = msg.current_gear().try_into() {
                    self.gear.set(gear);
                } else if self.ignition().ig3_on() {
                    defmt::warn!(
                        "VCU200 message sent invalid gear value {}",
                        msg.current_gear_raw()
                    );
                }
            }
            _ => (),
        }
        self.set_last_pcan_rx();
    }
}

impl Default for CarState {
    fn default() -> Self {
        Self::new()
    }
}

impl ChargeLock {
    pub fn is_locked(&self) -> bool {
        return *self == Self::Locked;
    }
}
