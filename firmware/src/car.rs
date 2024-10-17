use crate::dbc::pcan::Messages;
use crate::fresh::Fresh;
use defmt::Format;
use fugit::ExtU32;

#[derive(Clone, Format)]
pub struct CarState {
    /// Main ignition power state. Updated from hard wired inputs.
    ignition: Ignition,

    /// Main high voltage contactor state. Updated from BMS whenever IG1 or IG3 is on.
    contactor: Fresh<Contactor>,

    charge_port_locked: bool,
    is_braking: bool,

    v_batt: f32,
    i_batt: f32,
    v_inverter: u16,
    motor_rpm: f32,

    // Internal state of pre-charge relay. Used to update 'contactor' field. Updated from BMS.
    last_precharge: Fresh<bool>,
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

impl CarState {
    pub fn new() -> Self {
        // Note this is where all of the stale timeouts for the Fresh values are set
        Self {
            ignition: Ignition::Off,
            contactor: Fresh::new(3.secs()),
            charge_port_locked: false,
            is_braking: false,

            v_batt: 0.0,
            i_batt: 0.0,
            v_inverter: 0,
            motor_rpm: 0.0,

            last_precharge: Fresh::new(3.secs()),
        }
    }

    #[inline]
    pub fn ignition(&self) -> Ignition {
        self.ignition
    }

    #[inline]
    /// Return the contactor state.
    pub fn contactor(&self) -> Fresh<Contactor> {
        self.contactor
    }

    /// Return true if the vehicle is Ready to drive
    ///
    /// WARNING: This will return false if CAN comms are lost with the BMS
    #[inline]
    pub fn ready(&self) -> bool {
        // TODO: there probably is a VCU CAN message that gives a clearer indication of this state
        self.contactor.get() == Some(Contactor::Closed) && self.ignition == Ignition::On
    }

    pub fn set_ignition(&mut self, value: Ignition) {
        if value != self.ignition {
            defmt::info!("Ignition => {}", value);
            self.ignition = value;
        }
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
    pub fn charge_port_locked(&self) -> bool {
        self.charge_port_locked
    }

    #[inline]
    pub fn set_charge_port_locked(&mut self, value: bool) {
        if value != self.charge_port_locked {
            defmt::info!("Charge Port Locked => {}", value);
            self.charge_port_locked = value;
        }
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
                    defmt::warn!("BMS is sending contactor state without precharge state");
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
            Messages::BmsHvMonitor(msg) => {
                // Pre-charge relay state
                {
                    // Don't update the contactor state here, wait for the next Bms5a3 message
                    // and switch it there (this is to avoid races when switching in and out of
                    // pre-charge
                    self.last_precharge.set(msg.precharging());
                }

                // Raw battery stats
                {
                    // TODO: Log these periodically, maybe?
                    self.v_batt = msg.v_batt();
                    self.i_batt = msg.i_batt();
                }
            }
            Messages::InverterStatus(msg) => {
                self.v_inverter = msg.v_inverter();
                self.motor_rpm = msg.speed_abs();
            }
            _ => (),
        }
    }
}

impl Default for CarState {
    fn default() -> Self {
        Self::new()
    }
}
