use defmt::Format;

use crate::dbc::pcan::Messages;

#[derive(Clone, Debug, Format)]
pub struct CarState {
    /// Main ignition power state
    ignition: Ignition,
    /// Main high voltage contactor state
    contactor: Contactor,
    charge_port_locked: bool,
    is_braking: bool,

    v_batt: f32,
    i_batt: f32,
    v_inverter: u16,
    motor_rpm: f32,

    // Internal state tracking
    last_precharge: bool,
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum Ignition {
    /// Car  is off, and will probably transition to sleep soon
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
        Self {
            ignition: Ignition::Off,
            contactor: Contactor::Open,
            charge_port_locked: false,
            is_braking: false,

            v_batt: 0.0,
            i_batt: 0.0,
            v_inverter: 0,
            motor_rpm: 0.0,

            last_precharge: false,
        }
    }

    #[inline]
    pub fn ignition(&self) -> Ignition {
        self.ignition
    }

    /// Return true if the vehicle is Ready to drive
    #[inline]
    pub fn ready(&self) -> bool {
        // TODO: there probably is a VCU CAN message that gives a clearer indication of this state
        self.contactor == Contactor::Closed && self.ignition == Ignition::On
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

    // Contactor state only updates in response to CAN messages
    fn set_contactor(&mut self, new_state: Contactor) {
        if self.contactor != new_state {
            // Log contactor transitions, including warnings for unexpected
            // ones
            if matches!(
                (self.contactor, new_state),
                (Contactor::Open, Contactor::Closed) // Skipped precharge
                    | (Contactor::PreCharging, Contactor::Open) // Precharge failed
                    | (Contactor::Closed, Contactor::PreCharging) // ???
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
            self.contactor = new_state;
        }
    }

    pub fn update_state(&mut self, pcan_msg: &Messages) {
        match pcan_msg {
            Messages::Bms5a3(msg) => {
                self.set_contactor(match (msg.contactor_closed(), self.last_precharge) {
                    (true, _) => Contactor::Closed,
                    (false, true) => Contactor::PreCharging,
                    (false, false) => Contactor::Open,
                });
            }
            Messages::BmsHvMonitor(msg) => {
                // Pre-charge relay state
                {
                    // Don't update the contactor state here, wait for the next Bms5a3 message
                    // and switch it there (this is to avoid races when switching in and out of
                    // pre-charge
                    self.last_precharge = msg.precharging();
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
