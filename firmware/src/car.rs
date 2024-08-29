

#[derive(Clone, Debug)]
pub struct CarState {
    ignition_on: bool,
    charge_port_locked: bool,
    is_braking: bool,
}

impl CarState {
    pub fn new() -> Self {
        Self {
            charge_port_locked: false,
            ignition_on: false,
            is_braking: false,
        }
    }

    #[inline]
    pub fn ignition_on(&self) -> bool {
        self.ignition_on
    }

    #[inline]
    pub fn set_ignition_on(&mut self, value: bool) {
        if value != self.ignition_on {
            defmt::info!("Ignition => {}", value);
            self.ignition_on = value;
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
}
