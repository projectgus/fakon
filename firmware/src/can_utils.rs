// Some utility methods for CAN message contents

use crate::dbc::pcan;

// The simplest "checksum" of summing all bytes modulo 256
pub fn byte_checksum_simple(data: &[u8]) -> u8 {
    data.iter().fold(0, |s,n| n.wrapping_add(s))
}

// Update a byte counter field, masked against MASK if necessary.
// if MASK is upper bits then the value is shifted accordingly.
pub fn counter_update<const MASK: u8>(data: &mut u8) {
    let shift = MASK.trailing_zeros();
    let counter = (*data & MASK).wrapping_add(1 << shift);
    let rest = *data & !MASK;
    *data = counter | rest;
}

// Update a byte counter field, skipping a particular value each time.
pub fn counter_update_skip<const MASK: u8, const SKIP: u8>(data: &mut u8) {
    counter_update::<MASK>(data);
    if *data & MASK == SKIP {
        counter_update::<MASK>(data);
    }
}

/// Trait representing some common operations on a CAN message
pub(crate) trait OutgoingMessage {
    /// Increment the counter field(s) in the message
    fn increment_counter(&mut self);

    /// Update any checksum field(s) according to the contents of the message
    fn update_checksum(&mut self);

    /// Perform all regular "liveness" updates (counters, checksums)
    #[inline]
    fn update(&mut self) {
        self.increment_counter();
        self.update_checksum();
    }
}

// Additional methods for StabilityControl message ID 0x220
impl OutgoingMessage for pcan::StabilityControl {
    #[inline]
    fn increment_counter(&mut self) {
        self.set_counter(match self.counter() {
            Self::COUNTER_MAX.. => Self::COUNTER_MIN,
            n => n + 1,
        }).unwrap();
    }

    #[inline]
    fn update_checksum(&mut self) {
        self.set_checksum(0).unwrap();
        let new_sum = self.raw().iter().fold(0u8, |n, e| n.wrapping_add(*e));
        self.set_checksum((new_sum ^ 0x9) & 0xF).unwrap();
    }
}

// Additional methods for TractionControlFast message ID 0x153
impl OutgoingMessage for pcan::TractionControlFast {
    #[inline]
    fn increment_counter(&mut self) {
                self
                    .set_counter1(match self.counter1() {
                        Self::COUNTER1_MAX.. => Self::COUNTER1_MIN,
                        c => c + 1,
                    }).unwrap();

                self
                    .set_counter2(match self.counter2() {
                        Self::COUNTER2_MAX.. => Self::COUNTER2_MIN,
                        0x8 => 0xA,  // counter2 skips 0x9 rather than 0xF
                        c => c + 1,
                    }).unwrap();
    }

    #[inline]
    fn update_checksum(&mut self) {
        // No checksum on this message
    }
}

// Additional methods for TractionControlMed message ID 0x394
impl OutgoingMessage for pcan::TractionControlMed {
    fn increment_counter(&mut self) {
        self.set_counter(match self.counter() {
            Self::COUNTER_MAX.. => Self::COUNTER_MIN,
            n => n + 1,
        }).unwrap();
    }

    fn update_checksum(&mut self) {
        self.set_checksum(0).unwrap();

        // Checksum is the sum of the nibbles of all bytes except the last
        let new_sum = self.raw()[..7].iter().fold(0u8, |n, e| {
            (n + (e >> 4) + (e & 0xF)) & 0xF
        });

        // Then take binary complement, add one, and mask modulo 16
        self.set_checksum(((new_sum ^ 0x0F) + 1) & 0x0F).unwrap();
    }
}

impl OutgoingMessage for pcan::Unk471 {
    fn increment_counter(&mut self) {
        self.set_counter(match self.counter() {
            Self::COUNTER_MAX.. => Self::COUNTER_MIN,
            n => n + 1,
        }).unwrap();
    }

    fn update_checksum(&mut self) {
        // It's possible this message has a 2-bit checksum, but unclear
    }
}
