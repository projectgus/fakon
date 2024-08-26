// Some utility methods for CAN message contents

// The simplest "checksum" of summing all bytes modulo 256
pub fn byte_checksum_simple(data: &[u8]) -> u8 {
    data.into_iter().fold(0, |s,n| n.wrapping_add(s))
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
