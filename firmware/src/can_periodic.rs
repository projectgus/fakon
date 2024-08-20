use crate::{can_queue, hardware::Mono};
use fugit::{Instant, Rate};
use rtic_monotonics::Monotonic;

// Group of messages to be sent at a particular periodic rate
pub struct Period {
    next: Instant<u32, 1, 1000>, // The next time this group's messages are due to be sent
    //messages: &'a [Message], // Reference to the messages in this period group
    //messages: heapless::Vec<Box<Message>, 16>,
}

// pub trait Message<DLEN> {
//     fn period(&self) -> Rate;
//     fn id(&self) -> Id;
//     fn update(&mut self) -> &[u8];
// }

pub struct Message
{
    frame: can_queue::QueuedFrame,
    update_fn: dyn FnMut() -> (),
}

pub fn byte_checksum_simple(data: &[u8]) -> u8 {
    data.into_iter().fold(0, |s,n| n.wrapping_add(s))
}

pub async fn task_periodic_callback(rate: Rate<u32, 1000, 1>, mut callback: impl FnMut() -> ()) -> !
{
    let period = rate.into_duration();
    let mut next = Mono::now();
    loop {
        // Periodic delay
        next = match next.checked_add_duration(period) {
            Some(next) => {
                Mono::delay_until(next).await;
                next
            },
            None => {
                // Allow for some clock skew when the timer overflows
                Mono::delay(period).await;
                Mono::now()
            }
        };

        callback()
    }
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
//
// Currently the skipped value is the same as the masked value (i.e.
// all bits set).
pub fn counter_update_skip<const MASK: u8, const SKIP: u8>(data: &mut u8) {
    counter_update::<MASK>(data);
    if *data & MASK == SKIP {
        counter_update::<MASK>(data);
    }
}
