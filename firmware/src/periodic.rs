use crate::can_queue::{self, QueuedFrame};
use crate::car::CarState;
use crate::hardware::{self, Mono};
use crate::{Duration, Instant, Rate};
use fugit::ExtU32;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

/// Trait for a message which needs to be sent regularly via CAN.
///
/// Designed so you can pass a slice of these to the send_periodic()
/// async function and have them sent onto the bus.
pub trait PeriodicMessage {
    /// Sending rate for the message. Should return same value each time.
    fn rate(&self) -> Rate;

    /// Update the message and optionally return a QueuedFrame to transmit
    fn update_for_transmit(&mut self, car: &CarState) -> Option<QueuedFrame>;

    /// Default conversion from the rate to the period of sending in milliseconds
    fn period_ms(&self) -> u32 {
        let d: Duration = self.rate().into_duration();
        return d.to_millis();
    }
}

pub(crate) async fn send_periodic<MPCAN, MCAR>(
    msgs: &mut [&mut dyn PeriodicMessage],
    pcan_tx: &mut MPCAN,
    car: &mut MCAR,
) -> !
where
    MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = CarState>,
{
    // Determine the shortest sleep time needed
    let common_ms = msgs.iter().fold(msgs[0].period_ms(), |a, b| {
        num::integer::gcd(a, b.period_ms())
    });
    let common_period = common_ms.millis();

    defmt::trace!("Common period {}ms", common_ms);

    assert!(common_ms > 0);

    let mut n = 0u32;
    let mut next = Mono::now();

    loop {
        next += common_period;
        Mono::delay_until(next).await;
        // TODO: handle skipped deadlines

        for msg in &mut *msgs {
            // Testing every message on each "tick" is frustratingly
            // inefficient, what would be great would be to group 'msgs'
            // by rates and then would only need to test once.
            //
            // Doing this without allocating seems like a pain, though...
            // and 'msgs' is pretty short so probably this is fast enough.
            let send_every_n = msg.period_ms() / common_ms;
            defmt::trace!("Message period {}ms common {}ms send_every_n {} n {}",
                msg.period_ms(), common_ms, send_every_n, n);
            if send_every_n % n == 0 {
                if let Some(msg) = car.lock(|car| msg.update_for_transmit(car)) {
                    pcan_tx.lock(|can| can.transmit(&msg));
                }
            }
        }
        n += 1;
    }
}

// /// Set of messages to send at a particular period
// pub struct PeriodSet<'a> {
//     pub period: EPeriod,  // FIXME: can have constructor here?
//     pub messages: &'a [&'a mut dyn PeriodicMessage],
// }

// pub async fn send_periodic_messages<MPCAN, MCAR>(all_sets: &mut [&mut [&mut PeriodSet<'_>]], mut pcan_tx: MPCAN, mut car: MCAR) -> !
// where
//     MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
//     MCAR: Mutex<T = CarState> {
//     let shortest = EPeriod::shortest_duration().into();
//     let mut next = Mono::now();
//     let mut count = 0u32;

//     loop {
//         // Wake up every 'shortest' tick
//         next += shortest;
//         Mono::delay_until(next).await;

//         // Each iteration take a copy of the car state and use it for
//         // all messages sent on this iteration. Saves on critical sections,
//         // and means all the messages see a consistent snapshot of vehicle state
//         let car = car.lock(|car| car.clone());

//         // This is super ugly because 'all_sets' is a slice of references to slices
//         // of PeriodSets. The exact sizes and ordering aren't known at compile time,
//         // so we can't easily make a single array or Vec.
//         //
//         // The saving grace is that the total size is pretty small, and the recursive
//         // iteration should be cheap to do.
//         for sets in &mut *all_sets {
//             for set in sets.iter_mut().filter(|s| s.period.send_on_iter(count)) {
//                 for msg in &mut set.messages {
//                     if let Some(to_send) = msg.update_for_transmit(&car) {
//                         pcan_tx.lock(|can| can.transmit(&to_send));
//                     }
//                 }
//             }
//         }

//         count += 1;
//     }

// }

// Very simple structure to have a loop which does a bunch of periodic things
//
// Pattern is create: create a group and one or more Periods from it
// call group.next_poll().await;
// then call period.due(&group) on each Period
//
// This could probably be made clever in various ways, but it's nice and simple
// like this...

pub struct PeriodicGroup {
    last: Instant,      // Last time this period group was triggered
    shortest: Duration, // Shortest period for wakeups
}

pub struct Period {
    period: Duration,
    next: Instant,
}

impl PeriodicGroup {
    pub fn new(shortest: Rate) -> Self {
        PeriodicGroup {
            last: Mono::now(),
            shortest: shortest.into_duration(),
        }
    }

    pub async fn next_poll(&mut self) {
        self.last = self
            .last
            .checked_add_duration(self.shortest)
            .expect("overflows after 29 days");
        Mono::delay_until(self.last).await;
    }

    pub fn new_period(&self, period: Rate) -> Period {
        let duration = period.into_duration();
        assert!(duration.ticks() % self.shortest.ticks() == 0); // ?
        Period {
            period: duration,
            next: self.last,
        }
    }
}

impl Period {
    pub fn due(&mut self, group: &PeriodicGroup) -> bool {
        if self.next <= group.last {
            self.next = self
                .next
                .checked_add_duration(self.period)
                .expect("overflows after 29 days");
            // If the task isn't polling this on every loop (i.e. has an ignition check, then it can fall behind
            // by more than a whole tick period. This isn't ideal, should find a way to design this so it's not
            // likely
            if self.next < Mono::now() {
                self.next = Mono::now()
                    .checked_add_duration(self.period)
                    .expect("overflows after 29 days");
            }
            true
        } else {
            false
        }
    }
}
