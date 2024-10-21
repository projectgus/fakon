use crate::hardware::Mono;
use crate::{Duration, Instant, Rate};
use rtic_monotonics::Monotonic;

// Very simple structure to have a loop which does a bunch of periodic things
//
// Pattern is create: create a group and one or more Periods from it
// call group.next_poll().await;
// then call period.due(&group) on each Period
//
// This could probably be made clever in various ways, but it's nice and simple
// like this...

pub struct PeriodicGroup {
    last: Instant, // Last time this period group was triggered
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
        self.last = self.last.checked_add_duration(self.shortest).expect("overflows after 29 days");
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
            self.next = self.next.checked_add_duration(self.period).expect("overflows after 29 days");
            // If the task isn't polling this on every loop (i.e. has an ignition check, then it can fall behind
            // by more than a whole tick period. This isn't ideal, should find a way to design this so it's not
            // likely
            if self.next < Mono::now() {
                self.next = Mono::now().checked_add_duration(self.period).expect("overflows after 29 days");
            }
            true
        } else {
            false
        }
    }
}
