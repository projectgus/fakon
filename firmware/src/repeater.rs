use core::future::Future;

use crate::{hardware::Mono, Duration, Instant, Rate};
use futures::{future, FutureExt};
use rtic_monotonics::Monotonic;

/// Very simple wrapper around Monotonic to give you
/// something you can await at a constant 'Rate' with
/// no drift
pub(crate) struct Repeater {
    deadline: Instant,
    // Note: choosing not to make period a const generic param, for two reasons:
    // - Const param can't be a Duration due to const generic restrictions
    // - Doing it with 'const HZ: u32' adds >1KB of binary size!!
    period: Duration,
}

impl Repeater {
    pub fn new(rate: Rate) -> Self {
        let period = rate.into_duration();
        Repeater {
            deadline: Mono::now() + period,
            period,
        }
    }

    pub async fn next(&mut self) {
        let now = Mono::now();
        if now > self.deadline {
            // Should never enter here after the deadline, it means processing
            // has taken longer than the repeating period. Report that we've
            // skipped a cycle and push the deadline to the future rather than
            // returning immediately and risking a cascade failure.
            defmt::error!("{} Repeater behind by {}, cycle(s) skipped", self.period, now - self.deadline);
            while now > self.deadline {
                self.deadline += self.period;
            }
        }
        Mono::delay_until(self.deadline).await;
        self.deadline += self.period;
    }

    /// Convenience function for use in select_biased!() macros
    pub fn on_next(&mut self) -> future::Fuse<impl Future<Output = ()> + use<'_>> {
        self.next().fuse()
    }

}
