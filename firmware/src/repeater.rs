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
        Repeater {
            deadline: Mono::now(),
            period: rate.into_duration(),
        }
    }

    pub async fn next(&mut self) {
        Mono::delay_until(self.deadline).await;
        self.deadline += self.period;
    }

    /// Convenience function for use in select_biased!() macros
    pub fn on_next(&mut self) -> future::Fuse<impl Future<Output = ()> + use<'_>> {
        self.next().fuse()
    }

}
