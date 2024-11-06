use crate::{hardware::Mono, Duration, Instant, Rate};
use rtic_monotonics::Monotonic;

/// Very simple wrapper around Monotonic to give you
/// something you can await at a constant 'Rate' with
/// no drift
pub(crate) struct Every {
    deadline: Instant,
    // Note: choosing not to make period a const generic param, for two reasons:
    // - Const param can't be a Duration due to const generic restrictions
    // - Doing it with 'const HZ: u32' adds >1KB of binary size!!
    period: Duration,
}

impl Every {
    pub fn new(rate: Rate) -> Self {
        Every {
            deadline: Mono::now(),
            period: rate.into_duration(),
        }
    }

    pub async fn next(&mut self) {
        Mono::delay_until(self.deadline).await;
        self.deadline += self.period;
    }
}
