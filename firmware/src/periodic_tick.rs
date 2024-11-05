use crate::{hardware::Mono, Duration, Rate};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{Error, PubSubChannel, Subscriber},
};
use enum_iterator::{all, cardinality, Sequence};
use fugit::RateExtU32;
use rtic_monotonics::Monotonic;

#[derive(Clone, Copy, PartialEq, Sequence)]
pub enum Period {
    Hz1,
    Hz5,
    Hz10,
    Hz20,
    Hz50,
    Hz100,
}

impl Period {
    fn into_duration(self) -> Duration {
        let r: Rate = match self {
            Period::Hz1 => 1.Hz(),
            Period::Hz5 => 5.Hz(),
            Period::Hz10 => 10.Hz(),
            Period::Hz20 => 20.Hz(),
            Period::Hz50 => 50.Hz(),
            Period::Hz100 => 100.Hz(),
        };
        r.into_duration()
    }

    /// The minimum tick length that covers all the periods
    #[inline]
    fn ticker_duration() -> Duration {
        // Note: this is in theory compile-time calculable so fingers crossed compiler can optimise it...?
        let ticker_ms = all::<Period>().fold(Period::Hz1.into_duration().to_millis(), |a, b| {
            num::integer::gcd(a, b.into_duration().to_millis())
        });
        assert!(ticker_ms > 1);
        Duration::millis(ticker_ms)
    }

    /// How many ticker ticks this period runs on
    fn ticks_per_period(&self) -> u32 {
        self.into_duration().to_millis() / Self::ticker_duration().to_millis()
    }
}

const CAP: usize = cardinality::<Period>(); // Capacity of channel
const SUBS: usize = 4; // Max subscriber tasks
const PUBS: usize = 1; // Max publishers (1, implemented in 'run')

pub struct Ticker(PubSubChannel<CriticalSectionRawMutex, Period, CAP, SUBS, PUBS>);

pub struct TickListener<'a>(Subscriber<'a, CriticalSectionRawMutex, Period, CAP, SUBS, PUBS>);

impl Ticker {
    pub const fn new() -> Self {
        Ticker(PubSubChannel::new())
    }

    pub fn subscribe(&self) -> Result<TickListener, Error> {
        self.0.subscriber().map(TickListener)
    }

    /// Async function that publishes ticks for different periods indefinitely.
    /// (or until the monotonic instant wraps)
    ///
    /// Doesn't drift, although if the calling task is delayed then ticks will be delayed.
    pub async fn run(&mut self) -> Result<!, Error> {
        let publisher = self.0.publisher()?;
        let mut next = Mono::now();
        let mut n = 0u32;

        loop {
            Mono::delay_until(next).await;
            for period in all::<Period>() {
                if n % period.ticks_per_period() == 0 {
                    publisher.publish_immediate(period);
                }
            }
            n += 1;
            next += Period::ticker_duration();
        }
    }
}

impl TickListener<'_> {
    /// Wait for the next tick period.
    ///
    /// Infallible function, will log a warning if some periods have been missed
    /// due to the calling task running slower than the tick rate.
    pub async fn next_period(&mut self) -> Period {
        loop {
            match self.0.next_message().await {
                embassy_sync::pubsub::WaitResult::Lagged(missed) => {
                    defmt::warn!("TickListener missed {} tick periods", missed);
                }
                embassy_sync::pubsub::WaitResult::Message(period) => {
                    return period;
                }
            }
        }
    }
}

impl Default for Ticker {
    fn default() -> Self {
        Self::new()
    }
}
