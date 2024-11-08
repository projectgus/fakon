//! Simple async timer for creating a bunch of timers that go off at different
//! repeating intervals, all awaited from the same task.
use crate::{hardware::Mono, Duration, Instant, Rate};
use enumflags2::{bitflags, BitFlags};
use rtic_monotonics::Monotonic;

#[bitflags]
#[repr(u8)]
#[derive(Copy, Clone, defmt::Format, PartialEq)]
pub enum Period {
    Hz1,
    Hz5,
    Hz10,
    Hz20,
    Hz50,
    Hz100,
}

// This is the basic tick length for the repeater, all other
// tick lengths are a multiple of this one (asserts verify this
// at runtime but not currently at compile time.)
const TICK_BASE: Period = Period::Hz100;

impl Period {
    pub const fn rate(&self) -> Rate {
        match self {
            Period::Hz1 => Rate::Hz(1),
            Period::Hz5 => Rate::Hz(5),
            Period::Hz10 => Rate::Hz(10),
            Period::Hz20 => Rate::Hz(20),
            Period::Hz50 => Rate::Hz(50),
            Period::Hz100 => Rate::Hz(100),
        }
    }

    pub const fn duration(&self) -> Duration {
        self.rate().into_duration()
    }

    /// This period expires every N passes of the TICK_BASE
    const fn multiplier(&self) -> u32 {
        let res = TICK_BASE.rate().to_Hz() / self.rate().to_Hz();
        // All ticks should be a multiple of the TICK_BASE period
        assert!(TICK_BASE.rate().to_Hz() % self.rate().to_Hz() == 0);
        res
    }

    /// Should this period trigger on this tick?
    const fn due_on(&self, ticks: u32) -> bool {
        ticks % self.multiplier() == 0
    }
}

/// A set of Period values, implemented as bit flags
pub type PeriodSet = BitFlags<Period>;

/// Wrapper around Monotonic to give you something you can await for periodic
/// ticks at various frequencies, without drift and without needing to spawn
/// many async tasks
pub(crate) struct Repeater {
    /// Timestamp of the next tick expiry
    next_tick: Instant,

    /// Number of ticks so far (not expected to wrap, runtime too short)
    ticks: u32,
}

impl Repeater {
    pub fn new() -> Self {
        Repeater {
            next_tick: Mono::now() + TICK_BASE.duration(),
            ticks: 0,
        }
    }

    /// Delay until the next tick expires, and return a set of which time
    /// periods are due at this tick interval.
    pub async fn tick(&mut self) -> PeriodSet {
        self.tick_filtered(PeriodSet::ALL).await
    }

    /// Delay until the next tick expires, and return a set of which of the
    /// enabled time periods are due at this tick interval. At least one Period
    /// will be enabled in the result.
    pub async fn tick_filtered(&mut self, enabled: PeriodSet) -> PeriodSet {
        loop {
            Mono::delay_until(self.next_tick).await;

            // Set all of the flags which are both enabled in this call and
            // due at this tick
            let due = BitFlags::from_iter(enabled.iter().filter(|f| f.due_on(self.ticks)));

            // Set up for the next tick
            let orig_next_tick = self.next_tick;
            self.next_tick += TICK_BASE.duration();
            self.ticks += 1;

            if due.is_empty() {
                // Go around again until at least one period is enabled (This
                // approach a bit wasteful, but avoids the question of what to do
                // if this future is cancelled while waiting.)
                continue;
            }

            // Check for skipped ticks
            let now = Mono::now();
            let mut skipped = 0;
            while now > self.next_tick {
                self.next_tick += TICK_BASE.duration();
                self.ticks += 1;
                skipped += 1;
            }
            if skipped > 0 {
                // We've fallen behind more than an entire tick period, so log an error
                // ... don't try to catch up as this could create cascading failure
                defmt::error!(
                    "Repeater tick skipped {} tick(s), total lag {}",
                    skipped, now - orig_next_tick
                );
            }

            return due;
        }
    }

    // /// Convenience function for use in select_biased!() macros
    // pub fn on_tick(&mut self) -> future::Fuse<impl Future<Output = PeriodSet> + use<'_>> {
    //     self.tick().fuse()
    // }

    // /// Convenience function for use in select_biased!() macros
    // pub fn on_tick_filtered(&mut self, filter: PeriodSet) -> future::Fuse<impl Future<Output = PeriodSet> + use<'_>> {
    //     self.tick_filtered(filter).fuse()
    // }
}
