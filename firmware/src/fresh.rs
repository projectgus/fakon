use defmt::Format;
use rtic_monotonics::Monotonic;

use crate::{hardware::Mono, Duration, Instant};

/// Struct to wrap a periodic signal value that can be "fresh" or "stale"
#[derive(Clone, Copy)]
pub struct Fresh<VALUE>
where
    VALUE: Copy,
{
    value: Option<(Instant, VALUE)>, // (Last Set, Value)
    stale_after: Duration, // This could be a const generic param, but Duration not currently eligible
}

impl<VALUE> Fresh<VALUE>
where
    VALUE: Copy,
{
    pub fn new(stale_after: Duration) -> Self {
        Self {
            value: None,
            stale_after,
        }
    }

    /// Set the value and update its last set timestamp
    pub fn set(&mut self, value: VALUE) {
        self.value = Some((Mono::now(), value));
    }

    /// Get a reference to the value, or None if it's stale or was never set.
    pub fn get(&self) -> Option<VALUE> {
        if self.is_fresh() {
            self.get_unchecked()
        } else {
            None // Value never set
        }
    }

    /// Get the last set value, even if it is stale
    pub fn get_unchecked(&self) -> Option<VALUE> {
        self.value.map(|(_, value)| value)
    }

    /// Return true if the value is fresh (i.e. was last set within the stale_after period)
    pub fn is_fresh(&self) -> bool {
        self.value
            .map(|(last_set, _)| {
                let age = Mono::now()
                    .checked_duration_since(last_set)
                    .expect("now() not allowed to wrap");
                age < self.stale_after
            })
            .unwrap_or(false)
    }

    /// Return true if the value is stale (i.e. not fresh)
    #[inline]
    pub fn is_stale(&self) -> bool {
        !self.is_fresh()
    }
}

impl<VALUE> Format for Fresh<VALUE>
where
    VALUE: Format + Copy,
{
    fn format(&self, fmt: defmt::Formatter) {
        match &self.value {
            Some((_, value)) => {
                if self.is_fresh() {
                    defmt::write!(fmt, "{:?}", value);
                } else {
                    defmt::write!(fmt, "Stale({:?})", value);
                }
            }
            None => defmt::write!(fmt, "Stale(None)"),
        }
    }
}
