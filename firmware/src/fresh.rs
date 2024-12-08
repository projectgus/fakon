use defmt::Format;
use rtic_monotonics::Monotonic;

use crate::{hardware::Mono, Instant};

/// Struct to wrap a periodic signal value that can be "fresh" or "stale"
#[derive(Clone, Copy)]
pub struct Fresh<VALUE, const STALE_SECS: u32>
where
    VALUE: Copy,
{
    value: Option<(Instant, VALUE)>, // (Last Set, Value)
}

// Using a trait here allows return types to be "impl IsFresh<V>" instead of
// "Fresh<V, SECS>" which leaks the const parameter out unnecessarily.
//
// Making Format a supertrait here is semi-laziness so we can have
// functions "-> impl IsFresh<V>" instead of "-> impl IsFresh<V> + Format"
pub trait IsFresh<VALUE>: Format + Copy {
    /// Set the value and update its last set timestamp
    fn set(&mut self, value: VALUE);

    /// Get a reference to the value, or None if it's stale or was never set.
    fn get(&self) -> Option<VALUE>;

    /// Get the last set value, even if it is stale
    fn get_unchecked(&self) -> Option<VALUE>;

    /// Return true if the value is fresh (i.e. was last set within the stale_after period)
    fn is_fresh(&self) -> bool;

    /// Return true if the value is stale (i.e. not fresh)
    #[inline]
    fn is_stale(&self) -> bool {
        !self.is_fresh()
    }
}

impl<VALUE, const STALE_SECS: u32> Fresh<VALUE, STALE_SECS>
where
    VALUE: Copy,
{
    #[inline]
    pub fn new() -> Self {
        Self {
            value: None,
        }
    }
}

impl<VALUE, const STALE_SECS: u32> IsFresh<VALUE> for Fresh<VALUE, STALE_SECS>
where
    VALUE: Copy + Format,
{
    fn set(&mut self, value: VALUE) {
        self.value = Some((Mono::now(), value));
    }

    fn get(&self) -> Option<VALUE> {
        if self.is_fresh() {
            self.get_unchecked()
        } else {
            None // Value never set
        }
    }

    fn get_unchecked(&self) -> Option<VALUE> {
        self.value.map(|(_, value)| value)
    }

    fn is_fresh(&self) -> bool {
        self.value.is_some_and(|(last_set, _)| Mono::now()
                               .checked_duration_since(last_set)
                               .expect("now() not allowed to wrap")
                               .to_secs() < STALE_SECS)
    }
}

impl<VALUE, const STALE_SECS: u32> Format for Fresh<VALUE, STALE_SECS>
where
    VALUE: Copy + Format,
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
