//! Board drivers.
//!
//! All four are concrete rather than generic over `embedded-hal` traits, unlike their
//! pre-ariel-os ancestors in `UWB-anchor-firmware/src/peripherals/`. There is exactly one battery
//! monitor, one indicator LED, one power controller and one radio on this board, wired to exactly
//! the pins in `crate::pins`, so a trait plus an associated error type per driver bought nothing
//! but type-parameter noise and a `PinError<E1, E2>` enum whose variants were never matched on.

pub mod battery;
pub mod led;
pub mod power;
pub mod uwb;
