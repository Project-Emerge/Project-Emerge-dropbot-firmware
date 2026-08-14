//! The bicolour indicator LED: green above [`LOW_BATTERY_PERCENT`], red at or below it.

use ariel_os::gpio::Output;

/// Charge level below which the LED turns red. Deliberately not near zero: an anchor that dies
/// mid-superframe takes its slots with it, and on the master that also takes the whole network's
/// TDMA timing, so the warning has to arrive with enough charge left to act on it.
const LOW_BATTERY_PERCENT: u8 = 20;

pub struct IndicatorLed {
    red: Output<'static>,
    green: Output<'static>,
}

impl IndicatorLed {
    pub fn new(red: Output<'static>, green: Output<'static>) -> Self {
        Self { red, green }
    }

    pub fn show_battery_percentage(&mut self, percentage: u8) {
        let low = percentage <= LOW_BATTERY_PERCENT;
        self.red.set_level(low.into());
        self.green.set_level((!low).into());
    }
}
