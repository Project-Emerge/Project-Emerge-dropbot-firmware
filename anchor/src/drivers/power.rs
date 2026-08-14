//! Power path: the STM6600's `PS_HOLD` latch and the BQ24074's input-current straps.

use ariel_os::gpio::{Input, IntEnabledInput, Level, Output, Pull};
use ariel_os::hal::peripheral::Peri;
use ariel_os::hal::peripherals::{PA10, PB0, PB1, PB5, PB7};

/// Owns every pin that must stay driven for the anchor to keep running.
///
/// Constructed in `main()` and then moved into a task, never dropped: releasing `PS_HOLD` lets the
/// STM6600 cut the rail, and releasing the BQ24074 straps drops the charger back to its default
/// 100 mA input limit.
pub struct PowerLatch {
    ps_hold: Output<'static>,
    power_int: IntEnabledInput<'static>,
    _charger_enable: Output<'static>,
    _charger_en1: Output<'static>,
    _charger_en2: Output<'static>,
}

impl PowerLatch {
    /// Asserts `PS_HOLD` and straps the charger, both immediately.
    ///
    /// `PS_HOLD` goes high here rather than in [`Self::confirm_power_button`] on purpose: on a
    /// debugger-triggered boot there was no fresh button press, so the `PWR_INT` edge that call
    /// waits for may never come -- and the STM6600 cuts power if it sees `PS_HOLD` low for too
    /// long. Asserting first and confirming afterwards makes the wait an observation rather than a
    /// precondition for staying alive.
    pub fn new(
        ps_hold: Peri<'static, PB5>,
        power_int: Peri<'static, PB7>,
        charger_enable: Peri<'static, PA10>,
        charger_en1: Peri<'static, PB0>,
        charger_en2: Peri<'static, PB1>,
    ) -> Self {
        let ps_hold = Output::new(ps_hold, Level::High);
        let power_int = Input::builder(power_int, Pull::Up)
            .build_with_interrupt()
            .expect("registering the PWR_INT GPIO interrupt failed");

        // BQ24074 USB500: CE low enables the charger, EN1 high / EN2 low selects the 500 mA input
        // limit. Preserved from the pre-ariel-os anchor firmware -- the anchors are USB-charged
        // from a host port that can supply it.
        let _charger_enable = Output::new(charger_enable, Level::Low);
        let _charger_en1 = Output::new(charger_en1, Level::High);
        let _charger_en2 = Output::new(charger_en2, Level::Low);

        Self {
            ps_hold,
            power_int,
            _charger_enable,
            _charger_en1,
            _charger_en2,
        }
    }

    /// Waits for the STM6600 to report the button press that powered this board on, then
    /// re-asserts the latch.
    ///
    /// Never returns on a debugger-triggered boot, which is exactly why [`Self::new`] already
    /// asserted `PS_HOLD`: this call is then simply a task that parks forever, and the rail stays
    /// up regardless.
    pub async fn confirm_power_button(&mut self) {
        self.power_int.wait_for_high().await;
        self.ps_hold.set_high();
    }
}
