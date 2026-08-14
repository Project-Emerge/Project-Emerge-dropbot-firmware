//! Power path: the STM6600's `PS_HOLD` latch and the BQ24074's input-current straps.

use ariel_os::gpio::{Input, IntEnabledInput, Level, Output, Pull};
use ariel_os::hal::peripheral::Peri;
use ariel_os::hal::peripherals::{PA10, PB0, PB1, PB5, PB7};
use ariel_os::log::debug;
use ariel_os::time::{Duration, Timer, with_timeout};

/// How long the power button has to be held down before the anchor cuts its own supply. The same
/// threshold the dropbot uses in `src/tasks/power_button.rs`, so the two boards feel alike.
const POWER_OFF_HOLD: Duration = Duration::from_secs(3);
/// Contact bounce -- and the minimum `PWR_INT` pulse the STM6600 emits for a press far shorter than
/// that -- are ridden out by ignoring the line for this long after it goes active.
const DEBOUNCE: Duration = Duration::from_millis(30);

/// Owns every pin that must stay driven for the anchor to keep running.
///
/// Constructed in `main()` and then moved into a task. Never dropped, and released only through
/// [`Self::release`]: letting go of `PS_HOLD` lets the STM6600 cut the rail, and letting go of the
/// BQ24074 straps drops the charger back to its default 100 mA input limit.
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

    /// Returns once the power button has been held down for [`POWER_OFF_HOLD`], leaving the caller
    /// to [`Self::release`] the latch.
    ///
    /// `PWR_INT` is the STM6600's open-drain interrupt line, so it is low while the controller has
    /// something to report and high (through the internal pull-up) otherwise. In the on state that
    /// is a debounced press: the line follows the button, held low for at least the controller's
    /// minimum pulse and then for as long as the button stays down. Timing the hold is therefore the
    /// firmware's job -- the STM6600 only reports the press and waits for `PS_HOLD` to be dropped.
    ///
    /// Presses shorter than [`POWER_OFF_HOLD`] are dropped rather than reported. The anchor has no
    /// menu to page through and no display to page it on, and an anchor that could be switched off
    /// by being leaned against would take the network's timing down with it.
    ///
    /// The STM6600 also asserts `PWR_INT` on undervoltage, and holds it asserted while the cell
    /// stays below the threshold, so a flat battery arrives here as a long press and powers the
    /// anchor off. That is the right response to it anyway.
    pub async fn wait_for_power_off_request(&mut self) {
        loop {
            self.power_int.wait_for_low().await;
            Timer::after(DEBOUNCE).await;
            // Already back up before the bounce window closed: a glitch on the line, or the tail of
            // the pulse from a press that was over before this even started looking.
            if self.power_int.is_high() {
                continue;
            }

            // Still asserted when the hold window closes, so the button is genuinely being held.
            if with_timeout(POWER_OFF_HOLD - DEBOUNCE, self.power_int.wait_for_high())
                .await
                .is_err()
            {
                return;
            }
            debug!("anchor: short power-button press ignored");
        }
    }

    /// Drives `PS_HOLD` low, which is what tells the STM6600 to cut the rail.
    ///
    /// The board is gone by the time this returns -- unless it is running off the SWD programmer's
    /// supply, which bypasses the load switch entirely. Callers must not arm the button again
    /// afterwards: a board that survived the release should stay off until it is reset.
    pub fn release(&mut self) {
        self.ps_hold.set_low();
    }
}
