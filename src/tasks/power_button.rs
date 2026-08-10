use ariel_os::gpio::{Input, Level, Output, Pull};
use ariel_os::log::info;
use ariel_os::time::{Duration, Timer};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;

use crate::data::button::ButtonEvent;
use crate::pins;

/// How long the button has to be held down before the board powers itself off.
const POWER_OFF_HOLD: Duration = Duration::from_secs(3);
/// Contact bounce is ridden out by ignoring the line for this long after each transition.
const DEBOUNCE: Duration = Duration::from_millis(30);
/// Grace period between announcing the power-off and actually cutting the supply, so the
/// display has time to flush its framebuffer and the message stays readable for a moment.
const POWER_OFF_NOTICE: Duration = Duration::from_millis(700);

/// Owns the power button (`int`) and the power latch (`kill`).
///
/// Pressing the button closes the supply path long enough for the board to boot; it then
/// stays up only as long as `kill` is driven high, so releasing that line cuts the board's
/// own power. Once the latch is held, the button becomes a plain user input: a short press
/// advances the menu, holding it for [`POWER_OFF_HOLD`] powers the board off.
#[ariel_os::task]
pub async fn manage_power_button(
    pins: pins::PowerManagementPins,
    button_events: Sender<'static, CriticalSectionRawMutex, ButtonEvent, 2>,
) -> ! {
    // Latch the supply first: until this is high, the board is alive only because the user
    // is still holding the button down.
    let mut kill = Output::new(pins.kill, Level::High);
    kill.set_high();

    // Active low: the button pulls the line to ground, the internal pull-up holds it high
    // while the button is up.
    let mut button = Input::builder(pins.int, Pull::Up)
        .build_with_interrupt()
        .expect("button: registering the GPIO interrupt failed");

    // The board is powered up by pressing this very button, so it is most likely still down
    // at this point. Waiting it out keeps the power-on press from being read as a menu
    // press -- or, if the user is slow to let go, as a power-off request.
    if button.is_low() {
        info!("button: waiting for the power-on press to be released");
        button.wait_for_high().await;
        Timer::after(DEBOUNCE).await;
    }

    loop {
        button.wait_for_low().await;
        Timer::after(DEBOUNCE).await;
        // The line was already back up by the time the bounce window closed: a spike on the
        // input rather than a press.
        if button.is_high() {
            continue;
        }

        match select(
            button.wait_for_high(),
            Timer::after(POWER_OFF_HOLD - DEBOUNCE),
        )
        .await
        {
            Either::First(()) => {
                info!("button: short press");
                // Dropping the event when the display is not keeping up only costs a page
                // change; blocking here would make the button unresponsive, including for
                // powering off.
                let _ = button_events.try_send(ButtonEvent::ShortPress);
                // Swallow the bounce on release so it is not read as a second press.
                Timer::after(DEBOUNCE).await;
            }
            Either::Second(()) => {
                info!("button: held down, powering off");
                let _ = button_events.try_send(ButtonEvent::LongPress);
                Timer::after(POWER_OFF_NOTICE).await;
                kill.set_low();

                // The supply is gone by now. If it is not -- the board is running off the
                // programmer's USB, which bypasses the latch -- park here rather than arm
                // the button again, so the board stays "off" until it is reset.
                info!("button: power latch released");
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }
    }
}
