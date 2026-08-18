use ariel_os::gpio::{Input, Level, Output, Pull};
use ariel_os::log::info;
use ariel_os::time::{Duration, Timer};
use embassy_futures::select::{Either, select};

use crate::data::power::{PowerEvent, ShutdownReason};
use crate::pins;
use crate::task_sync::{PowerEventTx, ShutdownRequestRx};

/// How long the button has to be held down before the board powers itself off.
const POWER_OFF_HOLD: Duration = Duration::from_secs(3);
/// Contact bounce is ridden out by ignoring the line for this long after each transition.
const DEBOUNCE: Duration = Duration::from_millis(30);
/// Grace period between announcing the power-off and actually cutting the supply, so the
/// display has time to flush its framebuffer and the message stays readable for a moment.
const POWER_OFF_NOTICE: Duration = Duration::from_millis(700);
/// Low-battery shutdown gets a longer notice because it is automatic and the user needs time
/// to read why the robot is stopping.
const LOW_BATTERY_NOTICE: Duration = Duration::from_secs(2);

/// Messaging endpoints owned by the power-button and latch task.
pub struct PowerManagerPorts {
    pub power_events: PowerEventTx,
    pub shutdown_requests: ShutdownRequestRx,
}

/// Owns the power button (`int`) and the power latch (`kill`).
///
/// Pressing the button closes the supply path long enough for the board to boot; it then
/// stays up only as long as `kill` is driven high, so releasing that line cuts the board's
/// own power. Once the latch is held, the button becomes a plain user input: a short press
/// advances the menu, holding it for [`POWER_OFF_HOLD`] powers the board off.
#[ariel_os::task]
pub async fn manage_power_button(pins: pins::PowerManagementPins, ports: PowerManagerPorts) -> ! {
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
        match select(button.wait_for_low(), ports.shutdown_requests.receive()).await {
            Either::First(()) => {
                Timer::after(DEBOUNCE).await;
                // The line was already back up by the time the bounce window closed: a spike on
                // the input rather than a press.
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
                        let _ = ports.power_events.try_send(PowerEvent::ShortPress);
                        // Swallow the bounce on release so it is not read as a second press.
                        Timer::after(DEBOUNCE).await;
                    }
                    Either::Second(()) => {
                        info!("button: held down, powering off");
                        ports
                            .power_events
                            .send(PowerEvent::ShuttingDown(ShutdownReason::ButtonHeld))
                            .await;
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
            Either::Second(reason) => {
                match reason {
                    ShutdownReason::ButtonHeld => info!("button: shutdown requested"),
                    ShutdownReason::LowBattery => {
                        info!("button: low battery warning, powering off");
                    }
                }
                ports
                    .power_events
                    .send(PowerEvent::ShuttingDown(reason))
                    .await;
                let notice = match reason {
                    ShutdownReason::ButtonHeld => POWER_OFF_NOTICE,
                    ShutdownReason::LowBattery => LOW_BATTERY_NOTICE,
                };
                Timer::after(notice).await;
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
