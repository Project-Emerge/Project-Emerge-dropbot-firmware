use core::fmt::Write;

use ariel_os::config;
use ariel_os::log::{Debug2Format, error, info};
use ariel_os::reexports::embassy_net::Ipv4Address;
use ariel_os::time::{Duration, Timer};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Receiver as WatchReceiver;
use heapless::String;

use crate::FIRMWARE_VERSION;
use crate::data::battery::ChargerStatus;
use crate::data::button::ButtonEvent;
use crate::data::menu::MenuPage;
use crate::data::mqtt::BrokerStatus;
use crate::data::ota::OtaStatus;
use crate::drivers::display_driver::SD1306Driver;
use crate::drivers::shared_i2c::BoardI2cDevice;
use crate::traits::{BatteryPage, DisplayController, NetworkPage};

const POWER_OFF_TITLE: &str = "POWERING OFF";
const POWER_OFF_MESSAGE: &str = "Bye!";

/// The network the firmware was built to join. ariel-os reads the same variable to
/// configure the Wi-Fi driver, so this always names the network actually being joined.
const WIFI_SSID: &str = config::str_from_env_or!(
    "CONFIG_WIFI_NETWORK",
    "?",
    "Wi-Fi SSID (network name) shown on the network page",
);

/// Whatever moved, normalized into one type so the render loop stays flat.
enum DisplayEvent {
    Network(Option<Ipv4Address>),
    Ota(OtaStatus),
    Button(ButtonEvent),
    Broker(BrokerStatus),
    Charger(ChargerStatus),
}

#[ariel_os::task]
pub async fn manage_display(
    i2c: BoardI2cDevice,
    device_id: &'static str,
    network_status: &'static Signal<CriticalSectionRawMutex, Option<Ipv4Address>>,
    mut ota_status: WatchReceiver<'static, CriticalSectionRawMutex, OtaStatus, 2>,
    button_events: Receiver<'static, CriticalSectionRawMutex, ButtonEvent, 2>,
    mut broker_status: WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 5>,
    mut charger_status: WatchReceiver<'static, CriticalSectionRawMutex, ChargerStatus, 1>,
) -> ! {
    let mut display = SD1306Driver::new(i2c, 0x3C);
    match display.init().await {
        Ok(_) => {
            info!("display: initialized");
        }
        Err(e) => {
            error!("display: initialization failed: {:?}", Debug2Format(&e));
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    // Everything the pages are drawn from. Kept here rather than re-read on each redraw so
    // that a page switch can render immediately, without waiting for its sources to move.
    let mut address: Option<Ipv4Address> = None;
    let mut broker = BrokerStatus::default();
    let mut charger: Option<ChargerStatus> = None;
    let mut ota_active = false;
    // Which menu page the button task has selected. Short presses walk through them.
    let mut page = MenuPage::default();

    if let Err(e) = draw_page(&mut display, page, device_id, address, broker, charger).await {
        error!("display: initial draw failed: {:?}", Debug2Format(&e));
    }

    loop {
        let event = next_event(
            network_status,
            &mut ota_status,
            &button_events,
            &mut broker_status,
            &mut charger_status,
        )
        .await;

        // An update screen owns the panel until the update ends; state changes are still
        // recorded, and get drawn as part of restoring the page afterwards.
        let result = match event {
            DisplayEvent::Network(status) => {
                address = status;
                if ota_active {
                    continue;
                }
                draw_page(&mut display, page, device_id, address, broker, charger).await
            }
            DisplayEvent::Broker(status) => {
                broker = status;
                if ota_active || page != MenuPage::Network {
                    continue;
                }
                draw_page(&mut display, page, device_id, address, broker, charger).await
            }
            DisplayEvent::Charger(status) => {
                charger = Some(status);
                if ota_active || page != MenuPage::Battery {
                    continue;
                }
                draw_page(&mut display, page, device_id, address, broker, charger).await
            }
            DisplayEvent::Button(ButtonEvent::ShortPress) => {
                page = page.next();
                if ota_active {
                    continue;
                }
                draw_page(&mut display, page, device_id, address, broker, charger).await
            }
            DisplayEvent::Button(ButtonEvent::LongPress) => {
                if let Err(e) = display
                    .draw_notice(POWER_OFF_TITLE, POWER_OFF_MESSAGE)
                    .await
                {
                    error!("display: power-off draw failed: {:?}", Debug2Format(&e));
                }
                // The supply is about to be cut: hold this screen so nothing redraws over
                // the goodbye in whatever time is left.
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
            DisplayEvent::Ota(status) => {
                ota_active = status.is_active();
                match status {
                    OtaStatus::Idle => {
                        draw_page(&mut display, page, device_id, address, broker, charger).await
                    }
                    OtaStatus::Preparing => {
                        display
                            .draw_firmware_update("Loading new firmware...", None)
                            .await
                    }
                    OtaStatus::Downloading { .. } => {
                        display
                            .draw_firmware_update("Downloading firmware", status.percent())
                            .await
                    }
                    OtaStatus::Applying => {
                        display
                            .draw_firmware_update("Rebooting...", status.percent())
                            .await
                    }
                }
            }
        };

        if let Err(e) = result {
            error!("display: update failed: {:?}", Debug2Format(&e));
        }
    }
}

/// Waits for whichever input moves first. `select` only comes in fixed arities, so the five
/// sources are nested into threes and twos and flattened back into [`DisplayEvent`] here.
async fn next_event(
    network_status: &Signal<CriticalSectionRawMutex, Option<Ipv4Address>>,
    ota_status: &mut WatchReceiver<'static, CriticalSectionRawMutex, OtaStatus, 2>,
    button_events: &Receiver<'static, CriticalSectionRawMutex, ButtonEvent, 2>,
    broker_status: &mut WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 5>,
    charger_status: &mut WatchReceiver<'static, CriticalSectionRawMutex, ChargerStatus, 1>,
) -> DisplayEvent {
    match select3(
        select(network_status.wait(), ota_status.changed()),
        select(button_events.receive(), broker_status.changed()),
        charger_status.changed(),
    )
    .await
    {
        Either3::First(Either::First(status)) => DisplayEvent::Network(status),
        Either3::First(Either::Second(status)) => DisplayEvent::Ota(status),
        Either3::Second(Either::First(event)) => DisplayEvent::Button(event),
        Either3::Second(Either::Second(status)) => DisplayEvent::Broker(status),
        Either3::Third(status) => DisplayEvent::Charger(status),
    }
}

/// Draws the currently selected menu page.
async fn draw_page<D: DisplayController>(
    display: &mut D,
    page: MenuPage,
    device_id: &str,
    address: Option<Ipv4Address>,
    broker: BrokerStatus,
    charger: Option<ChargerStatus>,
) -> Result<(), D::Error> {
    match page {
        MenuPage::Network => {
            // `Ipv4Address` only formats through `Display`, so it has to be rendered into a
            // buffer that outlives the borrow the page payload takes of it.
            let mut ip_address: String<15> = String::new();
            if let Some(address) = address {
                let _ = write!(ip_address, "{address}");
            }

            display
                .draw_network_page(&NetworkPage {
                    device_id,
                    ssid: WIFI_SSID,
                    ip_address: address.map(|_| ip_address.as_str()),
                    broker_status: broker.label(),
                    firmware_version: FIRMWARE_VERSION,
                })
                .await
        }
        MenuPage::Battery => {
            display
                .draw_battery_page(&BatteryPage {
                    status: charger.as_ref(),
                    firmware_version: FIRMWARE_VERSION,
                })
                .await
        }
    }
}
