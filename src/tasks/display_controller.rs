use core::fmt::Write;

use ariel_os::hal;
use ariel_os::i2c::controller::{Kilohertz, highest_freq_in};
use ariel_os::log::{Debug2Format, error, info};
use ariel_os::reexports::embassy_net::Ipv4Address;
use ariel_os::time::Timer;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Receiver as WatchReceiver;
use heapless::String;

use crate::data::ota::OtaStatus;
use crate::drivers::display_driver::SD1306Driver;
use crate::pins;
use crate::pins::I2cBus;
use crate::traits::DisplayController;
use crate::{DEVICE_ID, FIRMWARE_VERSION};

const NO_NETWORK: &str = "---.---.---.---";

#[ariel_os::task]
pub async fn manage_display(
    pins: pins::I2cPins,
    network_status: &'static Signal<CriticalSectionRawMutex, Option<Ipv4Address>>,
    mut ota_status: WatchReceiver<'static, CriticalSectionRawMutex, OtaStatus, 2>,
) -> ! {
    let mut i2c_config = hal::i2c::controller::Config::default();
    i2c_config.frequency = const { highest_freq_in(Kilohertz::kHz(100)..=Kilohertz::kHz(400)) };
    let bus = I2cBus::new(pins.sda, pins.scl, i2c_config);
    let mut display = SD1306Driver::new(bus, 0x3C);
    match display.init().await {
        Ok(_) => {
            info!("display: initialized");
        }
        Err(e) => {
            error!("display: initialization failed: {:?}", Debug2Format(&e));
            loop {
                Timer::after(ariel_os::time::Duration::from_secs(1)).await;
            }
        }
    }

    // Last known network state, kept so the status mask can be restored unchanged once an
    // OTA update has taken the display over.
    let mut address: Option<Ipv4Address> = None;
    let mut ota_active = false;

    if let Err(e) = draw_status(&mut display, address).await {
        error!("display: initial draw failed: {:?}", Debug2Format(&e));
    }

    loop {
        let result = match select(network_status.wait(), ota_status.changed()).await {
            Either::First(status) => {
                address = status;
                // An update screen owns the panel until the update ends; the new address
                // is recorded and drawn as part of restoring the mask afterwards.
                if ota_active {
                    continue;
                }
                draw_status(&mut display, address).await
            }
            Either::Second(status) => {
                ota_active = status.is_active();
                match status {
                    OtaStatus::Idle => draw_status(&mut display, address).await,
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

async fn draw_status<D: DisplayController>(
    display: &mut D,
    address: Option<Ipv4Address>,
) -> Result<(), D::Error> {
    match address {
        Some(address) => {
            let mut ip_address: String<15> = String::new();
            let _ = write!(ip_address, "{}", address);
            display
                .draw_status(DEVICE_ID, ip_address.as_str(), None, true, FIRMWARE_VERSION)
                .await
        }
        None => {
            display
                .draw_status(DEVICE_ID, NO_NETWORK, None, false, FIRMWARE_VERSION)
                .await
        }
    }
}
