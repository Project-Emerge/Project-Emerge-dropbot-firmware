use core::fmt::Write;

use ariel_os::hal;
use ariel_os::i2c::controller::{Kilohertz, highest_freq_in};
use ariel_os::log::{Debug2Format, error, info};
use ariel_os::time::Timer;
use heapless::String;

use crate::drivers::display_driver::SD1306Driver;
use crate::pins;
use crate::pins::I2cBus;
use crate::traits::DisplayController;
use crate::{DEVICE_ID, NETWORK_STATUS};

#[ariel_os::task]
pub async fn manage_display(pins: pins::I2cPins) -> ! {
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
    const NO_NETWORK: &str = "---.---.---.---";

    if let Err(e) = display
        .draw_status(DEVICE_ID, NO_NETWORK, None, false)
        .await
    {
        error!("display: initial draw failed: {:?}", Debug2Format(&e));
    }

    loop {
        let result = match NETWORK_STATUS.wait().await {
            Some(address) => {
                let mut ip_address: String<15> = String::new();
                let _ = write!(ip_address, "{}", address);
                display
                    .draw_status(DEVICE_ID, ip_address.as_str(), None, true)
                    .await
            }
            None => {
                display
                    .draw_status(DEVICE_ID, NO_NETWORK, None, false)
                    .await
            }
        };

        if let Err(e) = result {
            error!("display: network update failed: {:?}", Debug2Format(&e));
        }
    }
}
