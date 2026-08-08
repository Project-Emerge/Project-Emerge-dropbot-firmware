use core::fmt;

use crate::data::battery::ChargerStatus;

pub enum MotorStatus {
    Stopped,
    Motoring { left: f32, right: f32 },
}

pub trait MotorController {
    type Error: fmt::Debug;

    /// Sets the speed of the left and right motors. The speed values should be between 0 and the configured maximum PWM speed.
    /// The caller is responsible for ensuring that the speed is between 0 and 100. If the speed exceeds the maximum, it will be capped at the maximum value.
    fn set_speed(&mut self, left: f32, right: f32) -> Result<(), Self::Error>;
    /// Stops the motors immediately.
    fn stop(&mut self) -> Result<(), Self::Error>;
    /// Retrieves the current status of the motors, including whether they are stopped or motoring and their current speeds.
    fn get_status(&self) -> Result<MotorStatus, Self::Error>;
    /// Puts the motor driver into sleep mode, reducing power consumption. The caller is responsible for ensuring that the motor driver is properly configured to wake up from sleep mode when needed.
    fn sleep(&mut self) -> Result<(), Self::Error>;
}

pub trait BatteryCharger {
    type Error: fmt::Debug;

    /// Prepares the charger for monitoring: identifies the part and starts its ADC. Must be
    /// called before [`BatteryCharger::read_status`] returns anything meaningful.
    async fn init(&mut self) -> Result<(), Self::Error>;
    /// Samples everything the charger knows about the pack and the input in one pass.
    async fn read_status(&mut self) -> Result<ChargerStatus, Self::Error>;
}

/// What the network page shows. Values that are not available yet render as placeholders.
pub struct NetworkPage<'a> {
    pub device_id: &'a str,
    pub ssid: &'a str,
    /// `None` while the link is down or DHCP has not returned a lease.
    pub ip_address: Option<&'a str>,
    pub broker_status: &'a str,
    pub firmware_version: &'a str,
}

/// What the battery page shows. `status` is `None` until the charger answers for the first
/// time, e.g. while it is still being probed or if it is not responding at all.
pub struct BatteryPage<'a> {
    pub status: Option<&'a ChargerStatus>,
    pub firmware_version: &'a str,
}

pub trait DisplayController {
    type Error: fmt::Debug;

    /// Initializes the display. This method should be called before any other display operations.
    async fn init(&mut self) -> Result<(), Self::Error>;
    /// Clears the display, removing all content.
    async fn clear(&mut self) -> Result<(), Self::Error>;
    /// Draws a string of text on the display at the specified coordinates (x, y).
    async fn draw_text(&mut self, x: u32, y: u32, text: &str) -> Result<(), Self::Error>;
    /// Draws the network page -- Wi-Fi link, address and broker session -- in a single
    /// framebuffer update.
    async fn draw_network_page(&mut self, page: &NetworkPage<'_>) -> Result<(), Self::Error>;
    /// Draws the battery page -- charge, voltages and charger health -- in a single
    /// framebuffer update.
    async fn draw_battery_page(&mut self, page: &BatteryPage<'_>) -> Result<(), Self::Error>;
    /// Draws the firmware-update screen, taking the display over for the duration of an
    /// OTA update. `message` is word-wrapped to fit. `percent` draws a progress bar when
    /// the transfer size is known; pass `None` for an indeterminate update.
    async fn draw_firmware_update(
        &mut self,
        message: &str,
        percent: Option<u8>,
    ) -> Result<(), Self::Error>;
    /// Draws a full-screen notice: `title` centered above a word-wrapped `message`. Used
    /// for states that take the panel over without any progress to report, such as the
    /// board powering off.
    async fn draw_notice(&mut self, title: &str, message: &str) -> Result<(), Self::Error>;
}
