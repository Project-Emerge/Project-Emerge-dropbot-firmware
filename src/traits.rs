use core::fmt;

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

pub trait DisplayController {
    type Error: fmt::Debug;

    /// Initializes the display. This method should be called before any other display operations.
    async fn init(&mut self) -> Result<(), Self::Error>;
    /// Clears the display, removing all content.
    async fn clear(&mut self) -> Result<(), Self::Error>;
    /// Draws a string of text on the display at the specified coordinates (x, y).
    async fn draw_text(&mut self, x: u32, y: u32, text: &str) -> Result<(), Self::Error>;
    /// Draws the complete status mask in a single framebuffer update.
    async fn draw_status(
        &mut self,
        ip_address: &str,
        rssi_dbm: Option<i32>,
        network_connected: bool,
    ) -> Result<(), Self::Error>;
}
