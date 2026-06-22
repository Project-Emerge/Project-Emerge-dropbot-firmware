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
