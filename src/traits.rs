use core::fmt;
use core::result::Result;

pub enum MotorStatus {
    Stopped,
    Motoring { left_speed: f32, right_speed: f32 },
}

trait MotorController {
    type Error: fmt::Debug;

    /// Sets the speed of the left and right motors.
    /// `left_speed` and `right_speed` should be in the range [-1.0, 1.0], where -1.0 is full reverse, 0.0 is stop, and 1.0 is full forward.
    fn set_speed(&mut self, left_speed: f32, right_speed: f32) -> Result<(), Self::Error>;

    /// Stops the motors immediately.
    fn stop(&mut self) -> Result<(), Self::Error>;

    /// Returns the current status of the motors.
    fn get_status(&self) -> Result<MotorStatus, Self::Error>;
}
