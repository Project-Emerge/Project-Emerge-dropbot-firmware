use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};

use crate::drivers::motor_driver::types;

pub mod motor_driver;

pub struct MotorDriver<AIN1, AIN2, BIN1, BIN2, SLEEP> {
    // Pins for controlling the left motor
    ain1: AIN1,
    ain2: AIN2,
    // Pins for controlling the right motor
    bin1: BIN1,
    bin2: BIN2,
    // Pin for putting the motors to sleep
    sleep: SLEEP,
    config: types::MotorConfig,
    left_speed: f32,
    right_speed: f32,
}

impl<AIN1, AIN2, BIN1, BIN2, SLEEP> MotorDriver<AIN1, AIN2, BIN1, BIN2, SLEEP>
where
    AIN1: SetDutyCycle,
    AIN2: SetDutyCycle,
    BIN1: SetDutyCycle,
    BIN2: SetDutyCycle,
    SLEEP: OutputPin,
{
    pub fn new(
        ain1: AIN1,
        ain2: AIN2,
        bin1: BIN1,
        bin2: BIN2,
        sleep: SLEEP,
        config: types::MotorConfig,
    ) -> Self {
        Self {
            ain1,
            ain2,
            bin1,
            bin2,
            sleep,
            config,
            left_speed: 0.0,
            right_speed: 0.0,
        }
    }
}
