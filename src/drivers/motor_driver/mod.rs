use embedded_hal::{
    digital::{ErrorType as DigitalErrorType, OutputPin},
    pwm::{ErrorType as PwmErrorType, SetDutyCycle},
};

use crate::{drivers::motor_driver::types::MotorConfig, traits::MotorController};

pub mod types;

#[derive(Debug)]
pub enum DRV8833Error<DigitalError, PwmError> {
    PwmError(PwmError),
    DigitalError(DigitalError),
}

pub struct DRV8833Driver<AIN1, AIN2, BIN1, BIN2, SLEEP> {
    // Pins for controlling right motor
    ain1: AIN1,
    ain2: AIN2,
    // Pins for controlling left motor
    bin1: BIN1,
    bin2: BIN2,
    // Pin for sleep mode
    sleep: SLEEP,
    // Configuration for the motor driver
    config: MotorConfig,
    // Speed state for left and right motors
    left_speed: f32,
    right_speed: f32,
}

enum Side {
    Left,
    Right,
}

impl<AIN1, AIN2, BIN1, BIN2, SLEEP> DRV8833Driver<AIN1, AIN2, BIN1, BIN2, SLEEP>
where
    AIN1: SetDutyCycle,
    AIN2: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    BIN1: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    BIN2: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    SLEEP: OutputPin,
{
    pub fn new(
        ain1: AIN1,
        ain2: AIN2,
        bin1: BIN1,
        bin2: BIN2,
        sleep: SLEEP,
        config: MotorConfig,
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

    fn set_motor_output<
        IN1: SetDutyCycle,
        IN2: SetDutyCycle<Error = <IN1 as PwmErrorType>::Error>,
    >(
        speed: f32,
        in1_pin: &mut IN1,
        in2_pin: &mut IN2,
    ) -> Result<(), <IN1 as PwmErrorType>::Error> {
        if speed > 0.0 {
            in1_pin.set_duty_cycle_percent((speed * 100.0) as u8)?;
            in2_pin.set_duty_cycle_fully_off()?;
        } else if speed < 0.0 {
            in1_pin.set_duty_cycle_fully_off()?;
            in2_pin.set_duty_cycle_percent((-speed * 100.0) as u8)?;
        } else {
            in1_pin.set_duty_cycle_percent(0)?;
            in2_pin.set_duty_cycle_percent(0)?;
        }
        Ok(())
    }

    fn set_speed_side(
        &mut self,
        side: Side,
        speed: f32,
    ) -> Result<(), DRV8833Error<<SLEEP as DigitalErrorType>::Error, <AIN1 as PwmErrorType>::Error>>
    {
        match side {
            Side::Left => {
                self.left_speed = speed;
                Self::set_motor_output(speed, &mut self.bin1, &mut self.bin2)
                    .map_err(DRV8833Error::PwmError)?;
            }
            Side::Right => {
                self.right_speed = speed;
                Self::set_motor_output(speed, &mut self.ain1, &mut self.ain2)
                    .map_err(DRV8833Error::PwmError)?;
            }
        }
        Ok(())
    }
}

impl<AIN1, AIN2, BIN1, BIN2, SLEEP> MotorController for DRV8833Driver<AIN1, AIN2, BIN1, BIN2, SLEEP>
where
    AIN1: SetDutyCycle,
    AIN2: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    BIN1: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    BIN2: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    SLEEP: OutputPin,
{
    type Error = DRV8833Error<<SLEEP as DigitalErrorType>::Error, <AIN1 as PwmErrorType>::Error>;

    fn set_speed(&mut self, left_speed: f32, right_speed: f32) -> Result<(), Self::Error> {
        let (left_speed, right_speed) = match self.config.ema_filter_alpha {
            Some(alpha) => {
                let left_speed = self.left_speed + alpha * (left_speed - self.left_speed);
                let right_speed = self.right_speed + alpha * (right_speed - self.right_speed);
                (left_speed, right_speed)
            }
            None => (left_speed, right_speed),
        };
        self.sleep.set_high().map_err(DRV8833Error::DigitalError)?;
        self.set_speed_side(Side::Left, left_speed)?;
        self.set_speed_side(Side::Right, right_speed)?;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.sleep.set_high().map_err(DRV8833Error::DigitalError)?;
        self.set_speed_side(Side::Left, 0.0)?;
        self.set_speed_side(Side::Right, 0.0)?;
        Ok(())
    }

    fn get_status(&self) -> Result<crate::traits::MotorStatus, Self::Error> {
        if self.left_speed == 0.0 && self.right_speed == 0.0 {
            Ok(crate::traits::MotorStatus::Stopped)
        } else {
            Ok(crate::traits::MotorStatus::Motoring {
                left: self.left_speed,
                right: self.right_speed,
            })
        }
    }

    fn sleep(&mut self) -> Result<(), Self::Error> {
        self.sleep.set_low().map_err(DRV8833Error::DigitalError)?;
        Ok(())
    }
}
