use embedded_hal::{
    digital::{ErrorType as DigitalErrorType, OutputPin},
    pwm::{ErrorType as PwmErrorType, SetDutyCycle},
};

use crate::{drivers::motor_driver::types, traits};

pub mod motor_driver;
pub mod ota_driver;

#[derive(Debug)]
pub struct MotorDriverError<PwmError, SleepError> {
    pub operation: MotorDriverOperation,
    pub source: MotorDriverErrorSource<PwmError, SleepError>,
}

impl<PwmError, SleepError> MotorDriverError<PwmError, SleepError> {
    fn pwm(operation: MotorDriverOperation, source: PwmError) -> Self {
        Self {
            operation,
            source: MotorDriverErrorSource::Pwm(source),
        }
    }

    fn sleep(operation: MotorDriverOperation, source: SleepError) -> Self {
        Self {
            operation,
            source: MotorDriverErrorSource::Sleep(source),
        }
    }
}

#[derive(Debug)]
pub enum MotorDriverErrorSource<PwmError, SleepError> {
    Pwm(PwmError),
    Sleep(SleepError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorDriverOperation {
    LeftForwardPwm,
    LeftReversePwm,
    RightForwardPwm,
    RightReversePwm,
    Wake,
    Sleep,
}

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

enum Side {
    Left,
    Right,
}

impl<AIN1, AIN2, BIN1, BIN2, SLEEP> MotorDriver<AIN1, AIN2, BIN1, BIN2, SLEEP>
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

    fn normalized_speed(speed: f32) -> f32 {
        if speed.is_finite() {
            speed.clamp(-1.0, 1.0)
        } else {
            0.0
        }
    }

    fn speed_limit(&self) -> f32 {
        if self.config.max_speed.is_finite() {
            self.config.max_speed.clamp(0.0, 1.0)
        } else {
            1.0
        }
    }

    fn duty_cycle_for_speed(speed: f32, max_speed: f32, max_duty_cycle: u16) -> u16 {
        let duty = speed.abs() * max_speed * f32::from(max_duty_cycle);

        if duty >= f32::from(max_duty_cycle) {
            max_duty_cycle
        } else {
            duty as u16
        }
    }

    fn set_motor_outputs<FORWARD, REVERSE>(
        forward: &mut FORWARD,
        reverse: &mut REVERSE,
        speed: f32,
        max_speed: f32,
        forward_operation: MotorDriverOperation,
        reverse_operation: MotorDriverOperation,
    ) -> Result<
        (),
        MotorDriverError<<FORWARD as PwmErrorType>::Error, <SLEEP as DigitalErrorType>::Error>,
    >
    where
        FORWARD: SetDutyCycle,
        REVERSE: SetDutyCycle<Error = <FORWARD as PwmErrorType>::Error>,
    {
        if speed > 0.0 {
            let duty_cycle = Self::duty_cycle_for_speed(speed, max_speed, forward.max_duty_cycle());
            forward
                .set_duty_cycle(duty_cycle)
                .map_err(|source| MotorDriverError::pwm(forward_operation, source))?;
            reverse
                .set_duty_cycle_fully_off()
                .map_err(|source| MotorDriverError::pwm(reverse_operation, source))?;
        } else if speed < 0.0 {
            let duty_cycle = Self::duty_cycle_for_speed(speed, max_speed, reverse.max_duty_cycle());
            forward
                .set_duty_cycle_fully_off()
                .map_err(|source| MotorDriverError::pwm(forward_operation, source))?;
            reverse
                .set_duty_cycle(duty_cycle)
                .map_err(|source| MotorDriverError::pwm(reverse_operation, source))?;
        } else {
            forward
                .set_duty_cycle_fully_off()
                .map_err(|source| MotorDriverError::pwm(forward_operation, source))?;
            reverse
                .set_duty_cycle_fully_off()
                .map_err(|source| MotorDriverError::pwm(reverse_operation, source))?;
        }

        Ok(())
    }

    fn set_side_motor_speed(
        &mut self,
        side: Side,
        speed: f32,
    ) -> Result<
        (),
        MotorDriverError<<AIN1 as PwmErrorType>::Error, <SLEEP as DigitalErrorType>::Error>,
    > {
        let speed = Self::normalized_speed(speed);
        let max_speed = self.speed_limit();

        match side {
            Side::Left => Self::set_motor_outputs(
                &mut self.ain1,
                &mut self.ain2,
                speed,
                max_speed,
                MotorDriverOperation::LeftForwardPwm,
                MotorDriverOperation::LeftReversePwm,
            )?,
            Side::Right => Self::set_motor_outputs(
                &mut self.bin1,
                &mut self.bin2,
                speed,
                max_speed,
                MotorDriverOperation::RightForwardPwm,
                MotorDriverOperation::RightReversePwm,
            )?,
        }

        Ok(())
    }
}

impl<AIN1, AIN2, BIN1, BIN2, SLEEP> traits::MotorController
    for MotorDriver<AIN1, AIN2, BIN1, BIN2, SLEEP>
where
    AIN1: SetDutyCycle,
    AIN2: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    BIN1: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    BIN2: SetDutyCycle<Error = <AIN1 as PwmErrorType>::Error>,
    SLEEP: OutputPin,
{
    type Error =
        MotorDriverError<<AIN1 as PwmErrorType>::Error, <SLEEP as DigitalErrorType>::Error>;

    fn set_speed(&mut self, left_speed: f32, right_speed: f32) -> Result<(), Self::Error> {
        let left_speed = Self::normalized_speed(left_speed);
        let right_speed = Self::normalized_speed(right_speed);

        if left_speed == 0.0 && right_speed == 0.0 {
            self.stop()?;
            return Ok(());
        }

        self.sleep
            .set_high()
            .map_err(|source| MotorDriverError::sleep(MotorDriverOperation::Wake, source))?;
        self.set_side_motor_speed(Side::Left, left_speed)?;
        self.set_side_motor_speed(Side::Right, right_speed)?;
        self.left_speed = left_speed;
        self.right_speed = right_speed;

        Ok(())
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        self.set_side_motor_speed(Side::Left, 0.0)?;
        self.set_side_motor_speed(Side::Right, 0.0)?;
        self.sleep
            .set_low()
            .map_err(|source| MotorDriverError::sleep(MotorDriverOperation::Sleep, source))?;
        self.left_speed = 0.0;
        self.right_speed = 0.0;

        Ok(())
    }

    fn get_status(&self) -> Result<traits::MotorStatus, Self::Error> {
        if self.left_speed == 0.0 && self.right_speed == 0.0 {
            Ok(traits::MotorStatus::Stopped)
        } else {
            Ok(traits::MotorStatus::Motoring {
                left_speed: self.left_speed,
                right_speed: self.right_speed,
            })
        }
    }
}
