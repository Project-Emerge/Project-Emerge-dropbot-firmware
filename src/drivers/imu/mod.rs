//! The board's nine-axis inertial stack.
//!
//! There is no single nine-axis part on the board: a BMI270 provides the accelerometer and
//! the gyroscope, an MMC5983MA the magnetometer, and both sit on the shared host I2C bus.
//! [`Imu9Axis`] is what makes them look like one sensor to the rest of the firmware -- it
//! samples both in a single [`Imu::read`] and hands back one [`ImuSample`].
//!
//! Everything here reports what the sensors measured, converted to physical units and
//! nothing more. Filtering, bias removal and the attitude estimate live in
//! [`crate::data::imu`], which is what keeps the raw readings available to publish alongside
//! the filtered ones.

mod bmi270;
mod mmc5983ma;

use embedded_hal_async::i2c::I2c;

use bmi270::{Bmi270, Bmi270Error};
use mmc5983ma::{Mmc5983ma, Mmc5983maError};

use crate::data::imu::{ImuSample, Vec3};
use crate::traits::Imu;

/// How each sensor's own axes sit in the board frame -- x forward, y left, z up.
///
/// Neither part is guaranteed to be placed the way that frame runs, and the two are separate
/// chips, so they can disagree with the board and with each other. This is the one place that
/// discrepancy belongs: correct it as the readings come off the sensors and everything
/// downstream sees a single consistent frame. Any correction must be a rotation rather than a
/// mirror, so negate an even number of axes -- negating one or three flips the handedness of
/// the frame, which silently reverses every cross product the filter takes.
///
/// The BMI270 needs nothing. This was briefly believed to be a half turn about the board's
/// left axis, on the strength of a bench reading of -9.81 m/s² on z with the board flat. That
/// reading was real but it was not the mounting: the attitude estimate was resting upside down
/// because the gravity correction in `data::imu` had its operands the wrong way round, and
/// every symptom that suggested a mounting was that bug seen from a different angle. Worth
/// keeping in mind here -- a signed permutation can absorb the symptoms of a sign error
/// elsewhere and look like it explains them. Check that the plain readings make physical sense
/// before concluding anything about how a part is placed.
fn inertial_to_board(sensor: Vec3) -> Vec3 {
    sensor
}

/// The magnetometer is unresolved, and passing it through is a placeholder rather than a
/// finding. One thing is known: on a level board at rest the part reports its z as *positive*,
/// a field coming up out of the ground, where in the northern hemisphere it dips downwards. So
/// its z runs opposite to the board's -- but a lone negation is a reflection, so a second axis
/// must go with it, and which one is not decidable from a board standing still. It takes one
/// reading with the nose pointed at magnetic north.
fn magnetic_to_board(sensor: Vec3) -> Vec3 {
    sensor
}

#[derive(Debug)]
pub enum ImuError<E> {
    /// The accelerometer/gyroscope half failed.
    Inertial(Bmi270Error<E>),
    /// The magnetometer half failed.
    Magnetic(Mmc5983maError<E>),
}

impl<E> From<Bmi270Error<E>> for ImuError<E> {
    fn from(error: Bmi270Error<E>) -> Self {
        Self::Inertial(error)
    }
}

impl<E> From<Mmc5983maError<E>> for ImuError<E> {
    fn from(error: Mmc5983maError<E>) -> Self {
        Self::Magnetic(error)
    }
}

pub struct Imu9Axis<I2C> {
    inertial: Bmi270<I2C>,
    magnetic: Mmc5983ma<I2C>,
}

impl<I2C: I2c> Imu9Axis<I2C> {
    /// Takes one bus handle per part rather than one for both: the two are addressed
    /// independently and each driver owns its own transactions, so sharing a single handle
    /// would only add a layer of borrowing without saving anything -- the bus itself is
    /// already shared, one transaction at a time, by the handles' mutex.
    pub fn new(inertial_bus: I2C, magnetic_bus: I2C) -> Self {
        Self {
            inertial: Bmi270::new(inertial_bus),
            magnetic: Mmc5983ma::new(magnetic_bus),
        }
    }
}

impl<I2C: I2c> Imu for Imu9Axis<I2C> {
    type Error = ImuError<I2C::Error>;

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.inertial.init().await?;
        self.magnetic.init().await?;
        Ok(())
    }

    async fn read(&mut self) -> Result<ImuSample, Self::Error> {
        let inertial = self.inertial.read().await?;
        let magnetometer = self.magnetic.read().await?;

        Ok(ImuSample {
            accelerometer: inertial_to_board(inertial.accelerometer),
            gyroscope: inertial_to_board(inertial.gyroscope),
            magnetometer: magnetic_to_board(magnetometer),
            temperature: inertial.temperature,
        })
    }
}
