//! MEMSIC MMC5983MA three-axis magnetometer, over I2C.
//!
//! Written against the datasheet rather than pulled in as a dependency: the one async crate
//! for this part, `mmc5983ma`, is an unmaintained release candidate that `panic!`s on a wrong
//! product ID and on a measurement timeout, and whose 16-bit read path shifts a `u16` by 16.
//! A panic in a driver takes the whole firmware down, including the power latch, so it is not
//! usable here.
//!
//! The part is left in continuous measurement mode with automatic set/reset enabled: every
//! sample is taken as the difference of two measurements with the sense element magnetised in
//! opposite directions, which cancels the sensor's own offset and its drift with temperature.
//! What that does *not* cancel is the hard iron of the board around it -- the magnets in the
//! drive motors, most of all -- which is estimated at runtime by `data::imu::ImuFilter`.

use ariel_os::time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

use crate::data::imu::Vec3;

/// The part has no address strapping.
const ADDRESS: u8 = 0x30;

/// What `PRODUCT_ID` reads on an MMC5983MA.
const PRODUCT_ID: u8 = 0x30;

mod register {
    /// First of the seven measurement bytes: two per axis plus a shared byte holding the low
    /// two bits of all three.
    pub const X_OUT_0: u8 = 0x00;
    pub const CONTROL_0: u8 = 0x09;
    pub const CONTROL_1: u8 = 0x0A;
    pub const CONTROL_2: u8 = 0x0B;
    pub const PRODUCT_ID: u8 = 0x2F;
}

/// `CONTROL_0`: take every measurement as a set/reset pair. See the module docs.
const CONTROL_0_AUTO_SET_RESET: u8 = 0b0010_0000;

/// `CONTROL_1`: restore the power-on defaults.
const CONTROL_1_SOFT_RESET: u8 = 0b1000_0000;
/// `CONTROL_1`: 8 ms conversions (100 Hz bandwidth) with all three channels enabled. The
/// slowest, quietest setting the part has, and still twice the rate this firmware polls at.
const CONTROL_1_BANDWIDTH_100HZ: u8 = 0b0000_0000;

/// `CONTROL_2`: continuous measurement at 100 Hz, degaussing the sense element every 250
/// measurements. Automatic set/reset removes the offset from each sample; the periodic set on
/// top of it recovers the element from a field strong enough to have saturated it, which for
/// a robot that drives up to other robots' motors is a matter of when rather than if.
const CONTROL_2_CONTINUOUS_100HZ: u8 = 0b1100_1101;

/// Bytes in a measurement burst.
const BURST_LEN: usize = 7;

/// An 18-bit reading is unsigned and centred here, not on zero.
const ZERO_FIELD_COUNTS: f32 = 131_072.0;
/// Counts per gauss in 18-bit mode.
const COUNTS_PER_GAUSS: f32 = 16_384.0;
/// Microtesla per gauss.
const MICROTESLA_PER_GAUSS: f32 = 100.0;

// Field contents are only ever surfaced through the derived `Debug` impl (via `Debug2Format`
// in `imu_monitor`'s `error!` logs), which the dead-code lint doesn't credit as a read.
#[derive(Debug)]
#[allow(dead_code)]
pub enum Mmc5983maError<E> {
    Bus(E),
    /// Something answered at the part's address but did not identify as an MMC5983MA.
    WrongProductId(u8),
}

pub struct Mmc5983ma<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> Mmc5983ma<I2C> {
    pub const fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    /// Resets the part and starts continuous measurement.
    ///
    /// Reading the product ID first doubles as a presence check, so a magnetometer that is
    /// not on the bus fails here rather than reporting a fixed field forever.
    pub async fn init(&mut self) -> Result<(), Mmc5983maError<I2C::Error>> {
        let product_id = self.read_register(register::PRODUCT_ID).await?;
        if product_id != PRODUCT_ID {
            return Err(Mmc5983maError::WrongProductId(product_id));
        }

        self.write_register(register::CONTROL_1, CONTROL_1_SOFT_RESET)
            .await?;
        // The datasheet gives 10 ms for the part to come back up.
        Timer::after(Duration::from_millis(15)).await;

        self.write_register(register::CONTROL_1, CONTROL_1_BANDWIDTH_100HZ)
            .await?;
        self.write_register(register::CONTROL_0, CONTROL_0_AUTO_SET_RESET)
            .await?;
        self.write_register(register::CONTROL_2, CONTROL_2_CONTINUOUS_100HZ)
            .await?;

        // Let the first conversion land, so the caller's first read is a measurement rather
        // than the reset value of the output registers.
        Timer::after(Duration::from_millis(20)).await;

        Ok(())
    }

    /// Reads the most recent measurement, in microtesla.
    ///
    /// In continuous mode the output registers always hold a complete sample, so this does
    /// not wait on the data-ready flag: polling slower than the 100 Hz measurement rate, the
    /// worst case is reading a sample up to one period old.
    pub async fn read(&mut self) -> Result<Vec3, Mmc5983maError<I2C::Error>> {
        let mut buffer = [0u8; BURST_LEN];
        self.i2c
            .write_read(ADDRESS, &[register::X_OUT_0], &mut buffer)
            .await
            .map_err(Mmc5983maError::Bus)?;

        // Each axis is spread over three registers: bits 17:10, bits 9:2, and -- shared with
        // the other two axes in the last byte -- bits 1:0.
        let axis = |high: usize, low: usize, shift: u8| {
            (u32::from(buffer[high]) << 10)
                | (u32::from(buffer[low]) << 2)
                | u32::from((buffer[6] >> shift) & 0b11)
        };

        Ok(Vec3 {
            x: microtesla(axis(0, 1, 6)),
            y: microtesla(axis(2, 3, 4)),
            z: microtesla(axis(4, 5, 2)),
        })
    }

    async fn write_register(
        &mut self,
        register: u8,
        value: u8,
    ) -> Result<(), Mmc5983maError<I2C::Error>> {
        self.i2c
            .write(ADDRESS, &[register, value])
            .await
            .map_err(Mmc5983maError::Bus)
    }

    async fn read_register(&mut self, register: u8) -> Result<u8, Mmc5983maError<I2C::Error>> {
        let mut value = [0u8; 1];
        self.i2c
            .write_read(ADDRESS, &[register], &mut value)
            .await
            .map_err(Mmc5983maError::Bus)?;
        Ok(value[0])
    }
}

fn microtesla(counts: u32) -> f32 {
    (counts as f32 - ZERO_FIELD_COUNTS) / COUNTS_PER_GAUSS * MICROTESLA_PER_GAUSS
}
