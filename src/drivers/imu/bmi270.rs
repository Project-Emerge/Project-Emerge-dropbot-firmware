//! Bosch BMI270 accelerometer + gyroscope, over I2C.
//!
//! Written against the datasheet rather than pulled in as a dependency: the two BMI270 crates
//! on crates.io are `bmi2`, which is blocking-only, and `bmi270`, which is built on
//! embedded-hal 0.2 and drags in `linux-embedded-hal`. This firmware's I2C bus is async and
//! shared behind a mutex, so neither can drive it.
//!
//! Only the accelerometer, the gyroscope and the die temperature are used. The part also has
//! a "feature engine" -- step counting, wrist gestures, any-motion -- which runs firmware the
//! host uploads after every power-on; none of that is used here, but the upload happens
//! anyway because the datasheet's initialization sequence is not complete without it and the
//! chip refuses to report `init_ok` until it has run.

use ariel_os::time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

use crate::data::imu::Vec3;

/// The address the part answers on with SDO tied low.
const ADDRESS_PRIMARY: u8 = 0x68;
/// ...and with SDO tied high. Which one the board uses is a strapping choice this driver does
/// not need to know: [`Bmi270::init`] tries both.
const ADDRESS_SECONDARY: u8 = 0x69;

/// What `CHIP_ID` reads on a BMI270.
const CHIP_ID: u8 = 0x24;

mod register {
    pub const CHIP_ID: u8 = 0x00;
    /// First of the six accelerometer bytes; the gyroscope's six follow it, and the
    /// temperature pair sits sixteen registers further on. See [`super::BURST_LEN`].
    pub const DATA_8: u8 = 0x0C;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const INIT_CTRL: u8 = 0x59;
    pub const INIT_ADDR_0: u8 = 0x5B;
    pub const INIT_ADDR_1: u8 = 0x5C;
    pub const INIT_DATA: u8 = 0x5E;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const CMD: u8 = 0x7E;
}

/// `CMD` value that restores every register to its power-on default.
const CMD_SOFT_RESET: u8 = 0xB6;

/// Low nibble of `INTERNAL_STATUS`, the field that reports how the feature-engine upload went.
const INTERNAL_STATUS_MESSAGE: u8 = 0x0F;
/// The one value of that field that means the upload took.
const INTERNAL_STATUS_INIT_OK: u8 = 0x01;

/// `PWR_CTRL`: run the accelerometer, the gyroscope and the temperature sensor. The auxiliary
/// (magnetometer pass-through) interface stays off -- the MMC5983MA hangs off the same host
/// bus as the BMI270, not off its auxiliary port.
const PWR_CTRL_ENABLE_ALL: u8 = 0b0000_1110;

/// `PWR_CONF`: advanced power save off, FIFO self-wake-up on. Power save gates the register
/// interface behind a wake-up, which would stall every poll; the board is awake and driving
/// motors whenever this runs, so there is nothing to save.
const PWR_CONF_PERFORMANCE: u8 = 0b0000_0010;

/// `ACC_CONF`: performance filter, `osr4_avg1` bandwidth, 200 Hz output.
///
/// The output rate is twice what `tasks::imu_monitor` polls at, so every poll gets a sample
/// at most one period old. The bandwidth setting is the part that matters for the
/// dead-reckoning this feeds: `osr4` puts the on-chip filter's corner at about 40 Hz, below
/// the 50 Hz Nyquist limit of the firmware's own 100 Hz sampling. Motor vibration above that
/// is attenuated here, where it can be, rather than folded down onto the signal band where
/// no later filter could tell it apart.
const ACC_CONF_200HZ: u8 = 0b1000_1001;

/// `GYR_CONF`: performance filter, performance noise mode, `osr4` bandwidth, 200 Hz. Same
/// reasoning as [`ACC_CONF_200HZ`], plus the low-noise mode: the gyroscope is what carries
/// heading between magnetometer corrections, and its noise integrates into that heading.
const GYR_CONF_200HZ: u8 = 0b1100_1001;

/// Bytes of the feature-engine image pushed per `INIT_DATA` burst. Any even chunk size works;
/// this one keeps the transaction buffer on the stack small while still cutting the upload to
/// 64 transactions.
const CONFIG_CHUNK: usize = 128;

/// Bosch's feature-engine firmware image, taken verbatim from `bmi270_config_file[]` in
/// <https://github.com/boschsensortec/BMI270_SensorAPI> (BSD-3-Clause, (c) Bosch Sensortec).
///
/// The part has no non-volatile store for it, so it is re-uploaded on every init.
const CONFIG_FILE: &[u8] = include_bytes!("bmi270_config_file.bin");

/// One burst covers the twelve data bytes at `DATA_8` and the temperature pair at
/// `TEMPERATURE_0`; the status registers in between are read and discarded. Cheaper on a bus
/// shared with the display than issuing two transactions.
const BURST_LEN: usize = 24;
/// Offset of `TEMPERATURE_0` within that burst.
const TEMPERATURE_OFFSET: usize = 22;
/// What the temperature registers read when the sensor has no valid measurement.
const TEMPERATURE_INVALID: i16 = -32768;
/// Counts per degree Celsius, and the offset the count is relative to.
const TEMPERATURE_LSB_PER_DEGREE: f32 = 512.0;
const TEMPERATURE_OFFSET_DEGREES: f32 = 23.0;

/// Standard gravity, used to convert the accelerometer's g-relative counts to m/s².
const GRAVITY: f32 = 9.806_65;

/// Full-scale span of a 16-bit signed reading.
const FULL_SCALE: f32 = 32768.0;

/// `ACC_RANGE`: ±4 g. Enough resolution to be useful for attitude while leaving headroom for
/// the knocks a floor robot takes.
const ACC_RANGE_4G: u8 = 0b01;
const ACCEL_RANGE_G: f32 = 4.0;
/// m/s² per count.
const ACCEL_SCALE: f32 = ACCEL_RANGE_G * GRAVITY / FULL_SCALE;

/// `GYR_RANGE`: ±1000 °/s, i.e. nearly three turns a second.
///
/// Two competing failures. Saturating the gyroscope loses heading outright, which argues for
/// the widest range; but heading is integrated, so quantization accumulates, which argues for
/// the narrowest. ±1000 °/s puts the step at 0.031 °/s -- comfortably under the part's own
/// ~0.07 °/s of noise, so quantization stops being the limiting error -- while still leaving
/// several times the margin over anything a differential drive of this size can spin at.
const GYR_RANGE_1000DPS: u8 = 0b001;
const GYRO_RANGE_DPS: f32 = 1000.0;
/// Degrees per second per count.
const GYRO_SCALE: f32 = GYRO_RANGE_DPS / FULL_SCALE;

#[derive(Debug)]
pub enum Bmi270Error<E> {
    Bus(E),
    /// Nothing answered on either of the part's two addresses.
    NotFound,
    /// Something answered but did not identify as a BMI270.
    WrongChipId(u8),
    /// The feature-engine upload did not take; carries `INTERNAL_STATUS`.
    InitFailed(u8),
}

/// One pass of the inertial half of the IMU.
pub struct Bmi270Sample {
    /// Proper acceleration in m/s², i.e. including the ~9.81 m/s² the board feels at rest.
    pub accelerometer: Vec3,
    /// Angular rate in degrees per second.
    pub gyroscope: Vec3,
    /// Die temperature in °C. `None` when the sensor reports its invalid code, which it does
    /// for the first few milliseconds after the temperature sensor is enabled.
    pub temperature: Option<f32>,
}

pub struct Bmi270<I2C> {
    i2c: I2C,
    /// Settled by [`Bmi270::init`]; the value before that is only a starting guess.
    address: u8,
}

impl<I2C: I2c> Bmi270<I2C> {
    pub const fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: ADDRESS_PRIMARY,
        }
    }

    /// Brings the part up: locates it, resets it, uploads the feature-engine image and starts
    /// the accelerometer, gyroscope and temperature sensor.
    ///
    /// Safe to call again on a part that stopped answering; it starts from a soft reset, so a
    /// chip that browned out and came back is reconfigured from scratch.
    pub async fn init(&mut self) -> Result<(), Bmi270Error<I2C::Error>> {
        self.address = self.probe().await?;

        self.write_register(register::CMD, CMD_SOFT_RESET).await?;
        Timer::after(Duration::from_millis(5)).await;

        // The upload only lands with advanced power save off, and the datasheet asks for
        // 450 µs of settling between disabling it and touching `INIT_CTRL`.
        self.write_register(register::PWR_CONF, 0x00).await?;
        Timer::after(Duration::from_micros(500)).await;

        // `INIT_CTRL` low means "an upload is in progress"; the rising edge afterwards is
        // what starts the feature engine.
        self.write_register(register::INIT_CTRL, 0x00).await?;
        self.upload_config_file().await?;
        self.write_register(register::INIT_CTRL, 0x01).await?;

        // The datasheet's floor is 20 ms. The margin costs nothing -- this runs once, at
        // startup -- and covers a slow bus on which the upload itself is still draining.
        Timer::after(Duration::from_millis(150)).await;
        let status = self.read_register(register::INTERNAL_STATUS).await?;
        if status & INTERNAL_STATUS_MESSAGE != INTERNAL_STATUS_INIT_OK {
            return Err(Bmi270Error::InitFailed(status));
        }

        self.write_register(register::PWR_CTRL, PWR_CTRL_ENABLE_ALL)
            .await?;
        self.write_register(register::ACC_CONF, ACC_CONF_200HZ)
            .await?;
        self.write_register(register::ACC_RANGE, ACC_RANGE_4G)
            .await?;
        self.write_register(register::GYR_CONF, GYR_CONF_200HZ)
            .await?;
        self.write_register(register::GYR_RANGE, GYR_RANGE_1000DPS)
            .await?;
        self.write_register(register::PWR_CONF, PWR_CONF_PERFORMANCE)
            .await?;

        // The gyroscope needs ~45 ms to spin up; reads before that return whatever the
        // output registers were last left holding.
        Timer::after(Duration::from_millis(50)).await;

        Ok(())
    }

    /// Samples the accelerometer, the gyroscope and the die temperature in one transaction.
    pub async fn read(&mut self) -> Result<Bmi270Sample, Bmi270Error<I2C::Error>> {
        let mut buffer = [0u8; BURST_LEN];
        self.read_registers(register::DATA_8, &mut buffer).await?;

        Ok(Bmi270Sample {
            accelerometer: Vec3 {
                x: f32::from(word(&buffer, 0)) * ACCEL_SCALE,
                y: f32::from(word(&buffer, 2)) * ACCEL_SCALE,
                z: f32::from(word(&buffer, 4)) * ACCEL_SCALE,
            },
            gyroscope: Vec3 {
                x: f32::from(word(&buffer, 6)) * GYRO_SCALE,
                y: f32::from(word(&buffer, 8)) * GYRO_SCALE,
                z: f32::from(word(&buffer, 10)) * GYRO_SCALE,
            },
            temperature: match word(&buffer, TEMPERATURE_OFFSET) {
                TEMPERATURE_INVALID => None,
                raw => {
                    Some(f32::from(raw) / TEMPERATURE_LSB_PER_DEGREE + TEMPERATURE_OFFSET_DEGREES)
                }
            },
        })
    }

    /// Finds which of the two strapping addresses the part sits on.
    async fn probe(&mut self) -> Result<u8, Bmi270Error<I2C::Error>> {
        let mut last_id = None;

        for address in [ADDRESS_PRIMARY, ADDRESS_SECONDARY] {
            let mut id = [0u8; 1];
            // A missing device NACKs its address, which is a bus error rather than a wrong
            // ID: keep trying the other address instead of giving up here.
            if self
                .i2c
                .write_read(address, &[register::CHIP_ID], &mut id)
                .await
                .is_err()
            {
                continue;
            }

            if id[0] == CHIP_ID {
                return Ok(address);
            }
            last_id = Some(id[0]);
        }

        // Something answering with the wrong ID is a different failure from nothing
        // answering at all -- the first means the board is not what this firmware expects,
        // the second that the part is absent or unpowered.
        match last_id {
            Some(id) => Err(Bmi270Error::WrongChipId(id)),
            None => Err(Bmi270Error::NotFound),
        }
    }

    /// Streams [`CONFIG_FILE`] into the part's feature-engine memory.
    async fn upload_config_file(&mut self) -> Result<(), Bmi270Error<I2C::Error>> {
        let mut buffer = [0u8; CONFIG_CHUNK + 1];
        buffer[0] = register::INIT_DATA;

        for (index, chunk) in CONFIG_FILE.chunks(CONFIG_CHUNK).enumerate() {
            // The write pointer is addressed in 16-bit words, split across two registers:
            // `INIT_ADDR_0` takes bits 3:0 and `INIT_ADDR_1` bits 11:4.
            let word_address = (index * CONFIG_CHUNK / 2) as u16;
            self.write_register(register::INIT_ADDR_0, (word_address & 0x0F) as u8)
                .await?;
            self.write_register(register::INIT_ADDR_1, ((word_address >> 4) & 0xFF) as u8)
                .await?;

            buffer[1..=chunk.len()].copy_from_slice(chunk);
            self.i2c
                .write(self.address, &buffer[..=chunk.len()])
                .await
                .map_err(Bmi270Error::Bus)?;
        }

        Ok(())
    }

    async fn write_register(
        &mut self,
        register: u8,
        value: u8,
    ) -> Result<(), Bmi270Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, value])
            .await
            .map_err(Bmi270Error::Bus)
    }

    async fn read_register(&mut self, register: u8) -> Result<u8, Bmi270Error<I2C::Error>> {
        let mut value = [0u8; 1];
        self.read_registers(register, &mut value).await?;
        Ok(value[0])
    }

    async fn read_registers(
        &mut self,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), Bmi270Error<I2C::Error>> {
        self.i2c
            .write_read(self.address, &[register], buffer)
            .await
            .map_err(Bmi270Error::Bus)
    }
}

/// Reads one little-endian signed 16-bit sample out of a burst.
fn word(buffer: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([buffer[offset], buffer[offset + 1]])
}
