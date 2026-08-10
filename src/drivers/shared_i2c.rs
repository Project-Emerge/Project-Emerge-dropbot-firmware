use ariel_os::hal;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embedded_hal_async::i2c::{ErrorType, I2c, Operation, SevenBitAddress};

/// The board's single I2C bus. The display and the battery charger both sit on it, so
/// neither can own it outright: it lives in a mutex and each task gets a
/// [`BoardI2cDevice`] handle.
pub type BoardI2cBus = Mutex<CriticalSectionRawMutex, hal::i2c::controller::I2c>;

/// A per-task handle onto [`BoardI2cBus`].
pub type BoardI2cDevice = SharedI2c<'static, CriticalSectionRawMutex, hal::i2c::controller::I2c>;

/// A handle onto an I2C bus shared between tasks.
///
/// Every transaction takes the mutex for its duration -- long enough to keep an
/// address/read pair atomic, short enough that a display flush only delays a charger poll
/// by a few tens of milliseconds.
///
/// ariel-os ships exactly this as `ariel_os::i2c::controller::I2cDevice`, but it is not
/// usable here: it is built on `embassy-sync` 0.7, while this firmware depends on 0.8, so
/// its mutex types are unrelated to ours and it would not accept our bus.
pub struct SharedI2c<'a, M: RawMutex, BUS> {
    bus: &'a Mutex<M, BUS>,
}

impl<'a, M: RawMutex, BUS> SharedI2c<'a, M, BUS> {
    pub const fn new(bus: &'a Mutex<M, BUS>) -> Self {
        Self { bus }
    }
}

impl<M: RawMutex, BUS: I2c> ErrorType for SharedI2c<'_, M, BUS> {
    type Error = BUS::Error;
}

impl<M: RawMutex, BUS: I2c> I2c for SharedI2c<'_, M, BUS> {
    async fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        // `read`, `write` and `write_read` are provided by the trait in terms of this one,
        // so holding the bus here covers all of them.
        self.bus.lock().await.transaction(address, operations).await
    }
}
