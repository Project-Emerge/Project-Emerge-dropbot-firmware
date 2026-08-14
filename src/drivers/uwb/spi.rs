//! Builds the SPI link to the DW3000, pulses its reset line, and re-claims both after a bring-up
//! attempt that could not be recovered.
//!
//! Deliberately bypasses ariel-os's own SPI wrapper (`ariel_os::hal::spi`/the `spi` laze
//! feature): that feature's `init()` takes and drops `SPI2` unconditionally
//! (`ariel-os-esp/src/spi/mod.rs`), and `ariel-os-hal` doesn't even re-export a `spi` module, so
//! the two are mutually exclusive with claiming `SPI2` here. Going straight to `esp_hal` also
//! buys the runtime `apply_config` the two-stage clock trick below needs, which ariel-os's
//! wrapper (`YieldingAsync<BlockingAsync<Spi<Blocking>>>`) has no way to expose.

use ariel_os::gpio::{Input, Level, Output, Pull};
use ariel_os::hal::peripherals::{GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7, GPIO15, SPI2};
use ariel_os::time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::Async;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;

/// The DW3000 only tolerates up to 7 MHz SPI while its clock is still running off INIT_RC, so
/// bring-up starts here.
const INIT_FREQUENCY: Rate = Rate::from_mhz(5);
/// Once `init()`/`config()` have locked the clock PLL, the part accepts up to 38 MHz.
/// Everything on the ranging critical path is register and frame-buffer traffic, so this is the
/// single biggest lever on the achievable update rate -- see [`raise_frequency`].
const RUNNING_FREQUENCY: Rate = Rate::from_mhz(20);

pub type UwbSpiBus = Spi<'static, Async>;
/// CS is managed here, as a plain push-pull `Output` driven by `embedded-hal-bus`, rather than
/// by the SPI peripheral's own hardware CS line -- matching the reference tag firmware, and
/// simpler than threading a second driver-owned pin through `esp_hal::spi::master::Config`.
pub type UwbSpiDevice = ExclusiveDevice<UwbSpiBus, Output<'static>, Delay>;

/// The peripherals one bring-up attempt needs, bundled so a retry can hand back a fresh set
/// without the caller juggling six separate arguments.
pub struct UwbPeripherals {
    pub rst: GPIO15<'static>,
    pub sck: GPIO4<'static>,
    pub mosi: GPIO6<'static>,
    pub miso: GPIO5<'static>,
    pub cs: GPIO7<'static>,
    pub spi: SPI2<'static>,
}

impl UwbPeripherals {
    /// Re-claims the peripherals a bring-up attempt needs.
    ///
    /// `dw3000_ng::DW3000<SPI, State>` never hands the SPI device back out on a failure path --
    /// not from `init()`/`config()` on a bad handshake, and not from `finish_sending`/
    /// `finish_receiving` on a stuck typestate (its own `ll` field is private) -- so recovering
    /// from any such failure means going back to the underlying peripheral singletons for a
    /// fresh reset and a fresh `Spi`/CS/RSTn, not reusing anything from the failed attempt.
    ///
    /// # Safety
    ///
    /// Sound exactly because that is true: the one legitimate instance of each of these
    /// peripherals -- originally handed to the caller via [`crate::pins::UwbPins`] -- has
    /// already been dropped along with the failed attempt by the time this runs. This mirrors
    /// the same "steal a singleton back after its driver was dropped" pattern `ariel-os-esp`'s
    /// own SPI driver uses internally (`ariel-os-esp/src/spi/main/mod.rs`), for the same reason:
    /// the driver, not the peripheral, is what Rust's ownership tracked, and the driver is gone.
    pub unsafe fn steal_for_retry() -> Self {
        unsafe {
            Self {
                rst: GPIO15::steal(),
                sck: GPIO4::steal(),
                mosi: GPIO6::steal(),
                miso: GPIO5::steal(),
                cs: GPIO7::steal(),
                spi: SPI2::steal(),
            }
        }
    }
}

/// Pulses reset and builds the SPI link at [`INIT_FREQUENCY`], ready for
/// `drivers::uwb::bring_up`.
///
/// Returns the reset pin too, now a floating `Input`: hold onto it for as long as the returned
/// `UwbSpiDevice` (or anything built from it) is in use, so nothing else drives `RSTn` low
/// underneath the radio.
pub async fn build(p: UwbPeripherals) -> (Input<'static>, UwbSpiDevice) {
    let rst_guard = reset(p.rst).await;

    let config = SpiConfig::default().with_frequency(INIT_FREQUENCY);
    let bus = Spi::new(p.spi, config)
        .expect("SPI2 configuration is a fixed frequency within range; cannot fail")
        .with_sck(p.sck)
        .with_mosi(p.mosi)
        .with_miso(p.miso)
        .into_async();
    let cs = Output::new(p.cs, Level::High);
    let device = ExclusiveDevice::new(bus, cs, Delay)
        .expect("CS is a plain push-pull GPIO output; setting it cannot fail");

    (rst_guard, device)
}

/// Raises the bus clock to [`RUNNING_FREQUENCY`]. Call once the DW3000's clock PLL has locked
/// (i.e. after `dw3000_ng`'s `init()` and `config()`), not before.
pub fn raise_frequency(bus: &mut UwbSpiBus) {
    bus.apply_config(&SpiConfig::default().with_frequency(RUNNING_FREQUENCY))
        .expect("frequency is within the peripheral's supported range; cannot fail");
}

/// Pulses the DW3000's `RSTn` line: driven low for 10 ms, then released to a floating input so
/// the part's own internal pull-up brings it back high.
///
/// `RSTn` is open-drain -- it must never be driven high directly, only released. The returned
/// `Input` is left floating rather than dropped, so nothing later reconfigures the pin's
/// electrical state; the caller is expected to hold it for as long as the radio built from this
/// same reset is in use.
async fn reset(mut rst: GPIO15<'static>) -> Input<'static> {
    {
        let mut low = Output::new(rst.reborrow(), Level::Low);
        low.set_low();
        Timer::after(Duration::from_millis(10)).await;
    }
    let floating = Input::new(rst, Pull::None);
    Timer::after(Duration::from_millis(5)).await;
    floating
}

/// Holds `WAKEUP` inactive (low) for the whole program.
///
/// The reference tag firmware doesn't wire this pin at all; the dropbot's `UwbPins` carries it
/// because the DW3000 can also be woken from `DEEPSLEEP` with a pulse on it. This firmware never
/// sleeps the radio, so the pin is simply held low. Revisit if a low-power mode is added later.
pub fn hold_wakeup_inactive(wup: GPIO3<'static>) -> Output<'static> {
    Output::new(wup, Level::Low)
}

/// Builds the interrupt-enabled input the ranging loop waits on for `TXFRS`/`RXFCG` events.
pub fn irq(irq: GPIO2<'static>) -> ariel_os::gpio::IntEnabledInput<'static> {
    Input::builder(irq, Pull::None)
        .build_with_interrupt()
        .expect("uwb: registering the IRQ GPIO interrupt failed")
}
