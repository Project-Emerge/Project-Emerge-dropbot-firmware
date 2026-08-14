//! The anchor's half of the DW3000 driver: SPI1 with DMA, the open-drain reset pulse, the IRQ
//! input, and this firmware's log facade wrapped around the shared [`dw3000_hal`] control layer.
//!
//! The structure mirrors the dropbot's `src/drivers/uwb/`: everything that is *about the DW3000*
//! lives in `crates/dw3000-hal`, and what is left here is this board's bus and pins plus the
//! `debug!`/`warn!` lines the library crate cannot emit itself. Keeping the two firmwares'
//! glue layers symmetrical is the point -- it is what makes a change to the radio protocol a
//! change in one shared place rather than two divergent ones.
//!
//! ariel-os's own SPI is bypassed deliberately. On STM32 it is
//! `YieldingAsync<BlockingAsync<Spi<Blocking>>>` -- no DMA -- its per-chip `MAX_FREQUENCY` table
//! has no entry for the STM32L432KC, and it exposes no way to reconfigure a live bus, which the
//! two-stage clock ramp below needs. GPIO and the interrupt input do come from ariel-os, so the
//! IRQ line the shared crate waits on is the same `IntEnabledInput` type on both boards.

use ariel_os::gpio::{Input, IntEnabledInput, Level, Output, Pull};
use ariel_os::hal::peripheral::Peri;
use ariel_os::hal::peripherals::{DMA1_CH2, DMA1_CH3, PA1, PA2, PA4, PA5, PA6, PA7, SPI1};
use ariel_os::log::{Debug2Format, debug, warn};
use ariel_os::reexports::embassy_time::Duration;
use ariel_os::time::{Delay, Timer};
use dw3000_hal::{AntennaDelay, RxMiss, TxMiss};
use dw3000_ng::hl::SendTime;
use dw3000_ng::{Config, Error};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embedded_hal_bus::spi::ExclusiveDevice;
use uwb_protocol::Packet;

pub use dw3000_hal::radio_config;

/// The DW3000 only tolerates SPI up to 7 MHz while its clock still runs off INIT_RC, so bring-up
/// starts here. embassy's own default is 1 MHz, which makes the driver's 129-byte frame-buffer
/// transfers slow enough to blow every reply deadline.
const INIT_FREQUENCY: Hertz = Hertz(5_000_000);

/// Once `init()`/`config()` have locked the clock PLL the part accepts up to 38 MHz. Everything on
/// the ranging critical path is register and frame-buffer traffic, so this is the single biggest
/// lever on the achievable update rate. 20 MHz needs PCLK2 at 80 MHz, which is what the RCC
/// override in `main.rs` is for.
const RUNNING_FREQUENCY: Hertz = Hertz(20_000_000);

/// Per-anchor antenna delays, indexed by `uwb_protocol::ANCHOR_IDS`.
///
/// One entry per board rather than one constant for all four, because each is a separate PCB with its
/// own antenna and its own trace lengths -- a single shared value cannot be right for more than one of
/// them. All four are still [`AntennaDelay::NOMINAL`], i.e. **uncalibrated**: that is Qorvo's nominal
/// figure for this PHY configuration, not a measurement, and it leaves a constant bias of up to a few
/// tens of centimetres.
///
/// To calibrate one: place a robot at a taped-out distance, read the raw ranges off `/uwb/{ID}`, and
/// adjust this entry. One tick is about 4.69 mm, and because the delay applies at both ends of an
/// exchange, changing it by N ticks moves the reported distance by roughly **2N** ticks (~9.4N mm).
/// Increase the delay to make the reported distance shorter. Do this before fitting
/// `uwb_protocol::RangeBias` on the robot -- see that type's docs for why the order matters.
const ANTENNA_DELAYS: [AntennaDelay; uwb_protocol::ACTIVE_ANCHOR_COUNT] = [
    AntennaDelay::NOMINAL, // A001, the TDMA master
    AntennaDelay::NOMINAL, // A002
    AntennaDelay::NOMINAL, // A003
    AntennaDelay::NOMINAL, // A004
];

/// This build's antenna delay.
pub const fn antenna_delay(anchor_index: usize) -> AntennaDelay {
    ANTENNA_DELAYS[anchor_index]
}

pub type UwbSpiBus = Spi<'static, Async>;
/// CS is a plain push-pull `Output` driven by `embedded-hal-bus` rather than SPI1's own hardware
/// NSS line, matching the pre-ariel-os anchor firmware and the dropbot.
pub type UwbSpiDevice = ExclusiveDevice<UwbSpiBus, Output<'static>, Delay>;
/// The radio once bring-up has finished.
pub type Radio = dw3000_hal::Radio<UwbSpiDevice>;

/// The peripherals one bring-up attempt needs, bundled so a retry can hand back a fresh set.
pub struct UwbPeripherals {
    pub rst: Peri<'static, PA1>,
    pub cs: Peri<'static, PA4>,
    pub sck: Peri<'static, PA5>,
    pub miso: Peri<'static, PA6>,
    pub mosi: Peri<'static, PA7>,
    pub spi: Peri<'static, SPI1>,
    pub tx_dma: Peri<'static, DMA1_CH3>,
    pub rx_dma: Peri<'static, DMA1_CH2>,
}

impl UwbPeripherals {
    /// Re-claims the peripherals a bring-up attempt needs.
    ///
    /// `dw3000_ng::DW3000<SPI, State>` never hands the SPI device back out on a failure path --
    /// not from `init()`/`config()` on a bad handshake, and not from
    /// `finish_sending`/`finish_receiving` on a stuck typestate, since its `ll` field is private --
    /// so recovering from any such failure means going back to the peripheral singletons for a
    /// fresh reset and a fresh `Spi`/CS/RSTn rather than reusing anything from the failed attempt.
    ///
    /// # Safety
    ///
    /// Sound exactly because that is true: the one legitimate instance of each of these
    /// peripherals -- originally handed to the caller via `crate::pins::UwbPins` -- has already
    /// been dropped along with the failed attempt by the time this runs.
    pub unsafe fn steal_for_retry() -> Self {
        unsafe {
            Self {
                rst: PA1::steal(),
                cs: PA4::steal(),
                sck: PA5::steal(),
                miso: PA6::steal(),
                mosi: PA7::steal(),
                spi: SPI1::steal(),
                tx_dma: DMA1_CH3::steal(),
                rx_dma: DMA1_CH2::steal(),
            }
        }
    }
}

/// Pulses reset and builds the SPI link at [`INIT_FREQUENCY`], ready for [`bring_up`].
///
/// Returns the reset pin too, now a floating `Input`: hold it for as long as the returned
/// `UwbSpiDevice` (or anything built from it) is in use, so nothing else drives `RSTn` low
/// underneath the radio.
pub async fn build(p: UwbPeripherals) -> (Input<'static>, UwbSpiDevice) {
    let rst_guard = reset(p.rst).await;

    let mut config = SpiConfig::default();
    config.frequency = INIT_FREQUENCY;
    let bus = Spi::new(p.spi, p.sck, p.mosi, p.miso, p.tx_dma, p.rx_dma, config);
    let cs = Output::new(p.cs, Level::High);
    let device = ExclusiveDevice::new(bus, cs, Delay)
        .expect("CS is a plain push-pull GPIO output; setting it cannot fail");

    (rst_guard, device)
}

/// Pulses the DW3000's `RSTn` line: driven low for 10 ms, then released to a floating input so the
/// part's own internal pull-up brings it back high.
///
/// `RSTn` is open-drain -- it must never be driven high directly, only released. The returned
/// `Input` is left floating rather than dropped, so nothing later reconfigures the pin's
/// electrical state.
async fn reset(mut rst: Peri<'static, PA1>) -> Input<'static> {
    {
        let mut low = Output::new(rst.reborrow(), Level::Low);
        low.set_low();
        Timer::after(Duration::from_millis(10)).await;
    }
    let floating = Input::new(rst, Pull::None);
    Timer::after(Duration::from_millis(5)).await;
    floating
}

/// Builds the interrupt-enabled input the ranging loop waits on for `TXFRS`/`RXFCG` events.
pub fn irq(irq: Peri<'static, PA2>) -> IntEnabledInput<'static> {
    Input::builder(irq, Pull::None)
        .build_with_interrupt()
        .expect("registering the DW3000 IRQ GPIO interrupt failed")
}

/// Initializes and configures the DW3000, ready to send and receive.
///
/// `anchor_id` becomes the radio's own 802.15.4 short address. With frame filtering off it is not
/// enforced in hardware, but `dw3000-ng` copies it into every frame's source address, so a sniffer
/// can still tell the four anchors apart.
pub async fn bring_up(
    spi_device: UwbSpiDevice,
    anchor_id: u16,
    antenna_delay: AntennaDelay,
) -> Result<Radio, Error<UwbSpiDevice>> {
    dw3000_hal::bring_up(
        spi_device,
        anchor_id,
        antenna_delay,
        Delay,
        // Runs once the clock PLL has locked, when the INIT_RC ceiling stops applying.
        // `set_config` on a live bus is exactly the operation no `embedded-hal` trait exposes,
        // which is why `dw3000-hal` takes it as a callback.
        |device| raise_frequency(device.bus_mut()),
    )
    .await
}

/// Raises the bus clock to [`RUNNING_FREQUENCY`]. Call only once the DW3000's clock PLL has locked.
fn raise_frequency(bus: &mut UwbSpiBus) {
    let mut config = bus.get_current_config();
    config.frequency = RUNNING_FREQUENCY;
    bus.set_config(&config)
        .expect("frequency is within the peripheral's supported range; cannot fail");
}

/// Waits up to `timeout` for one frame, decodes it if it is one of ours, and logs why not
/// otherwise.
///
/// Returns `Ok((radio, None))` both for "nothing arrived" and for "arrived but didn't decode":
/// both are routine on a shared medium, so the distinction is made in the log line rather than in
/// the return type. `Err` means the driver's typestate could not be walked back to `Ready` and
/// [`bring_up`] must run again from a fresh reset.
///
/// The pre-ariel-os anchor firmware `.expect()`ed all of these, on the grounds that ranging is the
/// whole program here. It is not quite: an anchor that panics stops answering, and if it is the
/// TDMA master it takes the whole network's timing with it, so recovering is worth the `Result`.
pub async fn receive_packet(
    radio: Radio,
    irq: &mut IntEnabledInput<'static>,
    config: Config,
    timeout: Duration,
) -> Result<(Radio, Option<(Packet, u64)>), Error<UwbSpiDevice>> {
    let received = dw3000_hal::receive_packet(radio, irq, config, timeout).await?;

    if let Some(reason) = received.stale {
        debug!("uwb: cleared stale RX flag before receive: {}", reason);
    }
    let packet = match received.outcome {
        Ok(packet) => Some(packet),
        Err(RxMiss::Timeout { latched }) => {
            if let Some(reason) = latched {
                debug!("uwb: RX aborted: {}", reason);
            }
            None
        }
        Err(RxMiss::Radio { latched }) => {
            debug!("uwb: RX failed: {}", latched.unwrap_or("no flag latched"));
            None
        }
        Err(RxMiss::Undecodable { payload_len }) => {
            debug!("uwb: undecodable payload ({} bytes)", payload_len);
            None
        }
        Err(RxMiss::Irq) => {
            warn!("uwb: IRQ pin error while waiting for RX");
            None
        }
    };
    Ok((received.radio, packet))
}

/// Sends one frame at `when` and reports the timestamp the radio actually used.
///
/// For a delayed send, `timeout` must cover the whole wait until that instant plus the usual
/// transmit-done margin: the radio genuinely does not fire until then, so a timeout sized for an
/// immediate send makes every delayed transmit look like a failure.
pub async fn send_packet(
    radio: Radio,
    irq: &mut IntEnabledInput<'static>,
    data: &[u8],
    when: SendTime,
    config: Config,
    timeout: Duration,
) -> Result<(Radio, Option<u64>), Error<UwbSpiDevice>> {
    let sent = dw3000_hal::send_packet(radio, irq, data, when, config, timeout).await?;

    let timestamp = match sent.outcome {
        Ok(ticks) => Some(ticks),
        Err(TxMiss::Timeout) => {
            debug!("uwb: TX IRQ timeout (missed delayed-send deadline?)");
            None
        }
        Err(TxMiss::Rejected) => {
            warn!("uwb: TX rejected by radio after IRQ");
            None
        }
        Err(TxMiss::Irq) => {
            warn!("uwb: IRQ pin error while waiting for TX");
            None
        }
    };
    Ok((sent.radio, timestamp))
}

/// Microseconds of radio time elapsed since `since_ticks`, per the DW3000's own clock.
///
/// This is the measurement that sizes the schedule's guard intervals: the deadline for a delayed
/// reply is set relative to a hardware receive timestamp, so it is the radio's clock, not the
/// MCU's, that decides whether the reply was programmed in time.
pub async fn elapsed_since_us(radio: &mut Radio, since_ticks: u64) -> Option<u32> {
    match dw3000_hal::elapsed_since_us(radio, since_ticks).await {
        Ok(elapsed) => Some(elapsed),
        Err(e) => {
            debug!("uwb: SYS_TIME read failed: {:?}", Debug2Format(&e));
            None
        }
    }
}

/// The radio's current time in DW3000 ticks, used by the timing master to place its first `Sync`.
pub async fn sys_time_ticks(radio: &mut Radio) -> Option<u64> {
    match dw3000_hal::sys_time_ticks(radio).await {
        Ok(ticks) => Some(ticks),
        Err(e) => {
            debug!("uwb: SYS_TIME read failed: {:?}", Debug2Format(&e));
            None
        }
    }
}
