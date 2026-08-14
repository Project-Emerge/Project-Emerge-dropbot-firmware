//! The dropbot's half of the DW3000 driver: this board's pins, this board's antenna delay, and
//! this firmware's log facade wrapped around the shared [`dw3000_hal`] control layer.
//!
//! Everything that is *about the DW3000* rather than about this board -- the INIT_RC clock
//! ceiling, the level-driven IRQ line, the `SYS_STATUS` flag-clearing workaround, the
//! delayed-send granularity, the antenna-delay argument order -- moved to `crates/dw3000-hal`,
//! which the anchor firmware in `anchor/` shares. What is left here is deliberately concrete over
//! this board's own SPI device type ([`spi::UwbSpiDevice`]): there is exactly one DW3000 on the
//! dropbot, wired to exactly these pins, so genericity would only add type-parameter noise at the
//! one place where it buys nothing.
//!
//! The wrappers below exist for one reason: `dw3000-hal` returns its diagnostics instead of
//! logging them, because a library crate has no access to `ariel_os::log` and the two firmwares
//! do not share a log facade. This module is where those diagnostics become `debug!`/`warn!`
//! lines, so the ranging task sees the same `Option`-shaped API it always did and no call site
//! has to match on a miss reason just to log it.
//!
//! Note what is *not* wrapped here: `dw3000_hal::elapsed_since_us`, which measures the
//! receive-to-transmit turnaround in radio time. The robot no longer has one to measure -- its only
//! transmission is a delayed `Poll` scheduled from the `Sync`, so it never turns a reception around
//! inside a deadline. That measurement now belongs entirely to the anchors, which are the nodes that
//! do (see `anchor/src/ranging.rs`).

pub mod anchors;
pub mod spi;
pub mod tag_id;

use ariel_os::gpio::IntEnabledInput;
use ariel_os::log::{debug, warn};
use ariel_os::reexports::embassy_time::Duration;
use dw3000_hal::{AntennaDelay, RxMiss, TxMiss};
use dw3000_ng::hl::SendTime;
use dw3000_ng::{Config, Error};
use uwb_protocol::Packet;

pub use dw3000_hal::radio_config;

use self::spi::UwbSpiDevice;

/// The radio once bring-up has finished.
pub type Radio = dw3000_hal::Radio<UwbSpiDevice>;

/// Initializes and configures the DW3000, ready to send and receive.
///
/// The caller must have pulsed `RSTn` first, which `spi::build` does.
///
/// `tag_id` becomes the radio's own 802.15.4 short address; every `Poll` addressed to it is what
/// the ranging responder answers to (see `tasks::uwb_ranging`).
pub async fn bring_up(
    spi_device: UwbSpiDevice,
    tag_id: u16,
    antenna_delay: AntennaDelay,
) -> Result<Radio, Error<UwbSpiDevice>> {
    dw3000_hal::bring_up(
        spi_device,
        tag_id,
        antenna_delay,
        ariel_os::time::Delay,
        // Runs once the clock PLL has locked, when the 7 MHz INIT_RC ceiling stops applying.
        // `esp-hal`'s `apply_config` on a live bus is exactly the operation no `embedded-hal`
        // trait exposes, which is why `dw3000-hal` takes it as a callback.
        |device| spi::raise_frequency(device.bus_mut()),
    )
    .await
}

/// Waits up to `timeout` for one frame, decodes it if it is one of ours, and logs why not
/// otherwise.
///
/// Returns `Ok((radio, None))` both for "nothing arrived" and for "arrived but didn't decode":
/// both are routine on a shared medium and not worth distinguishing to the caller, which is why
/// the distinction is made in the log line rather than in the return type. `Err` means the
/// driver's own typestate could not be walked back to `Ready`, i.e. there is no radio left to keep
/// using and [`bring_up`] must run again from a fresh reset -- the reference tag firmware treats
/// that as fatal (`.expect()`), but an embedded panic halts the whole MCU and would stop the
/// motors, the display and Wi-Fi along with ranging.
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

/// Sends one frame, delayed to `when`, and reports the timestamp the radio actually used.
///
/// Returns `Ok((radio, None))` if the send was not confirmed in time. A missed delayed-send
/// deadline is silent in hardware (`TXFRS` simply never fires), so it looks identical to an IRQ
/// that never arrived for any other reason, and both are routine enough not to abort the exchange
/// over. `Err` carries the same meaning as on [`receive_packet`].
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
