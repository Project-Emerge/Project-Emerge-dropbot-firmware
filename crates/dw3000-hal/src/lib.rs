//! Platform-agnostic DW3000 control: bring-up, the async transmit/receive helpers both ranging
//! roles are built from, and the two `dw3000-ng` workarounds neither firmware can do without.
//!
//! # Why this crate exists
//!
//! The anchors (STM32L432KC, `embassy-stm32`) and the robots (ESP32-C6, `esp-hal`) run the same
//! radio, the same PHY configuration and the same wire protocol, but reach the SPI bus through
//! completely different HALs. Everything that is *about the DW3000* -- the INIT_RC clock ceiling,
//! the level-driven IRQ line, the delayed-send granularity, the antenna-delay argument order --
//! is identical on both, and duplicating it once per firmware is how the two drifted apart in the
//! first place. So it lives here, generic over `embedded_hal_async`'s `SpiDevice` and `Wait`, and
//! each firmware supplies only the pin and bus glue.
//!
//! This is the *radio-touching* half of the split. The wire format, the TDMA timing tables and
//! the ranging arithmetic live in [`uwb_protocol`], which has no dependencies at all and is
//! host-testable; this crate deliberately is not, since every function here is an SPI
//! conversation. The boundary between them is raw `u64` ticks in both directions: no
//! `dw3000_ng::time::Instant` ever crosses into `uwb-protocol`.
//!
//! # No logging, no panics
//!
//! Nothing here logs. An embedded panic handler halts the whole MCU rather than just the task
//! that panicked, so on the robot an `.expect()` on an SPI glitch would stop the motors, the
//! display and Wi-Fi along with ranging. Every fallible call therefore returns a `Result` the
//! caller can recover from by dropping the radio and re-running [`bring_up`] from a fresh reset.
//!
//! For the same reason the diagnostics are *returned* rather than logged: the two firmwares use
//! different log facades (and `ariel_os::log` is not available to a plain library crate), so
//! [`Received::stale`], [`RxMiss`] and [`TxMiss`] carry everything the caller needs to say what
//! happened, in whatever facade it has.

#![no_std]

use dw3000_ng::configs::{BitRate, PreambleLength, PulseRepetitionFrequency, UwbChannel};
use dw3000_ng::hl::SendTime;
use dw3000_ng::{Config, Error, Ready, DW3000};
use embassy_time::{with_timeout, Duration};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use smoltcp::wire::{Ieee802154Address, Ieee802154Pan};
use uwb_protocol::Packet;

/// The radio once bring-up has finished: reset, initialized, configured, addressed, and with both
/// transmit and receive interrupts enabled.
pub type Radio<SPI> = DW3000<SPI, Ready>;

/// The largest frame the DW3000 will hand back, and the buffer size `r_wait` demands.
const RX_BUFFER_LEN: usize = 127;

/// Receive- and transmit-path antenna delays, in DW3000 clock ticks.
///
/// A struct rather than two arguments because `dw3000_ng::set_antenna_delay` takes them in
/// `(rx, tx)` order -- the reverse of how every datasheet, calibration note and conversation
/// about them is phrased. Swapping the two silently biases every distance this node reports, so
/// the order is fixed once, here, and callers name their fields.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AntennaDelay {
    pub rx: u16,
    pub tx: u16,
}

impl AntennaDelay {
    /// Qorvo's nominal value for the 64 MHz PRF configuration, applied to both paths.
    ///
    /// A starting point, **not** a calibration: it leaves a constant bias of up to a few tens of
    /// centimetres. One tick is about 4.69 mm, and because the delay applies at both ends of an
    /// exchange, changing it by N ticks moves the reported distance by roughly 2N ticks (~9.4N
    /// mm). Each board needs its own measured pair against a taped-out distance.
    pub const NOMINAL: Self = Self {
        rx: 16_385,
        tx: 16_385,
    };
}

/// Radio configuration shared by every node in the network.
///
/// Anchors and tags must agree on this or frames simply never decode. Note that a *timing* table
/// mismatch is not visible here and stays silent on air -- [`uwb_protocol::PROTOCOL_FINGERPRINT`]
/// is what catches that.
///
/// `frame_filtering` is off: addressing is checked in software against the payload IDs, because
/// the schedule (not the 802.15.4 header) is what decides who may answer when. `ranging_enable`
/// sets the ranging bit in `TX_FCTRL`. The remaining fields come from `Config::default()`, which
/// for channel 5 at 64 MHz PRF means preamble code 9, IEEE short SFD, STS off, and a derived PAC
/// size of 8.
pub fn radio_config() -> Config {
    Config {
        channel: UwbChannel::Channel5,
        pulse_repetition_frequency: PulseRepetitionFrequency::Mhz64,
        preamble_length: PreambleLength::Symbols64,
        bitrate: BitRate::Kbps6800,
        frame_filtering: false,
        ranging_enable: true,
        ..Config::default()
    }
}

/// Initializes and configures the DW3000, ready to send and receive.
///
/// The caller is responsible for having pulsed `RSTn` beforehand -- it is open-drain and must be
/// driven low and then *released*, never driven high, which is too HAL-specific to belong here.
///
/// `raise_clock` runs after `init()` and `config()` have locked the clock PLL, at which point the
/// 7 MHz INIT_RC SPI ceiling no longer applies and the part accepts up to 38 MHz. Everything on
/// the ranging critical path is register and frame-buffer traffic, so this is the single biggest
/// lever on the achievable update rate; it is a callback because reconfiguring a live bus is
/// exactly the operation no `embedded-hal` trait exposes.
///
/// `short_address` becomes the radio's own 802.15.4 short address. With frame filtering off it is
/// not enforced in hardware, but `dw3000-ng` copies it into the source address of every frame it
/// sends, so a sniffer can still tell the nodes apart.
pub async fn bring_up<SPI, DELAY>(
    spi_device: SPI,
    short_address: u16,
    antenna_delay: AntennaDelay,
    delay: DELAY,
    raise_clock: impl FnOnce(&mut SPI),
) -> Result<Radio<SPI>, Error<SPI>>
where
    SPI: SpiDevice<u8>,
    DELAY: DelayNs,
{
    let mut radio = DW3000::new(spi_device)
        .init()
        .await?
        .config(radio_config(), delay)
        .await?;

    raise_clock(&mut radio.ll().spi);

    radio
        .set_address(
            Ieee802154Pan(uwb_protocol::PAN_ID),
            Ieee802154Address::Short(short_address.to_be_bytes()),
        )
        .await?;
    radio
        .set_antenna_delay(antenna_delay.rx, antenna_delay.tx)
        .await?;
    radio.enable_rx_interrupts().await?;
    radio.enable_tx_interrupts().await?;

    Ok(radio)
}

/// Reports and clears any latched receive event in `SYS_STATUS`, naming the first one found.
///
/// The DW3000 drives its IRQ pin from the *level* of `SYS_STATUS & SYS_ENABLE`, and `dw3000-ng`
/// never clears the receive flags outside `r_wait`'s success path: its error paths return early,
/// and `finish_receiving` only forces the radio idle (unlike `finish_sending`, which also calls
/// `reset_flags`). A single latched receive event -- such as the SFD or preamble timeout that
/// happens routinely whenever the receiver is armed in the middle of someone else's transmission
/// -- therefore holds IRQ high forever, so `wait_for_rising_edge` never fires again and the node
/// stays deaf for good.
///
/// [`receive_packet`] runs this both before arming and after finishing, so a caller that only
/// uses these helpers never has to think about it. A caller driving the radio by hand must run it
/// after every receive, since a leftover flag blocks the next *transmit*'s IRQ wait too.
pub async fn take_rx_error<SPI>(radio: &mut Radio<SPI>) -> Option<&'static str>
where
    SPI: SpiDevice<u8>,
{
    let error = match radio.ll().sys_status().read().await {
        Ok(status) => {
            if status.rxphe() == 0b1 {
                Some("PHY header error")
            } else if status.rxfce() == 0b1 {
                Some("FCS error")
            } else if status.rxfsl() == 0b1 {
                Some("Reed-Solomon sync loss")
            } else if status.rxsto() == 0b1 {
                Some("SFD timeout")
            } else if status.rxpto() == 0b1 {
                Some("preamble timeout")
            } else if status.rxfto() == 0b1 {
                Some("frame wait timeout")
            } else if status.rxovrr() == 0b1 {
                Some("receiver overrun")
            } else {
                None
            }
        }
        Err(_) => Some("SYS_STATUS read failed"),
    };

    // `SYS_STATUS` is write-1-to-clear, and `write` leaves the bits it doesn't mention at zero, so
    // this clears exactly the receive events -- the same set `r_wait` clears when it succeeds.
    if radio
        .ll()
        .sys_status()
        .write(|w| {
            w.rxprd(0b1)
                .rxsfdd(0b1)
                .ciadone(0b1)
                .rxphd(0b1)
                .rxphe(0b1)
                .rxfr(0b1)
                .rxfcg(0b1)
                .rxfce(0b1)
                .rxfsl(0b1)
                .rxfto(0b1)
                .ciaerr(0b1)
                .rxovrr(0b1)
                .rxpto(0b1)
                .rxsto(0b1)
                .rxprej(0b1)
        })
        .await
        .is_err()
    {
        return Some("SYS_STATUS clear failed");
    }

    error
}

/// Why a receive produced no packet. All three are routine on a shared medium.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RxMiss {
    /// No IRQ edge before the deadline. `latched` names whatever receive flag was found while
    /// releasing the line afterwards, if any.
    Timeout { latched: Option<&'static str> },
    /// The IRQ fired but the frame did not survive `r_wait` -- a bad FCS, a truncated preamble,
    /// an overrun. `latched` names the flag `SYS_STATUS` reported.
    Radio { latched: Option<&'static str> },
    /// A frame arrived intact and was addressed to nobody in particular, but its payload is not
    /// this protocol's. Distinguished from the others because folding it in with them once made
    /// a decode bug look like radio silence.
    Undecodable { payload_len: usize },
    /// The interrupt pin itself reported an error while being waited on.
    Irq,
}

/// The outcome of one [`receive_packet`] call, plus the radio to keep using.
pub struct Received<SPI> {
    pub radio: Radio<SPI>,
    /// The decoded packet and its hardware receive timestamp, in DW3000 ticks.
    pub outcome: Result<(Packet, u64), RxMiss>,
    /// A receive flag that was already latched *before* the receiver was armed, cleared by this
    /// call. Its presence means the previous exchange left the IRQ line asserted; worth logging,
    /// but not an error here -- clearing it is exactly what keeps this node from going deaf.
    pub stale: Option<&'static str>,
}

/// Why a transmit was not confirmed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TxMiss {
    /// No `TXFRS` before the deadline. Also the symptom of a delayed send whose instant had
    /// already passed when it was programmed: the radio then does not transmit at all and raises
    /// no error, so a missed deadline and a dead IRQ line look identical from here.
    Timeout,
    /// The radio rejected the frame after the IRQ fired.
    Rejected,
    /// The interrupt pin itself reported an error while being waited on.
    Irq,
}

/// The outcome of one [`send_packet`] call, plus the radio to keep using.
pub struct Sent<SPI> {
    pub radio: Radio<SPI>,
    /// The hardware transmit timestamp the radio actually used, in DW3000 ticks.
    pub outcome: Result<u64, TxMiss>,
}

/// Waits up to `timeout` for one frame and decodes it if it is one of ours.
///
/// `Err` means the driver's own typestate could not be walked back to `Ready`, i.e. there is no
/// radio left to keep using and [`bring_up`] must run again from a fresh reset. Everything that
/// is merely a bad reception comes back as `Ok` with a [`RxMiss`].
pub async fn receive_packet<SPI, IRQ>(
    mut radio: Radio<SPI>,
    irq: &mut IRQ,
    config: Config,
    timeout: Duration,
) -> Result<Received<SPI>, Error<SPI>>
where
    SPI: SpiDevice<u8>,
    IRQ: Wait,
{
    // Release the IRQ line before arming the receiver, or a flag left over from an earlier failed
    // receive means no rising edge ever arrives.
    let stale = take_rx_error(&mut radio).await;

    let mut receiving = radio.receive(config).await?;
    let edge = with_timeout(timeout, irq.wait_for_rising_edge()).await;
    if !matches!(edge, Ok(Ok(()))) {
        let mut radio = match receiving.finish_receiving().await {
            Ok(radio) => radio,
            Err((_, error)) => return Err(error),
        };
        let latched = take_rx_error(&mut radio).await;
        let outcome = Err(match edge {
            Ok(Err(_)) => RxMiss::Irq,
            _ => RxMiss::Timeout { latched },
        });
        return Ok(Received {
            radio,
            outcome,
            stale,
        });
    }

    let mut buffer = [0u8; RX_BUFFER_LEN];
    let decoded = match receiving.r_wait(&mut buffer).await {
        Ok(message) => match message.frame.payload() {
            Some(payload) => match Packet::decode(payload) {
                Ok(packet) => Ok((packet, message.rx_time.value())),
                Err(_) => Err(RxMiss::Undecodable {
                    payload_len: payload.len(),
                }),
            },
            None => Err(RxMiss::Undecodable { payload_len: 0 }),
        },
        Err(_) => Err(RxMiss::Radio { latched: None }),
    };

    let mut radio = match receiving.finish_receiving().await {
        Ok(radio) => radio,
        Err((_, error)) => return Err(error),
    };
    // Always clear: `r_wait` only resets the flags when it succeeds, and a leftover flag would
    // also block the *next* transmission's IRQ wait.
    let latched = take_rx_error(&mut radio).await;
    let outcome = match decoded {
        Err(RxMiss::Radio { .. }) => Err(RxMiss::Radio { latched }),
        other => other,
    };

    Ok(Received {
        radio,
        outcome,
        stale,
    })
}

/// Sends one frame at `when` and reports the timestamp the radio actually used.
///
/// For a [`SendTime::Delayed`] send, `timeout` must cover the whole wait until that instant plus
/// the usual transmit-done margin -- the radio genuinely does not fire until then, so a timeout
/// sized for an immediate send makes every delayed transmit look like a failure.
///
/// `Err` carries the same "no radio left to keep using" meaning as on [`receive_packet`].
pub async fn send_packet<SPI, IRQ>(
    radio: Radio<SPI>,
    irq: &mut IRQ,
    data: &[u8],
    when: SendTime,
    config: Config,
    timeout: Duration,
) -> Result<Sent<SPI>, Error<SPI>>
where
    SPI: SpiDevice<u8>,
    IRQ: Wait,
{
    let mut sending = radio.send(data, when, config).await?;
    let outcome = match with_timeout(timeout, irq.wait_for_rising_edge()).await {
        Ok(Ok(())) => match sending.s_wait().await {
            Ok(instant) => Ok(instant.value()),
            Err(_) => Err(TxMiss::Rejected),
        },
        Ok(Err(_)) => Err(TxMiss::Irq),
        Err(_) => Err(TxMiss::Timeout),
    };
    let radio = match sending.finish_sending().await {
        Ok(radio) => radio,
        Err((_, error)) => return Err(error),
    };
    Ok(Sent { radio, outcome })
}

/// Microseconds of radio time elapsed since `since_ticks`, per the DW3000's own clock.
///
/// Used to measure the true receive-to-transmit turnaround, which is what sizes the schedule's
/// guard intervals. The deadline for a delayed reply is set relative to a hardware receive
/// timestamp, so it is the radio's clock -- not the MCU's -- that decides whether the reply is
/// programmed in time.
///
/// This costs an SPI round trip charged against that same deadline. The cost was measured against
/// the robot firmware's actual transmit-failure rate and found not to move it, so the visibility
/// is worth keeping rather than trading away for a cheaper measurement that cannot answer the
/// same question: a timestamp derived from a *successful* delayed send is, by construction, close
/// to `since_ticks + reply_delay` regardless of how much margin the software actually used, so it
/// cannot tell a comfortable send from one that only barely made it.
pub async fn elapsed_since_us<SPI>(
    radio: &mut Radio<SPI>,
    since_ticks: u64,
) -> Result<u32, Error<SPI>>
where
    SPI: SpiDevice<u8>,
{
    let now = sys_time_ticks(radio).await?;
    let ticks = now.wrapping_sub(since_ticks) & dw3000_ng::time::TIME_MAX;
    Ok((ticks / uwb_protocol::TICKS_PER_US) as u32)
}

/// The radio's current time, on the same 40-bit tick scale as every timestamp in the protocol.
///
/// Needed to bootstrap a schedule: the timing master has to place its first `Sync` at some instant in
/// the near future, and every instant after that is chained off the previous one. The low 8 bits are
/// always zero -- `SYS_TIME` exposes only the top 32 bits of the counter, which is also why `DX_TIME`
/// is programmed as `value >> 8` -- so this is accurate to about 4 ns, far finer than any guard
/// interval it gets added to.
pub async fn sys_time_ticks<SPI>(radio: &mut Radio<SPI>) -> Result<u64, Error<SPI>>
where
    SPI: SpiDevice<u8>,
{
    Ok((radio.sys_time().await? as u64) << 8)
}
