//! The anchor's TDMA superframe loop.
//!
//! One anchor -- index 0, `uwb_protocol::MASTER_ANCHOR_ID` -- is the timing master and broadcasts a
//! `Sync` once per superframe. Every node derives the superframe's time base from that frame: the
//! master from its own transmit timestamp, everyone else from their receive timestamp. Each robot
//! then owns one `uwb_protocol::robot_slot`, in which it broadcasts one `Poll` that all four anchors
//! answer, each in the sub-slot `uwb_protocol::response_offset` gives it.
//!
//! The anchor never learns a distance. It contributes the two timestamps only it can measure --
//! `poll_rx` and its own `response_tx` -- and the robot does the arithmetic, because the robot is the
//! node that has somewhere to send a position.
//!
//! # The one tight deadline in the protocol lives here
//!
//! `response_tx` is derived from the *schedule*, not from `poll_rx`: the anchor knows when its
//! sub-slot is the moment it knows the superframe's base, long before the `Poll` arrives. What it
//! still has to do inside the deadline is read the `Poll` out, build the reply and program the
//! delayed send before that instant passes -- `uwb_protocol::RESPONSE_GUARD_US` for whichever anchor
//! is in the first sub-slot, plus one `SUBSLOT_US` for each position later. That is the whole reason
//! `response_offset` rotates with the sequence number: an anchor permanently in the first sub-slot
//! would absorb every marginal turnaround on its own and look broken.
//!
//! Missing the deadline is silent in hardware -- `TXFRS` simply never fires -- so
//! [`Stats::response_tx_failed`] is indexed by sub-slot rather than pooled. Failures concentrated in
//! sub-slot 0 mean `RESPONSE_GUARD_US` is too tight; failures spread evenly mean something else.
//!
//! # Recovery instead of panicking
//!
//! The pre-ariel-os firmware `.expect()`ed every radio call. An anchor that panics stops answering
//! and, if it is the master, takes the whole network's timing with it -- so every failure path here
//! either counts the event and carries on, or abandons the session and re-runs bring-up from a fresh
//! reset after [`RETRY_INTERVAL`].

use ariel_os::gpio::IntEnabledInput;
use ariel_os::log::{Debug2Format, debug, error, info, warn};
use ariel_os::reexports::embassy_time::Duration;
use ariel_os::time::{Instant, Timer};
use dw3000_ng::hl::SendTime;
use dw3000_ng::time::Instant as DwInstant;
use uwb_protocol::Packet;

use crate::drivers::uwb::{self, Radio};
use crate::pins;

/// Backoff after the radio drops out -- either bring-up failed, or the driver's typestate machine got
/// stuck and had to be abandoned (see `drivers::uwb::UwbPeripherals::steal_for_retry`).
const RETRY_INTERVAL: Duration = Duration::from_secs(5);

/// How early the receiver is armed before a robot slot opens.
///
/// Absorbs the offset between this MCU's clock (which schedules the arming) and the DW3000's (which
/// the slot is defined in), plus `Instant::now()`'s granularity. Every slot is armed relative to when
/// the `Sync` was *seen*, not to when the previous slot's exchange happened to finish, so a slot that
/// runs long cannot push the following ones out of the grid.
///
/// A slave's `seen_at` (see [`await_sync`]) is read *after* `receive_packet` has already paid for
/// `r_wait`'s frame read-out and an extra `take_rx_error` round trip -- SPI work `transmit_sync` never
/// does on its own, lighter, post-IRQ path. That gap between the two roles is what this guard has to
/// absorb, not just clock granularity: measured on hardware, 400 us was not enough and every slave
/// anchor missed every robot `Poll` in every slot, while the master, whose `seen_at` comes from the
/// cheaper TX path, was unaffected.
const POLL_ARM_GUARD_US: u64 = 1500;

/// How often to log a summary: often enough to be useful while watching bring-up, rare enough not to
/// drown the RTT channel -- roughly 7 s at the 69.8 ms superframe.
const REPORT_EVERY_SUPERFRAMES: u32 = 100;

// The summary line below names each sub-slot individually rather than looping, since defmt's `info!`
// needs one argument per placeholder and cannot take `[u32; N]` directly. Fails the build instead of
// silently mis-logging if the anchor table changes size.
const _: () = assert!(uwb_protocol::ACTIVE_ANCHOR_COUNT == 4);

/// Counters for one reporting window.
#[derive(Default)]
struct Stats {
    superframes: u32,
    missed_sync: u32,
    sync_tx_failed: u32,
    polls_seen: u32,
    responses_sent: u32,
    /// Delayed sends that never fired, indexed by the sub-slot they were scheduled in rather than
    /// pooled -- see the module docs on why the distribution is the diagnostic, not the total.
    response_tx_failed: [u32; uwb_protocol::ACTIVE_ANCHOR_COUNT],
    /// Worst turnaround from `poll_rx` to having the reply programmed, in microseconds of *radio*
    /// time. A running maximum rather than a mean: the failure mode is a single exchange overrunning
    /// its guard, which a max catches and an average hides.
    max_response_setup_us: u32,
}

impl Stats {
    fn log_and_reset(&mut self, anchor_id: u16) {
        info!(
            "uwb: anchor {:04x}: {} superframes, {} missed-sync, {} sync-tx-failed, {} polls, \
             {} responses, tx-failed by sub-slot [0]={} [1]={} [2]={} [3]={}, \
             max response setup {} us of {} us (tightest sub-slot)",
            anchor_id,
            self.superframes,
            self.missed_sync,
            self.sync_tx_failed,
            self.polls_seen,
            self.responses_sent,
            self.response_tx_failed[0],
            self.response_tx_failed[1],
            self.response_tx_failed[2],
            self.response_tx_failed[3],
            self.max_response_setup_us,
            uwb_protocol::RESPONSE_GUARD_US,
        );
        *self = Self::default();
    }
}

/// This superframe's time base, in both clocks that need it.
struct Superframe {
    /// The `Sync` epoch in DW3000 ticks: the master's own transmit timestamp, or a slave's receive
    /// timestamp for the same frame. Delayed sends are scheduled from this.
    base_ticks: u64,
    /// MCU time at which the `Sync` was transmitted or received. Receiver arming is scheduled from
    /// this, since `Timer` runs on the MCU clock and not the radio's.
    seen_at: Instant,
    /// What goes in every `Response`'s `master_offset` field: zero on the master, `base_ticks` minus
    /// the master's own transmit timestamp elsewhere.
    master_offset: u64,
    sequence: u16,
}

/// Runs this anchor's half of the ranging protocol forever.
pub async fn run(
    pins: pins::UwbPins,
    anchor_index: usize,
    antenna_delay: dw3000_hal::AntennaDelay,
) -> ! {
    let anchor_id = uwb_protocol::ANCHOR_IDS[anchor_index];
    let is_master = anchor_id == uwb_protocol::MASTER_ANCHOR_ID;
    // Reused across every bring-up attempt: `irq` is never consumed into the DW3000 typestate chain,
    // so unlike the SPI and reset peripherals it never needs re-claiming.
    let mut irq = uwb::irq(pins.irq);

    // The first attempt uses the peripherals this task was legitimately handed; every attempt after a
    // failure re-claims them instead.
    let mut peripherals = Some(uwb::UwbPeripherals {
        rst: pins.rst,
        cs: pins.cs,
        sck: pins.sck,
        miso: pins.miso,
        mosi: pins.mosi,
        spi: pins.spi,
        tx_dma: pins.tx_dma,
        rx_dma: pins.rx_dma,
    });

    let mut stats = Stats::default();

    loop {
        let owned = match peripherals.take() {
            Some(owned) => owned,
            // SAFETY: the only legitimate instance was dropped along with the previous attempt's
            // `UwbSpiDevice` -- see `UwbPeripherals::steal_for_retry`'s docs.
            None => unsafe { uwb::UwbPeripherals::steal_for_retry() },
        };
        let (_rst_guard, spi_device) = uwb::build(owned).await;

        let mut radio = match uwb::bring_up(spi_device, anchor_id, antenna_delay).await {
            Ok(radio) => radio,
            Err(e) => {
                error!("uwb: bring-up failed: {:?}", Debug2Format(&e));
                Timer::after(RETRY_INTERVAL).await;
                continue;
            }
        };
        info!(
            "uwb: anchor {:04x} ready ({}), superframe {} us / {} robot slots of {} us, \
             protocol fingerprint {:08x}, PHY fingerprint {:08x}",
            anchor_id,
            if is_master { "TDMA master" } else { "slave" },
            uwb_protocol::SUPERFRAME_US,
            uwb_protocol::ACTIVE_TAG_COUNT,
            uwb_protocol::ROBOT_SLOT_US,
            uwb_protocol::PROTOCOL_FINGERPRINT,
            uwb_protocol::PHY_FINGERPRINT,
        );

        let config = uwb::radio_config();
        // Master only: the instant the next `Sync` is programmed for, carried forward from the
        // previous one so the grid never accumulates this MCU's software latency. `None` means "no
        // grid yet", i.e. schedule the first one relative to a freshly read `SYS_TIME`.
        let mut next_sync_ticks: Option<u64> = None;
        let mut sequence = 0u16;

        'session: loop {
            stats.superframes += 1;

            let superframe = if is_master {
                match transmit_sync(
                    radio,
                    &mut irq,
                    config,
                    sequence,
                    next_sync_ticks,
                    antenna_delay,
                )
                .await
                {
                    Ok((next_radio, outcome)) => {
                        radio = next_radio;
                        match outcome {
                            Some((superframe, scheduled)) => {
                                // Chain the next Sync off the *programmed* instant, not off the
                                // reported timestamp: the latter includes the TX antenna delay, so
                                // chaining from it would walk the grid forward by one antenna delay
                                // per superframe.
                                next_sync_ticks = Some(uwb_protocol::delayed_after(
                                    scheduled,
                                    uwb_protocol::SUPERFRAME_US,
                                ));
                                superframe
                            }
                            None => {
                                stats.sync_tx_failed += 1;
                                warn!("uwb: sync TX failed; re-establishing the grid");
                                // Drop the chain: whatever went wrong, the next Sync should be
                                // scheduled fresh rather than at an instant that may already be past.
                                next_sync_ticks = None;
                                continue 'session;
                            }
                        }
                    }
                    Err(e) => {
                        error!("uwb: radio failed, re-initializing: {:?}", Debug2Format(&e));
                        break 'session;
                    }
                }
            } else {
                match await_sync(radio, &mut irq, config).await {
                    Ok((next_radio, Some(superframe))) => {
                        radio = next_radio;
                        // Adopt whatever sequence the master actually broadcast rather than requiring
                        // it to match a stale local counter: a single missed `Sync` would otherwise
                        // leave this anchor waiting for a number the master never resends -- and the
                        // sequence also drives the sub-slot rotation, so disagreeing about it would
                        // put two anchors in the same sub-slot.
                        sequence = superframe.sequence;
                        superframe
                    }
                    Ok((next_radio, None)) => {
                        radio = next_radio;
                        stats.missed_sync += 1;
                        debug!("uwb: no Sync received this superframe");
                        continue 'session;
                    }
                    Err(e) => {
                        error!("uwb: radio failed, re-initializing: {:?}", Debug2Format(&e));
                        break 'session;
                    }
                }
            };

            for tag_index in 0..uwb_protocol::ACTIVE_TAG_COUNT {
                match serve_robot_slot(
                    radio,
                    &mut irq,
                    config,
                    anchor_id,
                    anchor_index,
                    tag_index,
                    &superframe,
                    antenna_delay,
                    &mut stats,
                )
                .await
                {
                    Ok(next_radio) => radio = next_radio,
                    Err(e) => {
                        error!("uwb: radio failed, re-initializing: {:?}", Debug2Format(&e));
                        break 'session;
                    }
                }
            }

            sequence = sequence.wrapping_add(1);

            if stats.superframes >= REPORT_EVERY_SUPERFRAMES {
                stats.log_and_reset(anchor_id);
            }
        }

        Timer::after(RETRY_INTERVAL).await;
    }
}

/// Master only: broadcasts the `Sync` that starts a superframe.
///
/// Returns the superframe base and the instant the send was *programmed* for, which the caller chains
/// the next `Sync` off. A delayed send rather than an immediate one so the transmit timestamp is known
/// before transmitting and can be carried in the frame itself -- that is what lets every listener
/// relate its own clock to the master's.
async fn transmit_sync(
    radio: Radio,
    irq: &mut IntEnabledInput<'static>,
    config: dw3000_ng::Config,
    sequence: u16,
    next_sync_ticks: Option<u64>,
    antenna_delay: dw3000_hal::AntennaDelay,
) -> Result<(Radio, Option<(Superframe, u64)>), dw3000_ng::Error<uwb::UwbSpiDevice>> {
    let mut radio = radio;
    let scheduled = match next_sync_ticks {
        Some(ticks) => ticks,
        None => {
            // No grid yet, so place this one relative to the radio's current time. Every subsequent
            // `Sync` is chained off this instant instead, so the cost of this read is paid once.
            let Some(now_ticks) = uwb::sys_time_ticks(&mut radio).await else {
                warn!("uwb: cannot read SYS_TIME to schedule the first Sync");
                return Ok((radio, None));
            };
            uwb_protocol::delayed_after(now_ticks, uwb_protocol::SYNC_SCHEDULE_GUARD_US)
        }
    };

    let predicted_tx = uwb_protocol::delayed_tx_timestamp(scheduled, antenna_delay.tx);
    let mut buffer = [0u8; Packet::MAX_LEN];
    let len = Packet::Sync {
        sequence,
        // The frame cannot carry a timestamp read after its own transmission, so it carries the
        // prediction. Receivers use this value, so what matters is that it matches what the radio
        // reports -- checked below.
        sync_tx: predicted_tx,
    }
    .encode(&mut buffer);

    // The radio does not fire until `scheduled`, so the wait has to cover the whole delay plus the
    // usual transmit-done margin.
    let wait_us = uwb_protocol::SYNC_SCHEDULE_GUARD_US + uwb_protocol::SUPERFRAME_US;
    let (radio, actual_tx) = uwb::send_packet(
        radio,
        irq,
        &buffer[..len],
        SendTime::Delayed(
            DwInstant::new(scheduled).expect("delayed_after masks into the DW3000's 40-bit range"),
        ),
        config,
        Duration::from_micros(wait_us.into()),
    )
    .await?;
    let seen_at = Instant::now();

    let Some(actual_tx) = actual_tx else {
        return Ok((radio, None));
    };
    if actual_tx != predicted_tx {
        // Not fatal, but it means `delayed_tx_timestamp`'s model of the radio disagrees with the
        // hardware, and every listener's `master_offset` is off by the difference. Worth seeing.
        debug!(
            "uwb: sync TX timestamp mismatch: predicted {}, actual {} ({} ticks)",
            predicted_tx,
            actual_tx,
            actual_tx.wrapping_sub(predicted_tx),
        );
    }

    Ok((
        radio,
        Some((
            Superframe {
                base_ticks: actual_tx,
                seen_at,
                master_offset: 0,
                sequence,
            },
            scheduled,
        )),
    ))
}

/// Slave only: listens up to one whole superframe for the master's `Sync`.
///
/// Listens once rather than looping until a `Sync` arrives: a frame that is *not* a `Sync` -- some
/// robot's `Poll`, another anchor's `Response` -- means this anchor is listening in the middle of
/// someone else's exchange, and the caller's next attempt opens a fresh full-superframe window
/// anyway. Looping here would instead keep re-arming inside the same window with progressively less
/// of it left.
async fn await_sync(
    radio: Radio,
    irq: &mut IntEnabledInput<'static>,
    config: dw3000_ng::Config,
) -> Result<(Radio, Option<Superframe>), dw3000_ng::Error<uwb::UwbSpiDevice>> {
    let (radio, received) = uwb::receive_packet(
        radio,
        irq,
        config,
        Duration::from_micros(uwb_protocol::SUPERFRAME_US.into()),
    )
    .await?;
    let seen_at = Instant::now();

    match received {
        Some((Packet::Sync { sequence, sync_tx }, rx_ticks)) => {
            debug!("uwb: RX Sync seq={}", sequence);
            Ok((
                radio,
                Some(Superframe {
                    base_ticks: rx_ticks,
                    seen_at,
                    // Clock offset plus the master-to-here flight time; see the field's docs.
                    master_offset: rx_ticks.wrapping_sub(sync_tx) & 0xFF_FFFF_FFFF,
                    sequence,
                }),
            ))
        }
        Some(_) => {
            debug!("uwb: RX non-Sync frame while waiting for Sync");
            Ok((radio, None))
        }
        None => Ok((radio, None)),
    }
}

/// Serves one robot's slot: waits for its broadcast `Poll` and answers with a `Response` scheduled
/// into this anchor's sub-slot.
///
/// Every "nothing useful happened" case returns `Ok(radio)` and is counted in `stats`: an absent
/// robot, a collision, a momentary drop-out are all routine, and most of the twelve provisioned slots
/// are typically empty in a partly-populated fleet.
#[expect(
    clippy::too_many_arguments,
    reason = "one slot's worth of schedule state"
)]
async fn serve_robot_slot(
    radio: Radio,
    irq: &mut IntEnabledInput<'static>,
    config: dw3000_ng::Config,
    anchor_id: u16,
    anchor_index: usize,
    tag_index: usize,
    superframe: &Superframe,
    antenna_delay: dw3000_hal::AntennaDelay,
    stats: &mut Stats,
) -> Result<Radio, dw3000_ng::Error<uwb::UwbSpiDevice>> {
    let tag_id = uwb_protocol::TAG_IDS[tag_index];
    let slot_start_us = uwb_protocol::robot_slot(tag_index);

    // Anchored to when the Sync was seen, so a slot that runs long cannot push the following ones out
    // of the grid.
    Timer::at(
        superframe.seen_at
            + Duration::from_micros(u64::from(slot_start_us).saturating_sub(POLL_ARM_GUARD_US)),
    )
    .await;

    // Long enough to catch a Poll that is early or late by the arming guard, short enough that
    // twelve empty slots' worth of waiting still leaves the grid intact -- the `Timer::at` above is
    // what actually enforces the latter.
    let poll_window_us =
        POLL_ARM_GUARD_US + u64::from(uwb_protocol::MAX_FRAME_AIRTIME_US) + POLL_ARM_GUARD_US;
    let (mut radio, received) =
        uwb::receive_packet(radio, irq, config, Duration::from_micros(poll_window_us)).await?;

    let Some((
        Packet::Poll {
            tag_id: polled_tag,
            sequence: poll_sequence,
            ..
        },
        poll_rx,
    )) = received
    else {
        return Ok(radio);
    };
    if polled_tag != tag_id {
        // Another robot transmitting in the wrong slot, or a stale frame from the previous slot. Not
        // this anchor's problem to fix, but worth seeing: it means two robots share a slot
        // assignment.
        debug!(
            "uwb: slot {} carried a Poll from {:04x}, expected {:04x}",
            tag_index, polled_tag, tag_id
        );
        return Ok(radio);
    }
    if poll_sequence != superframe.sequence {
        debug!(
            "uwb: Poll from {:04x} has sequence {}, this superframe is {}",
            tag_id, poll_sequence, superframe.sequence
        );
        return Ok(radio);
    }
    stats.polls_seen += 1;

    let subslot_offset_us = uwb_protocol::response_offset(anchor_index, superframe.sequence);
    let subslot = (anchor_index + superframe.sequence as usize) % uwb_protocol::ACTIVE_ANCHOR_COUNT;
    // Derived from the superframe base, *not* from `poll_rx`: this anchor knew when its sub-slot was
    // before the Poll ever arrived, and scheduling from the schedule is what keeps the four anchors
    // from drifting into each other.
    let response_tx_ticks =
        uwb_protocol::delayed_after(superframe.base_ticks, slot_start_us + subslot_offset_us);

    // Measured in *radio* time, from the hardware receive timestamp: it is the radio's clock, not the
    // MCU's, that decides whether the reply was programmed before its sub-slot arrived. Queried
    // before the reply is even built, so both a slow build and a slow send show up in the number.
    if let Some(elapsed) = uwb::elapsed_since_us(&mut radio, poll_rx).await {
        stats.max_response_setup_us = stats.max_response_setup_us.max(elapsed);
        debug!(
            "uwb: response setup {} us, sub-slot {} (budget {} us)",
            elapsed,
            subslot,
            uwb_protocol::RESPONSE_GUARD_US + subslot as u32 * uwb_protocol::SUBSLOT_US,
        );
    }

    let mut buffer = [0u8; Packet::MAX_LEN];
    let len = Packet::Response {
        anchor_id,
        tag_id,
        sequence: superframe.sequence,
        poll_rx,
        // The frame cannot carry a timestamp read after its own transmission, so predict what the
        // radio will report. Getting this wrong inflates every range by tens of metres.
        response_tx: uwb_protocol::delayed_tx_timestamp(response_tx_ticks, antenna_delay.tx),
        master_offset: superframe.master_offset,
    }
    .encode(&mut buffer);

    let (radio, sent) = uwb::send_packet(
        radio,
        irq,
        &buffer[..len],
        SendTime::Delayed(
            DwInstant::new(response_tx_ticks)
                .expect("delayed_after masks into the DW3000's 40-bit range"),
        ),
        config,
        // Covers the wait until the sub-slot plus the transmit-done margin.
        Duration::from_micros(u64::from(subslot_offset_us + uwb_protocol::RX_TIMEOUT_US)),
    )
    .await?;

    if sent.is_some() {
        stats.responses_sent += 1;
    } else {
        stats.response_tx_failed[subslot] += 1;
        warn!(
            "uwb: response TX failed for tag {:04x} in sub-slot {}",
            tag_id, subslot
        );
    }
    Ok(radio)
}
