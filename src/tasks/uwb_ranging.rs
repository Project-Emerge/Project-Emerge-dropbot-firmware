//! DW3000 robot-initiated ranging: one broadcast `Poll` per superframe, four `Response`s back.
//!
//! The robot listens promiscuously only for the `Sync` that starts each superframe -- and even that
//! only until it has caught one, after which it predicts the next from the last and narrows to a brief
//! window around it (see [`SYNC_PREDICT_GUARD_US`]). It then transmits a single `Poll` in its own slot
//! (`uwb_protocol::robot_slot`) and collects the four `Response`s the anchors send back, one per
//! sub-slot. That is six receptions per superframe out of the ~60 frames on air, and four ranges from
//! a single transmission -- a genuine simultaneous snapshot, which is what a position solver wants,
//! rather than four measurements smeared across a superframe.
//!
//! # Why the robot initiates
//!
//! The previous revision was anchor-initiated three-message DS-TWR, one slot per *(anchor, robot)*
//! pair: 3.65 Hz per robot, with the cost scaling as `anchors x robots`. Worse, it put the tight
//! deadline on the *robot*: a `Poll` arrived and a delayed `Request` had to be programmed within 2 ms,
//! on an MCU also running Wi-Fi, MQTT, an OLED and an IMU sampler.
//!
//! Now the robot's only transmission is a delayed send scheduled from the `Sync`, programmed
//! [`POLL_PROGRAM_LEAD_US`] ahead of its own slot. There is no receive-to-transmit turnaround left on
//! this node at all; the one remaining tight deadline lives on the anchors, which are 80 MHz STM32s
//! doing nothing else. The rate goes to ~14 Hz and, because a robot slot no longer contains one
//! exchange per anchor, adding a fifth anchor would cost one sub-slot rather than twelve slots.
//!
//! # The clock correction is not optional
//!
//! A two-message exchange is single-sided, so it does not cancel the two nodes' clock-rate
//! difference: at an untrimmed 20 ppm and the 3.2 ms reply delay of the last sub-slot the error is
//! 9.6 metres. [`uwb_protocol::ClockRatioTracker`] measures that ratio from timestamps the protocol
//! already carries, at zero extra air time, one tracker per anchor -- the four anchors have four
//! crystals, and the `Sync` aligns their epochs, not their rates. Until a tracker has a baseline its
//! ranges are published with `clock_ratio_converged: false` rather than withheld, so the convergence
//! transient is visible on `/uwb/{ID}` instead of looking like a dropout; consumers must not fuse
//! those.
//!
//! # Why this runs on its own thread
//!
//! [`range_uwb`] shares no data structure with any other task, but it does share the CPU. Every other
//! task runs as an `embassy` task on the one main executor, which is a single cooperatively-scheduled
//! thread; measured on hardware, that was enough to occasionally starve ranging of several
//! milliseconds. The old design lost a whole exchange to that, since it was mid-turnaround. The
//! current one loses a whole *superframe*: miss the moment to program the delayed `Poll` and the slot
//! passes empty. [`ranging_thread`] therefore still runs this on its own executor and its own
//! `ariel-os` thread at [`RANGING_THREAD_PRIORITY`], above the main executor's, so the scheduler
//! preempts whatever it is doing rather than making ranging wait behind it. [`start`] exists only
//! because `#[ariel_os::thread(autostart)]` functions cannot take arguments the way a spawned
//! `#[ariel_os::task]` can.

use ariel_os::gpio::IntEnabledInput;
use ariel_os::log::{Debug2Format, debug, error, info, warn};
use ariel_os::reexports::static_cell::StaticCell;
use ariel_os::time::{Duration, Instant, Timer};
use ariel_os_embassy::thread_executor::Executor as ThreadExecutor;
use dw3000_ng::hl::SendTime;
use dw3000_ng::time::Instant as DwInstant;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::watch::Receiver as WatchReceiver;
use uwb_protocol::{ClockRatioTracker, Packet};

use crate::data::configurations::TagAssignmentsConfiguration;
use crate::data::localization::RangeMeasurement;
use crate::drivers::uwb::spi::UwbSpiDevice;
use crate::drivers::uwb::{self, Radio, spi};
use crate::pins;

/// Stack size for [`ranging_thread`]. The call chain is a handful of `.await` points deep with small
/// buffers (a 24-byte `Packet` and a 127-byte receive buffer being the largest), nowhere near the
/// 64 KiB the main executor needs for OTA's TLS and JSON handling -- but a soft-float RV32 async state
/// machine still isn't free, so this leaves real headroom rather than trimming to the minimum observed.
const RANGING_THREAD_STACKSIZE: usize = 8192;

/// Priority [`ranging_thread`] runs at: above the main executor's (8 by default, see
/// `ariel-os-embassy-common::executor_thread::PRIORITY`) so its scheduler preempts whatever the main
/// executor -- Wi-Fi, MQTT, the display, the IMU sampler -- is doing. Not as high as it could go:
/// `esp-radio`'s own driver tasks run up to priority 15, and sharing a level with them would mean no
/// preemption *between* them either, which is a subtler way to reintroduce the same starvation this
/// thread exists to avoid.
const RANGING_THREAD_PRIORITY: u8 = 9;

/// Backoff after the radio drops out -- either bring-up failed, or the driver's typestate machine got
/// stuck and had to be abandoned (see `drivers::uwb::spi::UwbPeripherals`).
const RETRY_INTERVAL: Duration = Duration::from_secs(5);

/// How far ahead of its own slot the robot starts programming the delayed `Poll`.
///
/// This is the whole turnaround budget on this node, and unlike the old design's 2 ms reply deadline it
/// is a *choice* rather than a consequence of the protocol: the send is scheduled from the `Sync`, so
/// the lead can be as generous as the schedule allows. 2 ms is several times the measured worst-case
/// SPI setup cost, which is what makes ranging robust to the main executor stealing the CPU mid-setup.
///
/// The radio spends this lead armed for transmission rather than listening, which costs nothing: what
/// is on air during it is the *previous* robot's exchange.
const POLL_PROGRAM_LEAD_US: u64 = 2_000;

// The lead has to stay inside the previous robot's slot; reaching further back would mean two robots
// programming transmissions over each other.
const _: () = assert!(POLL_PROGRAM_LEAD_US < uwb_protocol::ROBOT_SLOT_US as u64);

// The lead also has to fit inside the guard before robot slot 0 with room to spare, since
// `robot_slot(0) == uwb_protocol::SYNC_GUARD_US`: if the two were ever equal again, tag_index 0
// would get `slot_start_us.saturating_sub(POLL_PROGRAM_LEAD_US) == 0` in `run_slot` below, i.e. no
// explicit sleep at all, leaning on `Timer::at` absorbing an already-elapsed deadline instead of the
// same sleep-then-send schedule every other robot gets.
const _: () = assert!(POLL_PROGRAM_LEAD_US < uwb_protocol::SYNC_GUARD_US as u64);

/// Guard band on each side of the *predicted* next `Sync`, once one has been caught this session.
///
/// Has to absorb a superframe's worth of accumulated clock drift and scheduling jitter: at 20 ppm the
/// drift across 69.8 ms is barely 2 us, so this is almost entirely about MCU timer jitter and this
/// thread's own scheduling latency. Down from 5 ms in the anchor-initiated revision, where a
/// superframe was five times longer. The `uwb: sync interval` log line is the measured version of that
/// jitter to tighten this against.
const SYNC_PREDICT_GUARD_US: u64 = 1_500;

/// How many predicted windows in a row may come up empty before falling back to a full promiscuous
/// listen to reacquire.
///
/// Not zero: a single missed window is far more likely to be a transient (this thread running late, a
/// collision, a moment of RF noise) than an actual loss of the anchors' timing, and predicting forward
/// from the last known-good `Sync` for another cycle or two costs only that cycle's ranges. Not
/// unbounded either: enough consecutive misses means tracking really has been lost -- the master power
/// cycled, this robot drifted out of range -- and predicting from an increasingly stale anchor would
/// just keep missing.
const MAX_CONSECUTIVE_SYNC_MISSES: u32 = 3;

/// How often to log a ranging summary: roughly 7 s at the 69.8 ms superframe.
const REPORT_EVERY_SUPERFRAMES: u32 = 100;

/// Whether to log every accepted range's raw distance as it is computed.
///
/// Normally off: at up to four anchors per superframe and ~14 Hz this is a lot of log lines. The
/// serial-console counterpart of `/config/estimation.publish_raw_ranges`, and the quickest way
/// to see whether the distances themselves are sane before suspecting anything downstream of them.
///
/// **Currently on for a ranging-quality investigation**, alongside [`LOG_RECEPTION_TIMING`] and that
/// raw-range MQTT publication. The line carries `uwb_protocol::response_subslot` because that is what tells
/// a clock-correction residual apart from an RF one: the reply delay this anchor was answering at
/// rotates every superframe, and the single-sided correction's error scales with it, so a distance
/// that tracks the sub-slot index is not a multipath problem. See that function's docs for the
/// millimetres-per-ppb arithmetic.
const LOG_RAW_RANGES: bool = true;

/// Whether to log the wall-clock offset (from `poll_sent_at`) at which the robot armed its receiver
/// for each of the four `Response` sub-slots, and whether that attempt caught a frame.
///
/// Normally off; the measurement that sizes [`uwb_protocol::SUBSLOT_US`]. Each
/// `uwb::receive_packet` does a real SPI round trip to read a frame out before the next attempt can
/// arm, so if that readout costs more than one sub-slot, every successful catch pushes the following
/// attempt past its own window and the robot silently caps out at fewer anchors per superframe than
/// answered. Compare the logged arm times against the sub-slot boundaries -- `MAX_FRAME_AIRTIME_US +
/// RESPONSE_GUARD_US + n * SUBSLOT_US`, i.e. 1000, 2000, 3000 and 4000 us after the `Poll` at today's
/// constants -- and a loop that is falling behind shows up as attempt `n` arming progressively later
/// than boundary `n`.
///
/// **Currently on**, and this time to size a change rather than to explain a past one: lengthening the
/// preamble from 64 to 128 symbols buys a much better first-path timestamp but costs about 80 us more
/// air time per frame, and that comes straight out of the margin `SUBSLOT_US` holds over
/// `MAX_FRAME_AIRTIME_US` plus this readout. Today's arithmetic is 200 + ~700 against 1000; at 280 it
/// would be 980 against 1000, which is no margin at all. The gap between each armed-at figure below
/// and its own boundary is that margin measured, so it is what decides whether `SUBSLOT_US` can stay
/// where it is or has to grow with the air time -- and growing it is the one change here that costs
/// update rate.
const LOG_RECEPTION_TIMING: bool = true;

// The summary line below names each anchor individually rather than looping, since defmt's `info!`
// needs one argument per placeholder and cannot take `[u32; N]` directly. Fails the build instead of
// silently mis-logging if the anchor table ever changes size.
const _: () = assert!(uwb_protocol::ACTIVE_ANCHOR_COUNT == 4);

/// Tracks this session's lock on the anchors' `Sync` timing.
///
/// `anchor` is always a `Sync` this task actually decoded -- never a prediction -- so predicting
/// forward from it (`anchor + (misses + 1) * SUPERFRAME_US`) never compounds error across more than
/// [`MAX_CONSECUTIVE_SYNC_MISSES`] cycles even while `misses` is nonzero.
struct SyncLock {
    anchor: Instant,
    misses: u32,
}

#[derive(Default)]
struct RangingStats {
    superframes: u32,
    missed_sync: u32,
    poll_tx_failed: u32,
    invalid_timestamps: u32,
    unexpected_frames: u32,
    /// Ranges accepted per anchor, indexed the same way `uwb_protocol::ANCHOR_IDS` is.
    accepted: [u32; uwb_protocol::ACTIVE_ANCHOR_COUNT],
    /// Superframes in which this robot polled successfully but heard nothing from a given anchor.
    ///
    /// Per anchor rather than pooled because this is the deployment's NLOS and coverage signal: with
    /// only four anchors, one that is persistently shadowed drops the fix from four ranges to three,
    /// and a pooled count cannot say which one or whether it is always the same one.
    missing: [u32; uwb_protocol::ACTIVE_ANCHOR_COUNT],
}

impl RangingStats {
    fn log_and_reset(&mut self, ratios: &[ClockRatioTracker; uwb_protocol::ACTIVE_ANCHOR_COUNT]) {
        info!(
            "uwb: {} superframes, {} missed-sync, {} poll-tx-failed, {} invalid, {} unexpected; \
             accepted/missing by anchor [{:04x}] {}/{} [{:04x}] {}/{} [{:04x}] {}/{} \
             [{:04x}] {}/{}; clock ratio ppb {} {} {} {}",
            self.superframes,
            self.missed_sync,
            self.poll_tx_failed,
            self.invalid_timestamps,
            self.unexpected_frames,
            uwb_protocol::ANCHOR_IDS[0],
            self.accepted[0],
            self.missing[0],
            uwb_protocol::ANCHOR_IDS[1],
            self.accepted[1],
            self.missing[1],
            uwb_protocol::ANCHOR_IDS[2],
            self.accepted[2],
            self.missing[2],
            uwb_protocol::ANCHOR_IDS[3],
            self.accepted[3],
            self.missing[3],
            ratios[0].ratio().offset_ppb,
            ratios[1].ratio().offset_ppb,
            ratios[2].ratio().offset_ppb,
            ratios[3].ratio().offset_ppb,
        );
        *self = Self::default();
    }
}

/// Resolves `device_id`'s tag index: an MQTT-delivered fleet assignment takes priority when present
/// and valid, falling back to the compiled `drivers::uwb::tag_id::TAG_ASSIGNMENTS` otherwise. That
/// order, not the reverse, is what lets ranging start immediately at boot from the compiled default
/// while still letting a fleet be reprovisioned later over MQTT without a reflash.
fn resolve_tag_index(
    device_id: &str,
    mqtt_config: Option<&TagAssignmentsConfiguration>,
) -> Option<usize> {
    if let Some(config) = mqtt_config
        && let Some(index) = uwb::tag_id::resolve_from_config(device_id, &config.assignments)
    {
        return Some(index);
    }
    uwb::tag_id::tag_index(device_id)
}

/// Which entry of `uwb_protocol::ANCHOR_IDS` an ID refers to.
///
/// The robot cannot assume the n-th `Response` it hears comes from anchor n: `response_offset` rotates
/// the sub-slot order every superframe so no anchor is permanently on the tightest deadline. It reads
/// the ID out of the frame instead, which makes the rotation entirely the anchors' business.
fn anchor_index_of(anchor_id: u16) -> Option<usize> {
    uwb_protocol::ANCHOR_IDS
        .iter()
        .position(|&id| id == anchor_id)
}

/// [`range_uwb`]'s dependencies, bundled so they can cross from `main()` to [`ranging_thread`] through
/// [`HANDOFF`] -- see the module docs on why that handoff exists at all.
struct UwbThreadArgs {
    pins: pins::UwbPins,
    device_id: &'static str,
    ranges: Sender<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
    tag_config: WatchReceiver<'static, CriticalSectionRawMutex, TagAssignmentsConfiguration, 1>,
    antenna_delay: dw3000_hal::AntennaDelay,
}

/// Carries [`UwbThreadArgs`] from `main()` to [`ranging_thread`], exactly once. Capacity 1 and a single
/// send are enough reason on their own to reach for `try_send` over the blocking `send`; the more basic
/// reason is that `main()` is not an async context at all.
static HANDOFF: Channel<CriticalSectionRawMutex, UwbThreadArgs, 1> = Channel::new();

/// Hands the ranging task's dependencies to its dedicated thread.
///
/// Called once from `main()`, in place of a `spawner.spawn(...)` call -- [`ranging_thread`] spawns
/// [`range_uwb`] itself, onto its own executor, once these arrive.
pub fn start(
    pins: pins::UwbPins,
    device_id: &'static str,
    ranges: Sender<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
    tag_config: WatchReceiver<'static, CriticalSectionRawMutex, TagAssignmentsConfiguration, 1>,
    antenna_delay: dw3000_hal::AntennaDelay,
) {
    HANDOFF
        .try_send(UwbThreadArgs {
            pins,
            device_id,
            ranges,
            tag_config,
            antenna_delay,
        })
        .unwrap_or_else(|_| unreachable!("HANDOFF has capacity 1 and start() runs exactly once"));
}

/// Runs [`range_uwb`] on its own thread and its own `embassy` executor -- see the module docs on why
/// ranging needs one.
#[ariel_os::thread(autostart, stacksize = RANGING_THREAD_STACKSIZE, priority = RANGING_THREAD_PRIORITY)]
fn ranging_thread() {
    let args = ariel_os::thread::block_on(HANDOFF.receive());

    static EXECUTOR: StaticCell<ThreadExecutor> = StaticCell::new();
    EXECUTOR.init_with(ThreadExecutor::new).run(|spawner| {
        spawner.must_spawn(range_uwb(
            args.pins,
            args.device_id,
            args.ranges,
            args.tag_config,
            args.antenna_delay,
        ));
    });
}

/// Polls the anchors once per superframe and reports each accepted distance on `ranges`.
///
/// `device_id` must resolve to a tag index -- via `tag_config` or the compiled
/// `drivers::uwb::tag_id::TAG_ASSIGNMENTS`, see [`resolve_tag_index`] -- or this task keeps retrying
/// rather than ranging under a fallback index: two robots sharing a slot would transmit `Poll`s over
/// each other, which is worse than one robot not ranging.
#[ariel_os::task]
async fn range_uwb(
    pins: pins::UwbPins,
    device_id: &'static str,
    ranges: Sender<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
    mut tag_config: WatchReceiver<'static, CriticalSectionRawMutex, TagAssignmentsConfiguration, 1>,
    antenna_delay: dw3000_hal::AntennaDelay,
) -> ! {
    // Held for the program's lifetime: `_wup_guard` keeps WAKEUP inactive, `irq` is reused across every
    // bring-up attempt (it is never consumed into the DW3000 typestate chain, so it never needs
    // re-claiming the way the SPI/reset peripherals below do).
    let _wup_guard = spi::hold_wakeup_inactive(pins.wup);
    let mut irq = spi::irq(pins.irq);

    // The first bring-up attempt uses the peripherals this task was legitimately handed. Every attempt
    // after a failure re-claims them instead -- see `UwbPeripherals::steal_for_retry`.
    let mut peripherals = Some(spi::UwbPeripherals {
        rst: pins.rst,
        sck: pins.sck,
        mosi: pins.mosi,
        miso: pins.miso,
        cs: pins.cs,
        spi: pins.spi,
    });

    let mut stats = RangingStats::default();

    loop {
        let Some(tag_index) = resolve_tag_index(device_id, tag_config.try_get().as_ref()) else {
            error!(
                "uwb: no tag assignment (compiled default or MQTT) for device_id {}; retrying",
                device_id
            );
            Timer::after(RETRY_INTERVAL).await;
            continue;
        };
        let tag_id = uwb_protocol::TAG_IDS[tag_index];

        let owned = match peripherals.take() {
            Some(owned) => owned,
            // SAFETY: the only legitimate instance was dropped along with the previous attempt's
            // `UwbSpiDevice` -- see `UwbPeripherals::steal_for_retry`'s docs.
            None => unsafe { spi::UwbPeripherals::steal_for_retry() },
        };
        let (_rst_guard, spi_device) = spi::build(owned).await;

        let mut radio = match uwb::bring_up(spi_device, tag_id, antenna_delay).await {
            Ok(radio) => radio,
            Err(e) => {
                error!("uwb: bring-up failed: {:?}", Debug2Format(&e));
                Timer::after(RETRY_INTERVAL).await;
                continue;
            }
        };
        info!(
            "uwb: radio ready, tag_id={:04x} tag_index={} slot at {} us of a {} us superframe, \
             protocol fingerprint {:08x}, PHY fingerprint {:08x}",
            tag_id,
            tag_index,
            uwb_protocol::robot_slot(tag_index),
            uwb_protocol::SUPERFRAME_US,
            uwb_protocol::PROTOCOL_FINGERPRINT,
            uwb_protocol::PHY_FINGERPRINT,
        );

        let config = uwb::radio_config();
        // One tracker per anchor: four separate crystals, and the `Sync` aligns their epochs, not their
        // rates. Fresh per session because the DW3000's counter restarts on re-initialization, so a
        // baseline from before it no longer refers to the same epoch.
        let mut ratios = [ClockRatioTracker::default(); uwb_protocol::ACTIVE_ANCHOR_COUNT];
        // `None` means listen promiscuously (freshly booted, or tracking was abandoned after
        // `MAX_CONSECUTIVE_SYNC_MISSES`); `Some` means predict the next `Sync` and listen only in a
        // brief window around it.
        let mut sync_lock: Option<SyncLock> = None;

        'session: loop {
            // Re-resolve every superframe so a fleet reprovision delivered over MQTT is picked up
            // promptly rather than only on the next radio failure. Cheap: a `Watch::try_get` and a scan
            // of at most twelve entries, against a ~70 ms cycle.
            if let Some(resolved) = resolve_tag_index(device_id, tag_config.try_get().as_ref())
                && resolved != tag_index
            {
                info!(
                    "uwb: tag assignment changed ({} -> {}), re-initializing",
                    tag_index, resolved
                );
                break 'session;
            }

            // Wait for the next `Sync`. The first time this session (or once tracking has been
            // abandoned) that means promiscuously, for a whole superframe, since the timing is not
            // known yet. Once locked on, sleep until shortly before the predicted next one and listen
            // only in a brief window instead.
            let listen_timeout_us = match &sync_lock {
                Some(lock) => {
                    let predicted_at = lock.anchor
                        + Duration::from_micros(
                            u64::from(lock.misses + 1) * u64::from(uwb_protocol::SUPERFRAME_US),
                        );
                    Timer::at(predicted_at - Duration::from_micros(SYNC_PREDICT_GUARD_US)).await;
                    2 * SYNC_PREDICT_GUARD_US
                }
                None => uwb_protocol::SUPERFRAME_US.into(),
            };
            let (next_radio, received) = match uwb::receive_packet(
                radio,
                &mut irq,
                config,
                Duration::from_micros(listen_timeout_us),
            )
            .await
            {
                Ok(result) => result,
                Err(e) => {
                    error!("uwb: radio failed, re-initializing: {:?}", Debug2Format(&e));
                    break 'session;
                }
            };
            radio = next_radio;
            stats.superframes += 1;

            let Some((Packet::Sync { sequence, .. }, sync_rx_ticks)) = received else {
                stats.missed_sync += 1;
                // A miss while locked on doesn't necessarily mean the prediction is wrong -- see
                // `MAX_CONSECUTIVE_SYNC_MISSES` -- so keep predicting from the same last known-good
                // `Sync` for a few more cycles before paying for a full promiscuous reacquisition.
                if let Some(lock) = &mut sync_lock {
                    lock.misses += 1;
                    if lock.misses >= MAX_CONSECUTIVE_SYNC_MISSES {
                        debug!(
                            "uwb: {} consecutive missed predicted sync windows, re-acquiring promiscuously",
                            lock.misses
                        );
                        sync_lock = None;
                    }
                }
                if stats.superframes >= REPORT_EVERY_SUPERFRAMES {
                    stats.log_and_reset(&ratios);
                }
                continue 'session;
            };

            // Anchors this superframe's slot to when the `Sync` was actually seen rather than to a
            // free-running local clock. `sync_rx_ticks` does the same on the radio's clock, and the two
            // are captured within microseconds of each other.
            let sync_seen_at = Instant::now();
            if let Some(lock) = &sync_lock {
                debug!(
                    "uwb: sync interval {} us (expected {} us, {} predicted window(s))",
                    sync_seen_at.duration_since(lock.anchor).as_micros(),
                    uwb_protocol::SUPERFRAME_US,
                    lock.misses + 1,
                );
            }
            sync_lock = Some(SyncLock {
                anchor: sync_seen_at,
                misses: 0,
            });

            match run_slot(
                radio,
                &mut irq,
                config,
                tag_id,
                tag_index,
                sequence,
                sync_seen_at,
                sync_rx_ticks,
                antenna_delay,
                &mut ratios,
                &ranges,
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

            if stats.superframes >= REPORT_EVERY_SUPERFRAMES {
                stats.log_and_reset(&ratios);
            }
        }

        Timer::after(RETRY_INTERVAL).await;
    }
}

/// Runs this robot's own slot: one broadcast `Poll`, then the anchors' `Response`s.
///
/// Every "nothing useful happened" case is counted in `stats` and does not abort the session: a missed
/// `Poll` transmission, an absent anchor, an unrangeable timestamp set are all routine on a shared
/// medium. `Err` is reserved for the radio's own typestate machine failing to recover, exactly as in
/// `drivers::uwb::receive_packet`.
#[expect(clippy::too_many_arguments, reason = "one superframe's worth of state")]
async fn run_slot(
    radio: Radio,
    irq: &mut IntEnabledInput<'static>,
    config: dw3000_ng::Config,
    tag_id: u16,
    tag_index: usize,
    sequence: u16,
    sync_seen_at: Instant,
    sync_rx_ticks: u64,
    antenna_delay: dw3000_hal::AntennaDelay,
    ratios: &mut [ClockRatioTracker; uwb_protocol::ACTIVE_ANCHOR_COUNT],
    ranges: &Sender<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
    stats: &mut RangingStats,
) -> Result<Radio, dw3000_ng::Error<UwbSpiDevice>> {
    let slot_start_us = u64::from(uwb_protocol::robot_slot(tag_index));
    let poll_tx_ticks =
        uwb_protocol::delayed_after(sync_rx_ticks, uwb_protocol::robot_slot(tag_index));

    // Sleep until it is time to program the delayed send, measured from the `Sync` on the MCU clock.
    Timer::at(
        sync_seen_at + Duration::from_micros(slot_start_us.saturating_sub(POLL_PROGRAM_LEAD_US)),
    )
    .await;

    let mut buffer = [0u8; Packet::MAX_LEN];
    let len = Packet::Poll {
        tag_id,
        sequence,
        // The frame cannot carry a timestamp read after its own transmission, so predict what the radio
        // will report. Only a passive listener reads this field; the ranging below uses the hardware
        // timestamp `s_wait` returns.
        poll_tx: uwb_protocol::delayed_tx_timestamp(poll_tx_ticks, antenna_delay.tx),
    }
    .encode(&mut buffer);

    // The radio does not fire until `poll_tx_ticks`, so the wait has to cover the whole lead plus the
    // frame's own air time and a transmit-done margin.
    let tx_wait_us = POLL_PROGRAM_LEAD_US
        + u64::from(uwb_protocol::MAX_FRAME_AIRTIME_US + uwb_protocol::RX_TIMEOUT_US);
    let (radio, actual_poll_tx) = uwb::send_packet(
        radio,
        irq,
        &buffer[..len],
        SendTime::Delayed(
            DwInstant::new(poll_tx_ticks)
                .expect("delayed_after masks into the DW3000's 40-bit range"),
        ),
        config,
        Duration::from_micros(tx_wait_us),
    )
    .await?;

    let Some(poll_tx) = actual_poll_tx else {
        stats.poll_tx_failed += 1;
        warn!("uwb: poll TX failed; this superframe yields no ranges");
        return Ok(radio);
    };
    let poll_sent_at = Instant::now();

    // The anchors answer in rotated sub-slots, so which one arrives first is not fixed. Collect
    // whatever turns up and attribute each frame by the anchor ID it carries.
    //
    // Deliberately minimal work inside this loop. Anything done here lands between one reception
    // finishing and the next arming, so it comes straight out of the next sub-slot's window:
    // [`LOG_RECEPTION_TIMING`] caught an earlier version of this loop capping out at two of four
    // responses no matter how many anchors answered, purely because each catch pushed the following
    // attempt past its own slot. Only identifying which anchor answered and stashing its raw
    // timestamps has to happen before the next arm; the clock-rate update, the ranging maths
    // (128-bit fixed-point) and the channel send do not, so they run below once all four are done.
    let mut radio = radio;
    let mut seen = [false; uwb_protocol::ACTIVE_ANCHOR_COUNT];
    let mut response_ticks: [Option<(u64, u64, u64)>; uwb_protocol::ACTIVE_ANCHOR_COUNT] =
        [None; uwb_protocol::ACTIVE_ANCHOR_COUNT];
    let mut reception_armed_at_us = [0u32; uwb_protocol::ACTIVE_ANCHOR_COUNT];
    let mut reception_hit = [false; uwb_protocol::ACTIVE_ANCHOR_COUNT];
    // Every attempt's window ends at the same instant: the end of this robot's reply block. See
    // `uwb_protocol::RESPONSE_BLOCK_US` for why one absolute deadline beats a fixed per-attempt
    // duration -- in short, it is what keeps one late arm from cascading into every later one.
    let block_end = poll_sent_at + Duration::from_micros(uwb_protocol::RESPONSE_BLOCK_US.into());

    // Bounded twice over: by the deadline, which no iteration can outlive, and by an attempt count
    // that leaves slack for a few frames that turn out not to be ours. The slack is the point of
    // not simply looping `ACTIVE_ANCHOR_COUNT` times -- an undecodable frame or one from another
    // robot's exchange returns early and used to burn one of exactly four chances.
    let mut attempt = 0usize;
    while attempt < 2 * uwb_protocol::ACTIVE_ANCHOR_COUNT && !seen.iter().all(|heard| *heard) {
        let armed_at = Instant::now();
        let Some(timeout) = block_end.checked_duration_since(armed_at) else {
            break;
        };
        if let Some(slot) = reception_armed_at_us.get_mut(attempt) {
            *slot = armed_at.duration_since(poll_sent_at).as_micros() as u32;
        }
        let (next_radio, received) = uwb::receive_packet(radio, irq, config, timeout).await?;
        radio = next_radio;
        if let Some(slot) = reception_hit.get_mut(attempt) {
            *slot = received.is_some();
        }
        attempt += 1;

        let Some((packet, response_rx)) = received else {
            // Not necessarily a timeout: `drivers::uwb::receive_packet` also reports a radio error
            // and an undecodable payload this way. Carrying on costs nothing now that the next
            // attempt's window is whatever is left of the block rather than another full sub-slot.
            continue;
        };
        let Packet::Response {
            anchor_id,
            tag_id: response_tag,
            sequence: response_sequence,
            poll_rx,
            response_tx,
            ..
        } = packet
        else {
            stats.unexpected_frames += 1;
            continue;
        };
        if response_tag != tag_id || response_sequence != sequence {
            // Somebody else's exchange, or a stale frame from the previous superframe.
            stats.unexpected_frames += 1;
            continue;
        }
        let Some(anchor_index) = anchor_index_of(anchor_id) else {
            stats.unexpected_frames += 1;
            debug!("uwb: response from unknown anchor {:04x}", anchor_id);
            continue;
        };
        seen[anchor_index] = true;
        response_ticks[anchor_index] = Some((poll_rx, response_tx, response_rx));
    }

    for (anchor_index, ticks) in response_ticks.iter().enumerate() {
        let Some((poll_rx, response_tx, response_rx)) = *ticks else {
            continue;
        };
        let anchor_id = uwb_protocol::ANCHOR_IDS[anchor_index];

        // Fold this pair into the anchor's clock-rate estimate before using it: the sample is part of
        // its own baseline, which is harmless and one superframe less stale.
        let ratio = ratios[anchor_index].update(response_tx, response_rx);
        let converged = ratios[anchor_index].is_converged();

        match uwb_protocol::distance_mm_ss_twr(poll_tx, poll_rx, response_tx, response_rx, ratio) {
            Some(distance_mm) => {
                stats.accepted[anchor_index] += 1;
                if LOG_RAW_RANGES {
                    info!(
                        "uwb: raw range from anchor {:04x}: {} mm, sequence {}, subslot {}, \
                         ratio {} ppb, converged {}",
                        anchor_id,
                        distance_mm,
                        sequence,
                        uwb_protocol::response_subslot(anchor_index, sequence),
                        ratio.offset_ppb,
                        converged
                    );
                }
                let measurement = RangeMeasurement {
                    anchor_id,
                    distance_mm,
                    sequence,
                    response_subslot: uwb_protocol::response_subslot(anchor_index, sequence) as u8,
                    timestamp_us: poll_sent_at.as_micros(),
                    clock_ratio_ppb: ratio.offset_ppb,
                    clock_ratio_converged: converged,
                };
                // A dropped measurement here just means the raw-range topic falls behind; the fusion
                // estimator reads from the same channel and matters far more than `/uwb/{ID}` keeping
                // up.
                let _ = ranges.try_send(measurement);
            }
            None => {
                stats.invalid_timestamps += 1;
                // Routine before the clock ratio converges -- an uncorrected offset in one direction
                // makes the computed flight negative -- and a real problem after.
                if converged {
                    warn!(
                        "uwb: unrangeable timestamps from anchor {:04x} (ratio {} ppb)",
                        anchor_id, ratio.offset_ppb
                    );
                } else {
                    debug!(
                        "uwb: unrangeable timestamps from anchor {:04x} while the clock ratio is \
                         still converging",
                        anchor_id
                    );
                }
            }
        }
    }

    for (anchor_index, heard) in seen.iter().enumerate() {
        if !heard {
            stats.missing[anchor_index] += 1;
        }
    }

    if LOG_RECEPTION_TIMING {
        // Sub-slot n's frame is expected at MAX_FRAME_AIRTIME_US + RESPONSE_GUARD_US + n * SUBSLOT_US
        // after `poll_sent_at`: 1000, 1700, 2400, 3100 us at today's constants. Compare those against
        // where each attempt actually armed to see whether the loop is keeping pace or falling behind.
        info!(
            "uwb: reception armed at (us after poll) [0] {} hit={} [1] {} hit={} [2] {} hit={} \
             [3] {} hit={}",
            reception_armed_at_us[0],
            reception_hit[0],
            reception_armed_at_us[1],
            reception_hit[1],
            reception_armed_at_us[2],
            reception_hit[2],
            reception_armed_at_us[3],
            reception_hit[3],
        );
    }

    Ok(radio)
}
