//! Fuses UWB ranges with the IMU into a pose, onboard.
//!
//! The estimator itself lives in `dropbot-estimation`, which is dependency-free and host-tested against
//! synthetic trajectories; this task is the plumbing that feeds it real data and publishes what comes
//! out. The split is what makes the filter testable at all -- none of the maths here can be exercised
//! on a robot in a way that tells you whether it is *right*.
//!
//! # Two rates, one filter
//!
//! Prediction runs off the IMU at its full 100 Hz sampling rate -- over `IMU_FUSION`, not over the
//! decimated copy that goes out on MQTT -- while range updates arrive in bursts of up to four every
//! superframe (~18 Hz). That asymmetry is the whole point of fusing: between range fixes the unicycle
//! model carries the pose forward, so a consumer sees a smooth estimate rather than an 18 Hz staircase,
//! and a superframe that yields no ranges at all degrades to dead reckoning instead of to nothing.
//!
//! Prediction is deliberately *not* gated on a range arriving. Doing that would silently run the model
//! at the ranging rate, which is the one mistake in this task that would still produce a plausible
//! pose.
//!
//! # Why the raw range topic is off by default
//!
//! This filter running onboard is what freed the network: nothing off-board needs the raw ranges to
//! compute a position any more. Four of them per superframe is ~0.6 Mbit/s across twelve robots in
//! small messages, each paying its own MQTT and TCP overhead, against a pose that carries the same
//! information for a twentieth of that. So they are gated behind [`PUBLISH_RAW_RANGES`] -- on for a
//! calibration session (they are what `uwb_protocol::RangeBias` is fitted from), off in a running
//! swarm.

use ariel_os::log::{debug, error, info, warn};
use ariel_os::time::{Duration, Instant, Timer};
use dropbot_estimation::{Anchor, FilterConfig, PoseFilter, RangeOutcome};
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender};
use heapless::Vec;

use crate::data::configurations::AnchorsConfiguration;
use crate::data::localization::{PoseEstimate, RangeMeasurement};
use crate::data::telemetry::IMUTelemetry;
use crate::drivers::uwb::anchors::{self, Layout};

/// Whether to forward every accepted range to `/uwb/{ID}` as well as publishing the pose.
///
/// Off by default -- see the module docs for the bandwidth arithmetic. Turn it on to collect the data
/// `uwb_protocol::RangeBias` is fitted from, then turn it back off.
///
/// Currently on: turned on to see raw per-anchor distances while chasing why no snapshot has enough
/// simultaneous anchors to bootstrap. Turn back off once that's resolved.
const PUBLISH_RAW_RANGES: bool = true;

/// Forward speed at full commanded duty, metres per second.
///
/// A calibration constant, and the only place the commanded duty cycle becomes a physical speed: the
/// robots have no wheel encoders, so nothing measures this and nothing can check it at runtime. Measure
/// it by driving a taped-out distance at full duty and timing it. Getting it wrong shows up as a
/// systematic lag or lead in the estimated position between range fixes, which the ranges then correct
/// -- so the cost is noisier tracking rather than a wrong answer.
const MAX_SPEED_M_S: f32 = 0.45;

/// Degrees per second to radians per second. `data::imu` reports its filtered rates in degrees, since
/// that is what the attitude code around it works in; the estimator is in radians.
const RADIANS_PER_DEGREE: f32 = core::f32::consts::PI / 180.0;

/// How long to keep collecting a superframe's ranges after the first one arrives.
///
/// The four `Response`s span at most `ACTIVE_ANCHOR_COUNT * SUBSLOT_US` on air, so anything that has
/// not arrived by then is not coming. A snapshot that already holds all four is fused immediately, so
/// this only bounds the *incomplete* case -- which is exactly the case where waiting for the next
/// superframe's first range to notice would cost a whole superframe of latency.
const SNAPSHOT_WINDOW: Duration = Duration::from_micros(
    (uwb_protocol::ACTIVE_ANCHOR_COUNT as u64) * (uwb_protocol::SUBSLOT_US as u64)
        + uwb_protocol::MAX_FRAME_AIRTIME_US as u64,
);

/// How long to wait on an idle timer when there is no snapshot pending.
///
/// Only exists so the `select3` below always has three branches. Long enough not to spin, short enough
/// that the periodic summary still gets logged on a robot that has stopped hearing anchors.
const IDLE_TICK: Duration = Duration::from_secs(1);

/// How many superframes' worth of ranges may fail to produce a fix before the bootstrap is redone.
///
/// A filter whose position has drifted far enough that every range trips the outlier gate cannot
/// recover on its own: the gate keeps rejecting, so nothing corrects it, so the gate keeps rejecting.
/// Re-bootstrapping from a fresh snapshot breaks that loop. Not one superframe: a single all-rejected
/// burst is a routine consequence of being briefly shadowed.
const MAX_BARREN_SUPERFRAMES: u32 = 10;

/// How often to log a summary: roughly every 5 s at the ~18 Hz superframe rate.
const REPORT_EVERY_SUPERFRAMES: u32 = 90;

/// One superframe's worth of ranges, keyed by anchor index.
///
/// Collected before being fused so the bootstrap has a genuine simultaneous snapshot to trilaterate
/// from -- all four ranges of a superframe come from one transmission, which is exactly what
/// trilateration assumes and what the previous anchor-initiated schedule could not provide.
type Snapshot = Vec<(Anchor, f32), { uwb_protocol::ACTIVE_ANCHOR_COUNT }>;

#[derive(Default)]
struct Stats {
    superframes: u32,
    accepted: u32,
    gated: u32,
    unfusable: u32,
    unconverged: u32,
    bootstraps: u32,
}

impl Stats {
    fn log_and_reset(&mut self, pose: &PoseEstimate, layout: &Layout) {
        info!(
            "pose: {} superframes, {} accepted, {} gated, {} unfusable, {} pre-convergence, \
             {} bootstraps; at ({}, {}) m heading {} rad, variance {} m2, {}/{} anchors surveyed",
            self.superframes,
            self.accepted,
            self.gated,
            self.unfusable,
            self.unconverged,
            self.bootstraps,
            pose.x_m,
            pose.y_m,
            pose.heading_rad,
            pose.position_variance_m2,
            layout.known_count(),
            uwb_protocol::ACTIVE_ANCHOR_COUNT,
        );
        *self = Self::default();
    }
}

/// Runs the fusion filter and publishes a pose per superframe.
#[ariel_os::task]
pub async fn estimate_pose(
    ranges_rx: Receiver<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
    mut imu_rx: WatchReceiver<'static, CriticalSectionRawMutex, IMUTelemetry, 1>,
    mut commanded_duty_rx: WatchReceiver<'static, CriticalSectionRawMutex, f32, 1>,
    mut anchors_rx: WatchReceiver<'static, CriticalSectionRawMutex, AnchorsConfiguration, 1>,
    pose_tx: WatchSender<'static, CriticalSectionRawMutex, PoseEstimate, 2>,
    raw_ranges_tx: Sender<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
) -> ! {
    let mut layout = Layout::default();
    let mut filter = PoseFilter::new(filter_config(&layout));
    let mut stats = Stats::default();
    let mut snapshot = Snapshot::new();
    let mut pending: Option<(u16, Instant)> = None;
    let mut last_imu_us: Option<u64> = None;
    let mut barren = 0u32;
    let mut warned_unsurveyed = false;

    if !layout.is_solvable() {
        info!(
            "pose: no compiled anchor geometry, waiting for a retained {} message",
            crate::topics::ANCHORS_TOPIC
        );
    }

    loop {
        // Cheap enough to poll every time round rather than giving it a `select` branch of its own: a
        // `Watch::try_changed` is a lock and a compare.
        if let Some(config) = anchors_rx.try_changed() {
            layout = anchors::resolve_from_config(&config);
            info!(
                "pose: anchor geometry updated, {}/{} surveyed, robot antenna at {} m",
                layout.known_count(),
                uwb_protocol::ACTIVE_ANCHOR_COUNT,
                layout.robot_antenna_height_m,
            );
            // The antenna height is part of every measurement model, so a new one invalidates the
            // filter's whole linearization history. Cheaper and more honest to start over.
            filter = PoseFilter::new(filter_config(&layout));
            snapshot.clear();
            pending = None;
            warned_unsurveyed = false;
        }

        // Three things can happen next, and waiting for any one of them is the point: a range to fuse,
        // an IMU sample to predict with, or the deadline on a partial snapshot. Prediction must *not*
        // be gated on a range arriving -- that would run the model at the ~18 Hz ranging rate instead
        // of the 100 Hz sampling rate, which is the whole reason for fusing in the first place.
        let deadline = pending.map_or_else(|| Instant::now() + IDLE_TICK, |(_, at)| at);
        match select3(ranges_rx.receive(), imu_rx.changed(), Timer::at(deadline)).await {
            Either3::Second(sample) => {
                let dt_s = match last_imu_us {
                    Some(previous) => {
                        (sample.timestamp_us.saturating_sub(previous) as f32) * 1.0e-6
                    }
                    None => 0.0,
                };
                last_imu_us = Some(sample.timestamp_us);

                // `filtered.gyroscope` already has the AHRS's own zero-rate estimate removed; the
                // filter's `gyro_bias` state tracks whatever that estimate has not, which with a 10 s
                // time constant and updates only while stationary is not nothing.
                let yaw_rate = sample.filtered.gyroscope.z * RADIANS_PER_DEGREE;
                let commanded = commanded_duty_rx.try_get().unwrap_or(0.0) * MAX_SPEED_M_S;
                filter.predict(dt_s, yaw_rate, commanded);
                if sample.filtered.is_stationary {
                    filter.update_zero_velocity(yaw_rate);
                }
                continue;
            }
            Either3::Third(()) => {
                // A partial snapshot whose window has closed: the missing anchors are not coming, so
                // fuse what did arrive rather than waiting for the next superframe to notice.
                if pending.take().is_some() {
                    flush(
                        &mut filter,
                        &snapshot,
                        &layout,
                        &pose_tx,
                        &mut stats,
                        &mut barren,
                    );
                    snapshot.clear();
                    maybe_report(&mut stats, &filter, &layout);
                }
                continue;
            }
            Either3::First(measurement) => {
                if PUBLISH_RAW_RANGES {
                    // Never blocks: a stale range is worth less than the pose it would delay.
                    let _ = raw_ranges_tx.try_send(measurement);
                }

                // A single-sided exchange with an assumed clock ratio can be metres out -- see
                // `RangeMeasurement::clock_ratio_converged`. Publishing those raw is useful for
                // watching convergence; fusing them is not.
                if !measurement.clock_ratio_converged {
                    stats.unconverged += 1;
                    continue;
                }

                let Some(anchor_index) = uwb_protocol::ANCHOR_IDS
                    .iter()
                    .position(|&id| id == measurement.anchor_id)
                else {
                    continue;
                };
                let Some(anchor) = layout.anchors[anchor_index] else {
                    if !warned_unsurveyed {
                        warn!(
                            "pose: ranges from anchor {:04x} but no surveyed position for it; \
                             publish a {} message",
                            measurement.anchor_id,
                            crate::topics::ANCHORS_TOPIC
                        );
                        warned_unsurveyed = true;
                    }
                    continue;
                };

                let corrected_mm = layout.biases[anchor_index].correct(measurement.distance_mm);
                let range_m = corrected_mm as f32 * 1.0e-3;

                // Ranges arrive in per-superframe bursts tagged with the sequence they belong to. A
                // burst from a new sequence means the previous one is over, however incomplete it was.
                match pending {
                    Some((sequence, _)) if sequence == measurement.sequence => {}
                    _ => {
                        if pending.take().is_some() {
                            flush(
                                &mut filter,
                                &snapshot,
                                &layout,
                                &pose_tx,
                                &mut stats,
                                &mut barren,
                            );
                            maybe_report(&mut stats, &filter, &layout);
                        }
                        snapshot.clear();
                        pending = Some((measurement.sequence, Instant::now() + SNAPSHOT_WINDOW));
                    }
                }
                // Cannot overflow while each anchor answers once per superframe; a duplicate is
                // dropped rather than panicking.
                let _ = snapshot.push((anchor, range_m));

                // The common case: all four arrived, so there is nothing left to wait for. Fusing here
                // rather than on the window deadline is what keeps the pose's latency at the burst's
                // own length rather than at `SNAPSHOT_WINDOW`.
                if snapshot.len() == uwb_protocol::ACTIVE_ANCHOR_COUNT {
                    pending = None;
                    flush(
                        &mut filter,
                        &snapshot,
                        &layout,
                        &pose_tx,
                        &mut stats,
                        &mut barren,
                    );
                    snapshot.clear();
                    maybe_report(&mut stats, &filter, &layout);
                }
            }
        }
    }
}

fn maybe_report(stats: &mut Stats, filter: &PoseFilter, layout: &Layout) {
    if stats.superframes >= REPORT_EVERY_SUPERFRAMES {
        let pose = estimate_of(filter, 0);
        stats.log_and_reset(&pose, layout);
    }
}

/// Fuses one superframe's snapshot and publishes the resulting pose.
fn flush(
    filter: &mut PoseFilter,
    snapshot: &Snapshot,
    layout: &Layout,
    pose_tx: &WatchSender<'static, CriticalSectionRawMutex, PoseEstimate, 2>,
    stats: &mut Stats,
    barren: &mut u32,
) {
    stats.superframes += 1;
    if snapshot.is_empty() || !layout.is_solvable() {
        return;
    }

    if !filter.is_initialized() {
        if filter.bootstrap(snapshot) {
            stats.bootstraps += 1;
            *barren = 0;
            info!(
                "pose: bootstrapped from {} ranges at ({}, {}) m",
                snapshot.len(),
                filter.pose().x_m,
                filter.pose().y_m
            );
        } else {
            debug!(
                "pose: {} ranges are not enough to bootstrap, or the geometry is degenerate",
                snapshot.len()
            );
        }
        return;
    }

    let mut used = 0u8;
    for (anchor, range_m) in snapshot.iter() {
        match filter.update_range(anchor, *range_m) {
            RangeOutcome::Accepted { .. } => {
                stats.accepted += 1;
                used += 1;
            }
            RangeOutcome::Rejected { residual_m, .. } => {
                stats.gated += 1;
                debug!("pose: gated a range, residual {} m", residual_m);
            }
            RangeOutcome::NotInitialized => stats.unfusable += 1,
        }
    }

    // A filter whose position has drifted far enough that the gate rejects everything cannot recover on
    // its own -- see `MAX_BARREN_SUPERFRAMES`. Re-bootstrapping is what breaks that loop.
    if used == 0 {
        *barren += 1;
        if *barren >= MAX_BARREN_SUPERFRAMES {
            error!(
                "pose: {} superframes with every range gated out, re-bootstrapping",
                barren
            );
            if filter.bootstrap(snapshot) {
                stats.bootstraps += 1;
            }
            *barren = 0;
        }
    } else {
        *barren = 0;
    }

    pose_tx.send(estimate_of(filter, used));
}

fn estimate_of(filter: &PoseFilter, anchors_used: u8) -> PoseEstimate {
    let pose = filter.pose();
    PoseEstimate {
        x_m: pose.x_m,
        y_m: pose.y_m,
        heading_rad: pose.heading_rad,
        speed_m_s: pose.speed_m_s,
        position_variance_m2: pose.position_variance_m2,
        anchors_used,
        timestamp_us: Instant::now().as_micros(),
    }
}

/// The filter's tuning, with the one field that comes from configuration rather than from defaults.
fn filter_config(layout: &Layout) -> FilterConfig {
    FilterConfig {
        robot_antenna_height_m: layout.robot_antenna_height_m,
        ..FilterConfig::default()
    }
}
