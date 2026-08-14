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
//! superframe (~14 Hz). That asymmetry is the whole point of fusing: between range fixes the unicycle
//! model carries the pose forward, so a consumer sees a smooth estimate rather than a 14 Hz staircase,
//! and a superframe that yields no ranges at all degrades to dead reckoning instead of to nothing.
//!
//! Prediction is deliberately *not* gated on a range arriving. Doing that would silently run the model
//! at the ranging rate, which is the one mistake in this task that would still produce a plausible
//! pose.
//!
//! # Why the raw range topic is off by default
//!
//! This filter running onboard is what freed the network: nothing off-board needs the raw ranges to
//! compute a position any more. Four of them per superframe is ~0.5 Mbit/s across twelve robots in
//! small messages, each paying its own MQTT and TCP overhead, against a pose that carries the same
//! information for a twentieth of that. So `/config/estimation.publish_raw_ranges` gates them -- on
//! for a calibration session (they are what `uwb_protocol::RangeBias` is fitted from), off in a
//! running swarm.
//!
//! # Running without the filter
//!
//! `/config/estimation` can turn the fusion off entirely (see
//! [`crate::data::configurations::EstimationConfiguration`]), leaving the pose as the plain
//! trilateration of each superframe's ranges: no prediction between fixes, no outlier gate, nothing
//! carried over from the previous superframe. That is a diagnostic mode, and it is worth having
//! because a filter cannot be checked against itself -- an EKF tracking the wrong place reports the
//! same shrinking covariance as one tracking the right place, and there is no ground truth onboard
//! to tell the two apart. What the ranges say on their own is the comparison.

use ariel_os::log::{debug, error, info, warn};
use ariel_os::time::{Duration, Instant, Timer};
use dropbot_estimation::{Anchor, FilterConfig, Motion, PoseFilter, RangeOutcome, trilaterate};
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender};
use heapless::Vec;
use libm::sqrtf;

use crate::data::calibration::CalibrationCapture;
use crate::data::configurations::{
    AnchorsConfiguration, EstimationConfiguration, LocalizationConfiguration,
};
use crate::data::localization::{PoseEstimate, RangeMeasurement};
use crate::data::telemetry::IMUTelemetry;
use crate::drivers::uwb::anchors::{self, Layout};

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

/// How often to log a summary: roughly every 6 s at the ~14 Hz superframe rate.
const REPORT_EVERY_SUPERFRAMES: u32 = 90;

/// What the periodic summary reports before a first pose has been published.
///
/// All zeros, which is what it used to print by reading a pre-bootstrap `PoseFilter::pose()`. The
/// counters printed alongside it -- `unfusable`, `pre-convergence`, `{}/{} anchors surveyed` -- are
/// what say *why* there is nothing yet.
const NOTHING_PUBLISHED: PoseEstimate = PoseEstimate {
    x_m: 0.0,
    y_m: 0.0,
    heading_rad: 0.0,
    speed_m_s: 0.0,
    position_variance_m2: 0.0,
    anchors_used: 0,
    timestamp_us: 0,
};

/// One superframe's worth of ranges, keyed by anchor index.
///
/// Collected before being fused so the bootstrap has a genuine simultaneous snapshot to trilaterate
/// from -- all four ranges of a superframe come from one transmission, which is exactly what
/// trilateration assumes and what the previous anchor-initiated schedule could not provide.
#[derive(Clone, Copy)]
struct Observation {
    anchor: Anchor,
    range_m: f32,
    sigma_m: f32,
}

type Snapshot = Vec<Observation, { uwb_protocol::ACTIVE_ANCHOR_COUNT }>;

fn range_pairs(snapshot: &Snapshot) -> Vec<(Anchor, f32), { uwb_protocol::ACTIVE_ANCHOR_COUNT }> {
    let mut pairs = Vec::new();
    for observation in snapshot {
        // `pairs` has exactly the same capacity as `snapshot`, so this cannot overflow.
        let _ = pairs.push((observation.anchor, observation.range_m));
    }
    pairs
}

/// Running mean and standard deviation of one residual stream.
///
/// Welford's method rather than accumulating the sum and the sum of squares, because the case worth
/// diagnosing is exactly the one where the naive form loses its precision: a tight spread sitting on
/// a large constant offset. That is what an uncalibrated antenna delay looks like, and telling it
/// apart from genuine noise is the whole point of logging this.
#[derive(Default)]
struct Residuals {
    count: u32,
    mean_m: f32,
    sum_squared_deviation: f32,
}

impl Residuals {
    fn observe(&mut self, residual_m: f32) {
        self.count += 1;
        let before = residual_m - self.mean_m;
        self.mean_m += before / self.count as f32;
        self.sum_squared_deviation += before * (residual_m - self.mean_m);
    }

    /// Population standard deviation, or zero before there are two samples to spread.
    fn sigma_m(&self) -> f32 {
        if self.count < 2 {
            0.0
        } else {
            sqrtf(self.sum_squared_deviation / self.count as f32)
        }
    }
}

#[derive(Default)]
struct Stats {
    superframes: u32,
    accepted: u32,
    gated: u32,
    unfusable: u32,
    unconverged: u32,
    bootstraps: u32,
    /// Residuals of the ranges that survived the gate, and of the ones that did not.
    ///
    /// Split rather than pooled because the shape of the *rejected* set is the diagnostic. A gate
    /// that is doing its job rejects a handful of ranges scattered on both sides of zero; a
    /// systematic range bias rejects most of them, clustered tightly around one positive value. The
    /// two look identical in the `gated` count alone, and they call for completely different fixes
    /// -- the second is a `uwb_protocol::RangeBias` calibration and nothing to do with this filter.
    accepted_residuals: Residuals,
    rejected_residuals: Residuals,
}

impl Stats {
    fn log_and_reset(&mut self, mode: &str, pose: &PoseEstimate, layout: &Layout) {
        info!(
            "pose ({}): {} superframes, {} accepted, {} gated, {} unfusable, {} pre-convergence, \
             {} bootstraps; residual accepted {} +/- {} m, rejected {} +/- {} m; \
             at ({}, {}) m heading {} rad, variance {} m2, {}/{} anchors surveyed",
            mode,
            self.superframes,
            self.accepted,
            self.gated,
            self.unfusable,
            self.unconverged,
            self.bootstraps,
            self.accepted_residuals.mean_m,
            self.accepted_residuals.sigma_m(),
            self.rejected_residuals.mean_m,
            self.rejected_residuals.sigma_m(),
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
    mut commanded_duty_rx: WatchReceiver<'static, CriticalSectionRawMutex, f32, 2>,
    mut anchors_rx: WatchReceiver<'static, CriticalSectionRawMutex, AnchorsConfiguration, 1>,
    mut estimation_rx: WatchReceiver<'static, CriticalSectionRawMutex, EstimationConfiguration, 1>,
    mut localization_rx: WatchReceiver<
        'static,
        CriticalSectionRawMutex,
        LocalizationConfiguration,
        1,
    >,
    mut calibration_capture_rx: WatchReceiver<
        'static,
        CriticalSectionRawMutex,
        Option<CalibrationCapture>,
        2,
    >,
    pose_tx: WatchSender<'static, CriticalSectionRawMutex, PoseEstimate, 2>,
    raw_ranges_tx: Sender<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
) -> ! {
    let mut layout = Layout::default();
    let mut estimation = EstimationConfiguration::default();
    let mut localization = LocalizationConfiguration::default();
    let mut filter = PoseFilter::new(filter_config(&layout, &estimation, &localization));
    let mut stats = Stats::default();
    let mut snapshot = Snapshot::new();
    let mut pending: Option<(u16, Instant)> = None;
    let mut last_imu_us: Option<u64> = None;
    let mut barren = 0u32;
    let mut warned_unsurveyed = false;
    // Fused until a retained `/config/estimation` message says otherwise: the filter is what this
    // task is for, and a robot that never hears from the broker should not be running the
    // diagnostic mode.
    let mut fusion_enabled = estimation.fusion_enabled;
    // What was last put on `pose_tx`, kept only so the periodic summary can report a pose in either
    // mode. Reading it off the filter, as this used to, reports zeros while the fusion is off --
    // that state is not the estimate any more.
    let mut last_pose: Option<PoseEstimate> = None;

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
            filter = PoseFilter::new(filter_config(&layout, &estimation, &localization));
            snapshot.clear();
            pending = None;
            warned_unsurveyed = false;
        }

        // Polled the same way and for the same reason as the geometry above.
        if let Some(config) = estimation_rx.try_changed() {
            let config = config.sanitized();
            let reset_filter = config.fusion_enabled != estimation.fusion_enabled
                || config.filter != estimation.filter;
            let raw_changed = config.publish_raw_ranges != estimation.publish_raw_ranges;
            estimation = config;
            fusion_enabled = estimation.fusion_enabled;

            if reset_filter {
                info!(
                    "pose: fusion {}, sigma {} m, gates +{} / -{} sigma",
                    if fusion_enabled {
                        "enabled"
                    } else {
                        "disabled"
                    },
                    estimation.filter.range_sigma_m,
                    estimation.filter.gate_long,
                    estimation.filter.gate_short,
                );
                // A different measurement/process model invalidates the covariance history. Start
                // clean and let the next solvable snapshot bootstrap against the new parameters.
                filter = PoseFilter::new(filter_config(&layout, &estimation, &localization));
                snapshot.clear();
                pending = None;
                barren = 0;
            }
            if raw_changed {
                info!(
                    "pose: raw UWB publication {}",
                    if estimation.publish_raw_ranges {
                        "enabled"
                    } else {
                        "disabled"
                    }
                );
            }
        }

        if let Some(config) = localization_rx.try_changed() {
            let config = config.sanitized();
            if config != localization {
                localization = config;
                info!(
                    "pose: robot calibration updated, range offset {} mm, full-duty speed {} m/s",
                    localization.range_offset_mm, localization.full_duty_speed_m_s,
                );
                filter = PoseFilter::new(filter_config(&layout, &estimation, &localization));
                snapshot.clear();
                pending = None;
                barren = 0;
            }
        }

        // Three things can happen next, and waiting for any one of them is the point: a range to fuse,
        // an IMU sample to predict with, or the deadline on a partial snapshot. Prediction must *not*
        // be gated on a range arriving -- that would run the model at the ~14 Hz ranging rate instead
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

                // With the fusion off there is nothing to predict *into*: the pose is whatever the
                // last superframe's ranges said and nothing carries it forward. The timestamp above
                // is still tracked, so switching the filter back on starts from a real timestep
                // rather than from however long the diagnostic session lasted.
                if !fusion_enabled {
                    continue;
                }

                // `filtered.gyroscope` already has the AHRS's own zero-rate estimate removed; the
                // filter's `gyro_bias` state tracks whatever that estimate has not, which with a 10 s
                // time constant and updates only while stationary is not nothing.
                let yaw_rate = sample.filtered.gyroscope.z * RADIANS_PER_DEGREE;
                let commanded =
                    commanded_duty_rx.try_get().unwrap_or(0.0) * localization.full_duty_speed_m_s;
                // One reading of the standstill flag driving both halves of what a park is worth: the
                // tighter process model, and the two pseudo-measurements. They are separate calls
                // because they answer different questions -- `Motion` says which model propagated this
                // step, `update_zero_velocity` fuses what standstill measures -- but a robot that got
                // one without the other would be worse off than one that got neither.
                let motion = Motion::from_stationary(sample.filtered.is_stationary);
                filter.predict(dt_s, yaw_rate, commanded, motion);
                if motion == Motion::Still {
                    filter.update_zero_velocity(yaw_rate);
                }
                continue;
            }
            Either3::Third(()) => {
                // A partial snapshot whose window has closed: the missing anchors are not coming, so
                // fuse what did arrive rather than waiting for the next superframe to notice.
                if pending.take().is_some() {
                    last_pose = flush(
                        &mut filter,
                        &snapshot,
                        &layout,
                        &pose_tx,
                        &mut stats,
                        &mut barren,
                        fusion_enabled,
                    )
                    .or(last_pose);
                    snapshot.clear();
                    maybe_report(&mut stats, last_pose, &layout, fusion_enabled);
                }
                continue;
            }
            Either3::First(measurement) => {
                let capture_active = calibration_capture_rx
                    .try_get()
                    .flatten()
                    .is_some_and(|capture| Instant::now().as_micros() < capture.expires_at_us);
                if estimation.publish_raw_ranges || capture_active {
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

                let robot_corrected_mm = localization.correct_range(measurement.distance_mm);
                let corrected_mm = layout.biases[anchor_index].correct(robot_corrected_mm);
                let range_m = corrected_mm as f32 * 1.0e-3;
                let sigma_m =
                    layout.range_sigmas_m[anchor_index].unwrap_or(estimation.filter.range_sigma_m);

                // Ranges arrive in per-superframe bursts tagged with the sequence they belong to. A
                // burst from a new sequence means the previous one is over, however incomplete it was.
                match pending {
                    Some((sequence, _)) if sequence == measurement.sequence => {}
                    _ => {
                        if pending.take().is_some() {
                            last_pose = flush(
                                &mut filter,
                                &snapshot,
                                &layout,
                                &pose_tx,
                                &mut stats,
                                &mut barren,
                                fusion_enabled,
                            )
                            .or(last_pose);
                            maybe_report(&mut stats, last_pose, &layout, fusion_enabled);
                        }
                        snapshot.clear();
                        pending = Some((measurement.sequence, Instant::now() + SNAPSHOT_WINDOW));
                    }
                }
                // Cannot overflow while each anchor answers once per superframe; a duplicate is
                // dropped rather than panicking.
                let _ = snapshot.push(Observation {
                    anchor,
                    range_m,
                    sigma_m,
                });

                // The common case: all four arrived, so there is nothing left to wait for. Fusing here
                // rather than on the window deadline is what keeps the pose's latency at the burst's
                // own length rather than at `SNAPSHOT_WINDOW`.
                if snapshot.len() == uwb_protocol::ACTIVE_ANCHOR_COUNT {
                    pending = None;
                    last_pose = flush(
                        &mut filter,
                        &snapshot,
                        &layout,
                        &pose_tx,
                        &mut stats,
                        &mut barren,
                        fusion_enabled,
                    )
                    .or(last_pose);
                    snapshot.clear();
                    maybe_report(&mut stats, last_pose, &layout, fusion_enabled);
                }
            }
        }
    }
}

fn maybe_report(
    stats: &mut Stats,
    last_pose: Option<PoseEstimate>,
    layout: &Layout,
    fusion_enabled: bool,
) {
    if stats.superframes >= REPORT_EVERY_SUPERFRAMES {
        let mode = if fusion_enabled { "fused" } else { "raw" };
        stats.log_and_reset(mode, &last_pose.unwrap_or(NOTHING_PUBLISHED), layout);
    }
}

/// Turns one superframe's snapshot into a pose and publishes it, returning what was published.
///
/// `None` means nothing went out: too few ranges, no surveyed geometry, or -- with the fusion on --
/// a filter that has not bootstrapped yet. Not publishing is the point in all three cases, since the
/// alternative is a repeat of the previous pose, which downstream reads as a robot standing still
/// rather than as a robot that is not being seen.
fn flush(
    filter: &mut PoseFilter,
    snapshot: &Snapshot,
    layout: &Layout,
    pose_tx: &WatchSender<'static, CriticalSectionRawMutex, PoseEstimate, 2>,
    stats: &mut Stats,
    barren: &mut u32,
    fusion_enabled: bool,
) -> Option<PoseEstimate> {
    stats.superframes += 1;
    if snapshot.is_empty() || !layout.is_solvable() {
        return None;
    }

    if !fusion_enabled {
        // `range_sigma_m` is the filter's tuning rather than this snapshot's, but it is the number
        // this firmware holds for how good a range is, and a raw fix has no covariance of its own to
        // report -- see [`raw_fix`].
        let pose = raw_fix(snapshot, layout, filter.config().range_sigma_m, stats)?;
        pose_tx.send(pose);
        return Some(pose);
    }

    if !filter.is_initialized() {
        let pairs = range_pairs(snapshot);
        if filter.bootstrap(&pairs) {
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
        return None;
    }

    let mut used = 0u8;
    for observation in snapshot.iter() {
        match filter.update_range_with_sigma(
            &observation.anchor,
            observation.range_m,
            observation.sigma_m,
        ) {
            RangeOutcome::Accepted { residual_m } => {
                stats.accepted += 1;
                stats.accepted_residuals.observe(residual_m);
                used += 1;
            }
            RangeOutcome::Rejected { residual_m, .. } => {
                stats.gated += 1;
                stats.rejected_residuals.observe(residual_m);
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
            let pairs = range_pairs(snapshot);
            if filter.bootstrap(&pairs) {
                stats.bootstraps += 1;
            }
            *barren = 0;
        }
    } else {
        *barren = 0;
    }

    let pose = estimate_of(filter, used);
    pose_tx.send(pose);
    Some(pose)
}

/// The position one superframe's ranges give on their own, with nothing carried over from the last.
///
/// This is [`dropbot_estimation::trilaterate`] and nothing else: the same closed-form least squares
/// the filter bootstraps from, published directly instead of being handed to the filter as a
/// starting point. Every range in the snapshot is used, including the ones the outlier gate would
/// have thrown away -- a gate needs a prediction to compare against, and in this mode there is not
/// one.
///
/// Heading and speed come back **zero**. Ranges do not observe either: heading is only ever
/// recovered through the filter's course-over-ground cross-covariances, and speed through its
/// commanded-duty model, so with the filter off nothing estimates them. Zero says that more plainly
/// than a stale value from before the filter was switched off would.
///
/// `position_variance_m2` is the mean squared range residual of the fix rather than a covariance,
/// floored at the range noise the filter is tuned for. It is deliberately in the same units and the
/// same role as the filtered field, so a consumer's trust threshold keeps meaning something, but it
/// measures a different thing: how far the ranges disagree with the position solved from them, which
/// is what a non-line-of-sight range or a bad calibration shows up in. The floor is what keeps a
/// three-range fix -- exactly determined, so its residuals are zero by construction -- from
/// reporting itself as perfect.
fn raw_fix(
    snapshot: &Snapshot,
    layout: &Layout,
    range_sigma_m: f32,
    stats: &mut Stats,
) -> Option<PoseEstimate> {
    let pairs = range_pairs(snapshot);
    let Some((x_m, y_m)) = trilaterate(&pairs, layout.robot_antenna_height_m) else {
        stats.unfusable += 1;
        debug!(
            "pose: {} ranges do not determine a raw fix, or the geometry is degenerate",
            snapshot.len()
        );
        return None;
    };

    let mut sum_squared_residual = 0.0;
    for observation in snapshot.iter() {
        let dx = x_m - observation.anchor.x_m;
        let dy = y_m - observation.anchor.y_m;
        let dz = layout.robot_antenna_height_m - observation.anchor.z_m;
        let residual_m = observation.range_m - sqrtf(dx * dx + dy * dy + dz * dz);
        sum_squared_residual += residual_m * residual_m;
        // Counted as accepted because in this mode they all are: the residual stream in the periodic
        // summary is the reason to be running raw in the first place, and it stays comparable with
        // the fused one this way.
        stats.accepted += 1;
        stats.accepted_residuals.observe(residual_m);
    }

    // `f32::max` returns the other operand against a NaN, so a fix whose residuals went non-finite
    // reports the floor rather than propagating a NaN into a published variance.
    let variance_m2 =
        (sum_squared_residual / snapshot.len() as f32).max(range_sigma_m * range_sigma_m);

    Some(PoseEstimate {
        x_m,
        y_m,
        heading_rad: 0.0,
        speed_m_s: 0.0,
        position_variance_m2: variance_m2,
        // Not "survived the gate" as in the fused mode, since there is no gate here: it is how many
        // ranges the fix was solved from, which is the same coverage signal.
        anchors_used: snapshot.len() as u8,
        timestamp_us: Instant::now().as_micros(),
    })
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

fn filter_config(
    layout: &Layout,
    estimation: &EstimationConfiguration,
    localization: &LocalizationConfiguration,
) -> FilterConfig {
    estimation.filter.as_filter_config(
        layout.robot_antenna_height_m,
        localization.antenna_offset_x_m,
        localization.antenna_offset_y_m,
    )
}
