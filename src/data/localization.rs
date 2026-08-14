//! Wire types for localization: the raw ranges going in and the pose estimate coming out.
//!
//! Both are split from `data::telemetry` because they publish on their own topics at their own rates
//! -- `/uwb/{ID}` once per accepted range and `/pose/{ID}` once per superframe -- rather than folding
//! into the once-a-second aggregate.

use serde::{Deserialize, Serialize};

/// One accepted range, as computed by the robot from a single-sided two-way exchange.
///
/// Quality fields (first-path/total signal power, the NLOS delta they imply) are not here yet:
/// enabling them costs four extra register reads per exchange, which is worth doing only once the
/// plain range is confirmed working end to end. They land with the fusion estimator, which is what
/// will actually consume them -- an NLOS indication is only useful to something that can weight or
/// reject a measurement because of it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct RangeMeasurement {
    pub anchor_id: u16,
    pub distance_mm: u32,
    pub sequence: u16,
    /// MCU time this exchange's `Poll` went out, microseconds since boot.
    ///
    /// Not the DW3000's own hardware timestamp -- that is in radio ticks, tied to the ranging maths
    /// in `uwb_protocol`, and not comparable across a reset. This is what lets a consumer -- the
    /// fusion estimator, or just a human reading `/uwb/{ID}` -- tell how stale a measurement is
    /// against `Instant::now()`. All four of a superframe's ranges share it, because all four come
    /// from that one transmission.
    pub timestamp_us: u64,
    /// The clock-rate correction applied to this exchange, in parts per billion.
    ///
    /// Published because it is a diagnostic in its own right: it is a direct read on the anchor's
    /// crystal relative to this robot's, so a value drifting with temperature, or one anchor sitting
    /// far from the others, is visible without instrumenting anything else.
    pub clock_ratio_ppb: i32,
    /// Whether that correction came from a measured baseline rather than being assumed.
    ///
    /// **A consumer must not fuse a measurement with this false.** A single-sided exchange does not
    /// cancel the two nodes' clock-rate difference, so an assumed ratio leaves an error of
    /// `k/2 * t_reply` -- metres, not centimetres, at two untrimmed crystals and the reply delay of
    /// the last anchor sub-slot. These are published rather than withheld so the convergence
    /// transient after a radio re-initialization is visible on `/uwb/{ID}` instead of looking like a
    /// dropout.
    pub clock_ratio_converged: bool,
}

/// The robot's own estimate of where it is, from `tasks::pose_estimator`.
///
/// This is the product the whole UWB stack exists to produce, and it is computed onboard rather than
/// on a server: the robot is where the answer is needed for closed-loop control, the latency of a
/// Wi-Fi round trip is larger than the ranging period it would be correcting, and a broker outage
/// should not cost the robot its position.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct PoseEstimate {
    /// Position in the arena's world frame, metres -- the same frame the anchor positions are given in
    /// (`data::configurations::AnchorsConfiguration`).
    pub x_m: f32,
    pub y_m: f32,
    /// Heading in radians, normalized to `(-pi, pi]`, zero along the world x axis.
    ///
    /// Derived from the gyroscope and from course over ground, *not* from the magnetometer: indoors,
    /// next to two motors and whatever steel is in the arena, a magnetic heading is the least reliable
    /// of the three.
    pub heading_rad: f32,
    /// Forward speed in metres per second. Estimated, not measured: the robots have no wheel encoders,
    /// so this comes from the commanded duty cycle through a first-order model, corrected by the
    /// ranges.
    pub speed_m_s: f32,
    /// Trace of the position covariance, square metres -- the number to read as "how much should this
    /// be trusted". It grows while the robot dead-reckons and shrinks as ranges are accepted.
    pub position_variance_m2: f32,
    /// How many of this superframe's ranges survived the outlier gate.
    ///
    /// With only four anchors this is the coverage signal that matters: two or fewer means the fix is
    /// leaning on the motion model, and a consumer doing anything safety-relevant should treat that
    /// differently from a full four.
    pub anchors_used: u8,
    /// MCU time this estimate was produced, microseconds since boot.
    pub timestamp_us: u64,
}
