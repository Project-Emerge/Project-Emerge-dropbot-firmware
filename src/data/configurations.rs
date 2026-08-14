// `MotorsConfiguration`/`Configuration` are not constructed directly anywhere yet -- `Deserialize`
// targets for a remote-configuration command that hasn't been wired up to `data::commands` yet, which
// the dead-code lint doesn't credit as a use. Everything else below *is* wired up: see
// `topics::TAG_ASSIGNMENTS_TOPIC` and `topics::ANCHORS_TOPIC`, `tasks::mqtt_client`,
// `tasks::uwb_ranging` and `tasks::pose_estimator`.
#![allow(dead_code)]

use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::device_id::DEVICE_ID_LEN;

#[derive(Serialize, Deserialize)]
pub struct MotorsConfiguration {
    pub ema_filter_alpha: Option<f32>,
    pub max_speed: f32,
}

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub motors: MotorsConfiguration,
}

/// One robot's TDMA slot, as published in a [`TagAssignmentsConfiguration`].
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TagAssignment {
    pub device_id: String<DEVICE_ID_LEN>,
    /// Index into `uwb_protocol::TAG_IDS`.
    pub tag_index: u8,
}

/// A fleet-wide override of `drivers::uwb::tag_id::TAG_ASSIGNMENTS`, published retained on
/// `topics::TAG_ASSIGNMENTS_TOPIC` so the fleet can be reprovisioned without a reflash.
///
/// Whole-fleet in one message rather than one message per robot: it lets a receiver check the
/// *entire* list for a duplicate `device_id` or `tag_index` before trusting any of it (see
/// `drivers::uwb::tag_id::resolve_from_config`), and it makes a reassignment atomic from a
/// subscriber's point of view instead of racing between two retained per-device messages.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TagAssignmentsConfiguration {
    pub assignments: Vec<TagAssignment, 12>,
}

/// One anchor's surveyed position and range calibration.
///
/// Position and calibration together in one entry, rather than two parallel tables, because they are
/// only ever meaningful as a pair: a position without its calibration produces a confidently biased
/// fix, and a calibration without a position cannot be applied to anything. Keyed by `anchor_id`
/// rather than by index, so reordering the list cannot silently reassign one anchor's calibration to
/// another.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct AnchorCalibration {
    /// The anchor's ID, as it appears in `uwb_protocol::ANCHOR_IDS`.
    pub anchor_id: u16,
    /// Position in the arena's world frame, metres.
    pub x: f32,
    pub y: f32,
    /// Height above the floor plane, metres. Not optional: the anchors are mounted well above the
    /// robots on purpose -- robot-to-robot shadowing at floor level is the dominant non-line-of-sight
    /// source in a twelve-robot swarm -- so a range is a 3D hypotenuse and assuming this away biases
    /// every measurement inward by an amount that varies across the floor.
    pub z: f32,
    /// Millimetres to subtract from every range to this anchor. See `uwb_protocol::RangeBias`.
    #[serde(default)]
    pub offset_mm: i32,
    /// Parts per million of range to subtract. See `uwb_protocol::RangeBias`.
    #[serde(default)]
    pub scale_ppm: i32,
}

/// The arena's anchor geometry and calibration, published retained on `topics::ANCHORS_TOPIC`.
///
/// Fleet-shared like the tag assignments, and for a stronger reason: the anchors are physically shared
/// hardware, so two robots holding different geometries for the same arena would produce poses in two
/// different coordinate frames. One retained message is the only representation where that cannot
/// happen.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct AnchorsConfiguration {
    /// Height of a robot's own UWB antenna above the floor plane, metres. Fleet-wide because the
    /// robots are identical.
    pub robot_antenna_height_m: f32,
    pub anchors: Vec<AnchorCalibration, 8>,
}
