//! The arena's anchor geometry and range calibration, as this robot knows it.
//!
//! Same two-source pattern as `tag_id`: a compiled table for cold boots with no network, overridden by
//! a retained MQTT message so the arena can be re-surveyed or re-calibrated without a reflash. The
//! difference is which way the default leans.
//!
//! # Why the compiled default is empty
//!
//! [`DEFAULT_ANCHORS`] ships empty, so a robot with no `/config/anchors` message produces **no pose at
//! all** rather than a pose from placeholder geometry. That is deliberate. A wrong anchor position does
//! not degrade the estimate, it relocates it: the filter converges confidently onto a position in the
//! wrong frame, the covariance shrinks as it does, and every consumer -- including anything driving
//! the robot -- is told the fix is good. "No fix" is a state a consumer can handle; "a confident fix in
//! the wrong coordinate frame" is not.
//!
//! Fill the table in for a bench setup by uncommenting the example below; the arena itself should come
//! over MQTT, so all twelve robots are guaranteed to share one frame.

use dropbot_estimation::Anchor;
use uwb_protocol::RangeBias;

use crate::data::configurations::AnchorsConfiguration;

/// Height of this robot's UWB antenna above the floor plane, metres, when no configuration has
/// arrived. Only used alongside [`DEFAULT_ANCHORS`], so only relevant to a bench setup.
pub const DEFAULT_ROBOT_ANTENNA_HEIGHT_M: f32 = 0.06;

/// Compiled fallback geometry, indexed the same way `uwb_protocol::ANCHOR_IDS` is.
///
/// `None` means "this anchor's position is unknown", which is not the same as "this anchor is absent":
/// ranges to it still arrive and are still published raw, they just cannot be fused.
///
/// Example for a 6 x 6 m bench arena with the anchors at the corners, above the robot plane and at
/// deliberately unequal heights (equal heights make the vertical term common-mode, so it cancels out
/// of every difference and stops constraining anything):
///
/// ```ignore
/// pub const DEFAULT_ANCHORS: [Option<Anchor>; ACTIVE_ANCHOR_COUNT] = [
///     Some(Anchor { x_m: 0.0, y_m: 0.0, z_m: 2.00 }), // A001
///     Some(Anchor { x_m: 6.0, y_m: 0.0, z_m: 2.20 }), // A002
///     Some(Anchor { x_m: 6.0, y_m: 6.0, z_m: 1.90 }), // A003
///     Some(Anchor { x_m: 0.0, y_m: 6.0, z_m: 2.10 }), // A004
/// ];
/// ```
pub const DEFAULT_ANCHORS: [Option<Anchor>; uwb_protocol::ACTIVE_ANCHOR_COUNT] = [None; 4];

/// Compiled fallback range calibration, indexed the same way. All `NONE` until each anchor has been
/// measured -- see `uwb_protocol::RangeBias` for the procedure.
pub const DEFAULT_BIASES: [RangeBias; uwb_protocol::ACTIVE_ANCHOR_COUNT] =
    [RangeBias::NONE; uwb_protocol::ACTIVE_ANCHOR_COUNT];

/// `None` delegates to the fleet-wide EKF `range_sigma_m` tuning.
pub const DEFAULT_RANGE_SIGMAS_M: [Option<f32>; uwb_protocol::ACTIVE_ANCHOR_COUNT] = [None; 4];

/// Everything `tasks::pose_estimator` needs to turn ranges into a position.
#[derive(Clone, Copy, Debug)]
pub struct Layout {
    pub anchors: [Option<Anchor>; uwb_protocol::ACTIVE_ANCHOR_COUNT],
    pub biases: [RangeBias; uwb_protocol::ACTIVE_ANCHOR_COUNT],
    pub range_sigmas_m: [Option<f32>; uwb_protocol::ACTIVE_ANCHOR_COUNT],
    pub robot_antenna_height_m: f32,
}

impl Default for Layout {
    fn default() -> Self {
        Self {
            anchors: DEFAULT_ANCHORS,
            biases: DEFAULT_BIASES,
            range_sigmas_m: DEFAULT_RANGE_SIGMAS_M,
            robot_antenna_height_m: DEFAULT_ROBOT_ANTENNA_HEIGHT_M,
        }
    }
}

impl Layout {
    /// How many anchors have a known position.
    pub fn known_count(&self) -> usize {
        self.anchors.iter().filter(|slot| slot.is_some()).count()
    }

    /// Whether there are enough known positions to bootstrap a 2D fix.
    ///
    /// Three, not two: two anchors leave the mirror-image ambiguity about the line joining them, and
    /// an EKF handed the wrong side of it tracks that side forever.
    pub fn is_solvable(&self) -> bool {
        self.known_count() >= 3
    }
}

/// Builds a [`Layout`] from a retained `/config/anchors` message, falling back per anchor.
///
/// Entries are matched by `anchor_id` rather than by position in the list, so a reordered or partial
/// message cannot silently give one anchor another's calibration. An entry naming an ID that is not in
/// `uwb_protocol::ANCHOR_IDS` is ignored rather than rejecting the whole message: an arena with a fifth
/// anchor installed and this firmware built for four should still use the four it knows.
pub fn resolve_from_config(config: &AnchorsConfiguration) -> Layout {
    let mut layout = Layout {
        anchors: DEFAULT_ANCHORS,
        biases: DEFAULT_BIASES,
        range_sigmas_m: DEFAULT_RANGE_SIGMAS_M,
        robot_antenna_height_m: if config.robot_antenna_height_m.is_finite()
            && (0.0..=1.0).contains(&config.robot_antenna_height_m)
        {
            config.robot_antenna_height_m
        } else {
            DEFAULT_ROBOT_ANTENNA_HEIGHT_M
        },
    };

    for entry in &config.anchors {
        let Some(index) = uwb_protocol::ANCHOR_IDS
            .iter()
            .position(|&id| id == entry.anchor_id)
        else {
            continue;
        };
        if !entry.x.is_finite() || !entry.y.is_finite() || !entry.z.is_finite() || entry.z < 0.0 {
            continue;
        }
        layout.anchors[index] = Some(Anchor {
            x_m: entry.x,
            y_m: entry.y,
            z_m: entry.z,
        });
        layout.biases[index] = RangeBias {
            offset_mm: entry.offset_mm.clamp(-2_000, 2_000),
            scale_ppm: entry.scale_ppm.clamp(-100_000, 100_000),
        };
        layout.range_sigmas_m[index] = entry
            .range_sigma_m
            .filter(|sigma| sigma.is_finite())
            .map(|sigma| sigma.clamp(0.02, 0.5));
    }

    layout
}
