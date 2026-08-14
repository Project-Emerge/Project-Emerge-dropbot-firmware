// Every type below is wired up: see `topics::TAG_ASSIGNMENTS_TOPIC`, `topics::ANCHORS_TOPIC`,
// `topics::ESTIMATION_TOPIC` and `Topics::robot_config`, `tasks::mqtt_client`,
// `tasks::motor_controller`, `tasks::uwb_ranging` and `tasks::pose_estimator`.

use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::device_id::DEVICE_ID_LEN;
use dropbot_estimation::FilterConfig;

/// How `tasks::motor_controller` drives the motors, as published retained on this robot's
/// `/config/robots/{ID}` topic.
///
/// Per-robot rather than fleet-shared, unlike every other configuration in this module: these are
/// the two knobs whose right value depends on the individual chassis -- how much its drivetrain
/// slack wants smoothing out, and how fast it may be driven -- not on the arena the fleet shares.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct MotorsConfiguration {
    /// Smoothing factor of the driver's exponential moving average over commanded duty, in
    /// `(0, 1]`; `null` turns the filter off and applies each command as it arrives.
    ///
    /// Smaller is smoother and slower to respond. The filter runs once per accepted command rather
    /// than on a fixed tick, so its time constant is set by the *command* rate: at the dashboard's
    /// rate a small alpha is several hundred milliseconds of lag, which is why it is worth being
    /// able to turn off from the dashboard rather than only at compile time.
    #[serde(default = "default_ema_filter_alpha")]
    pub ema_filter_alpha: Option<f32>,
    /// Ceiling on the magnitude of either side's duty cycle, in `[0, 1]`.
    ///
    /// A clamp, not a scale: a command already within the ceiling is applied unchanged, so lowering
    /// this slows the robot's top speed without dulling its response to small commands.
    #[serde(default = "default_max_speed")]
    pub max_speed: f32,
}

const fn default_ema_filter_alpha() -> Option<f32> {
    Some(0.1)
}

const fn default_max_speed() -> f32 {
    1.0
}

impl Default for MotorsConfiguration {
    fn default() -> Self {
        Self {
            ema_filter_alpha: default_ema_filter_alpha(),
            max_speed: default_max_speed(),
        }
    }
}

impl MotorsConfiguration {
    /// The same configuration with every field forced into the range the driver can act on.
    ///
    /// The dashboard validates before publishing, but it is not the only thing that can publish
    /// here, and these values reach the PWM duty registers: an alpha outside `(0, 1]` makes the
    /// EMA diverge instead of converge, and a `max_speed` above 1.0 is a duty cycle over 100%.
    /// A NaN would defeat both `clamp` and the driver's own comparisons, so it falls back to the
    /// default rather than propagating.
    #[must_use]
    pub fn sanitized(self) -> Self {
        Self {
            // `0.0` would freeze the output at its current value forever rather than filtering it,
            // so it is treated as "no useful filter" and mapped to off rather than clamped up into
            // a filter nobody asked for.
            ema_filter_alpha: self
                .ema_filter_alpha
                .filter(|alpha| alpha.is_finite() && *alpha > 0.0)
                .map(|alpha| alpha.min(1.0)),
            max_speed: if self.max_speed.is_finite() {
                self.max_speed.clamp(0.0, 1.0)
            } else {
                default_max_speed()
            },
        }
    }
}

/// One robot's own settings, published retained on `/config/robots/{ID}`.
///
/// The stable envelope for chassis-specific settings. Keeping motor and localization calibration in
/// one retained message makes replacing a robot's complete setup atomic from the subscriber's view.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct RobotConfiguration {
    pub motors: MotorsConfiguration,
    /// Chassis-specific localization calibration. This belongs here rather than on the shared
    /// `/config/anchors` topic because the DWM3000 antenna delay and the duty-to-speed conversion
    /// vary from robot to robot.
    pub localization: LocalizationConfiguration,
}

impl Default for RobotConfiguration {
    fn default() -> Self {
        Self {
            motors: MotorsConfiguration::default(),
            localization: LocalizationConfiguration::default(),
        }
    }
}

/// Per-robot localization calibration, carried inside `/config/robots/{ID}`.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(default)]
pub struct LocalizationConfiguration {
    /// Millimetres subtracted from every raw range before the per-anchor correction is applied.
    /// Positive means this robot's antenna reads long. Prefer calibrating the DW3000 antenna delay
    /// first; this removes the residual without requiring a reflash.
    pub range_offset_mm: i32,
    /// Forward speed represented by an applied mean motor duty of `1.0`, in metres per second.
    /// There are no wheel encoders, so this is the EKF's only chassis-specific speed calibration.
    pub full_duty_speed_m_s: f32,
    /// Forward displacement of the DWM3000 antenna phase centre from the robot pose origin.
    pub antenna_offset_x_m: f32,
    /// Leftward displacement of the DWM3000 antenna phase centre from the robot pose origin.
    pub antenna_offset_y_m: f32,
}

impl Default for LocalizationConfiguration {
    fn default() -> Self {
        Self {
            range_offset_mm: 0,
            full_duty_speed_m_s: 0.5,
            antenna_offset_x_m: 0.0,
            antenna_offset_y_m: 0.0,
        }
    }
}

impl LocalizationConfiguration {
    #[must_use]
    pub fn sanitized(self) -> Self {
        let defaults = Self::default();
        Self {
            // A residual beyond two metres is not an antenna calibration; allowing it would turn a
            // malformed retained message into a plausible-looking but relocated pose.
            range_offset_mm: self.range_offset_mm.clamp(-2_000, 2_000),
            full_duty_speed_m_s: finite_clamp(
                self.full_duty_speed_m_s,
                0.05,
                1.2,
                defaults.full_duty_speed_m_s,
            ),
            // The chassis is 110 mm in diameter. A larger lever arm cannot be on this robot and is
            // almost certainly a unit error in a retained message.
            antenna_offset_x_m: finite_clamp(
                self.antenna_offset_x_m,
                -0.055,
                0.055,
                defaults.antenna_offset_x_m,
            ),
            antenna_offset_y_m: finite_clamp(
                self.antenna_offset_y_m,
                -0.055,
                0.055,
                defaults.antenna_offset_y_m,
            ),
        }
    }

    /// Applies the robot-wide correction and saturates rather than wrapping below zero.
    #[must_use]
    pub fn correct_range(self, raw_mm: u32) -> u32 {
        let corrected = i64::from(raw_mm) - i64::from(self.range_offset_mm);
        u32::try_from(corrected.max(0)).unwrap_or(u32::MAX)
    }
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
    /// Measured LOS standard deviation for this anchor in metres. `null` uses the fleet filter's
    /// `range_sigma_m`. A separate value is important with the robot antenna mounted horizontally:
    /// polarization and floor multipath do not affect all four links equally.
    #[serde(default)]
    pub range_sigma_m: Option<f32>,
}

/// How `tasks::pose_estimator` turns ranges into the pose it publishes, published retained on
/// `topics::ESTIMATION_TOPIC`.
///
/// Exists so the fusion filter can be taken out of the loop without a reflash. Comparing
/// `/pose/{ID}` with the filter on against the same run with it off is the only way to tell a filter
/// that is genuinely smoothing from one that is confidently tracking the wrong place -- an EKF
/// answers with the same shape either way, and onboard there is no ground truth to check it against.
/// Its own topic rather than a field on [`AnchorsConfiguration`], because it says nothing about the
/// arena: the geometry is a survey of shared hardware, this is a choice about how to use it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(default)]
pub struct EstimationConfiguration {
    /// Whether to fuse ranges with the IMU (`true`, the default) or publish the raw trilateration
    /// fix from each superframe's ranges alone (`false`).
    ///
    /// With this false nothing predicts between fixes and nothing rejects an outlier, so the pose
    /// steps at the ~14 Hz superframe rate and every non-line-of-sight range lands in it directly.
    /// That is the point -- it is what the ranges alone say -- but it is a diagnostic mode, not a
    /// cheaper way to run a swarm.
    ///
    /// Defaulted rather than required so that a message which only sets some later field cannot
    /// silently turn the filter off.
    pub fusion_enabled: bool,
    /// Forward accepted ranges to `/uwb/{ID}`. Keep this off during normal swarm operation and turn
    /// it on for one robot while collecting calibration data.
    pub publish_raw_ranges: bool,
    /// EKF process, measurement and gate tuning. Every field is defaulted independently, so an old
    /// retained payload containing only `fusion_enabled` remains valid.
    pub filter: EstimationFilterConfiguration,
}

const fn fusion_enabled_default() -> bool {
    true
}

impl Default for EstimationConfiguration {
    fn default() -> Self {
        Self {
            fusion_enabled: fusion_enabled_default(),
            publish_raw_ranges: false,
            filter: EstimationFilterConfiguration::default(),
        }
    }
}

impl EstimationConfiguration {
    #[must_use]
    pub fn sanitized(self) -> Self {
        Self {
            fusion_enabled: self.fusion_enabled,
            publish_raw_ranges: self.publish_raw_ranges,
            filter: self.filter.sanitized(),
        }
    }
}

/// MQTT representation of [`FilterConfig`], separated so serde can default a partially specified
/// nested object and so invalid broker input is clamped before it reaches the covariance matrix.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(default)]
pub struct EstimationFilterConfiguration {
    pub speed_tau_s: f32,
    pub position_noise_m2_s: f32,
    pub heading_noise_rad2_s: f32,
    pub speed_noise_m2_s3: f32,
    pub position_noise_still_m2_s: f32,
    pub heading_noise_still_rad2_s: f32,
    pub speed_noise_still_m2_s3: f32,
    pub gyro_bias_noise_rad2_s3: f32,
    pub range_sigma_m: f32,
    pub zero_velocity_sigma_m_s: f32,
    pub gyro_bias_sigma_rad_s: f32,
    pub gate_long: f32,
    pub gate_short: f32,
}

impl Default for EstimationFilterConfiguration {
    fn default() -> Self {
        let config = FilterConfig::default();
        Self {
            speed_tau_s: config.speed_tau_s,
            position_noise_m2_s: config.position_noise_m2_s,
            heading_noise_rad2_s: config.heading_noise_rad2_s,
            speed_noise_m2_s3: config.speed_noise_m2_s3,
            position_noise_still_m2_s: config.position_noise_still_m2_s,
            heading_noise_still_rad2_s: config.heading_noise_still_rad2_s,
            speed_noise_still_m2_s3: config.speed_noise_still_m2_s3,
            gyro_bias_noise_rad2_s3: config.gyro_bias_noise_rad2_s3,
            range_sigma_m: config.range_sigma_m,
            zero_velocity_sigma_m_s: config.zero_velocity_sigma_m_s,
            gyro_bias_sigma_rad_s: config.gyro_bias_sigma_rad_s,
            gate_long: config.gate_long,
            gate_short: config.gate_short,
        }
    }
}

impl EstimationFilterConfiguration {
    #[must_use]
    pub fn sanitized(self) -> Self {
        let d = Self::default();
        Self {
            speed_tau_s: finite_clamp(self.speed_tau_s, 0.02, 2.0, d.speed_tau_s),
            position_noise_m2_s: finite_clamp(
                self.position_noise_m2_s,
                1.0e-5,
                1.0,
                d.position_noise_m2_s,
            ),
            heading_noise_rad2_s: finite_clamp(
                self.heading_noise_rad2_s,
                1.0e-6,
                1.0,
                d.heading_noise_rad2_s,
            ),
            speed_noise_m2_s3: finite_clamp(
                self.speed_noise_m2_s3,
                1.0e-4,
                4.0,
                d.speed_noise_m2_s3,
            ),
            position_noise_still_m2_s: finite_clamp(
                self.position_noise_still_m2_s,
                1.0e-8,
                0.02,
                d.position_noise_still_m2_s,
            ),
            heading_noise_still_rad2_s: finite_clamp(
                self.heading_noise_still_rad2_s,
                1.0e-8,
                0.02,
                d.heading_noise_still_rad2_s,
            ),
            speed_noise_still_m2_s3: finite_clamp(
                self.speed_noise_still_m2_s3,
                1.0e-5,
                0.5,
                d.speed_noise_still_m2_s3,
            ),
            gyro_bias_noise_rad2_s3: finite_clamp(
                self.gyro_bias_noise_rad2_s3,
                1.0e-10,
                0.01,
                d.gyro_bias_noise_rad2_s3,
            ),
            range_sigma_m: finite_clamp(self.range_sigma_m, 0.02, 0.5, d.range_sigma_m),
            zero_velocity_sigma_m_s: finite_clamp(
                self.zero_velocity_sigma_m_s,
                0.002,
                0.2,
                d.zero_velocity_sigma_m_s,
            ),
            gyro_bias_sigma_rad_s: finite_clamp(
                self.gyro_bias_sigma_rad_s,
                0.0005,
                0.05,
                d.gyro_bias_sigma_rad_s,
            ),
            gate_long: finite_clamp(self.gate_long, 1.5, 10.0, d.gate_long),
            gate_short: finite_clamp(self.gate_short, 1.5, 15.0, d.gate_short),
        }
    }

    #[must_use]
    pub fn as_filter_config(
        self,
        robot_antenna_height_m: f32,
        robot_antenna_offset_x_m: f32,
        robot_antenna_offset_y_m: f32,
    ) -> FilterConfig {
        let config = self.sanitized();
        FilterConfig {
            speed_tau_s: config.speed_tau_s,
            position_noise_m2_s: config.position_noise_m2_s,
            heading_noise_rad2_s: config.heading_noise_rad2_s,
            speed_noise_m2_s3: config.speed_noise_m2_s3,
            position_noise_still_m2_s: config.position_noise_still_m2_s,
            heading_noise_still_rad2_s: config.heading_noise_still_rad2_s,
            speed_noise_still_m2_s3: config.speed_noise_still_m2_s3,
            gyro_bias_noise_rad2_s3: config.gyro_bias_noise_rad2_s3,
            range_sigma_m: config.range_sigma_m,
            zero_velocity_sigma_m_s: config.zero_velocity_sigma_m_s,
            gyro_bias_sigma_rad_s: config.gyro_bias_sigma_rad_s,
            gate_long: config.gate_long,
            gate_short: config.gate_short,
            robot_antenna_height_m,
            robot_antenna_offset_x_m,
            robot_antenna_offset_y_m,
        }
    }
}

fn finite_clamp(value: f32, min: f32, max: f32, fallback: f32) -> f32 {
    if value.is_finite() {
        value.clamp(min, max)
    } else {
        fallback
    }
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
