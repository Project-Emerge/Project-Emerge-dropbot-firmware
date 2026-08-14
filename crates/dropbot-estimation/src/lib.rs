//! Onboard 2D pose estimation: a trilateration bootstrap followed by a unicycle EKF fusing UWB
//! ranges with the IMU.
//!
//! Host-testable and hardware-free on purpose -- every input is a plain number, so the whole filter
//! is exercised against synthetic trajectories in this crate's tests rather than only on a robot.
//!
//! # What is being estimated, and why this shape
//!
//! The robots sit on a floor plane, so only `x`, `y` and a heading are wanted. The state is
//!
//! ```text
//! [ x, y, psi, v, gyro_bias ]
//! ```
//!
//! -- world position in metres, heading in radians, forward speed in metres per second, and the
//! gyroscope's zero-rate offset in radians per second.
//!
//! Forward speed rather than a velocity vector, because this is a **unicycle**: a differential-drive
//! robot cannot move sideways, and encoding that constraint in the process model is what makes the
//! heading observable at all. A constant-velocity `[x, y, vx, vy]` model would have to infer heading
//! from the velocity direction as a separate step and would happily drift into velocities the chassis
//! cannot produce. Here, `psi` is corrected by every range that disagrees with where the model
//! thought the robot was going -- course over ground, at no extra cost.
//!
//! That matters because the alternative heading sources are both weak. The robots have **no wheel
//! encoders**, so there is no odometry; and the magnetometer is unreliable indoors next to two
//! motors, a battery and whatever steel is in the arena. What is left is the gyroscope, which is
//! excellent over seconds and useless over minutes -- hence `gyro_bias` in the state, corrected
//! whenever the robot is provably still ([`PoseFilter::update_zero_velocity`]).
//!
//! Forward speed is driven by the *commanded* duty cycle through a first-order lag, since that is the
//! only forward-motion input available. An unmodelled slip or stall therefore shows up as a speed
//! error until the next range arrives, which at the protocol's ~14 Hz is at most ~3.5 cm at 0.5 m/s.
//!
//! # Ranges are fused one at a time
//!
//! [`PoseFilter::update_range`] takes a single range, not a snapshot, and applies a scalar update.
//! With only four anchors that is what makes the filter degrade gracefully: a robot shadowed from one
//! anchor keeps being corrected by the other three instead of losing the whole fix, and one bad range
//! can be rejected without discarding its three good siblings.
//!
//! Rejection uses an **asymmetric** normalized-residual gate. Non-line-of-sight propagation only ever
//! makes a range *longer* -- the signal took a detour -- so a range that reads long is much more
//! likely to be wrong than one that reads short, and [`FilterConfig::gate_long`] is correspondingly
//! tighter than [`FilterConfig::gate_short`]. The gate normalizes by the predicted covariance, so it
//! widens by itself while the filter is still uncertain and tightens once it has converged, instead of
//! rejecting everything right after a bootstrap.
//!
//! # Standstill is a different process model, not a quieter one
//!
//! [`FilterConfig::position_noise_m2_s`] and its two companions exist to cover what the unicycle model
//! cannot express: wheel slip, a stall, a `commanded_speed_m_s` whose calibration is off. Every one of
//! those mechanisms needs the wheels to be turning. So [`Motion::Still`] propagates against a second,
//! much tighter set of process noises rather than against a scaled-down version of the same ones.
//!
//! This is what actually holds a parked pose still, and it is worth being precise about why
//! [`PoseFilter::update_zero_velocity`] is not. That update constrains forward speed and gyroscope
//! bias, and neither of their measurement Jacobians touches `x` or `y` -- position stays governed by
//! its own process noise, which at the moving value re-inflates `P` by some 1.4e-3 m^2 per superframe.
//! That keeps the Kalman gain high enough that each range's noise lands more or less directly in the
//! published position. Measured on the parked trajectory in this crate's tests, with 10 cm of range
//! noise: removing the zero-velocity update altogether costs about 5% of the standstill jitter, while
//! switching the process noise removes about 85% of it -- 4.2 cm of spread down to 0.6 cm -- and
//! leaves the moving RMSE unchanged to three figures.
//!
//! The gain is not only in the jitter. A stiffer parked filter is also harder for one lying anchor to
//! drag: a robot that parks in front of another's line of sight and adds 30 cm of non-line-of-sight
//! path moves the parked estimate by 15 cm rather than 27 cm.
//!
//! [`Motion`] is an argument to [`PoseFilter::predict`] rather than state on the filter, so that it
//! cannot go stale. A caller that stopped updating a stored flag would leave a moving robot
//! propagating against a model that says it cannot move -- a failure with no symptom until the robot
//! has driven away from its own estimate.

#![no_std]
// Two of clippy's style lints argue with this file's subject matter rather than with its code.
//
// `needless_range_loop`: these loops walk matrix indices, and the index arithmetic *is* the
// algorithm. Mirroring a triangle onto its transpose, or scattering one column across the matching
// row, says what it means as `p[i][j]` and `p[j][i]` and stops saying it when rewritten as an
// iterator chain over rows.
//
// `neg_cmp_op_on_partial_ord`: every negated float comparison here is load-bearing. `!(x <= y)` and
// `x > y` differ in exactly one case -- when a side is NaN -- and this filter cannot recover once a
// NaN reaches its state, so each guard is written the way round that sends NaN to the branch which
// rejects. Taking clippy's suggestion would quietly invert that on all five of them.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord)]

use libm::{ceilf, fabsf, sincosf, sqrtf};

/// State dimension: `[x, y, psi, v, gyro_bias]`.
pub const N: usize = 5;

const IX: usize = 0;
const IY: usize = 1;
const IPSI: usize = 2;
const IV: usize = 3;
const IB: usize = 4;

const TAU: f32 = core::f32::consts::TAU;

/// Largest magnitude [`wrap_angle`] will try to reduce; see there for why a bigger one is not
/// wrappable rather than merely awkward.
const WRAPPABLE_LIMIT: f32 = 1.0e6;

/// Lower bound on each state's variance, indexed like the state vector.
///
/// See [`PoseFilter::floor_variances`] for why this exists. The values sit well below where each
/// state converges in practice -- position to a few centimetres squared, speed to around `7e-4`,
/// gyroscope bias to around `5e-7` -- so a floor that binds is a signal that something upstream is
/// wrong, not a working part of the tuning.
const VARIANCE_FLOOR: [f32; N] = [1.0e-6, 1.0e-6, 1.0e-6, 1.0e-5, 1.0e-8];

/// A fixed anchor, in world coordinates and metres.
///
/// `z` matters even though the estimate is 2D: the anchors are mounted well above the robot plane --
/// deliberately, since the dominant non-line-of-sight source in a twelve-robot swarm is the robots
/// shadowing each other at floor level -- so a range is a 3D hypotenuse. Projecting it by assuming
/// `z` away would bias every measurement inward by an amount that varies across the floor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Anchor {
    pub x_m: f32,
    pub y_m: f32,
    pub z_m: f32,
}

/// The estimate this filter produces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub x_m: f32,
    pub y_m: f32,
    /// Heading in radians, normalized to `(-pi, pi]`. Zero is the world x axis.
    pub heading_rad: f32,
    pub speed_m_s: f32,
    /// Estimated gyroscope zero-rate offset, radians per second. Exposed because it is a health
    /// signal: a value that keeps growing means the zero-velocity updates are not firing, which means
    /// the robot is never being detected as still.
    pub gyro_bias_rad_s: f32,
    /// Trace of the position block of the covariance, in square metres.
    ///
    /// The single number worth publishing as a quality metric: it grows while the robot dead-reckons
    /// and shrinks as ranges are accepted, so it is directly readable as "how much should this pose be
    /// trusted".
    pub position_variance_m2: f32,
}

/// What happened to one range.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RangeOutcome {
    Accepted {
        /// Residual in metres: measured minus predicted. Positive means the range read long.
        residual_m: f32,
    },
    /// Rejected by the asymmetric gate; the filter was left untouched.
    Rejected {
        residual_m: f32,
        /// `residual / sqrt(S)`, i.e. how many predicted standard deviations out it was.
        normalized: f32,
    },
    /// The filter has no position yet -- call [`PoseFilter::bootstrap`] first.
    NotInitialized,
}

/// Whether the robot was moving over the step being propagated.
///
/// Selects which set of process noises [`PoseFilter::predict`] applies -- see the module docs for why
/// standstill deserves its own set, and why this is an argument rather than filter state.
///
/// `tasks::pose_estimator` derives it from `data::imu`'s `is_stationary`, whose thresholds are
/// deliberately strict, and the asymmetry is the right way round: a false [`Motion::Still`] tightens
/// the process noise on a robot that is actually moving, while a false [`Motion::Moving`] merely gives
/// up the standstill's extra stability for one sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Motion {
    Moving,
    Still,
}

impl Motion {
    /// From `data::imu`'s standstill flag.
    #[must_use]
    pub fn from_stationary(stationary: bool) -> Self {
        if stationary {
            Self::Still
        } else {
            Self::Moving
        }
    }
}

/// Tuning. The defaults are the starting point documented against each field; every one of them is a
/// hardware measurement waiting to happen, not a fundamental constant.
#[derive(Clone, Copy, Debug)]
pub struct FilterConfig {
    /// Time constant of the first-order lag from commanded to actual forward speed, in seconds.
    ///
    /// N20 gear motors on a light chassis settle quickly; this is the number to fit from a step
    /// response once the pose estimate itself is trustworthy enough to measure it.
    pub speed_tau_s: f32,
    /// Process noise on position while [`Motion::Moving`], m^2 per second. Covers everything the
    /// unicycle model cannot express: wheel slip, being nudged, a floor that is not quite flat.
    pub position_noise_m2_s: f32,
    /// Process noise on heading while [`Motion::Moving`], rad^2 per second. Gyroscope angle random
    /// walk plus the model error from integrating a single-axis rate on a robot that pitches slightly
    /// as it accelerates.
    pub heading_noise_rad2_s: f32,
    /// Process noise on forward speed while [`Motion::Moving`], (m/s)^2 per second. Sized against how
    /// wrong the commanded-speed model can be over one superframe, which with no encoders is the
    /// filter's weakest input.
    pub speed_noise_m2_s3: f32,
    /// Process noise on position while [`Motion::Still`], m^2 per second.
    ///
    /// Three orders of magnitude under the moving value, because every mechanism that one covers needs
    /// the wheels to be turning. What is left is whatever could move a parked robot without the IMU
    /// noticing it, and this deployment has no such mechanism -- the robots are not pushed and do not
    /// slide.
    ///
    /// Standstill jitter falls as roughly the fourth root of this, so the last factor of ten buys
    /// little. It is set where the *reported* position variance stays near the centimetre that the
    /// ranging can actually support, rather than at the smallest value that still converges:
    /// [`Pose::position_variance_m2`] is published, and a consumer is entitled to believe it.
    pub position_noise_still_m2_s: f32,
    /// Process noise on heading while [`Motion::Still`], rad^2 per second.
    ///
    /// At the moving value a one-minute park grows the heading variance by 1.2 rad^2, so a filter
    /// comes out of a long standstill with no usable heading at all -- and ranges cannot rebuild one,
    /// since their Jacobian is zero on `psi` and the cross-covariances that would carry a correction
    /// are proportional to a forward speed that is also zero. The robot then has to drive several
    /// metres before its heading means anything again.
    ///
    /// A parked robot's heading does not change, so the honest figure here is the gyroscope's angle
    /// random walk, nearer 1e-7 for this part. This sits well above it to cover what the bias estimate
    /// has not yet caught; measure it from a stationary IMU log rather than trusting the default.
    pub heading_noise_still_rad2_s: f32,
    /// Process noise on forward speed while [`Motion::Still`], (m/s)^2 per second.
    ///
    /// The commanded-speed model is at its most trustworthy here -- commanded zero, actual zero -- so
    /// this is only keeping the speed variance from collapsing onto the zero-velocity update, not
    /// tracking anything. It reopens within a step or two of [`Motion::Moving`] coming back, which is
    /// why a stopped robot does not lag when it pulls away.
    pub speed_noise_still_m2_s3: f32,
    /// Process noise on the gyroscope bias, (rad/s)^2 per second. Thermal drift only, so small: the
    /// bias is nearly constant over a run, which is exactly why it is worth estimating.
    ///
    /// Deliberately not split by [`Motion`], unlike the four above. Thermal drift does not care
    /// whether the wheels are turning, and standstill is the only time the bias is *observable* at
    /// all -- tightening this while parked would slow down the one estimate that a park exists to
    /// improve.
    pub gyro_bias_noise_rad2_s3: f32,
    /// Range measurement noise standard deviation, metres.
    ///
    /// Line-of-sight DW3000 ranging is good to a few centimetres once the antenna delay is calibrated;
    /// this is deliberately looser than that, because an *uncalibrated* constant bias and the
    /// residual clock-ratio error both land here until they are measured out.
    pub range_sigma_m: f32,
    /// Standard deviation of the zero-velocity pseudo-measurement, metres per second. Tight: when the
    /// IMU says the robot is still, it really is still.
    pub zero_velocity_sigma_m_s: f32,
    /// Standard deviation of the gyroscope-bias pseudo-measurement taken while still, rad/s.
    pub gyro_bias_sigma_rad_s: f32,
    /// Gate for a range that reads **long**, in predicted standard deviations.
    ///
    /// Tighter than [`Self::gate_short`] on purpose: non-line-of-sight propagation only lengthens a
    /// range, so this is the side where outliers actually live. See the module docs.
    pub gate_long: f32,
    /// Gate for a range that reads **short**, in predicted standard deviations. Looser, because a
    /// short reading has no comparable physical mechanism behind it -- it is just noise, or the
    /// filter's own position being wrong.
    pub gate_short: f32,
    /// Height of the robot's own UWB antenna above the floor plane, metres.
    pub robot_antenna_height_m: f32,
    /// Forward displacement of the UWB antenna phase centre from the pose origin, metres.
    /// Positive is along the robot's forward axis.
    pub robot_antenna_offset_x_m: f32,
    /// Leftward displacement of the UWB antenna phase centre from the pose origin, metres.
    pub robot_antenna_offset_y_m: f32,
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            speed_tau_s: 0.15,
            position_noise_m2_s: 0.02,
            heading_noise_rad2_s: 0.02,
            speed_noise_m2_s3: 0.5,
            position_noise_still_m2_s: 1.0e-5,
            heading_noise_still_rad2_s: 1.0e-4,
            speed_noise_still_m2_s3: 5.0e-3,
            gyro_bias_noise_rad2_s3: 1.0e-6,
            range_sigma_m: 0.15,
            zero_velocity_sigma_m_s: 0.01,
            gyro_bias_sigma_rad_s: 0.005,
            gate_long: 3.0,
            gate_short: 5.0,
            robot_antenna_height_m: 0.06,
            robot_antenna_offset_x_m: 0.0,
            robot_antenna_offset_y_m: 0.0,
        }
    }
}

/// Two-stage estimator: [`Self::bootstrap`] to get a position at all, then predict/update forever.
#[derive(Clone, Debug)]
pub struct PoseFilter {
    config: FilterConfig,
    /// The three measurement variances, squared once here rather than on every update.
    ///
    /// `config` is fixed for the filter's lifetime -- `tasks::pose_estimator` builds a whole new
    /// `PoseFilter` when the anchor layout changes rather than mutating this one -- so squaring the
    /// configured standard deviations per call was recomputing a constant at 100 Hz.
    range_variance_m2: f32,
    zero_velocity_variance_m2_s2: f32,
    gyro_bias_variance_rad2_s2: f32,
    x: [f32; N],
    p: [[f32; N]; N],
    initialized: bool,
}

impl PoseFilter {
    pub fn new(config: FilterConfig) -> Self {
        Self {
            range_variance_m2: config.range_sigma_m * config.range_sigma_m,
            zero_velocity_variance_m2_s2: config.zero_velocity_sigma_m_s
                * config.zero_velocity_sigma_m_s,
            gyro_bias_variance_rad2_s2: config.gyro_bias_sigma_rad_s * config.gyro_bias_sigma_rad_s,
            config,
            x: [0.0; N],
            p: [[0.0; N]; N],
            initialized: false,
        }
    }

    pub fn config(&self) -> &FilterConfig {
        &self.config
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Fixes an initial position from one simultaneous snapshot of ranges.
    ///
    /// Needs at least three anchors: two would leave the classic mirror-image ambiguity about the line
    /// joining them, and picking the wrong side is a failure the EKF would then happily track forever.
    ///
    /// On a **first** bootstrap, heading is not initialized -- it cannot be, from ranges alone, and a
    /// stationary robot has no course over ground either. It starts at zero with a large variance and
    /// is corrected by the first few metres of motion. `speed` starts at zero, which is true whenever
    /// this is called at standstill and close enough otherwise.
    ///
    /// On a **re-bootstrap** it keeps the heading and speed it had already learnt -- see
    /// [`Self::rebootstrap_position`], which is what a caller recovering a diverged filter is really
    /// asking for.
    ///
    /// Returns `false` and changes nothing if the snapshot is too small or geometrically degenerate
    /// (anchors collinear as seen in plan view, which makes the normal equations singular).
    pub fn bootstrap(&mut self, snapshot: &[(Anchor, f32)]) -> bool {
        let Some((antenna_x_m, antenna_y_m)) =
            trilaterate(snapshot, self.config.robot_antenna_height_m)
        else {
            return false;
        };

        // Trilateration locates the antenna phase centre. The public pose, motion model and motor
        // controller all refer to the chassis centre, so translate the result by the lever arm at
        // the best heading currently available. On a first bootstrap heading is unknown and starts
        // at zero; the range Jacobian below then makes a non-zero lever arm directly observable.
        let heading = if self.initialized { self.x[IPSI] } else { 0.0 };
        let (sin_heading, cos_heading) = sincosf(heading);
        let lever_x = cos_heading * self.config.robot_antenna_offset_x_m
            - sin_heading * self.config.robot_antenna_offset_y_m;
        let lever_y = sin_heading * self.config.robot_antenna_offset_x_m
            + cos_heading * self.config.robot_antenna_offset_y_m;
        let x_m = antenna_x_m - lever_x;
        let y_m = antenna_y_m - lever_y;

        if self.initialized {
            self.rebootstrap_position(x_m, y_m);
            return true;
        }

        self.x = [x_m, y_m, 0.0, 0.0, self.x[IB]];
        self.p = [[0.0; N]; N];
        // Position: a trilateration fix from four noisy ranges is good to a few tens of centimetres,
        // so start at 0.5 m of standard deviation and let the ranges pull it in.
        self.p[IX][IX] = 0.25;
        self.p[IY][IY] = 0.25;
        // Heading: completely unknown. Slightly less than (pi^2 / 3), the variance of a uniform
        // distribution over a full turn, since a variance that large makes the first linearization
        // meaningless anyway.
        self.p[IPSI][IPSI] = 3.0;
        self.p[IV][IV] = 0.25;
        self.p[IB][IB] = 1.0e-2;
        self.initialized = true;
        true
    }

    /// Moves an already-initialized filter to a freshly trilaterated position, keeping what it knows.
    ///
    /// A re-bootstrap means the position estimate has drifted far enough that every range trips the
    /// outlier gate -- see `tasks::pose_estimator`'s barren-superframe counter, which is what calls
    /// this. That is a statement about **position** and nothing else, so resetting the rest of the
    /// state throws away hard-won information for no reason.
    ///
    /// Heading is the expensive one. Ranges do not observe it: the measurement Jacobian is zero on
    /// `psi`, so heading is only ever corrected through the `x`/`y` cross-covariances that
    /// [`Self::predict`] accumulates, and those are proportional to forward speed. A robot that is
    /// stopped, or barely moving, therefore has no path back to a heading at all -- so zeroing it and
    /// setting its variance to 3.0 does not merely lose the estimate, it can strand the filter without
    /// the means to rebuild one. Worse, it closes a loop: a wrong heading makes dead reckoning between
    /// fixes wrong, which trips the gate, which triggers another re-bootstrap.
    ///
    /// Position variance is inflated rather than reset, and the cross-covariances linking position to
    /// the states being kept are dropped, since the new fix is independent of whatever produced them.
    fn rebootstrap_position(&mut self, x_m: f32, y_m: f32) {
        self.x[IX] = x_m;
        self.x[IY] = y_m;

        for i in 0..N {
            self.p[IX][i] = 0.0;
            self.p[i][IX] = 0.0;
            self.p[IY][i] = 0.0;
            self.p[i][IY] = 0.0;
        }
        self.p[IX][IX] = 0.25;
        self.p[IY][IY] = 0.25;

        // Heading and speed survive, but the filter has just been shown to be wrong about where it is,
        // so its confidence in the states that steer dead reckoning should not come through untouched.
        // Inflating -- rather than resetting -- keeps the estimate while widening the gate enough to
        // let the next few ranges correct it.
        self.p[IPSI][IPSI] = (self.p[IPSI][IPSI] * 4.0).min(3.0);
        self.p[IV][IV] = (self.p[IV][IV] * 4.0).min(0.25);
        // Gyroscope bias is a property of the sensor, not of the position fix, and re-learning it
        // costs a stationary period -- so both the estimate and its variance carry across untouched.
    }

    /// Advances the model by `dt_s`.
    ///
    /// `gyro_z_rad_s` is the raw yaw rate in the robot's body frame -- raw, not bias-corrected: the
    /// filter subtracts its own bias estimate, which is the whole point of having one.
    /// `commanded_speed_m_s` is the forward speed the motor controller was told to produce. `motion`
    /// says whether the robot was moving over this step, and picks which set of process noises to
    /// propagate against -- see the module docs.
    pub fn predict(
        &mut self,
        dt_s: f32,
        gyro_z_rad_s: f32,
        commanded_speed_m_s: f32,
        motion: Motion,
    ) {
        // `is_finite` before the sign test, because `dt_s <= 0.0` on its own is false for NaN and
        // `f32::clamp` propagates a NaN rather than trapping it. All three inputs are screened, not
        // just the one with an obvious range: a single NaN reaching `self.p` is unrecoverable -- it
        // spreads to every entry on the next propagation, and neither this crate nor
        // `tasks::pose_estimator` resets the filter on anything but an anchor-layout change. An
        // infinite timestep is refused rather than clamped, since unlike a merely large one it is not
        // a measurement of anything.
        if !self.initialized || !dt_s.is_finite() || dt_s <= 0.0 {
            return;
        }
        if !gyro_z_rad_s.is_finite() || !commanded_speed_m_s.is_finite() {
            return;
        }
        // Clamped for the same reason `data::imu`'s filters clamp theirs: a timestep from a stalled
        // sampler would otherwise be extrapolated as if it were real motion.
        let dt = dt_s.clamp(0.0005, 0.2);

        let psi = self.x[IPSI];
        let v = self.x[IV];
        // One argument reduction for both, rather than two.
        let (sin_psi, cos_psi) = sincosf(psi);
        let yaw_rate = gyro_z_rad_s - self.x[IB];
        // `dt / (tau + dt)`, not `dt / tau`. The latter passes 1 as soon as `dt > tau`, which at
        // `speed_tau_s = 0.15` sits *inside* the clamp above -- so it is reachable exactly in the
        // stalled-sampler case the clamp exists to survive. Past 1 the speed update overshoots its
        // target and `f[IV][IV] = 1 - lag` turns negative, flipping the sign of that state's own
        // Jacobian. This form stays in `(0, 1)` for every positive `dt`, and it is the same
        // first-order blend `data::imu::correction_weight` uses for every other lag in this firmware.
        let lag = dt / (self.config.speed_tau_s + dt);

        // Shared by the state update and the Jacobian, so the two cannot disagree even in the last
        // bit -- float multiplication does not reassociate, so writing `v * cos_psi * dt` in one place
        // and `cos_psi * dt` in the other leaves the compiler no way to make them consistent.
        let cos_dt = cos_psi * dt;
        let sin_dt = sin_psi * dt;
        let v_cos_dt = v * cos_dt;
        let v_sin_dt = v * sin_dt;

        self.x[IX] += v_cos_dt;
        self.x[IY] += v_sin_dt;
        self.x[IPSI] = wrap_angle(psi + yaw_rate * dt);
        self.x[IV] += (commanded_speed_m_s - v) * lag;
        // Bias is a random walk: unchanged in the mean, growing only in covariance.

        // `P = F P F^T + Q`, with `F` written out rather than built.
        //
        // `F` is the identity except for five off-diagonal terms and one replaced diagonal, so ten of
        // its twenty-five entries are nonzero and the rest are structural zeros known here at compile
        // time. Materializing it and running two general 5x5 products spent most of its arithmetic
        // multiplying by those zeros -- and the transpose product had no zero-skip at all, so it ran
        // the full 125 multiply-adds every time. Naming the six terms and expanding the congruence by
        // hand measures at **82 float operations against 400**, counted by routing every operation
        // through a counter and running both forms on the same input.
        //
        // It also drops the data-dependent branch that used to make this function's execution time a
        // function of its input, which is worth having in a task sharing a cooperative executor with
        // the network stack: what this buys is not really the duty cycle, it is a shorter worst-case
        // stretch during which nothing else on that executor can run.
        //
        //   F = [ 1  0  a  b  0 ]      a = -v*sin(psi)*dt    b = cos(psi)*dt
        //       [ 0  1  c  d  0 ]      c =  v*cos(psi)*dt    d = sin(psi)*dt
        //       [ 0  0  1  0  e ]      e = -dt
        //       [ 0  0  0  g  0 ]      g =  1 - lag
        //       [ 0  0  0  0  1 ]
        let a = -v_sin_dt;
        let b = cos_dt;
        let c = v_cos_dt;
        let d = sin_dt;
        let e = -dt;
        let g = 1.0 - lag;

        // Stage one, `A = F P`. Not symmetric, so all twenty-five entries are needed. Row IB is a
        // straight copy, which is why it is read from `self.p` below rather than recomputed.
        let p = self.p;
        let mut fp = p;
        for j in 0..N {
            fp[IX][j] = p[IX][j] + (a * p[IPSI][j] + b * p[IV][j]);
            fp[IY][j] = p[IY][j] + (c * p[IPSI][j] + d * p[IV][j]);
            fp[IPSI][j] = p[IPSI][j] + e * p[IB][j];
            fp[IV][j] = g * p[IV][j];
            // fp[IB][j] is p[IB][j], already there from the copy above.
        }

        // Stage two, `P' = A F^T`. The result is symmetric, so only the upper triangle is computed and
        // each value is stored into both slots -- which makes the two triangles bit-identical rather
        // than merely equal to within rounding, and retires the `symmetrize` pass that used to be
        // needed to repair exactly that drift.
        //
        // The parenthesisation is load-bearing. Rust folds `x + y + z` as `(x + y) + z`, so grouping
        // the two dt-scaled corrections together and adding the dominant term last keeps them from
        // being rounded away against a much larger accumulator: this matrix's diagonal spans about six
        // orders of magnitude, from a bootstrap heading variance of 3.0 down to a converged gyroscope
        // bias variance near 5e-7.
        // One loop per column, each running only down to the diagonal: column `j` needs rows `0..=j`
        // and the rest of it is the mirror of a row already written. That is fifteen entries rather
        // than twenty-five, and the ten it skips are the ten it would immediately overwrite.
        let mut next = [[0.0f32; N]; N];
        for i in 0..=IX {
            let value = fp[i][IX] + (a * fp[i][IPSI] + b * fp[i][IV]);
            next[i][IX] = value;
            next[IX][i] = value;
        }
        for i in 0..=IY {
            let value = fp[i][IY] + (c * fp[i][IPSI] + d * fp[i][IV]);
            next[i][IY] = value;
            next[IY][i] = value;
        }
        for i in 0..=IPSI {
            let value = fp[i][IPSI] + e * fp[i][IB];
            next[i][IPSI] = value;
            next[IPSI][i] = value;
        }
        for i in 0..=IV {
            let value = g * fp[i][IV];
            next[i][IV] = value;
            next[IV][i] = value;
        }
        for i in 0..=IB {
            let value = fp[i][IB];
            next[i][IB] = value;
            next[IB][i] = value;
        }
        self.p = next;

        // A parked robot is a different process model, not a quieter one: everything the moving
        // noises cover -- slip, a stall, a mis-calibrated speed constant -- needs the wheels to be
        // turning. See the module docs for what this is worth, and for why the zero-velocity update
        // alone cannot do it.
        let (position_noise, heading_noise, speed_noise) = match motion {
            Motion::Moving => (
                self.config.position_noise_m2_s,
                self.config.heading_noise_rad2_s,
                self.config.speed_noise_m2_s3,
            ),
            Motion::Still => (
                self.config.position_noise_still_m2_s,
                self.config.heading_noise_still_rad2_s,
                self.config.speed_noise_still_m2_s3,
            ),
        };
        self.p[IX][IX] += position_noise * dt;
        self.p[IY][IY] += position_noise * dt;
        self.p[IPSI][IPSI] += heading_noise * dt;
        self.p[IV][IV] += speed_noise * dt;
        // Not switched on `motion`: thermal drift does not care whether the wheels are turning, and
        // standstill is the only time this state is observable at all.
        self.p[IB][IB] += self.config.gyro_bias_noise_rad2_s3 * dt;
    }

    /// Fuses one range to one anchor.
    pub fn update_range(&mut self, anchor: &Anchor, range_m: f32) -> RangeOutcome {
        self.update_range_with_sigma(anchor, range_m, self.config.range_sigma_m)
    }

    /// Fuses one range with a measurement standard deviation specific to this observation.
    ///
    /// The horizontal DWM3000 mounting makes link quality depend on robot heading and anchor
    /// orientation, so one fleet-wide variance is not always honest. The ordinary
    /// [`Self::update_range`] remains the convenient default; this variant is what the firmware uses
    /// when `/config/anchors` carries a measured per-anchor LOS sigma.
    pub fn update_range_with_sigma(
        &mut self,
        anchor: &Anchor,
        range_m: f32,
        range_sigma_m: f32,
    ) -> RangeOutcome {
        if !self.initialized {
            return RangeOutcome::NotInitialized;
        }

        let (predicted, hx, hy, hpsi) = range_model(&self.config, &self.x, anchor);
        // Every test in this function is written negated, so that a NaN -- in `range_m`, in the state,
        // or in the covariance -- lands in a `Rejected` arm. The natural phrasing accepts instead:
        // comparisons against NaN are all false, so `predicted < 1e-3`, `s <= 0.0` and
        // `fabsf(normalized) > limit` would each wave it through to `apply_scalar_update`, which writes
        // it into `self.x` and `self.p`. From there `s` is NaN on every later call, so every later gate
        // accepts too, and the filter never returns a finite pose again. Nothing in this crate or in
        // `tasks::pose_estimator` resets it on anything but an anchor-layout change, so there is no
        // recovering: this has to fail closed.
        if !(predicted >= 1.0e-3) {
            // Standing exactly under an anchor: the range gradient is undefined there, and no useful
            // information can be extracted from this measurement.
            return RangeOutcome::Rejected {
                residual_m: range_m - predicted,
                normalized: f32::INFINITY,
            };
        }

        // A range is measured at the antenna rather than at the pose origin. A non-zero lever arm
        // therefore gives it a heading derivative as well as the two position derivatives.
        let mut ph = [0.0f32; N];
        for (i, entry) in ph.iter_mut().enumerate() {
            *entry = self.p[i][IX] * hx + self.p[i][IY] * hy + self.p[i][IPSI] * hpsi;
        }

        let residual = range_m - predicted;
        if !range_sigma_m.is_finite() || range_sigma_m <= 0.0 {
            return RangeOutcome::Rejected {
                residual_m: residual,
                normalized: f32::INFINITY,
            };
        }
        let range_variance_m2 = if range_sigma_m == self.config.range_sigma_m {
            self.range_variance_m2
        } else {
            range_sigma_m * range_sigma_m
        };
        let s = hx * ph[IX] + hy * ph[IY] + hpsi * ph[IPSI] + range_variance_m2;
        if !(s > 0.0) {
            return RangeOutcome::Rejected {
                residual_m: residual,
                normalized: f32::INFINITY,
            };
        }

        // Asymmetric: a long reading is the one with a physical mechanism behind it. See the module
        // docs.
        let limit = if residual > 0.0 {
            self.config.gate_long
        } else {
            self.config.gate_short
        };
        // Squared, so the accepted path -- the common one -- never pays for a square root. With
        // `s > 0` this is the same test as `|residual| / sqrt(s) > limit`, and the root is only worth
        // taking in the rejection arm, which reports the normalized value.
        if !(residual * residual <= limit * limit * s) {
            return RangeOutcome::Rejected {
                residual_m: residual,
                normalized: residual / sqrtf(s),
            };
        }

        self.apply_scalar_update(&ph, s, residual);
        RangeOutcome::Accepted {
            residual_m: residual,
        }
    }

    /// Tells the filter the robot is provably stationary.
    ///
    /// Two pseudo-measurements: forward speed is zero, and whatever the gyroscope is reading right now
    /// *is* its bias. The second is what keeps heading from drifting over a run, and it is the reason
    /// `data::imu`'s `is_stationary` flag is worth publishing at all.
    ///
    /// It is *not* what holds a parked position still, which is the intuition worth heading off: both
    /// pseudo-measurements have a Jacobian that is zero on `x` and `y`, so neither can stop the
    /// position wandering under its own process noise. Passing [`Motion::Still`] to [`Self::predict`]
    /// is what does that, and the two belong together at every call site.
    pub fn update_zero_velocity(&mut self, gyro_z_rad_s: f32) {
        if !self.initialized || !gyro_z_rad_s.is_finite() {
            return;
        }

        let speed_variance = self.zero_velocity_variance_m2_s2;
        let speed_residual = -self.x[IV];
        self.apply_unit_state_update(IV, speed_variance, speed_residual);

        let bias_variance = self.gyro_bias_variance_rad2_s2;
        let bias_residual = gyro_z_rad_s - self.x[IB];
        self.apply_unit_state_update(IB, bias_variance, bias_residual);
    }

    pub fn pose(&self) -> Pose {
        Pose {
            x_m: self.x[IX],
            y_m: self.x[IY],
            heading_rad: self.x[IPSI],
            speed_m_s: self.x[IV],
            gyro_bias_rad_s: self.x[IB],
            position_variance_m2: self.p[IX][IX] + self.p[IY][IY],
        }
    }

    /// `x += K * residual`, `P -= K * (P H^T)^T`, with `K = P H^T / S`.
    ///
    /// Takes `P H^T` rather than `H`, since every caller has already computed it to get `S`.
    fn apply_scalar_update(&mut self, ph: &[f32; N], s: f32, residual: f32) {
        // One reciprocal rather than five divisions. `__divsf3` is the most expensive of the basic
        // soft-float routines on this target, and the gain is the same `ph[i] / s` for every state.
        let inv_s = 1.0 / s;
        let weight = residual * inv_s;
        for (state, ph_i) in self.x.iter_mut().zip(ph.iter()) {
            *state += ph_i * weight;
        }
        self.x[IPSI] = wrap_angle(self.x[IPSI]);

        // `K (P H^T)^T` is `ph ph^T / s`, which is symmetric. Computing all 25 entries therefore
        // evaluated every off-diagonal twice, down two different rounding paths, and `symmetrize` then
        // averaged the two answers back together. Storing one value into both slots is both cheaper
        // and stronger: the two triangles come out bit-identical rather than merely close, so there is
        // nothing left for a symmetrization pass to repair.
        for i in 0..N {
            let scaled = ph[i] * inv_s;
            for j in i..N {
                let updated = self.p[i][j] - scaled * ph[j];
                self.p[i][j] = updated;
                self.p[j][i] = updated;
            }
        }
        self.floor_variances();
    }

    /// Fuses a pseudo-measurement of a single state, i.e. one where `H` is the unit vector `e_index`.
    ///
    /// Worth a path of its own rather than building an `h` and calling [`Self::apply_scalar_update`],
    /// because for a unit `H` the general form collapses. `P H^T` is just column `index` of `P`,
    /// `H P H^T` is one diagonal entry, and the whole of row and column `index` -- the diagonal
    /// included -- comes out scaled by a single factor:
    ///
    /// ```text
    /// P'[i][index] = P[i][index] - P[i][index] * P[index][index] / S
    ///              = P[i][index] * (S - P[index][index]) / S
    ///              = P[i][index] * R / S
    /// ```
    ///
    /// That matters for more than the arithmetic it saves. This state's variance ends up *multiplied*
    /// by a positive factor instead of having a subtraction land on it, so cancellation cannot drive
    /// it negative. That is the guarantee a Joseph-form update would have been reached for, and it
    /// lands on precisely the two states where nothing else would notice it going wrong -- see
    /// [`Self::floor_variances`]. Joseph would have cost more per update than the whole covariance
    /// propagation above saves. This costs less than the general form it replaces.
    ///
    /// Measured over the pair of pseudo-measurements [`Self::update_zero_velocity`] runs: **92 float
    /// operations against 314**, with divisions weighted at three times a multiply to reflect what
    /// `__divsf3` costs against `__mulsf3` on a soft-float target. Ten of those divisions become two.
    fn apply_unit_state_update(&mut self, index: usize, r: f32, residual: f32) {
        let s = self.p[index][index] + r;
        if !(s > 0.0) || !residual.is_finite() {
            return;
        }
        let inv_s = 1.0 / s;

        // Taken before anything below overwrites it, since the scaling writes into this same column.
        let mut column = [0.0f32; N];
        for (entry, row) in column.iter_mut().zip(self.p.iter()) {
            *entry = row[index];
        }

        let weight = residual * inv_s;
        for (state, column_i) in self.x.iter_mut().zip(column.iter()) {
            *state += column_i * weight;
        }
        self.x[IPSI] = wrap_angle(self.x[IPSI]);

        let alpha = r * inv_s;
        for i in 0..N {
            let scaled = column[i] * alpha;
            self.p[i][index] = scaled;
            self.p[index][i] = scaled;
        }
        // Everything off that row and column takes the ordinary downdate.
        for i in 0..N {
            if i == index {
                continue;
            }
            let gain = column[i] * inv_s;
            for j in i..N {
                if j == index {
                    continue;
                }
                let updated = self.p[i][j] - gain * column[j];
                self.p[i][j] = updated;
                self.p[j][i] = updated;
            }
        }
        self.floor_variances();
    }

    /// Holds each variance up off its floor after a measurement update.
    ///
    /// The short-form downdate above is algebraically exact but only conditionally stable in `f32`.
    /// It subtracts, so rounding can leave a diagonal entry at or below zero. A negative variance is
    /// not an inaccurate answer but a meaningless one: the gain derived from it pulls the state the
    /// wrong way.
    ///
    /// Nothing downstream would report it, either. `update_range`'s `h` is zero on heading, speed and
    /// bias, so `S` never involves those three and the `s > 0` guard cannot see them. A negative
    /// `P[IPSI][IPSI]` would simply sit there.
    ///
    /// The floors are an order of magnitude or more below where each state actually settles, so this
    /// does not fire in normal operation. It is a backstop against a rounding path going wrong, not a
    /// tuning parameter -- if a floor is ever observed to bind, the tuning above it is what to look
    /// at. Written negated so that a NaN diagonal is also caught and replaced.
    fn floor_variances(&mut self) {
        for (i, floor) in VARIANCE_FLOOR.iter().enumerate() {
            if !(self.p[i][i] >= *floor) {
                self.p[i][i] = *floor;
            }
        }
    }
}

/// Predicted anchor range and its non-zero state derivatives `(x, y, heading)`.
fn range_model(config: &FilterConfig, state: &[f32; N], anchor: &Anchor) -> (f32, f32, f32, f32) {
    let psi = state[IPSI];
    let (sin_psi, cos_psi) = sincosf(psi);
    let lever_x =
        cos_psi * config.robot_antenna_offset_x_m - sin_psi * config.robot_antenna_offset_y_m;
    let lever_y =
        sin_psi * config.robot_antenna_offset_x_m + cos_psi * config.robot_antenna_offset_y_m;
    let dx = state[IX] + lever_x - anchor.x_m;
    let dy = state[IY] + lever_y - anchor.y_m;
    let dz = config.robot_antenna_height_m - anchor.z_m;
    let predicted = sqrtf(dx * dx + dy * dy + dz * dz);

    if !(predicted >= 1.0e-3) {
        return (predicted, 0.0, 0.0, 0.0);
    }

    let inv_predicted = 1.0 / predicted;
    let antenna_dx_dpsi =
        -sin_psi * config.robot_antenna_offset_x_m - cos_psi * config.robot_antenna_offset_y_m;
    let antenna_dy_dpsi =
        cos_psi * config.robot_antenna_offset_x_m - sin_psi * config.robot_antenna_offset_y_m;
    (
        predicted,
        dx * inv_predicted,
        dy * inv_predicted,
        (dx * antenna_dx_dpsi + dy * antenna_dy_dpsi) * inv_predicted,
    )
}

/// Closed-form 2D position from a snapshot of ranges, by linear least squares on the squared ranges.
///
/// Subtracting the first anchor's equation from each of the others cancels the quadratic term and
/// leaves a linear system in `(x, y)`, which for three or more anchors is solved by 2x2 normal
/// equations. That is the standard bootstrap: it needs no starting guess, which is exactly what an
/// EKF cannot provide for itself.
///
/// Each range is first projected onto the floor plane using the known height difference. A range
/// shorter than that height difference is geometrically impossible -- noise, or an uncalibrated
/// antenna delay reading short -- and is clamped to zero rather than producing a NaN.
///
/// Returns `None` for fewer than three anchors, or when the anchors are collinear in plan view, which
/// makes the normal equations singular.
pub fn trilaterate(snapshot: &[(Anchor, f32)], robot_height_m: f32) -> Option<(f32, f32)> {
    if snapshot.len() < 3 {
        return None;
    }

    let planar = |(anchor, range): &(Anchor, f32)| -> f32 {
        let dz = robot_height_m - anchor.z_m;
        let squared = range * range - dz * dz;
        if squared > 0.0 {
            squared
        } else {
            0.0
        }
    };

    let (reference, _) = &snapshot[0];
    let rho0 = planar(&snapshot[0]);
    let k0 = reference.x_m * reference.x_m + reference.y_m * reference.y_m;

    // Normal equations for A^T A [x y]^T = A^T b, accumulated in place.
    let (mut a11, mut a12, mut a22) = (0.0f32, 0.0f32, 0.0f32);
    let (mut b1, mut b2) = (0.0f32, 0.0f32);
    for entry in &snapshot[1..] {
        let (anchor, _) = entry;
        let ax = 2.0 * (anchor.x_m - reference.x_m);
        let ay = 2.0 * (anchor.y_m - reference.y_m);
        let ki = anchor.x_m * anchor.x_m + anchor.y_m * anchor.y_m;
        let bi = rho0 - planar(entry) + ki - k0;

        a11 += ax * ax;
        a12 += ax * ay;
        a22 += ay * ay;
        b1 += ax * bi;
        b2 += ay * bi;
    }

    let det = a11 * a22 - a12 * a12;
    // Scaled against the matrix's own magnitude rather than an absolute epsilon, so the test means
    // "collinear" at any room size.
    if fabsf(det) < 1.0e-6 * (a11 * a22 + 1.0) {
        return None;
    }
    Some(((a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det))
}

/// Normalizes an angle to `(-pi, pi]`, in constant time.
///
/// The obvious `while angle > PI { angle -= TAU }` is not merely slow on a large input, it does not
/// terminate. `f32` has a 24-bit mantissa, so once `|angle|` reaches about `2^27` the ulp exceeds
/// `2 * TAU` and `angle - TAU` rounds straight back to `angle` -- an infinite loop, in a task with no
/// watchdog behind it. `+inf` hangs the same way for the same reason. Below that threshold it merely
/// degenerates: at `angle = 1e7` it is some 1.6 million soft-float subtractions, which on this MCU is
/// most of a second with the main executor frozen solid.
///
/// Neither call site can promise a small input. [`PoseFilter::predict`] wraps
/// `psi + (gyro - bias) * dt` straight off the IMU, and [`PoseFilter::apply_scalar_update`] wraps a
/// heading that a gain of `ph / s` has just moved by an unbounded amount if `s` collapsed.
///
/// A non-finite input has no wrapped form to return, so it comes back unchanged rather than as the
/// NaN that `inf - inf` would otherwise produce here -- the caller's own non-finite guards decide
/// what to do about it, and silently manufacturing a NaN inside a normalization would hide that.
pub fn wrap_angle(angle: f32) -> f32 {
    if !angle.is_finite() {
        return angle;
    }
    // Past this magnitude an `f32` ulp is a good fraction of a turn -- at 1e6 radians it is about
    // 0.12 rad, and by `f32::MAX` it is wider than a full cycle many times over -- so the input no
    // longer says where in the cycle it sits and the subtraction below cancels into noise. There is no
    // right answer to give back there. Returning an in-range one anyway keeps the postcondition that
    // every caller, and the published `heading_rad`, is entitled to assume.
    if fabsf(angle) > WRAPPABLE_LIMIT {
        return 0.0;
    }
    // `ceilf(x - 0.5)` rounds halves *down*, which is what puts the boundary where the loop above put
    // it: `+pi` stays `+pi` and `-pi` comes back as `+pi`, i.e. the range is open at the bottom and
    // closed at the top. `roundf` is the obvious choice here and is wrong -- it rounds halves away
    // from zero, so it maps `+pi` to `-pi` *and* `-pi` to `+pi`, leaving the two ends inconsistent
    // with each other rather than merely shifted.
    angle - TAU * ceilf(angle / TAU - 0.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    use libm::{cosf, sinf};

    /// Only the tests need this: [`wrap_angle`] works in whole turns and is written against `TAU`
    /// alone, but the bounds it promises are naturally phrased in half-turns.
    const PI: f32 = core::f32::consts::PI;

    // The dense linear algebra this filter used to run on, kept verbatim as the reference the sparse
    // hot paths are checked against. Every one of these was production code until the specialized
    // versions replaced it, so the oracle is not a fresh implementation that could be wrong in its own
    // way -- it *is* the previous shipping behaviour, and `sparse_predict_matches_the_dense_reference`
    // is the proof that nothing moved when it was retired.

    fn reference_identity() -> [[f32; N]; N] {
        let mut m = [[0.0f32; N]; N];
        for i in 0..N {
            m[i][i] = 1.0;
        }
        m
    }

    fn reference_mat_mul(a: &[[f32; N]; N], b: &[[f32; N]; N]) -> [[f32; N]; N] {
        let mut out = [[0.0f32; N]; N];
        for i in 0..N {
            for k in 0..N {
                let aik = a[i][k];
                if aik == 0.0 {
                    continue;
                }
                for j in 0..N {
                    out[i][j] += aik * b[k][j];
                }
            }
        }
        out
    }

    /// `a * b^T`.
    fn reference_mat_mul_transpose(a: &[[f32; N]; N], b: &[[f32; N]; N]) -> [[f32; N]; N] {
        let mut out = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..N {
                let mut sum = 0.0;
                for k in 0..N {
                    sum += a[i][k] * b[j][k];
                }
                out[i][j] = sum;
            }
        }
        out
    }

    fn reference_mat_vec(m: &[[f32; N]; N], v: &[f32; N]) -> [f32; N] {
        let mut out = [0.0f32; N];
        for i in 0..N {
            let mut sum = 0.0;
            for j in 0..N {
                sum += m[i][j] * v[j];
            }
            out[i] = sum;
        }
        out
    }

    fn reference_dot(a: &[f32; N], b: &[f32; N]) -> f32 {
        let mut sum = 0.0;
        for i in 0..N {
            sum += a[i] * b[i];
        }
        sum
    }

    fn reference_symmetrize(p: &mut [[f32; N]; N]) {
        for i in 0..N {
            for j in (i + 1)..N {
                let mean = 0.5 * (p[i][j] + p[j][i]);
                p[i][j] = mean;
                p[j][i] = mean;
            }
        }
    }

    /// The angle normalization this crate shipped before [`wrap_angle`] became branch-free.
    ///
    /// Only ever called with `|angle| < 100` -- it does not terminate otherwise, which is the whole
    /// reason it is no longer production code.
    fn reference_wrap_angle(mut angle: f32) -> f32 {
        while angle > PI {
            angle -= TAU;
        }
        while angle <= -PI {
            angle += TAU;
        }
        angle
    }

    /// `P = F P F^T + Q` the way `predict` used to compute it, for the differential test.
    fn reference_propagate(
        p: &[[f32; N]; N],
        f: &[[f32; N]; N],
        q_diagonal: &[f32; N],
    ) -> [[f32; N]; N] {
        let fp = reference_mat_mul(f, p);
        let mut out = reference_mat_mul_transpose(&fp, f);
        for i in 0..N {
            out[i][i] += q_diagonal[i];
        }
        reference_symmetrize(&mut out);
        out
    }

    /// Four anchors at the corners of a 6 x 6 m arena, 2 m above the floor.
    ///
    /// Unequal heights on purpose: it is what the deployment notes call for, and it keeps the vertical
    /// geometry from being a common-mode term that cancels out of every difference.
    fn arena() -> [Anchor; 4] {
        [
            Anchor {
                x_m: 0.0,
                y_m: 0.0,
                z_m: 2.0,
            },
            Anchor {
                x_m: 6.0,
                y_m: 0.0,
                z_m: 2.2,
            },
            Anchor {
                x_m: 6.0,
                y_m: 6.0,
                z_m: 1.9,
            },
            Anchor {
                x_m: 0.0,
                y_m: 6.0,
                z_m: 2.1,
            },
        ]
    }

    fn true_range(anchor: &Anchor, x: f32, y: f32, height: f32) -> f32 {
        let dx = x - anchor.x_m;
        let dy = y - anchor.y_m;
        let dz = height - anchor.z_m;
        sqrtf(dx * dx + dy * dy + dz * dz)
    }

    /// Deterministic pseudo-random noise. A tiny LCG rather than `rand`: this crate has no
    /// dev-dependencies, and a fixed seed makes a failure reproducible instead of flaky.
    struct Noise(u32);

    impl Noise {
        fn new() -> Self {
            Self(0x2545_F491)
        }

        /// Roughly uniform in `[-1, 1)`.
        fn uniform(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            ((self.0 >> 8) as f32 / 8_388_608.0) - 1.0
        }

        /// Approximately Gaussian with the given standard deviation, by summing three uniforms
        /// (Irwin-Hall). Good enough to exercise a gate; nobody is estimating tail probabilities here.
        fn gaussian(&mut self, sigma: f32) -> f32 {
            (self.uniform() + self.uniform() + self.uniform()) * sigma
        }
    }

    #[test]
    fn trilateration_recovers_an_exact_position() {
        let anchors = arena();
        let height = 0.06;
        let (tx, ty) = (2.0f32, 4.5f32);
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], tx, ty, height)));

        let (x, y) = trilaterate(&snapshot, height).expect("four good ranges must solve");
        assert!(
            fabsf(x - tx) < 0.01 && fabsf(y - ty) < 0.01,
            "expected ({tx}, {ty}), got ({x}, {y})"
        );
    }

    #[test]
    fn trilateration_rejects_degenerate_geometry() {
        let height = 0.06;
        // Three anchors on a line in plan view: the normal equations are singular, and any point
        // mirrored across that line fits equally well.
        let collinear = [
            Anchor {
                x_m: 0.0,
                y_m: 0.0,
                z_m: 2.0,
            },
            Anchor {
                x_m: 3.0,
                y_m: 0.0,
                z_m: 2.0,
            },
            Anchor {
                x_m: 6.0,
                y_m: 0.0,
                z_m: 2.0,
            },
        ];
        let snapshot: [(Anchor, f32); 3] =
            core::array::from_fn(|i| (collinear[i], true_range(&collinear[i], 2.0, 3.0, height)));
        assert_eq!(trilaterate(&snapshot, height), None);

        // And two anchors are never enough, however well placed.
        let anchors = arena();
        let pair = [
            (anchors[0], true_range(&anchors[0], 2.0, 3.0, height)),
            (anchors[1], true_range(&anchors[1], 2.0, 3.0, height)),
        ];
        assert_eq!(trilaterate(&pair, height), None);
    }

    #[test]
    fn trilateration_tolerates_a_range_shorter_than_the_anchor_height() {
        // An uncalibrated antenna delay reading short, or a robot almost directly beneath an anchor:
        // the planar projection would be the square root of a negative number.
        let anchors = arena();
        let height = 0.06;
        let mut snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 0.05, 0.05, height)));
        snapshot[0].1 = 0.5; // shorter than the 1.94 m height difference
                             // Must return *something* rather than a NaN; accuracy is not the point here.
        let solved = trilaterate(&snapshot, height).expect("must not produce a NaN");
        assert!(solved.0.is_finite() && solved.1.is_finite());
    }

    /// Drives a full circle at constant speed, then stops, then drives a straight leg.
    ///
    /// Returns the RMSE in metres and how many ranges the gate rejected.
    fn run_trajectory(inject_nlos: bool) -> (f32, u32, u32) {
        let anchors = arena();
        let config = FilterConfig::default();
        let height = config.robot_antenna_height_m;
        let mut filter = PoseFilter::new(config);
        let mut noise = Noise::new();

        // 100 Hz IMU against the protocol's 69.8 ms superframe: one superframe every 6.98 steps, so 7
        // as an integer cadence. Worth keeping faithful to `uwb_protocol::SUPERFRAME_US` rather than
        // rounding to something convenient -- the ratio between the two rates is the thing this test
        // exists to exercise, since it sets how far the filter dead-reckons between corrections.
        let dt = 0.01f32;
        let steps_per_superframe = 7;

        // Circle of radius 1.5 m about the arena centre at 0.35 m/s, then 1 s still, then straight.
        let radius = 1.5f32;
        let speed = 0.35f32;
        let omega = speed / radius;
        let gyro_bias = 0.012f32; // 0.7 deg/s, a realistic BMI270 zero-rate offset

        let mut true_x;
        let mut true_y;
        let mut true_psi = PI / 2.0;
        let mut theta = 0.0f32;
        let mut sum_squared = 0.0f32;
        let mut samples = 0u32;
        let mut accepted = 0u32;
        let mut rejected = 0u32;
        let mut bootstrapped = false;

        for step in 0..1_200u32 {
            // Three phases: circle, standstill, straight line.
            let (commanded, stationary) = if step < 700 {
                (speed, false)
            } else if step < 800 {
                (0.0, true)
            } else {
                (speed, false)
            };

            if step < 700 {
                theta += omega * dt;
                true_psi = wrap_angle(theta + PI / 2.0);
            } else if step >= 800 {
                // Straight on from wherever the circle ended.
                theta += 0.0;
            }
            true_x = 3.0 + radius * cosf(theta);
            true_y = 3.0 + radius * sinf(theta);
            if step >= 800 {
                let travelled = speed * dt * (step - 800) as f32;
                true_x += travelled * cosf(true_psi);
                true_y += travelled * sinf(true_psi);
            }

            let yaw_rate = if stationary { 0.0 } else { omega };
            let gyro = yaw_rate + gyro_bias + noise.gaussian(0.004);

            let motion = Motion::from_stationary(stationary);
            if bootstrapped {
                filter.predict(dt, gyro, commanded, motion);
                if motion == Motion::Still {
                    filter.update_zero_velocity(gyro);
                }
            }

            if step % steps_per_superframe == 0 {
                let mut snapshot = [(anchors[0], 0.0f32); 4];
                for (i, anchor) in anchors.iter().enumerate() {
                    let mut range =
                        true_range(anchor, true_x, true_y, height) + noise.gaussian(0.04);
                    // A robot shadowing anchor 2, on and off: non-line-of-sight only ever adds path.
                    if inject_nlos && i == 2 && (step / steps_per_superframe) % 7 == 0 {
                        range += 1.2;
                    }
                    snapshot[i] = (*anchor, range);
                }

                if !bootstrapped {
                    bootstrapped = filter.bootstrap(&snapshot);
                    continue;
                }
                for (anchor, range) in &snapshot {
                    match filter.update_range(anchor, *range) {
                        RangeOutcome::Accepted { .. } => accepted += 1,
                        RangeOutcome::Rejected { .. } => rejected += 1,
                        RangeOutcome::NotInitialized => unreachable!("bootstrapped above"),
                    }
                }
            }

            // Score only after the heading has had a chance to converge from course over ground: it
            // starts completely unknown, so the first second is a transient by construction, not an
            // error the filter should be judged on.
            if bootstrapped && step > 200 {
                let pose = filter.pose();
                let ex = pose.x_m - true_x;
                let ey = pose.y_m - true_y;
                sum_squared += ex * ex + ey * ey;
                samples += 1;
            }
        }

        assert!(samples > 500, "trajectory produced too few scored samples");
        assert!(
            accepted > 100,
            "almost nothing was fused: {accepted} accepted"
        );
        (sqrtf(sum_squared / samples as f32), accepted, rejected)
    }

    #[test]
    fn tracks_a_clean_trajectory_to_well_under_the_accuracy_target() {
        let (rmse, accepted, rejected) = run_trajectory(false);
        // Measured at 2.3 cm with 4 cm of range noise. The bound is 5 cm rather than the deployment's
        // 10-20 cm target so that a regression which merely halves the accuracy still fails here --
        // a test that only asserts the target would pass all the way down to a filter three times
        // worse than this one. Note this is synthetic noise: real ranging adds multipath and an
        // uncalibrated antenna delay on top, which is what the hardware accuracy gate measures.
        assert!(
            rmse < 0.05,
            "expected RMSE under 5 cm, got {rmse} m ({accepted} accepted, {rejected} rejected)"
        );
        assert_eq!(rejected, 0, "clean ranges must not be gated out");
    }

    #[test]
    fn survives_injected_nlos_outliers() {
        let (clean_rmse, clean_accepted, clean_rejected) = run_trajectory(false);
        let (rmse, accepted, rejected) = run_trajectory(true);

        // One anchor is shadowed on every seventh superframe, i.e. `clean_accepted / 4 / 7` times.
        let injected = clean_accepted / 4 / 7;
        assert!(
            rejected >= injected,
            "the gate must reject every injected 1.2 m outlier: {rejected} rejected of {injected} \
             injected ({clean_rejected} on clean data)"
        );
        // 2.30 cm measured, against 2.27 cm clean: rejecting the outliers costs almost nothing, which
        // is the property worth pinning. A filter that fused them instead would land near 20 cm.
        assert!(
            rmse < 1.5 * clean_rmse + 0.01,
            "outliers must barely move the RMSE: {rmse} m against {clean_rmse} m clean \
             ({accepted} accepted, {rejected} rejected)"
        );
    }

    #[test]
    fn zero_velocity_updates_learn_the_gyroscope_bias() {
        let mut filter = PoseFilter::new(FilterConfig::default());
        let anchors = arena();
        let height = filter.config().robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        assert!(filter.bootstrap(&snapshot));

        let bias = 0.02f32;
        for _ in 0..500 {
            filter.predict(0.01, bias, 0.0, Motion::Still);
            filter.update_zero_velocity(bias);
        }

        let pose = filter.pose();
        assert!(
            fabsf(pose.gyro_bias_rad_s - bias) < 0.002,
            "expected the bias to converge to {bias}, got {}",
            pose.gyro_bias_rad_s
        );
        // And with the bias learnt, a still robot's heading must not walk.
        assert!(
            fabsf(pose.heading_rad) < 0.05,
            "heading drifted to {} rad while stationary",
            pose.heading_rad
        );
    }

    /// Parks a robot on a known point and measures how far the published position wanders, in metres
    /// RMS about its own mean.
    fn parked_scatter(motion: Motion, seconds: f32) -> f32 {
        let anchors = arena();
        let config = FilterConfig::default();
        let height = config.robot_antenna_height_m;
        let mut filter = PoseFilter::new(config);
        let mut noise = Noise::new();

        let (parked_x, parked_y) = (2.0f32, 4.5f32);
        let dt = 0.01f32;
        let steps_per_superframe = 7;
        let gyro_bias = 0.012f32;
        let steps = (seconds / dt) as u32;
        // Ranges are noisier than the 4 cm the trajectory tests use, because standstill jitter is
        // linear in the range noise and the point here is what a real arena does, not a best case.
        let range_sigma = 0.10f32;

        // Welford again, and for the same reason `tasks::pose_estimator::Residuals` uses it: the
        // spread being measured is millimetres sitting on a mean of several metres.
        let mut count = 0u32;
        let (mut mean_x, mut mean_y) = (0.0f32, 0.0f32);
        let mut sum_squared_deviation = 0.0f32;
        let mut bootstrapped = false;

        for step in 0..steps {
            let gyro = gyro_bias + noise.gaussian(0.004);
            if bootstrapped {
                filter.predict(dt, gyro, 0.0, motion);
                filter.update_zero_velocity(gyro);
            }

            if step % steps_per_superframe == 0 {
                let mut snapshot = [(anchors[0], 0.0f32); 4];
                for (i, anchor) in anchors.iter().enumerate() {
                    snapshot[i] = (
                        *anchor,
                        true_range(anchor, parked_x, parked_y, height)
                            + noise.gaussian(range_sigma),
                    );
                }
                if !bootstrapped {
                    bootstrapped = filter.bootstrap(&snapshot);
                    continue;
                }
                for (anchor, range) in &snapshot {
                    filter.update_range(anchor, *range);
                }
            }

            // Scored only once the bootstrap's 0.5 m of initial uncertainty has been ranged away;
            // that transient is not jitter.
            if bootstrapped && step > steps / 3 {
                let pose = filter.pose();
                count += 1;
                let before_x = pose.x_m - mean_x;
                let before_y = pose.y_m - mean_y;
                mean_x += before_x / count as f32;
                mean_y += before_y / count as f32;
                sum_squared_deviation +=
                    before_x * (pose.x_m - mean_x) + before_y * (pose.y_m - mean_y);
            }
        }

        assert!(count > 500, "parked run produced too few scored samples");
        sqrtf(sum_squared_deviation / count as f32)
    }

    #[test]
    fn the_standstill_process_noise_is_what_stops_a_parked_pose_wandering() {
        // Both runs get the zero-velocity update. The only difference is which process noise the
        // propagation uses, which is exactly the claim: the pseudo-measurements cannot hold a position
        // still, because their Jacobians are zero on x and y.
        let still = parked_scatter(Motion::Still, 30.0);
        let moving = parked_scatter(Motion::Moving, 30.0);

        // 0.6 cm against 4.2 cm measured. The bound is a third rather than the seventh actually
        // achieved so that the test pins the mechanism rather than the exact tuning -- but a
        // regression that quietly returned `position_noise_still_m2_s` to within an order of magnitude
        // of the moving value would still fail it.
        assert!(
            still < 0.34 * moving,
            "the standstill noise must cut the parked scatter to a third: {still} m against {moving} m"
        );
        assert!(
            still < 0.015,
            "a parked robot should hold position to well under a centimetre and a half, got {still} m"
        );
    }

    #[test]
    fn a_park_does_not_throw_away_the_heading_the_robot_had_learnt() {
        // Heading has to be *earned* first -- from course over ground, since ranges do not observe it
        // -- or this measures the bootstrap's deliberately enormous initial variance instead of what a
        // standstill does to a converged one.
        let drive_then_park = |motion: Motion| -> (f32, f32) {
            let anchors = arena();
            let config = FilterConfig::default();
            let height = config.robot_antenna_height_m;
            let mut filter = PoseFilter::new(config);

            let at = |x: f32, y: f32| -> [(Anchor, f32); 4] {
                core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], x, y, height)))
            };
            // Six seconds of it: heading is only observable through the position cross-covariances
            // that forward motion accumulates, so this converges by the metre driven rather than by
            // the range fused. It settles at about 0.31 rad of spread and stays there -- that is the
            // driving equilibrium between the heading process noise and course over ground, not a
            // transient still on its way down.
            assert!(filter.bootstrap(&at(3.0, 3.0)));
            for step in 0..600 {
                filter.predict(0.01, 0.0, 0.35, Motion::Moving);
                if step % 7 == 0 {
                    let travelled = 3.0 + 0.35 * (step as f32 * 0.01);
                    for (anchor, range) in &at(travelled, 3.0) {
                        filter.update_range(anchor, *range);
                    }
                }
            }
            let learnt = sqrtf(filter.p[IPSI][IPSI]);
            assert!(
                learnt < 0.4,
                "the drive should have taught it a heading, got {learnt} rad of spread"
            );

            // Sixty seconds parked on the spot.
            let parked = 3.0 + 0.35 * 6.0;
            for step in 0..6000 {
                filter.predict(0.01, 0.0, 0.0, motion);
                filter.update_zero_velocity(0.0);
                if step % 7 == 0 {
                    for (anchor, range) in &at(parked, 3.0) {
                        filter.update_range(anchor, *range);
                    }
                }
            }
            (learnt, sqrtf(filter.p[IPSI][IPSI]))
        };

        let (learnt, still) = drive_then_park(Motion::Still);
        let (_, moving) = drive_then_park(Motion::Moving);

        // At the moving heading noise a minute of standstill adds 1.2 rad^2, and nothing can take it
        // back: a range's Jacobian is zero on psi, and the cross-covariances that would carry a
        // correction scale with a forward speed that is also zero. The robot then has to drive several
        // metres to recover a heading it never actually lost. Measured at 1.14 rad against 0.32.
        assert!(
            moving > 3.0 * learnt,
            "the moving noise should have wrecked the heading over a minute parked: {moving} rad \
             against the {learnt} rad it parked with"
        );
        assert!(
            still < 1.1 * learnt,
            "a park must leave the heading where the drive left it: {still} rad against {learnt} rad"
        );
    }

    #[test]
    fn heading_drifts_without_zero_velocity_updates() {
        // The counterpart to the test above: it is the zero-velocity update, not the model, that keeps
        // heading honest. Without it a 0.02 rad/s bias integrates to 0.1 rad in 5 s.
        let mut filter = PoseFilter::new(FilterConfig::default());
        let anchors = arena();
        let height = filter.config().robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        assert!(filter.bootstrap(&snapshot));

        for _ in 0..500 {
            filter.predict(0.01, 0.02, 0.0, Motion::Still);
        }
        assert!(
            fabsf(filter.pose().heading_rad) > 0.05,
            "expected heading to drift without zero-velocity updates"
        );
    }

    #[test]
    fn the_gate_is_asymmetric() {
        let anchors = arena();
        let config = FilterConfig::default();
        let height = config.robot_antenna_height_m;
        let truth = true_range(&anchors[0], 3.0, 3.0, height);

        // Converge tightly first, so the gate is driven by `range_sigma` rather than by a large P.
        let mut filter = PoseFilter::new(config);
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        assert!(filter.bootstrap(&snapshot));
        for _ in 0..40 {
            for (anchor, range) in &snapshot {
                filter.update_range(anchor, *range);
            }
        }

        // A residual that trips the long gate but not the short one must be rejected one way round and
        // accepted the other. 0.6 m against a 0.15 m sigma is 4 sigma: over `gate_long` (3), under
        // `gate_short` (5).
        let long = filter.clone().update_range(&anchors[0], truth + 0.6);
        let short = filter.clone().update_range(&anchors[0], truth - 0.6);
        assert!(
            matches!(long, RangeOutcome::Rejected { .. }),
            "a 4-sigma long reading must be rejected, got {long:?}"
        );
        assert!(
            matches!(short, RangeOutcome::Accepted { .. }),
            "a 4-sigma short reading must still be accepted, got {short:?}"
        );
    }

    #[test]
    fn a_per_measurement_sigma_controls_the_range_gate() {
        let anchors = arena();
        let config = FilterConfig::default();
        let height = config.robot_antenna_height_m;
        let truth = true_range(&anchors[0], 3.0, 3.0, height);
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));

        let mut filter = PoseFilter::new(config);
        assert!(filter.bootstrap(&snapshot));
        for _ in 0..40 {
            for (anchor, range) in &snapshot {
                filter.update_range(anchor, *range);
            }
        }

        let tight = filter
            .clone()
            .update_range_with_sigma(&anchors[0], truth + 0.25, 0.05);
        let loose = filter
            .clone()
            .update_range_with_sigma(&anchors[0], truth + 0.25, 0.20);
        assert!(matches!(tight, RangeOutcome::Rejected { .. }));
        assert!(matches!(loose, RangeOutcome::Accepted { .. }));

        let invalid = filter.update_range_with_sigma(&anchors[0], truth, f32::NAN);
        assert!(matches!(invalid, RangeOutcome::Rejected { .. }));
        assert!(filter.pose().x_m.is_finite());
    }

    #[test]
    fn the_antenna_lever_arm_heading_jacobian_matches_finite_differences() {
        let config = FilterConfig {
            robot_antenna_offset_x_m: 0.031,
            robot_antenna_offset_y_m: -0.014,
            ..FilterConfig::default()
        };
        let anchor = Anchor {
            x_m: 3.7,
            y_m: -0.4,
            z_m: 0.55,
        };
        let mut state = [1.2, 0.8, 0.73, 0.0, 0.0];
        let (_, _, _, analytic) = range_model(&config, &state, &anchor);

        let epsilon = 1.0e-3;
        state[IPSI] += epsilon;
        let (plus, _, _, _) = range_model(&config, &state, &anchor);
        state[IPSI] -= 2.0 * epsilon;
        let (minus, _, _, _) = range_model(&config, &state, &anchor);
        let finite_difference = (plus - minus) / (2.0 * epsilon);

        assert!(
            fabsf(analytic - finite_difference) < 2.0e-4,
            "heading derivative {analytic} differs from finite difference {finite_difference}"
        );
    }

    #[test]
    fn bootstrap_reports_the_chassis_centre_not_the_antenna() {
        let anchors = arena();
        let config = FilterConfig {
            robot_antenna_offset_x_m: 0.04,
            robot_antenna_offset_y_m: -0.02,
            ..FilterConfig::default()
        };
        let centre = (2.1, 3.2);
        // A first bootstrap starts at heading zero, so the phase centre is this fixed translation.
        let antenna = (
            centre.0 + config.robot_antenna_offset_x_m,
            centre.1 + config.robot_antenna_offset_y_m,
        );
        let snapshot: [(Anchor, f32); 4] = core::array::from_fn(|i| {
            (
                anchors[i],
                true_range(
                    &anchors[i],
                    antenna.0,
                    antenna.1,
                    config.robot_antenna_height_m,
                ),
            )
        });
        let mut filter = PoseFilter::new(config);
        assert!(filter.bootstrap(&snapshot));
        assert!(fabsf(filter.pose().x_m - centre.0) < 1.0e-4);
        assert!(fabsf(filter.pose().y_m - centre.1) < 1.0e-4);
    }

    #[test]
    fn updates_are_refused_before_a_bootstrap() {
        let mut filter = PoseFilter::new(FilterConfig::default());
        let anchors = arena();
        assert_eq!(
            filter.update_range(&anchors[0], 3.0),
            RangeOutcome::NotInitialized
        );
        // And predicting without a position must be a no-op rather than integrating from the origin.
        filter.predict(0.01, 0.0, 0.5, Motion::Moving);
        assert_eq!(filter.pose().x_m, 0.0);
        assert!(!filter.is_initialized());
    }

    #[test]
    fn covariance_grows_while_dead_reckoning_and_shrinks_on_ranges() {
        let mut filter = PoseFilter::new(FilterConfig::default());
        let anchors = arena();
        let height = filter.config().robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        assert!(filter.bootstrap(&snapshot));

        for _ in 0..100 {
            filter.predict(0.01, 0.0, 0.3, Motion::Moving);
        }
        let dead_reckoned = filter.pose().position_variance_m2;

        for _ in 0..5 {
            for (anchor, range) in &snapshot {
                filter.update_range(anchor, *range);
            }
        }
        let corrected = filter.pose().position_variance_m2;
        assert!(
            corrected < dead_reckoned,
            "ranges must reduce the position variance: {corrected} vs {dead_reckoned}"
        );
    }

    /// A random symmetric positive-definite covariance, as `L L^T`.
    ///
    /// `spread` sets how far apart the diagonal entries are pulled. The interesting case is not a
    /// well-conditioned toy matrix but the one the filter actually runs on, whose diagonal spans
    /// about six orders of magnitude: a bootstrap heading variance near 3.0 against a converged
    /// gyroscope-bias variance near 5e-7. That span is where a summation written in the wrong order
    /// stops being equivalent to one written in the right order.
    fn random_covariance(noise: &mut Noise, spread: bool) -> [[f32; N]; N] {
        // These are Cholesky-factor magnitudes, so they square into the covariance: the diagonal
        // comes out spanning roughly 3.0 down to 5e-7, which is the range the real filter occupies
        // between a bootstrap heading variance and a converged gyroscope-bias variance. Deliberately
        // not smaller than that -- a synthetic covariance below `VARIANCE_FLOOR` would have the floor
        // bind on every case and the test would be measuring the clamp instead of the arithmetic.
        let scale = [0.1f32, 0.1, 1.7, 0.026, 7.0e-4];
        let mut l = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..=i {
                let magnitude = if spread { scale[i] } else { 1.0 };
                l[i][j] = (0.3 + 0.7 * fabsf(noise.uniform())) * magnitude;
            }
        }
        let mut p = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..N {
                let mut sum = 0.0;
                for k in 0..N {
                    sum += l[i][k] * l[j][k];
                }
                p[i][j] = sum;
            }
        }
        p
    }

    /// The `F` that [`PoseFilter::predict`] builds, as a dense matrix for the reference path.
    fn dense_jacobian(psi: f32, v: f32, dt: f32, lag: f32) -> [[f32; N]; N] {
        let (sin_psi, cos_psi) = sincosf(psi);
        let cos_dt = cos_psi * dt;
        let sin_dt = sin_psi * dt;
        let mut f = reference_identity();
        f[IX][IPSI] = -(v * sin_dt);
        f[IX][IV] = cos_dt;
        f[IY][IPSI] = v * cos_dt;
        f[IY][IV] = sin_dt;
        f[IPSI][IB] = -dt;
        f[IV][IV] = 1.0 - lag;
        f
    }

    #[test]
    fn sparse_predict_matches_the_dense_reference() {
        let config = FilterConfig::default();
        let mut noise = Noise::new();

        for case in 0..400 {
            let spread = case % 2 == 0;
            let p = random_covariance(&mut noise, spread);
            let psi = noise.uniform() * PI;
            let v = 0.5 * noise.uniform();
            let dt = 0.001 + 0.05 * fabsf(noise.uniform());
            let gyro = 0.5 * noise.uniform();
            let commanded = 0.5 * noise.uniform();

            // Both arms of the process-noise switch feed the same congruence, so both are checked
            // against the reference rather than only the moving one -- and the still arm is where the
            // added terms are smallest against the matrix they land on, which is where a summation
            // written in the wrong order would show up first.
            let motion = if case % 3 == 0 {
                Motion::Still
            } else {
                Motion::Moving
            };

            let mut filter = PoseFilter::new(config);
            filter.x = [
                noise.uniform(),
                noise.uniform(),
                psi,
                v,
                0.01 * noise.uniform(),
            ];
            filter.p = p;
            filter.initialized = true;
            filter.predict(dt, gyro, commanded, motion);

            let lag = dt / (config.speed_tau_s + dt);
            let f = dense_jacobian(psi, v, dt, lag);
            let (position_noise, heading_noise, speed_noise) = match motion {
                Motion::Moving => (
                    config.position_noise_m2_s,
                    config.heading_noise_rad2_s,
                    config.speed_noise_m2_s3,
                ),
                Motion::Still => (
                    config.position_noise_still_m2_s,
                    config.heading_noise_still_rad2_s,
                    config.speed_noise_still_m2_s3,
                ),
            };
            let q = [
                position_noise * dt,
                position_noise * dt,
                heading_noise * dt,
                speed_noise * dt,
                config.gyro_bias_noise_rad2_s3 * dt,
            ];
            let expected = reference_propagate(&p, &f, &q);

            for i in 0..N {
                for j in 0..N {
                    let got = filter.p[i][j];
                    let want = expected[i][j];
                    // Scaled against the entry going in as well as the one coming out. The congruence
                    // adds terms of opposing sign, so an entry whose result is far smaller than its
                    // input is one where the two paths' differing summation orders are being compared
                    // through a cancellation -- and there the surviving digits are a property of f32,
                    // not of either implementation.
                    let scale = fabsf(want).max(fabsf(p[i][j])).max(1.0e-12);
                    assert!(
                        fabsf(got - want) <= 1.0e-5 * scale,
                        "case {case} (spread {spread}, {motion:?}) entry [{i}][{j}]: {got} vs \
                         reference {want}"
                    );
                }
            }

            // Stronger than the reference manages. `symmetrize` averaged two independently rounded
            // halves back together; storing one value into both slots makes them the same bits.
            for i in 0..N {
                for j in (i + 1)..N {
                    assert_eq!(
                        filter.p[i][j].to_bits(),
                        filter.p[j][i].to_bits(),
                        "case {case}: P[{i}][{j}] and P[{j}][{i}] must be bit-identical"
                    );
                }
            }
            // Row and column IB of F are the identity's, so the smallest number in the matrix is only
            // ever touched by its own process noise -- never summed against the large ones.
            assert_eq!(
                filter.p[IB][IB].to_bits(),
                (p[IB][IB] + config.gyro_bias_noise_rad2_s3 * dt).to_bits(),
                "case {case}: the bias variance must pass through the congruence untouched"
            );
        }
    }

    /// One scalar update with a unit `H`, computed in `f64`.
    ///
    /// Deliberately *not* the old `f32` path. On row and column `index` that path forms
    /// `P - P * P[index][index] / S`, and when `R` is small against `P[index][index]` the two terms
    /// very nearly cancel -- at `zero_velocity_sigma_m_s = 0.01` against a speed variance of order
    /// 0.1 it loses three decimal digits right there. Checking the specialized path against it would
    /// be measuring agreement with a worse implementation rather than correctness, and would have to
    /// be loosened until it stopped testing anything. `f64` has ten more digits than either, so it
    /// answers the question actually worth asking.
    fn reference_unit_update_f64(
        x: &[f32; N],
        p: &[[f32; N]; N],
        index: usize,
        r: f32,
        residual: f32,
    ) -> ([f64; N], [[f64; N]; N]) {
        let mut x64 = [0.0f64; N];
        let mut p64 = [[0.0f64; N]; N];
        for i in 0..N {
            x64[i] = f64::from(x[i]);
            for j in 0..N {
                p64[i][j] = f64::from(p[i][j]);
            }
        }

        let ph: [f64; N] = core::array::from_fn(|i| p64[i][index]);
        let s = p64[index][index] + f64::from(r);
        let residual = f64::from(residual);

        for i in 0..N {
            x64[i] += (ph[i] / s) * residual;
        }
        // The real update normalizes the heading; without the same step here the comparison trips on
        // a difference of exactly one full turn, which is agreement rather than error.
        let tau = f64::from(TAU);
        x64[IPSI] -= tau * libm::ceil(x64[IPSI] / tau - 0.5);
        let mut out = [[0.0f64; N]; N];
        for i in 0..N {
            for j in 0..N {
                out[i][j] = p64[i][j] - (ph[i] / s) * ph[j];
            }
        }
        (x64, out)
    }

    #[test]
    fn the_unit_state_update_matches_a_wider_precision_reference() {
        let config = FilterConfig::default();
        let mut noise = Noise::new();

        // One update per case, not the two that `update_zero_velocity` chains. Chaining would fold
        // this claim -- that `alpha = R/S` is the same operator as the general scalar update -- in
        // with a second, unrelated question about how f32 error accumulates when the first update's
        // block downdate happens to cancel. The trajectory tests already cover the chained behaviour.
        for case in 0..400 {
            let p = random_covariance(&mut noise, case % 2 == 0);
            let x = [
                noise.uniform(),
                noise.uniform(),
                noise.uniform(),
                0.3 * noise.uniform(),
                0.02 * noise.uniform(),
            ];
            let (index, r) = if case % 4 < 2 {
                (
                    IV,
                    config.zero_velocity_sigma_m_s * config.zero_velocity_sigma_m_s,
                )
            } else {
                (
                    IB,
                    config.gyro_bias_sigma_rad_s * config.gyro_bias_sigma_rad_s,
                )
            };
            let residual = 0.05 * noise.uniform();

            let mut fast = PoseFilter::new(config);
            fast.x = x;
            fast.p = p;
            fast.initialized = true;
            fast.apply_unit_state_update(index, r, residual);

            let (want_x, want_p) = reference_unit_update_f64(&x, &p, index, r, residual);

            for i in 0..N {
                let scale = want_x[i].abs().max(f64::from(x[i]).abs()).max(1.0e-12);
                assert!(
                    (f64::from(fast.x[i]) - want_x[i]).abs() <= 1.0e-5 * scale,
                    "case {case} (state {index}) x[{i}]: {} vs {}",
                    fast.x[i],
                    want_x[i]
                );
                for j in 0..N {
                    let want = want_p[i][j];
                    // Scaled against the magnitudes going *into* the entry, not the one coming out.
                    // Where the downdate cancels, the result is small precisely because two similar
                    // numbers were subtracted, and demanding a small error relative to that difference
                    // would be demanding better than f32 can represent of the inputs.
                    let scale = want.abs().max(f64::from(p[i][j]).abs()).max(1.0e-12);
                    assert!(
                        (f64::from(fast.p[i][j]) - want).abs() <= 1.0e-5 * scale,
                        "case {case} (state {index}) P[{i}][{j}]: {} vs {want}",
                        fast.p[i][j]
                    );
                }
            }

            // The property the specialization exists for: this state's variance is *scaled* by a
            // positive factor rather than having a subtraction land on it, so it cannot be driven
            // negative by cancellation however ill-conditioned the covariance is.
            assert!(
                fast.p[index][index] > 0.0,
                "case {case}: the measured state's variance must stay positive, got {}",
                fast.p[index][index]
            );
        }
    }

    #[test]
    fn covariance_stays_positive_semidefinite() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let mut noise = Noise::new();
        let mut filter = PoseFilter::new(config);
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        assert!(filter.bootstrap(&snapshot));

        let check = |filter: &PoseFilter, step: usize| {
            for i in 0..N {
                assert!(
                    filter.p[i][i] > 0.0,
                    "step {step}: variance [{i}] went non-positive: {}",
                    filter.p[i][i]
                );
                for j in (i + 1)..N {
                    assert_eq!(
                        filter.p[i][j].to_bits(),
                        filter.p[j][i].to_bits(),
                        "step {step}: P[{i}][{j}] lost bit-exact symmetry"
                    );
                    // Cauchy-Schwarz. Necessary for positive semi-definiteness, cheap to check, and it
                    // catches every way the downdate can actually go wrong -- a full Cholesky would
                    // only add sufficiency against failures this filter has no mechanism to produce.
                    let bound = filter.p[i][i] * filter.p[j][j] * 1.05;
                    let covariance = filter.p[i][j] * filter.p[i][j];
                    assert!(
                        covariance <= bound,
                        "step {step}: P[{i}][{j}]^2 = {covariance} exceeds P[{i}][{i}]*P[{j}][{j}] \
                         = {bound}"
                    );
                }
            }
        };

        for step in 0..4000 {
            // Long stationary stretches are where the covariance gets smallest, so they are where a
            // downdate is most likely to cross zero -- and the standstill process noise makes them
            // smaller still, which is the whole reason this test propagates them as still rather than
            // only feeding the zero-velocity update.
            let parked = (600..1400).contains(&step);
            let motion = Motion::from_stationary(parked);
            filter.predict(0.01, 0.01 + 0.004 * noise.uniform(), 0.35, motion);
            if step % 5 == 0 {
                for (anchor, range) in &snapshot {
                    filter.update_range(anchor, range + 0.04 * noise.gaussian(1.0));
                }
            }
            if parked {
                filter.update_zero_velocity(0.01 + 0.004 * noise.uniform());
            }
            check(&filter, step);
        }
    }

    #[test]
    fn wrap_angle_terminates_and_is_bounded() {
        // That this test returns at all is the assertion for the finite cases: the loop this replaced
        // does not terminate for the last few entries, and hangs an MCU with no watchdog behind it.
        for angle in [
            0.0,
            PI,
            -PI,
            PI - 1.0e-6,
            -PI + 1.0e-6,
            10.0 * TAU + 1.0,
            -1.0e6,
            1.0e8,
            f32::MAX,
            f32::MIN,
        ] {
            let wrapped = wrap_angle(angle);
            assert!(
                wrapped > -PI && wrapped <= PI,
                "wrap_angle({angle}) = {wrapped}, outside (-pi, pi]"
            );
        }

        // Non-finite input has no wrapped form, so it is handed back for the caller's own guards.
        assert!(wrap_angle(f32::INFINITY).is_infinite());
        assert!(wrap_angle(f32::NEG_INFINITY).is_infinite());
        assert!(wrap_angle(f32::NAN).is_nan());
    }

    #[test]
    fn wrap_angle_agrees_with_the_loop_over_the_ordinary_range() {
        let mut noise = Noise::new();
        for _ in 0..2000 {
            let angle = 100.0 * noise.uniform();
            let got = wrap_angle(angle);
            let want = reference_wrap_angle(angle);
            // No allowance for a boundary difference: the branch-free form was chosen to land on the
            // same convention as the loop, so over the range the loop can survive they agree outright.
            assert!(
                fabsf(got - want) < 1.0e-4,
                "wrap_angle({angle}) = {got}, reference = {want}"
            );
        }
    }

    #[test]
    fn a_nan_range_leaves_the_filter_untouched() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));

        for poison in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut filter = PoseFilter::new(config);
            assert!(filter.bootstrap(&snapshot));
            let before = filter.clone();

            let outcome = filter.update_range(&anchors[0], poison);
            assert!(
                matches!(outcome, RangeOutcome::Rejected { .. }),
                "a {poison} range must be rejected, got {outcome:?}"
            );
            for i in 0..N {
                assert_eq!(
                    filter.x[i].to_bits(),
                    before.x[i].to_bits(),
                    "a rejected range must not move the state"
                );
                for j in 0..N {
                    assert_eq!(
                        filter.p[i][j].to_bits(),
                        before.p[i][j].to_bits(),
                        "a rejected range must not move the covariance"
                    );
                }
            }
        }
    }

    #[test]
    fn a_nan_or_nonpositive_timestep_is_a_no_op() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));

        // Timestep, then gyroscope rate, then commanded speed: a NaN in any of the three would spread
        // to every entry of the covariance on the next propagation and never wash out.
        let cases: [(f32, f32, f32); 7] = [
            (f32::NAN, 0.0, 0.3),
            (0.0, 0.0, 0.3),
            (-0.01, 0.0, 0.3),
            (f32::INFINITY, 0.0, 0.3),
            (0.01, f32::NAN, 0.3),
            (0.01, 0.0, f32::NAN),
            (0.01, f32::INFINITY, 0.3),
        ];

        for (dt, gyro, commanded) in cases {
            let mut filter = PoseFilter::new(config);
            assert!(filter.bootstrap(&snapshot));
            let before = filter.clone();

            filter.predict(dt, gyro, commanded, Motion::Moving);

            for i in 0..N {
                assert_eq!(
                    filter.x[i].to_bits(),
                    before.x[i].to_bits(),
                    "predict({dt}, {gyro}, {commanded}) moved state [{i}]"
                );
                for j in 0..N {
                    assert_eq!(
                        filter.p[i][j].to_bits(),
                        before.p[i][j].to_bits(),
                        "predict({dt}, {gyro}, {commanded}) moved covariance [{i}][{j}]"
                    );
                }
            }
        }
    }

    #[test]
    fn a_large_timestep_cannot_destabilise_the_speed_lag() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        let mut filter = PoseFilter::new(config);
        assert!(filter.bootstrap(&snapshot));

        // 0.2 s is the top of the clamp in `predict`, and comfortably past `speed_tau_s` -- the regime
        // where `dt / tau` would exceed 1, overshoot the commanded speed, and flip the sign of
        // `f[IV][IV]`.
        let target = 0.5;
        let mut previous = filter.pose().speed_m_s;
        for step in 0..60 {
            filter.predict(0.2, 0.0, target, Motion::Moving);
            let speed = filter.pose().speed_m_s;
            assert!(
                speed >= previous - 1.0e-6,
                "step {step}: speed went backwards, {speed} after {previous}"
            );
            assert!(
                speed <= target + 1.0e-6,
                "step {step}: speed overshot the command, {speed} against {target}"
            );
            assert!(
                filter.p[IV][IV] > 0.0,
                "step {step}: speed variance went non-positive: {}",
                filter.p[IV][IV]
            );
            previous = speed;
        }
        assert!(
            fabsf(previous - target) < 0.01,
            "speed should have converged on the command, got {previous}"
        );
    }

    #[test]
    fn the_squared_gate_accepts_the_same_set_as_the_sqrt_gate() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));

        let mut filter = PoseFilter::new(config);
        assert!(filter.bootstrap(&snapshot));
        for _ in 0..40 {
            for (anchor, range) in &snapshot {
                filter.update_range(anchor, *range);
            }
        }

        // Sweep both sides of both gates, including right up against the boundary.
        for step in -400..=400 {
            let offset = step as f32 * 0.005;
            let mut probe = filter.clone();
            let truth = true_range(&anchors[0], 3.0, 3.0, height);
            let outcome = probe.update_range(&anchors[0], truth + offset);

            // Recompute the old test independently: |residual| / sqrt(S) against the same limit.
            let reference = filter.clone();
            let dx = reference.x[IX] - anchors[0].x_m;
            let dy = reference.x[IY] - anchors[0].y_m;
            let dz = height - anchors[0].z_m;
            let predicted = sqrtf(dx * dx + dy * dy + dz * dz);
            let mut h = [0.0f32; N];
            h[IX] = dx / predicted;
            h[IY] = dy / predicted;
            let ph = reference_mat_vec(&reference.p, &h);
            let s = reference_dot(&h, &ph) + config.range_sigma_m * config.range_sigma_m;
            let residual = truth + offset - predicted;
            let limit = if residual > 0.0 {
                config.gate_long
            } else {
                config.gate_short
            };
            let want_accept = fabsf(residual / sqrtf(s)) <= limit;

            let got_accept = matches!(outcome, RangeOutcome::Accepted { .. });
            assert_eq!(
                got_accept, want_accept,
                "offset {offset}: squared gate said accept={got_accept}, sqrt gate said \
                 accept={want_accept}"
            );
        }
    }

    #[test]
    fn a_long_stationary_period_does_not_make_the_filter_deaf() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let snapshot: [(Anchor, f32); 4] =
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], 3.0, 3.0, height)));
        let mut filter = PoseFilter::new(config);
        assert!(filter.bootstrap(&snapshot));

        // Sixty seconds parked, with the gyroscope reading a steady bias.
        let settled_bias = 0.01;
        for _ in 0..6000 {
            filter.predict(0.01, settled_bias, 0.0, Motion::Still);
            filter.update_zero_velocity(settled_bias);
        }
        assert!(
            fabsf(filter.pose().gyro_bias_rad_s - settled_bias) < 0.002,
            "the bias should have been learnt by now, got {}",
            filter.pose().gyro_bias_rad_s
        );
        assert!(
            filter.p[IB][IB] >= VARIANCE_FLOOR[IB],
            "the bias variance fell through its floor: {}",
            filter.p[IB][IB]
        );

        // Now the bias steps. A filter whose variance had collapsed would take many minutes to notice;
        // the process noise has to keep enough of the door open to track it in seconds.
        let stepped_bias = settled_bias + 0.01;
        for _ in 0..3000 {
            filter.predict(0.01, stepped_bias, 0.0, Motion::Still);
            filter.update_zero_velocity(stepped_bias);
        }
        assert!(
            fabsf(filter.pose().gyro_bias_rad_s - stepped_bias) < 0.002,
            "a bias step must still be tracked after a long standstill, got {} for {stepped_bias}",
            filter.pose().gyro_bias_rad_s
        );
    }

    #[test]
    fn a_re_bootstrap_keeps_the_heading_it_had_learnt() {
        let config = FilterConfig::default();
        let anchors = arena();
        let height = config.robot_antenna_height_m;
        let mut filter = PoseFilter::new(config);

        let at = |x: f32, y: f32| -> [(Anchor, f32); 4] {
            core::array::from_fn(|i| (anchors[i], true_range(&anchors[i], x, y, height)))
        };
        assert!(filter.bootstrap(&at(3.0, 3.0)));

        // Drive east for a couple of seconds so course over ground fixes a heading near zero.
        for step in 0..200 {
            filter.predict(0.01, 0.0, 0.35, Motion::Moving);
            if step % 5 == 0 {
                let travelled = 3.0 + 0.35 * (step as f32 * 0.01);
                for (anchor, range) in &at(travelled, 3.0) {
                    filter.update_range(anchor, *range);
                }
            }
        }
        let learnt_heading = filter.pose().heading_rad;
        let learnt_bias = filter.pose().gyro_bias_rad_s;
        assert!(
            fabsf(learnt_heading) < 0.3,
            "the trajectory should have taught it a heading near zero, got {learnt_heading}"
        );

        // Now re-bootstrap, exactly as the barren-superframe recovery in `tasks::pose_estimator` does.
        assert!(filter.bootstrap(&at(4.0, 3.0)));

        assert!(
            fabsf(filter.pose().heading_rad - learnt_heading) < 1.0e-6,
            "a re-bootstrap must keep the heading: {} was {learnt_heading}",
            filter.pose().heading_rad
        );
        assert_eq!(
            filter.pose().gyro_bias_rad_s.to_bits(),
            learnt_bias.to_bits(),
            "a re-bootstrap must keep the gyroscope bias"
        );
        assert!(
            fabsf(filter.pose().x_m - 4.0) < 0.15,
            "the position should be the new fix, got {}",
            filter.pose().x_m
        );
        assert!(
            filter.p[IPSI][IPSI] <= 3.0,
            "heading variance must be inflated, not reset past its bootstrap value: {}",
            filter.p[IPSI][IPSI]
        );
    }
}
