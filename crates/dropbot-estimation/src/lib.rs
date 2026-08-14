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
//! error until the next range arrives, which at the protocol's ~18 Hz is at most ~3 cm at 0.5 m/s.
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

#![no_std]

use libm::{cosf, fabsf, sinf, sqrtf};

/// State dimension: `[x, y, psi, v, gyro_bias]`.
pub const N: usize = 5;

const IX: usize = 0;
const IY: usize = 1;
const IPSI: usize = 2;
const IV: usize = 3;
const IB: usize = 4;

const PI: f32 = core::f32::consts::PI;
const TAU: f32 = core::f32::consts::TAU;

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

/// Tuning. The defaults are the starting point documented against each field; every one of them is a
/// hardware measurement waiting to happen, not a fundamental constant.
#[derive(Clone, Copy, Debug)]
pub struct FilterConfig {
    /// Time constant of the first-order lag from commanded to actual forward speed, in seconds.
    ///
    /// N20 gear motors on a light chassis settle quickly; this is the number to fit from a step
    /// response once the pose estimate itself is trustworthy enough to measure it.
    pub speed_tau_s: f32,
    /// Process noise on position, m^2 per second. Covers everything the unicycle model cannot express:
    /// wheel slip, being nudged, a floor that is not quite flat.
    pub position_noise_m2_s: f32,
    /// Process noise on heading, rad^2 per second. Gyroscope angle random walk plus the model error
    /// from integrating a single-axis rate on a robot that pitches slightly as it accelerates.
    pub heading_noise_rad2_s: f32,
    /// Process noise on forward speed, (m/s)^2 per second. Sized against how wrong the commanded-speed
    /// model can be over one superframe, which with no encoders is the filter's weakest input.
    pub speed_noise_m2_s3: f32,
    /// Process noise on the gyroscope bias, (rad/s)^2 per second. Thermal drift only, so small: the
    /// bias is nearly constant over a run, which is exactly why it is worth estimating.
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
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            speed_tau_s: 0.15,
            position_noise_m2_s: 0.02,
            heading_noise_rad2_s: 0.02,
            speed_noise_m2_s3: 0.5,
            gyro_bias_noise_rad2_s3: 1.0e-6,
            range_sigma_m: 0.15,
            zero_velocity_sigma_m_s: 0.01,
            gyro_bias_sigma_rad_s: 0.005,
            gate_long: 3.0,
            gate_short: 5.0,
            robot_antenna_height_m: 0.06,
        }
    }
}

/// Two-stage estimator: [`Self::bootstrap`] to get a position at all, then predict/update forever.
#[derive(Clone, Debug)]
pub struct PoseFilter {
    config: FilterConfig,
    x: [f32; N],
    p: [[f32; N]; N],
    initialized: bool,
}

impl PoseFilter {
    pub fn new(config: FilterConfig) -> Self {
        Self {
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
    /// Heading is *not* initialized -- it cannot be, from ranges alone, and a stationary robot has no
    /// course over ground either. It starts at zero with a large variance and is corrected by the
    /// first few metres of motion. `speed` starts at zero, which is true whenever this is called at
    /// standstill and close enough otherwise.
    ///
    /// Returns `false` and changes nothing if the snapshot is too small or geometrically degenerate
    /// (anchors collinear as seen in plan view, which makes the normal equations singular).
    pub fn bootstrap(&mut self, snapshot: &[(Anchor, f32)]) -> bool {
        let Some((x_m, y_m)) = trilaterate(snapshot, self.config.robot_antenna_height_m) else {
            return false;
        };

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
        // Gyroscope bias: carried across a re-bootstrap rather than reset, since it is a property of
        // the sensor and not of the position fix, and re-learning it costs a stationary period.
        self.p[IB][IB] = if self.initialized {
            self.p[IB][IB].max(1.0e-4)
        } else {
            1.0e-2
        };
        self.initialized = true;
        true
    }

    /// Advances the model by `dt_s`.
    ///
    /// `gyro_z_rad_s` is the raw yaw rate in the robot's body frame -- raw, not bias-corrected: the
    /// filter subtracts its own bias estimate, which is the whole point of having one.
    /// `commanded_speed_m_s` is the forward speed the motor controller was told to produce.
    pub fn predict(&mut self, dt_s: f32, gyro_z_rad_s: f32, commanded_speed_m_s: f32) {
        if !self.initialized || dt_s <= 0.0 {
            return;
        }
        // Clamped for the same reason `data::imu`'s filters clamp theirs: a timestep from a stalled
        // sampler would otherwise be extrapolated as if it were real motion.
        let dt = dt_s.clamp(0.0005, 0.2);

        let psi = self.x[IPSI];
        let v = self.x[IV];
        let (sin_psi, cos_psi) = (sinf(psi), cosf(psi));
        let yaw_rate = gyro_z_rad_s - self.x[IB];
        let lag = dt / self.config.speed_tau_s;

        self.x[IX] += v * cos_psi * dt;
        self.x[IY] += v * sin_psi * dt;
        self.x[IPSI] = wrap_angle(psi + yaw_rate * dt);
        self.x[IV] += (commanded_speed_m_s - v) * lag;
        // Bias is a random walk: unchanged in the mean, growing only in covariance.

        // F = d f / d x, evaluated at the previous state. Identity everywhere except the six terms
        // the model actually couples.
        let mut f = identity();
        f[IX][IPSI] = -v * sin_psi * dt;
        f[IX][IV] = cos_psi * dt;
        f[IY][IPSI] = v * cos_psi * dt;
        f[IY][IV] = sin_psi * dt;
        f[IPSI][IB] = -dt;
        f[IV][IV] = 1.0 - lag;

        // P = F P F^T + Q
        let fp = mat_mul(&f, &self.p);
        self.p = mat_mul_transpose(&fp, &f);
        self.p[IX][IX] += self.config.position_noise_m2_s * dt;
        self.p[IY][IY] += self.config.position_noise_m2_s * dt;
        self.p[IPSI][IPSI] += self.config.heading_noise_rad2_s * dt;
        self.p[IV][IV] += self.config.speed_noise_m2_s3 * dt;
        self.p[IB][IB] += self.config.gyro_bias_noise_rad2_s3 * dt;
        symmetrize(&mut self.p);
    }

    /// Fuses one range to one anchor.
    pub fn update_range(&mut self, anchor: &Anchor, range_m: f32) -> RangeOutcome {
        if !self.initialized {
            return RangeOutcome::NotInitialized;
        }

        let dx = self.x[IX] - anchor.x_m;
        let dy = self.x[IY] - anchor.y_m;
        let dz = self.config.robot_antenna_height_m - anchor.z_m;
        let predicted = sqrtf(dx * dx + dy * dy + dz * dz);
        if predicted < 1.0e-3 {
            // Standing exactly under an anchor: the range gradient is undefined there, and no useful
            // information can be extracted from this measurement.
            return RangeOutcome::Rejected {
                residual_m: range_m - predicted,
                normalized: f32::INFINITY,
            };
        }

        let mut h = [0.0f32; N];
        h[IX] = dx / predicted;
        h[IY] = dy / predicted;

        let residual = range_m - predicted;
        let r = self.config.range_sigma_m * self.config.range_sigma_m;
        let ph = mat_vec(&self.p, &h);
        let s = dot(&h, &ph) + r;
        if s <= 0.0 {
            return RangeOutcome::Rejected {
                residual_m: residual,
                normalized: f32::INFINITY,
            };
        }
        let normalized = residual / sqrtf(s);

        // Asymmetric: a long reading is the one with a physical mechanism behind it. See the module
        // docs.
        let limit = if residual > 0.0 {
            self.config.gate_long
        } else {
            self.config.gate_short
        };
        if fabsf(normalized) > limit {
            return RangeOutcome::Rejected {
                residual_m: residual,
                normalized,
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
    pub fn update_zero_velocity(&mut self, gyro_z_rad_s: f32) {
        if !self.initialized {
            return;
        }

        let mut h = [0.0f32; N];
        h[IV] = 1.0;
        let r = self.config.zero_velocity_sigma_m_s * self.config.zero_velocity_sigma_m_s;
        let ph = mat_vec(&self.p, &h);
        let s = dot(&h, &ph) + r;
        if s > 0.0 {
            self.apply_scalar_update(&ph, s, -self.x[IV]);
        }

        let mut h = [0.0f32; N];
        h[IB] = 1.0;
        let r = self.config.gyro_bias_sigma_rad_s * self.config.gyro_bias_sigma_rad_s;
        let ph = mat_vec(&self.p, &h);
        let s = dot(&h, &ph) + r;
        if s > 0.0 {
            self.apply_scalar_update(&ph, s, gyro_z_rad_s - self.x[IB]);
        }
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
        let mut gain = [0.0f32; N];
        for i in 0..N {
            gain[i] = ph[i] / s;
            self.x[i] += gain[i] * residual;
        }
        self.x[IPSI] = wrap_angle(self.x[IPSI]);

        for i in 0..N {
            for j in 0..N {
                self.p[i][j] -= gain[i] * ph[j];
            }
        }
        symmetrize(&mut self.p);
    }
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

/// Normalizes an angle to `(-pi, pi]`.
pub fn wrap_angle(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= TAU;
    }
    while angle <= -PI {
        angle += TAU;
    }
    angle
}

fn identity() -> [[f32; N]; N] {
    let mut m = [[0.0f32; N]; N];
    for i in 0..N {
        m[i][i] = 1.0;
    }
    m
}

fn mat_mul(a: &[[f32; N]; N], b: &[[f32; N]; N]) -> [[f32; N]; N] {
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
fn mat_mul_transpose(a: &[[f32; N]; N], b: &[[f32; N]; N]) -> [[f32; N]; N] {
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

fn mat_vec(m: &[[f32; N]; N], v: &[f32; N]) -> [f32; N] {
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

fn dot(a: &[f32; N], b: &[f32; N]) -> f32 {
    let mut sum = 0.0;
    for i in 0..N {
        sum += a[i] * b[i];
    }
    sum
}

/// Forces exact symmetry.
///
/// `P` is symmetric in theory and drifts out of it in f32 after a few thousand updates; once it does,
/// `S = H P H^T + R` can go negative and the filter diverges in a way that looks like a hardware
/// fault. Averaging the off-diagonals costs 10 additions and removes the failure mode entirely.
fn symmetrize(p: &mut [[f32; N]; N]) {
    for i in 0..N {
        for j in (i + 1)..N {
            let mean = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = mean;
            p[j][i] = mean;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

        // 100 Hz IMU, ~18 Hz ranging: one superframe every 5.4 steps, so 5 as an integer cadence.
        let dt = 0.01f32;
        let steps_per_superframe = 5;

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

            if bootstrapped {
                filter.predict(dt, gyro, commanded);
                if stationary {
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
        // Measured at 1.9 cm with 4 cm of range noise. The bound is 5 cm rather than the deployment's
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
        // 2.0 cm measured, against 1.9 cm clean: rejecting the outliers costs almost nothing, which is
        // the property worth pinning. A filter that fused them instead would land near 20 cm.
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
            filter.predict(0.01, bias, 0.0);
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
            filter.predict(0.01, 0.02, 0.0);
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
    fn updates_are_refused_before_a_bootstrap() {
        let mut filter = PoseFilter::new(FilterConfig::default());
        let anchors = arena();
        assert_eq!(
            filter.update_range(&anchors[0], 3.0),
            RangeOutcome::NotInitialized
        );
        // And predicting without a position must be a no-op rather than integrating from the origin.
        filter.predict(0.01, 0.0, 0.5);
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
            filter.predict(0.01, 0.0, 0.3);
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
}
