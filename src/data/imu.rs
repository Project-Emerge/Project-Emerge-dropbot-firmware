//! What the nine-axis IMU measures, and what this firmware makes of it.
//!
//! [`ImuSample`] is the sensors' own view: physical units, no processing. [`ImuFilter`] turns
//! a stream of those into [`FilteredImu`] -- de-noised axes, a gyroscope with its zero-rate
//! offset removed, a magnetometer with the board's hard iron subtracted, and the attitude
//! that falls out of combining all three. Both survive to the telemetry payload: the filtered
//! values are what a consumer should act on, the raw ones are what lets anyone check the
//! filter's work after the fact, or run a better one offline.
//!
//! # Axes
//!
//! Everything here treats the two sensors as sharing one right-handed board frame with **x
//! forward, y left and z up**. Roll, pitch and heading are only meaningful under that
//! assumption; if the parts are mounted differently on the PCB, the fix belongs in
//! `drivers::imu` -- permute and negate the components as they come off each sensor -- rather
//! than in the filter or in whatever consumes the telemetry.
//!
//! # Feeding a UWB fusion filter
//!
//! This IMU is the dead-reckoning half of an indoor localization estimate whose other half is
//! UWB ranging: the ranges fix absolute position at a low rate, the IMU carries the estimate
//! between them. That is what the filtering here is tuned for, and it is a different job from
//! producing a pretty number for a display, in three ways.
//!
//! **Phase lag is a bias.** A downstream estimator treats every sample as the state at its
//! timestamp. A low-pass with a 32 ms group delay hands it a state that is 32 ms old, which
//! at a metre per second is three centimetres of position error the filter has no way to
//! know about. So the cut-offs here sit as high as the noise allows rather than as low as
//! looks smooth, and the anti-aliasing that lets them is done on-chip by the BMI270's own
//! decimation filter -- see `drivers::imu::bmi270`.
//!
//! **Bias beats noise.** Integrated twice into position, a constant offset grows with the
//! square of time while zero-mean noise grows with its square root. Estimating the
//! gyroscope's zero-rate offset while the board is provably still is worth more than any
//! amount of extra smoothing -- and it is also what lets the attitude corrections be slowed
//! down enough to stop mistaking the robot's own acceleration for a change in tilt.
//!
//! **Standstill is information.** [`FilteredImu::is_stationary`] is published so the fusion
//! filter can apply a zero-velocity update: while it holds, velocity is known to be zero,
//! which resets the drift that dead reckoning accumulates while moving. For a robot that
//! spends much of its time parked this is the single most valuable bit in the payload.

use libm::{atan2f, sqrtf};
use serde::{Deserialize, Serialize};

/// Standard gravity: what the accelerometer's magnitude reads when the board is at rest.
const GRAVITY: f32 = 9.806_65;

const DEGREES_PER_RADIAN: f32 = 180.0 / core::f32::consts::PI;
const RADIANS_PER_DEGREE: f32 = core::f32::consts::PI / 180.0;

/// The world frame's upward axis, which is the direction an accelerometer reads at rest.
const WORLD_UP: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

/// Cut-off of the accelerometer low-pass, in Hz. Set from the group delay it costs -- about
/// `1 / (2*pi*fc)`, so 8 ms here -- rather than from how smooth the output looks: see the
/// module docs on phase lag. Aliasing is not this filter's job; the BMI270 is configured to
/// band-limit to ~40 Hz before the firmware ever sees a sample.
const ACCELEROMETER_CUTOFF_HZ: f32 = 20.0;
/// Cut-off of the gyroscope low-pass, in Hz. Higher still: the rate signal is the one the
/// attitude estimate integrates, so delaying it delays the heading, and the part is quiet
/// enough that there is little to gain by smoothing it harder.
const GYROSCOPE_CUTOFF_HZ: f32 = 30.0;
/// Cut-off of the magnetometer low-pass, in Hz. The one axis where heavy smoothing is still
/// right: the field only changes as fast as the robot turns, the heading correction it feeds
/// is deliberately slow anyway, and every step in motor current shows up in it.
const MAGNETOMETER_CUTOFF_HZ: f32 = 5.0;

/// Time constant of the gyroscope zero-rate estimate. Long, because it is only allowed to
/// move while the board is still and a wrong bias is worse than a stale one.
///
/// There is deliberately no matching estimate for the accelerometer. At standstill a
/// horizontal accelerometer offset and a small tilt produce the identical reading, so the
/// two are not separable from the accelerometer alone -- the complementary filter below
/// absorbs such an offset into the attitude, which is why
/// [`FilteredImu::linear_acceleration`] correctly reads zero at rest either way. Telling
/// them apart takes an independent position reference, which is exactly what the UWB half of
/// the localization filter has and this firmware does not; that is the other reason the raw
/// axes go out on the wire.
const GYRO_BIAS_TAU_S: f32 = 10.0;

/// The board counts as still when no filtered rate exceeds this, in °/s...
const STILL_RATE_DPS: f32 = 1.5;
/// ...and the measured acceleration is within this of gravity, in m/s². Strict, because
/// everything gated on standstill -- both bias estimates, and the zero-velocity update the
/// fusion filter runs off [`FilteredImu::is_stationary`] -- is corrupted by a false positive
/// and merely delayed by a false negative.
const STILL_ACCELERATION_TOLERANCE: f32 = 0.35;

/// Time constant of the gravity correction: above it the attitude follows the gyroscope,
/// below it the accelerometer pulls it back.
///
/// Lengthened rather than shortened for tracking, which is the counter-intuitive half of
/// this tuning. A robot accelerating at a metre per second squared tilts its apparent
/// gravity by six degrees while changing its magnitude by only 0.05 m/s², so
/// [`GRAVITY_REFERENCE_TOLERANCE`] cannot catch it; the only defence is to average it away
/// over longer than the acceleration lasts. Three seconds lets in about a sixth of a
/// second-long push instead of the two thirds a one-second constant would, and the drift it
/// gives up in exchange -- a residual gyroscope bias times this constant, so a few tenths of
/// a degree -- is much the smaller error. That trade only works because
/// [`GYRO_BIAS_TAU_S`] keeps the bias small: it is the bias estimate, not a fast correction,
/// that buys short-term accuracy here.
///
/// Attitude error is not a cosmetic problem downstream: it leaks straight into
/// [`FilteredImu::linear_acceleration`] as phantom horizontal acceleration, at 0.17 m/s² per
/// degree, and gets integrated twice.
const GRAVITY_CORRECTION_TAU_S: f32 = 3.0;

/// Time constant of the magnetic heading correction. Slower still, for the same reason taken
/// further: near the drive motors the field is the least trustworthy thing the board
/// measures, while a bias-corrected gyroscope holds a heading to within a degree or so over
/// tens of seconds. At ten seconds a field disturbance lasting a second drags the heading by
/// a tenth of its size, where a two-second constant would let through two fifths of it.
const HEADING_CORRECTION_TAU_S: f32 = 10.0;

/// The accelerometer is ignored as a gravity reference when its magnitude is further than
/// this from g, in m/s². This catches what the time constant above cannot: an impact, a
/// wheel dropping off a ledge, a robot being picked up.
const GRAVITY_REFERENCE_TOLERANCE: f32 = 1.5;

/// The horizontal field direction is bucketed into this many sectors, and this many of them
/// must have been visited before the hard-iron estimate is worth applying.
///
/// Eight of twelve, chosen by measuring the estimate's error against the sectors seen rather
/// than by picking a fraction of a turn that sounded right. The error falls off a cliff
/// between six sectors and seven -- 1.8 µT to 0.03 µT -- and stays there, so eight sits one
/// sector past the knee. Ten, the first guess, cost a further sixty degrees of driving and
/// bought nothing.
const HARD_IRON_SECTORS: u32 = 12;
const HARD_IRON_SECTORS_REQUIRED: u32 = 8;

/// A sample only votes for a sector when its horizontal distance from the centre estimated so
/// far is at least this, in µT. Comfortably above the part's noise, comfortably below the
/// earth's horizontal field anywhere it matters.
const HARD_IRON_MIN_RADIUS_UT: f32 = 5.0;

/// Bounds on the integration step, in seconds. A step shorter than this is noise in the
/// timer; one longer than this means the task was starved -- by a display flush holding the
/// I2C bus, say -- and integrating across the whole gap would kick the attitude estimate.
const MIN_TIMESTEP_S: f32 = 0.0005;
const MAX_TIMESTEP_S: f32 = 0.2;

/// The three-axis reading type, and the rotation type the attitude is carried in.
///
/// Both come from `glam`, built with its `libm` backend so it works without `std`. The
/// arithmetic underneath -- cross and dot products, quaternion composition, rotating a vector
/// by a rotation -- is not the interesting part of this module and is not worth hand-rolling;
/// what is interesting, and what stays here, is which physical convention each number is in.
/// That lives in [`BoardAttitude`].
///
/// These are deliberately not the types the telemetry payload is built from. `glam`
/// serializes a vector as a JSON array, and the wire format here is an object with named
/// axes, so `data::telemetry` keeps its own pair and converts at the boundary. A published
/// format should not move because a maths library changed its mind.
pub use glam::{Quat, Vec3};

/// One pass of the nine-axis stack, exactly as the sensors reported it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default)]
pub struct ImuSample {
    /// Proper acceleration in m/s², i.e. including the ~9.81 m/s² the board feels at rest.
    pub accelerometer: Vec3,
    /// Angular rate in °/s.
    pub gyroscope: Vec3,
    /// Magnetic flux density in µT.
    pub magnetometer: Vec3,
    /// BMI270 die temperature in °C, or `None` while the sensor reports its invalid code.
    pub temperature: Option<f32>,
}

/// What [`ImuFilter`] makes of a sample.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default)]
pub struct FilteredImu {
    /// Low-passed acceleration, m/s². Still includes gravity -- see
    /// [`FilteredImu::linear_acceleration`] for the part that moves the robot.
    pub accelerometer: Vec3,
    /// Low-passed angular rate with the zero-rate offset removed, °/s.
    pub gyroscope: Vec3,
    /// Low-passed magnetic flux density with the hard-iron offset removed once it is known,
    /// µT.
    pub magnetometer: Vec3,
    /// Acceleration in the board frame with gravity taken out, m/s². This is what a
    /// dead-reckoning filter integrates, after rotating it out of the board frame with
    /// [`FilteredImu::quaternion`].
    pub linear_acceleration: Vec3,
    /// The attitude the other three angles are read off, and the one to compute with.
    ///
    /// Unlike them it is well defined at every attitude, needs no angle wrapping, and
    /// rotates a vector in one multiplication instead of a reconstruction that is
    /// ill-conditioned near a vertical nose.
    pub quaternion: Quat,
    /// Rotation about the forward axis, in degrees. Positive leans the robot to its right.
    pub roll: f32,
    /// Rotation about the left axis, in degrees. Positive is **nose down**.
    ///
    /// Surprising only until the frame is taken seriously: the right-hand rule about a y
    /// axis that points left sends the nose downwards. It is the same convention ROS uses
    /// for a body frame (REP 103), which is the one this is likeliest to be read next to.
    pub pitch: f32,
    /// Magnetic heading in degrees, 0 at magnetic north and increasing clockwise. `None`
    /// until the board has turned far enough for the hard-iron estimate to hold up; see
    /// [`HardIron::is_usable`].
    ///
    /// Magnetic north is not the UWB anchor frame's north: a consumer fusing the two needs a
    /// constant offset between them, which is a property of the room and belongs wherever
    /// the anchor positions are configured, not here.
    pub heading: Option<f32>,
    /// Whether the board is at rest, in the strong sense that both bias estimates trust:
    /// no measurable rotation and no acceleration beyond gravity. A fusion filter should
    /// treat this as a zero-velocity update.
    pub is_stationary: bool,
}

/// The filter chain applied to the IMU stream.
///
/// One instance per sensor session: `tasks::imu_monitor` builds a fresh one after every
/// re-initialization, so a filter never carries state from before a sensor dropped off the
/// bus across to after it came back.
pub struct ImuFilter {
    accelerometer: LowPass,
    gyroscope: LowPass,
    magnetometer: LowPass,
    hard_iron: HardIron,
    /// Gyroscope zero-rate offset, °/s.
    gyro_bias: Vec3,
    /// Attitude carried between updates.
    orientation: Quat,
    /// Whether the attitude has been seeded from the first accelerometer reading. Without
    /// this the estimate would start level and take several seconds to find the truth.
    seeded: bool,
    /// Whether the heading has been snapped to the magnetometer, which cannot happen until
    /// the hard-iron estimate is usable.
    heading_seeded: bool,
}

impl ImuFilter {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            accelerometer: LowPass::new(ACCELEROMETER_CUTOFF_HZ),
            gyroscope: LowPass::new(GYROSCOPE_CUTOFF_HZ),
            magnetometer: LowPass::new(MAGNETOMETER_CUTOFF_HZ),
            hard_iron: HardIron::new(),
            gyro_bias: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            seeded: false,
            heading_seeded: false,
        }
    }

    /// Folds one sample in, `timestep` seconds after the previous one.
    pub fn update(&mut self, sample: &ImuSample, timestep: f32) -> FilteredImu {
        let timestep = timestep.clamp(MIN_TIMESTEP_S, MAX_TIMESTEP_S);

        let accelerometer = self.accelerometer.update(sample.accelerometer, timestep);
        let gyroscope = self.gyroscope.update(sample.gyroscope, timestep) - self.gyro_bias;
        let magnetometer = self.filter_magnetometer(sample.magnetometer, timestep);

        let is_stationary = is_stationary(accelerometer, gyroscope);
        if is_stationary {
            // `gyroscope` already has the current estimate subtracted, so what is left of a
            // still board's rate is the error in that estimate; fold it in slowly.
            self.gyro_bias += gyroscope * correction_weight(GYRO_BIAS_TAU_S, timestep);
        }

        self.update_attitude(accelerometer, gyroscope, magnetometer, timestep);

        let (roll, pitch) = self.orientation.roll_pitch();

        FilteredImu {
            accelerometer,
            gyroscope,
            magnetometer,
            // Gravity is subtracted in the board frame, using the attitude to work out which
            // way down currently points -- so this is only as good as the attitude estimate,
            // and a degree of error leaks in as 0.17 m/s² of phantom horizontal acceleration.
            linear_acceleration: accelerometer
                - self.orientation.to_body(Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: GRAVITY,
                }),
            quaternion: self.orientation,
            roll,
            pitch,
            heading: self
                .hard_iron
                .is_usable()
                .then(|| self.orientation.heading()),
            is_stationary,
        }
    }

    fn filter_magnetometer(&mut self, sample: Vec3, timestep: f32) -> Vec3 {
        let magnetometer = self.magnetometer.update(sample, timestep);
        self.hard_iron.observe(magnetometer);
        // Subtracting an offset estimated from too small an arc would be worse than
        // subtracting nothing: it would bake whatever direction the board happens to be
        // facing into the "corrected" field.
        if self.hard_iron.is_usable() {
            magnetometer - self.hard_iron.offset()
        } else {
            magnetometer
        }
    }

    /// Complementary filter: rotate the attitude by the gyroscope for the short term, nudge
    /// it back towards the accelerometer and the magnetometer for the long term.
    ///
    /// Integrating the gyroscope alone drifts without bound; trusting the accelerometer
    /// alone means reading every bump and every turn's centripetal acceleration as a change
    /// in attitude. Blending them gets the responsiveness of the first and the stability of
    /// the second, at a fraction of the arithmetic a Kalman filter would cost.
    ///
    /// The blend happens on the *rate*, not on angles: each reference contributes a small
    /// correcting rotation which is added to the measured rate before it is integrated. That
    /// is what lets the state stay a quaternion -- see [`Quaternion`] for why integrating
    /// roll, pitch and heading separately is not an option.
    fn update_attitude(
        &mut self,
        accelerometer: Vec3,
        gyroscope: Vec3,
        magnetometer: Vec3,
        timestep: f32,
    ) {
        let gravity_usable = gravity_is_trustworthy(accelerometer);

        if !self.seeded {
            // Start from the truth rather than from level, so the estimate does not spend
            // its first seconds converging out of a pose it was never in. Heading has no
            // reference yet, so it starts at zero and gets snapped below once it does.
            if gravity_usable && let Some(up) = accelerometer.try_normalize() {
                self.orientation = Quat::from_gravity(up);
                self.seeded = true;
            }
            return;
        }

        // The heading reference only exists once the hard-iron estimate does, which takes a
        // turn or two. Jumping to it the moment it appears beats letting a ten-second
        // correction crawl there from an arbitrary starting angle.
        if !self.heading_seeded
            && let Some(heading) = self.magnetic_heading(magnetometer)
        {
            self.orientation = self.orientation.with_heading(heading);
            self.heading_seeded = true;
        }

        // Correcting rotations, in °/s, in the body frame.
        let mut correction = Vec3::ZERO;

        if gravity_usable && let Some(measured_up) = accelerometer.try_normalize() {
            // Where the accelerometer says "up" is, against where the current attitude
            // predicts it. Their cross product is the axis that closes the gap, scaled by the
            // sine of the angle between them -- for the small errors this runs at, that is
            // the error itself, in radians.
            //
            // The order of the operands is the whole correction, not a detail. A body-frame
            // rate `w` moves the predicted direction by `p x w`, so feeding back
            // `w = k (m x p)` gives `k (m - p cos0)`, which travels towards the measurement.
            // The other order gives `k (p cos0 - m)`, which travels away from it -- and since
            // the cross product also vanishes when the two are exactly opposed, "away" has a
            // resting place: the attitude settles upside down and stays there, reporting a
            // level board while subtracting gravity with the wrong sign. It is a sign error
            // that a noiseless simulation seeded at the right attitude cannot see, because
            // both signs sit still at the answer.
            let predicted_up = self.orientation.to_body(WORLD_UP);

            // An estimate more than a right angle from the measured gravity is not something
            // a first-order correction should be asked to walk off. The correcting rotation
            // is a sine, so it weakens as the error grows past a right angle and vanishes
            // altogether at half a turn: an attitude that far out either takes minutes to
            // come back or, at exactly half a turn, never does. Snap it to what gravity
            // says, keeping the heading, and let the ordinary correction take it from there.
            if measured_up.dot(predicted_up) < 0.0 {
                self.orientation =
                    Quat::from_gravity(measured_up).with_heading(self.orientation.heading());
                return;
            }

            correction +=
                measured_up.cross(predicted_up) * (DEGREES_PER_RADIAN / GRAVITY_CORRECTION_TAU_S);
        }

        if let Some(from_magnetometer) = self.magnetic_heading(magnetometer) {
            // Taken the shortest way round, so a gyro estimate at 359° and a magnetic one at
            // 1° pull together rather than the long way apart. Applied about the world's
            // vertical only: a disturbed field must not be able to tip roll or pitch.
            let error = wrap_180(from_magnetometer - self.orientation.heading());
            let about_vertical = Vec3 {
                x: 0.0,
                y: 0.0,
                // A compass heading grows clockwise seen from above, while a positive
                // rotation about the upward axis of a right-handed frame is
                // counter-clockwise: hence the sign.
                z: -error / HEADING_CORRECTION_TAU_S,
            };
            correction += self.orientation.to_body(about_vertical);
        }

        self.orientation = self
            .orientation
            .integrated(gyroscope + correction, timestep);
    }

    /// Magnetic heading from the field, projected onto the horizontal plane using the
    /// attitude estimate. Without that projection a tilted board reads a heading error of
    /// the same order as the tilt.
    fn magnetic_heading(&self, magnetometer: Vec3) -> Option<f32> {
        if !self.hard_iron.is_usable() {
            return None;
        }

        // Undo roll and pitch but not heading, leaving the field in a frame that is level
        // yet still points where the board points. Its horizontal part then points at
        // magnetic north, and the angle from the nose round to it is the bearing.
        //
        // Done by rotating with the attitude rather than by the usual expansion in sines and
        // cosines of roll and pitch. That expansion is written in most references for a
        // frame whose y axis points *right*; this one's points left, which flips the sign of
        // the sideways term. Reusing the rotation that is already exercised everywhere else
        // here leaves nowhere for that mismatch to hide.
        let (roll, pitch) = self.orientation.roll_pitch();
        let level = Quat::from_board_euler(roll, pitch, 0.0).to_world(magnetometer);

        // `level.x` is how much of north lies along the nose, `level.y` how much lies to the
        // left. Facing north puts all of it on the nose; facing east puts all of it to the
        // left, which must read as 90°, hence the leftward component is the one that leads.
        Some(wrap_360(atan2f(level.y, level.x) * DEGREES_PER_RADIAN))
    }
}

impl Default for ImuFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// The board's own conventions, layered onto `glam`'s rotation type.
///
/// The attitude is carried as a unit quaternion taking board-frame vectors to the world
/// frame. The obvious alternative -- carrying roll, pitch and heading and integrating each
/// against its own gyroscope axis -- is wrong, and wrong in a way that looks fine in the
/// tests one naturally writes for it. Those three rates are only independent while the board
/// is level: tilt it, and turning about its own vertical axis changes roll and pitch too, by
/// terms proportional to the tilt. Dropping those terms costs about 1.5° of attitude error
/// per degree of tilt while turning, which for the dead reckoning this feeds means 0.17 m/s²
/// of phantom acceleration per degree, integrated twice.
///
/// Writing the coupling terms out instead is exact but divides by the cosine of the pitch,
/// so it blows up when the board is nose-up -- the classic gimbal lock. A quaternion has
/// neither problem: rotation composes correctly at every attitude, and there is nothing to
/// divide by. Roll, pitch and heading are then only ever an output format -- and one that
/// still degenerates at a vertical nose, which is why the quaternion goes out on the wire
/// alongside them.
///
/// # World frame
///
/// **x magnetic north, y west, z up**, right-handed. Not the more usual ENU: it is the frame
/// in which `heading` is simply the azimuth of the board's nose. A consumer that wants ENU
/// has `east = -y`, `north = x`, `up = z`.
pub trait BoardAttitude {
    /// The attitude implied by a measured "up" direction in the board frame, with the
    /// heading left at zero because gravity says nothing about it.
    fn from_gravity(up_in_body: Vec3) -> Self;
    /// Built from the board's three angles, in degrees.
    fn from_board_euler(roll: f32, pitch: f32, heading: f32) -> Self;
    /// This attitude with its heading replaced, leaving roll and pitch alone.
    fn with_heading(self, heading: f32) -> Self;
    /// Advanced by a board-frame angular rate in °/s over `timestep` seconds.
    fn integrated(self, rate_dps: Vec3, timestep: f32) -> Self;
    /// Roll and pitch in degrees, read off the direction this attitude puts "up" in the
    /// board frame -- i.e. exactly what a perfect accelerometer would report.
    fn roll_pitch(self) -> (f32, f32);
    /// Compass heading in degrees: where the board's nose points, projected onto the
    /// horizontal plane, clockwise from north.
    fn heading(self) -> f32;
    /// Rotates a world-frame vector into the board frame.
    fn to_body(self, world: Vec3) -> Vec3;
    /// Rotates a board-frame vector into the world frame.
    fn to_world(self, body: Vec3) -> Vec3;
}

impl BoardAttitude for Quat {
    fn from_gravity(up_in_body: Vec3) -> Self {
        let roll = atan2f(up_in_body.y, up_in_body.z) * DEGREES_PER_RADIAN;
        let pitch = atan2f(
            -up_in_body.x,
            sqrtf(up_in_body.y * up_in_body.y + up_in_body.z * up_in_body.z),
        ) * DEGREES_PER_RADIAN;
        Self::from_board_euler(roll, pitch, 0.0)
    }

    fn from_board_euler(roll: f32, pitch: f32, heading: f32) -> Self {
        // A compass heading runs clockwise; a rotation about the world's upward axis runs
        // the other way, hence the negation.
        Quat::from_rotation_z(-heading * RADIANS_PER_DEGREE)
            * Quat::from_rotation_y(pitch * RADIANS_PER_DEGREE)
            * Quat::from_rotation_x(roll * RADIANS_PER_DEGREE)
    }

    fn with_heading(self, heading: f32) -> Self {
        let (roll, pitch) = self.roll_pitch();
        Self::from_board_euler(roll, pitch, heading)
    }

    fn integrated(self, rate_dps: Vec3, timestep: f32) -> Self {
        // `from_scaled_axis` takes the rotation vector directly -- axis times angle, in
        // radians -- which is exactly what a rate times a timestep is. An exact rotation by
        // the angle swept, with no small-angle assumption left anywhere.
        (self * Quat::from_scaled_axis(rate_dps * RADIANS_PER_DEGREE * timestep)).normalize()
    }

    fn roll_pitch(self) -> (f32, f32) {
        let up = self.to_body(WORLD_UP);
        (
            atan2f(up.y, up.z) * DEGREES_PER_RADIAN,
            atan2f(-up.x, sqrtf(up.y * up.y + up.z * up.z)) * DEGREES_PER_RADIAN,
        )
    }

    fn heading(self) -> f32 {
        let nose = self.to_world(Vec3::X);
        wrap_360(atan2f(-nose.y, nose.x) * DEGREES_PER_RADIAN)
    }

    fn to_body(self, world: Vec3) -> Vec3 {
        self.conjugate() * world
    }

    fn to_world(self, body: Vec3) -> Vec3 {
        self * body
    }
}

/// First-order IIR low-pass over a three-axis signal.
///
/// The coefficient is recomputed from the actual timestep on every update rather than fixed
/// at construction: this task shares its I2C bus with the display and the charger, so its
/// period is only nominally regular, and a fixed coefficient would make the effective
/// cut-off wander with the bus contention.
struct LowPass {
    cutoff_hz: f32,
    state: Option<Vec3>,
}

impl LowPass {
    const fn new(cutoff_hz: f32) -> Self {
        Self {
            cutoff_hz,
            state: None,
        }
    }

    fn update(&mut self, sample: Vec3, timestep: f32) -> Vec3 {
        let state = match self.state {
            // Start on the first sample rather than at zero, so the output does not have to
            // climb out of a hole that says nothing about the signal.
            None => sample,
            Some(previous) => {
                let time_constant = 1.0 / (core::f32::consts::TAU * self.cutoff_hz);
                previous + (sample - previous) * correction_weight(time_constant, timestep)
            }
        };

        self.state = Some(state);
        state
    }
}

/// Running estimate of the constant field the board carries with it.
///
/// Motor magnets, the battery's tabs and any steel near the magnetometer add a fixed vector
/// to every reading in the board frame -- hard iron. Turning the board sweeps the earth's
/// field around a circle whose centre is that vector, so the midpoint of what each axis has
/// been seen to span estimates it. This is only as good as the arc the robot has actually
/// driven through, hence [`HardIron::is_usable`].
struct HardIron {
    min: Vec3,
    max: Vec3,
    seen: bool,
    /// One bit per sector of the horizontal field direction already visited.
    sectors: u16,
}

impl HardIron {
    const fn new() -> Self {
        Self {
            min: Vec3::ZERO,
            max: Vec3::ZERO,
            seen: false,
            sectors: 0,
        }
    }

    fn observe(&mut self, sample: Vec3) {
        if !self.seen {
            self.min = sample;
            self.max = sample;
            self.seen = true;
            return;
        }

        self.min = Vec3 {
            x: self.min.x.min(sample.x),
            y: self.min.y.min(sample.y),
            z: self.min.z.min(sample.z),
        };
        self.max = Vec3 {
            x: self.max.x.max(sample.x),
            y: self.max.y.max(sample.y),
            z: self.max.z.max(sample.z),
        };

        // Which way round the circle this sample sits, measured from the centre estimated so
        // far. Samples too close to that centre have no meaningful direction -- a board
        // standing still would otherwise let its own noise wander across every sector and
        // announce itself calibrated.
        let centred = sample - self.offset();
        if sqrtf(centred.x * centred.x + centred.y * centred.y) >= HARD_IRON_MIN_RADIUS_UT {
            let bearing = wrap_360(atan2f(centred.y, centred.x) * DEGREES_PER_RADIAN);
            let sector = (bearing / (360.0 / HARD_IRON_SECTORS as f32)) as u32;
            self.sectors |= 1 << sector.min(HARD_IRON_SECTORS - 1);
        }
    }

    fn offset(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Whether the board has been round enough of the circle for [`HardIron::offset`] to be
    /// an estimate rather than a guess.
    ///
    /// Counting sectors rather than measuring how far each axis has swung, which is the
    /// obvious test and does not work: a quarter turn already swings both horizontal axes
    /// across most of their range, so a span threshold passes while the box is still a
    /// quarter of the circle and its centre is out by most of the field strength. Measured on
    /// the bench, the span test announced itself ready after 86° with the offset wrong by
    /// 15.7 µT -- and a wrong offset does not merely make the bearing wrong, it makes it wrong
    /// in a way that slowly corrects itself as the estimate improves, which reads as a bearing
    /// that drifts for half a minute after the robot stops.
    ///
    /// Only the horizontal plane is considered: a robot driving on a floor turns about the
    /// vertical axis, which therefore never sweeps and would hold the estimate back forever.
    fn is_usable(&self) -> bool {
        self.sectors.count_ones() >= HARD_IRON_SECTORS_REQUIRED
    }
}

/// Weight a first-order blend gives its correcting term after `timestep` seconds, for a
/// filter whose time constant is `tau_s`. The complementary filter, both bias estimates and
/// every low-pass here are the same arithmetic under different names.
fn correction_weight(tau_s: f32, timestep: f32) -> f32 {
    timestep / (tau_s + timestep)
}

/// Whether the board is at rest in the strong sense both bias estimates need: not turning,
/// and feeling nothing but gravity.
fn is_stationary(accelerometer: Vec3, gyroscope: Vec3) -> bool {
    gyroscope.abs().max_element() < STILL_RATE_DPS
        && f32::abs(accelerometer.length() - GRAVITY) < STILL_ACCELERATION_TOLERANCE
}

/// Whether the accelerometer currently says anything useful about which way down is. It does
/// not in free fall, under an impact, or while the robot is being carried.
fn gravity_is_trustworthy(accelerometer: Vec3) -> bool {
    f32::abs(accelerometer.length() - GRAVITY) < GRAVITY_REFERENCE_TOLERANCE
}

/// Folds an angle in degrees into `[0, 360)`.
fn wrap_360(degrees: f32) -> f32 {
    let wrapped = degrees % 360.0;
    if wrapped < 0.0 {
        wrapped + 360.0
    } else {
        wrapped
    }
}

/// Folds an angle in degrees into `[-180, 180)`, i.e. the shortest way round.
fn wrap_180(degrees: f32) -> f32 {
    let wrapped = wrap_360(degrees);
    if wrapped >= 180.0 {
        wrapped - 360.0
    } else {
        wrapped
    }
}
