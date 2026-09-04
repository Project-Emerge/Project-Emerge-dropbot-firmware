/// Configuration for a motor driver, including maximum speed and optional EMA filter alpha value.
pub struct MotorConfig {
    /// Optional alpha value for an Exponential Moving Average (EMA) filter to smooth speed changes.
    /// If `None`, no EMA filtering will be applied.
    pub ema_filter_alpha: Option<f32>,
    /// Minimum duty cycle (0.0-1.0) applied to any nonzero speed command, to compensate for motor
    /// stiction: below this duty cycle the motor lacks the torque to overcome static friction and
    /// stalls even though PWM is being driven. Nonzero commands are remapped from `(0.0, 1.0]` onto
    /// `[min_duty_cycle, 1.0]` so small commands still produce motion; a command of exactly `0.0`
    /// still fully stops the motor. Tune to the lowest duty cycle at which the motor reliably starts.
    pub min_duty_cycle: f32,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            ema_filter_alpha: Some(0.1),
            min_duty_cycle: 0.3,
        }
    }
}
