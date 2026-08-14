use crate::data::configurations::MotorsConfiguration;

/// Configuration for a motor driver, including maximum speed and optional EMA filter alpha value.
pub struct MotorConfig {
    /// Optional alpha value for an Exponential Moving Average (EMA) filter to smooth speed changes.
    /// If `None`, no EMA filtering will be applied.
    pub ema_filter_alpha: Option<f32>,
    /// Ceiling on the magnitude of either side's duty cycle, as a fraction in `[0, 1]`.
    ///
    /// Applied as a clamp on the filter's *output*, so it bounds what actually reaches the H-bridge
    /// rather than only what was asked for -- an EMA converging towards an over-range command would
    /// otherwise walk past the ceiling on its way there.
    pub max_speed: f32,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self::from(MotorsConfiguration::default())
    }
}

/// The values that arrive over MQTT are the values the driver runs on; this is the only conversion
/// between the two, and it sanitizes on the way through so the driver never holds an alpha or a
/// ceiling it cannot act on. See [`MotorsConfiguration::sanitized`].
impl From<MotorsConfiguration> for MotorConfig {
    fn from(configuration: MotorsConfiguration) -> Self {
        let configuration = configuration.sanitized();
        Self {
            ema_filter_alpha: configuration.ema_filter_alpha,
            max_speed: configuration.max_speed,
        }
    }
}
