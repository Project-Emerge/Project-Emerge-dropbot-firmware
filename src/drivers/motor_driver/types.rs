/// Configuration for a motor driver, including maximum speed and optional EMA filter alpha value.
pub struct MotorConfig {
    /// Optional alpha value for an Exponential Moving Average (EMA) filter to smooth speed changes.
    /// If `None`, no EMA filtering will be applied.
    pub ema_filter_alpha: Option<f32>
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            ema_filter_alpha: Some(0.1),
        }
    }
}
