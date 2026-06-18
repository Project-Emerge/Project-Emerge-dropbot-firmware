pub struct MotorConfig {
    pub max_speed: f32,
    // pub acceleration: f32,
    pub pwm_frequency_hz: u32,
    pub ema_filter_alpha: Option<f32>,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self {
            max_speed: 1.0,
            // acceleration: 0.5,
            pwm_frequency_hz: 1000,
            ema_filter_alpha: Some(0.1),
        }
    }
}
