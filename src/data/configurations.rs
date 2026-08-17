// These types are not constructed directly yet: they are `Deserialize` targets for a remote
// configuration command that has not been wired up to `data::commands`.
#![allow(dead_code)]

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct MotorsConfiguration {
    pub ema_filter_alpha: Option<f32>,
    pub max_speed: f32,
}

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub motors: MotorsConfiguration,
}
