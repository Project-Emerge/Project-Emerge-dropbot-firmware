use heapless::Vec;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct MotorsConfiguration {
    pub ema_filter_alpha: Option<f32>,
    pub max_speed: f32,
}

#[derive(Serialize, Deserialize)]
pub struct AnchorPosition {
    pub x: f32,
    pub y: f32,
    pub z: Option<f32>,
}

#[derive(Serialize, Deserialize)]
pub struct AnchorsDisplacementConfiguration {
    pub anchors: Vec<AnchorPosition, 8>,
}

#[derive(Serialize, Deserialize)]
pub struct Configuration {
    pub motors: MotorsConfiguration,
    pub anchors_displacement: AnchorsDisplacementConfiguration,
}
