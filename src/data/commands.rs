use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub enum DriveCommand {
    Move { left: f32, right: f32 },
    Stop,
}
