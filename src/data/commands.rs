use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub enum DriveCommand {
    Move { left: f32, right: f32 },
    Stop,
}
