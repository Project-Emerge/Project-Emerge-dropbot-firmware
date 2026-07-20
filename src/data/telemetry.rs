use heapless::String;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct MotorTelemetry {
    pub left_motor_rpm: f32,
    pub right_motor_rpm: f32,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct BatteryTelemetry {
    pub voltage: f32,
    pub current: f32,
    pub temperature: f32,
    pub is_charging: bool,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct AccelerometerTelemetry {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct GyroscopeTelemetry {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct MagnetometerTelemetry {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct IMUTelemetry {
    pub accelerometer: AccelerometerTelemetry,
    pub gyroscope: GyroscopeTelemetry,
    pub magnetometer: MagnetometerTelemetry,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct NetworkTelemetry {
    pub rssi: i32,
    pub ip_address: Option<String<16>>,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Telemetry {
    pub motor_telemetry: MotorTelemetry,
    pub battery_telemetry: BatteryTelemetry,
    pub imu_telemetry: IMUTelemetry,
    pub network_telemetry: NetworkTelemetry,
}
