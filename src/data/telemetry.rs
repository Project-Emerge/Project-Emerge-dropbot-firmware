use heapless::String;
use serde::{Deserialize, Serialize};

use crate::data::imu::{FilteredImu, ImuSample};

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub enum MotorTelemetry {
    Stopped,
    Motoring { left: f32, right: f32 },
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct BatteryTelemetry {
    /// Pack voltage in volts, i.e. both cells of the 2S pack in series.
    pub voltage: f32,
    /// Charge current into the pack, in amps.
    pub current: f32,
    /// Charger junction temperature in degrees Celsius. The BQ25887 only exposes a coarse
    /// hot/cold classification for the pack itself, so this is the IC's own die.
    pub temperature: f32,
    pub is_charging: bool,
    /// Rough state of charge in percent; see `data::battery::ChargerStatus::state_of_charge`.
    pub state_of_charge: u8,
}

/// One IMU pass as it goes out on the wire.
///
/// The same payload is published two ways: once per second inside [`Telemetry`], as a
/// summary of what the robot is doing, and at the rate `tasks::imu_monitor` streams at on
/// its own topic for observers and offline analysis.
///
/// The two halves are the filter's own types, serialized as they are. Vectors come out as
/// `[x, y, z]` and the attitude quaternion as `[x, y, z, w]` -- note the scalar part last,
/// which is `glam`'s ordering.
#[derive(Serialize, Deserialize, Clone, Copy, Default, Debug)]
pub struct IMUTelemetry {
    /// Microseconds since the board booted, read as close to the sensor transaction as the
    /// scheduler allows.
    ///
    /// The arrival time of an MQTT message is not the sample time, since it also carries
    /// whatever delay the queue, Wi-Fi link and broker added. This value is relative to boot
    /// and jumps back to zero if the board resets.
    pub timestamp_us: u64,
    /// Unfiltered: physical units in the board frame, and nothing else. No low-pass, no bias
    /// removal, no hard-iron correction. Published alongside the filtered values so a
    /// consumer can check the filter's work, or run a better one of its own.
    pub raw: ImuSample,
    /// What `data::imu::ImuFilter` made of the same readings, plus what it derives from
    /// them: gravity-free acceleration, attitude, and whether the robot is standing still.
    /// This is the half to act on.
    pub filtered: FilteredImu,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct NetworkTelemetry {
    pub rssi: i32,
    pub ip_address: Option<String<16>>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Telemetry {
    pub motor_telemetry: MotorTelemetry,
    pub battery_telemetry: BatteryTelemetry,
    pub imu_telemetry: IMUTelemetry,
    pub network_telemetry: NetworkTelemetry,
}
