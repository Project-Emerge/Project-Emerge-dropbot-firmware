use ariel_os::time::Timer;

use crate::data;
use crate::task_sync::{
    AggregatedTelemetryTx, BatteryTelemetryRx, ImuTelemetryRx, MotorTelemetryRx, NetworkTelemetryRx,
};

/// Messaging endpoints owned by the telemetry aggregation task.
pub struct TelemetryAggregatorPorts {
    pub motor_telemetry: MotorTelemetryRx,
    pub battery_telemetry: BatteryTelemetryRx,
    pub imu_telemetry: ImuTelemetryRx,
    pub network_telemetry: NetworkTelemetryRx,
    pub aggregated_telemetry: AggregatedTelemetryTx,
}

#[ariel_os::task]
pub async fn aggregate_telemetry(ports: TelemetryAggregatorPorts) -> ! {
    let mut motor_telemetry = data::telemetry::MotorTelemetry::Stopped;
    let mut battery_telemetry = data::telemetry::BatteryTelemetry {
        voltage: 0.0,
        current: 0.0,
        temperature: 0.0,
        is_charging: false,
        state_of_charge: 0,
    };
    let mut imu_telemetry = data::telemetry::IMUTelemetry::default();
    let mut network_telemetry = data::telemetry::NetworkTelemetry {
        rssi: 0,
        ip_address: None,
    };

    loop {
        if let Ok(motor) = ports.motor_telemetry.try_receive() {
            motor_telemetry = motor;
        }
        if let Ok(battery) = ports.battery_telemetry.try_receive() {
            battery_telemetry = battery;
        }
        if let Ok(imu) = ports.imu_telemetry.try_receive() {
            imu_telemetry = imu;
        }
        if let Ok(network) = ports.network_telemetry.try_receive() {
            network_telemetry = network;
        }

        let telemetry = data::telemetry::Telemetry {
            motor_telemetry,
            battery_telemetry,
            imu_telemetry,
            network_telemetry: network_telemetry.clone(), // Clone the network telemetry to avoid ownership issues
        };

        let _ = ports.aggregated_telemetry.try_send(telemetry);

        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}
