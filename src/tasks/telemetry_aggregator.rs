use ariel_os::time::Timer;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};

use crate::data;

#[ariel_os::task]
pub async fn aggregate_telemetry(
    motor_telemetry_rx: Receiver<
        'static,
        CriticalSectionRawMutex,
        data::telemetry::MotorTelemetry,
        2,
    >,
    battery_telemetry_rx: Receiver<
        'static,
        CriticalSectionRawMutex,
        data::telemetry::BatteryTelemetry,
        2,
    >,
    imu_telemetry_rx: Receiver<'static, CriticalSectionRawMutex, data::telemetry::IMUTelemetry, 2>,
    network_telemetry_rx: Receiver<
        'static,
        CriticalSectionRawMutex,
        data::telemetry::NetworkTelemetry,
        2,
    >,
    aggregated_telemetry_tx: Sender<
        'static,
        CriticalSectionRawMutex,
        data::telemetry::Telemetry,
        1,
    >,
) -> ! {
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
        if let Ok(motor) = motor_telemetry_rx.try_receive() {
            motor_telemetry = motor;
        }
        if let Ok(battery) = battery_telemetry_rx.try_receive() {
            battery_telemetry = battery;
        }
        if let Ok(imu) = imu_telemetry_rx.try_receive() {
            imu_telemetry = imu;
        }
        if let Ok(network) = network_telemetry_rx.try_receive() {
            network_telemetry = network;
        }

        let telemetry = data::telemetry::Telemetry {
            motor_telemetry,
            battery_telemetry,
            imu_telemetry,
            network_telemetry: network_telemetry.clone(), // Clone the network telemetry to avoid ownership issues
        };

        let _ = aggregated_telemetry_tx.try_send(telemetry);

        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}
