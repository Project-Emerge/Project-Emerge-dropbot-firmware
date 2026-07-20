use ariel_os::time::Timer;
use crate::data;
use crate::{MOTOR_TELEMETRY, BATTERY_TELEMETRY, IMU_TELEMETRY, NETWORK_TELEMETRY, AGGREGATED_TELEMETRY};

#[ariel_os::task]
pub async fn aggregate_telemetry() -> ! {
    let mut motor_telemetry = data::telemetry::MotorTelemetry {
        left_motor_rpm: 0.0,
        right_motor_rpm: 0.0,
    };
    let mut battery_telemetry = data::telemetry::BatteryTelemetry {
        voltage: 0.0,
        current: 0.0,
        temperature: 0.0,
        is_charging: false,
    };
    let mut imu_telemetry = data::telemetry::IMUTelemetry {
        accelerometer: data::telemetry::AccelerometerTelemetry {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        gyroscope: data::telemetry::GyroscopeTelemetry {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        magnetometer: data::telemetry::MagnetometerTelemetry {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    };
    let mut network_telemetry = data::telemetry::NetworkTelemetry {
        rssi: 0,
        ip_address: None,
    };

    loop {
        if let Ok(motor) = MOTOR_TELEMETRY.try_receive() {
            motor_telemetry = motor;
        }
        if let Ok(battery) = BATTERY_TELEMETRY.try_receive() {
            battery_telemetry = battery;
        }
        if let Ok(imu) = IMU_TELEMETRY.try_receive() {
            imu_telemetry = imu;
        }
        if let Ok(network) = NETWORK_TELEMETRY.try_receive() {
            network_telemetry = network;
        }

        let telemetry = data::telemetry::Telemetry {
            motor_telemetry,
            battery_telemetry,
            imu_telemetry,
            network_telemetry: network_telemetry.clone(), // Clone the network telemetry to avoid ownership issues
        };

        AGGREGATED_TELEMETRY.send(telemetry).await;

        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}
