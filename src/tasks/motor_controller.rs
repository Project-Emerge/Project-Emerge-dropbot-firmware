use ariel_os::gpio::Output;
use ariel_os::log::{Debug2Format, error, info, warn};
use ariel_os::time::Timer;
use embassy_futures::select::{Either3, select3};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::time::Rate;

use crate::data;
use crate::data::telemetry::MotorTelemetry;
use crate::drivers::motor_driver::{DRV8833Driver, types::MotorConfig};
use crate::pins;
use crate::task_sync::{MotorCommandRx, MotorConfigurationRx, MotorTelemetryTx, OtaStatusRx};
use crate::traits::{MotorController, MotorStatus};

/// Messaging endpoints owned by the motor-controller task.
pub struct MotorControllerPorts {
    pub motor_telemetry: MotorTelemetryTx,
    pub motor_commands: MotorCommandRx,
    pub motor_configurations: MotorConfigurationRx,
    pub ota_status: OtaStatusRx,
}

#[ariel_os::task]
pub async fn manage_motor_controller(
    pins: pins::MotorDriverPins,
    mut ports: MotorControllerPorts,
) -> ! {
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    let mut pwm_module = McPwm::new(pins.pwm_device, clock_cfg);
    pwm_module.operator0.set_timer(&pwm_module.timer0);
    pwm_module.operator1.set_timer(&pwm_module.timer0);
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(1599, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();
    pwm_module.timer0.start(timer_clock_cfg);

    let (ain1, ain2) = pwm_module.operator0.with_pins(
        pins.ain1,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pins.ain2,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let (bin1, bin2) = pwm_module.operator1.with_pins(
        pins.bin1,
        PwmPinConfig::UP_ACTIVE_HIGH,
        pins.bin2,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let sleep_pin = Output::new(pins.sleep, ariel_os::gpio::Level::High);
    // DRV8833 tWAKE: up to 1ms needed after nSLEEP goes high before the H-bridge is operational.
    Timer::after(ariel_os::time::Duration::from_millis(1)).await;
    let mut motor_driver =
        DRV8833Driver::new(ain1, ain2, bin1, bin2, sleep_pin, MotorConfig::default());

    loop {
        // An OTA update is about to overwrite this firmware and reboot into it. Cut the
        // motors and hold them de-energized for its whole duration, so the robot cannot
        // drive off unattended across the reboot. `set_speed` drives SLEEP high again, so
        // a failed update needs no explicit wake-up here.
        if let Some(status) = ports.ota_status.try_changed() && status.is_active() {
            if let Err(e) = motor_driver.stop().and_then(|()| motor_driver.sleep()) {
                error!("motors: shutdown for OTA failed: {:?}", Debug2Format(&e));
            }
            info!("motors: disabled for OTA update");

            while ports.ota_status.changed().await.is_active() {}
            info!("motors: OTA update ended, resuming");
        }

        match select3(
            ports.motor_commands.receive(),
            ports.motor_configurations.receive(),
            Timer::after(ariel_os::time::Duration::from_secs(3)),
        )
        .await
        {
            Either3::First(command) => {
                info!("motors: received command: {:?}", Debug2Format(&command));
                match command {
                    data::commands::DriveCommand::Move { left, right } => {
                        if let Err(e) = motor_driver.set_speed(left, right) {
                            error!(
                                "motors: set_speed({}, {}) failed: {:?}",
                                left,
                                right,
                                Debug2Format(&e)
                            );
                        }
                    }
                    data::commands::DriveCommand::Stop => {
                        if let Err(e) = motor_driver.stop() {
                            error!("motors: stop failed: {:?}", Debug2Format(&e));
                        }
                    }
                }
            }
            Either3::Second(config) => {
                info!("motors: received configuration: {:?}", Debug2Format(&config));
                let motor_config = MotorConfig {ema_filter_alpha: config.ema_filter_alpha, min_duty_cycle: config.min_duty_cycle };
                if let Err(e) = motor_driver.set_config(motor_config) {
                    error!(
                        "motors: set_config({:?}) failed: {:?}",
                        Debug2Format(&config),
                        Debug2Format(&e)
                    );
                }
            },
            Either3::Third(()) => {
                warn!("motors: no command received for 3s, stopping");
                if let Err(e) = motor_driver.stop() {
                    error!("motors: stop failed: {:?}", Debug2Format(&e));
                }
            }
        };
        match motor_driver.get_status() {
            Ok(MotorStatus::Stopped) => {
                ports
                    .motor_telemetry
                    .try_send(MotorTelemetry::Stopped)
                    .unwrap_or_else(|e| {
                        error!("motors: failed to send telemetry: {:?}", Debug2Format(&e))
                    });
            }
            Ok(MotorStatus::Motoring { left, right }) => {
                ports
                    .motor_telemetry
                    .try_send(MotorTelemetry::Motoring { left, right })
                    .unwrap_or_else(|e| {
                        error!("motors: failed to send telemetry: {:?}", Debug2Format(&e))
                    });
            }
            Err(e) => error!("motors: get_status failed: {:?}", Debug2Format(&e)),
        }
        Timer::after(ariel_os::time::Duration::from_millis(10)).await;
    }
}
