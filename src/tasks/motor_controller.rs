use ariel_os::gpio::Output;
use ariel_os::log::{Debug2Format, error, info, warn};
use ariel_os::time::Timer;
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::time::Rate;

use crate::data;
use crate::data::configurations::MotorsConfiguration;
use crate::data::ota::OtaStatus;
use crate::data::telemetry::MotorTelemetry;
use crate::drivers::motor_driver::{DRV8833Driver, types::MotorConfig};
use crate::pins;
use crate::traits::{MotorController, MotorStatus};

#[ariel_os::task]
pub async fn manage_motor_controller(
    pins: pins::MotorDriverPins,
    motor_telemetry: Sender<'static, CriticalSectionRawMutex, data::telemetry::MotorTelemetry, 2>,
    motor_command: Receiver<'static, CriticalSectionRawMutex, data::commands::DriveCommand, 2>,
    mut ota_status: WatchReceiver<'static, CriticalSectionRawMutex, OtaStatus, 2>,
    forward_duty: WatchSender<'static, CriticalSectionRawMutex, f32, 2>,
    mut motors_config: WatchReceiver<'static, CriticalSectionRawMutex, MotorsConfiguration, 1>,
    mut calibration_interlock: WatchReceiver<'static, CriticalSectionRawMutex, bool, 1>,
    calibration_safe: Sender<'static, CriticalSectionRawMutex, (), 1>,
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
        // Polled at the top of every pass rather than given a `select` branch of its own, the same
        // way `tasks::pose_estimator` polls its own configuration: a `Watch::try_changed` is a lock
        // and a compare, and this loop already runs at least every 3 s.
        if let Some(config) = motors_config.try_changed() {
            let config = config.sanitized();
            info!(
                "motors: configuration updated, max speed {}, ema alpha {:?}",
                config.max_speed,
                Debug2Format(&config.ema_filter_alpha)
            );
            if let Err(e) = motor_driver.set_config(MotorConfig::from(config)) {
                error!(
                    "motors: applying new configuration failed: {:?}",
                    Debug2Format(&e)
                );
            }
        }

        // An OTA update is about to overwrite this firmware and reboot into it. Cut the
        // motors and hold them de-energized for its whole duration, so the robot cannot
        // drive off unattended across the reboot. `set_speed` drives SLEEP high again, so
        // a failed update needs no explicit wake-up here.
        if let Some(status) = ota_status.try_changed()
            && status.is_active()
        {
            if let Err(e) = motor_driver.stop().and_then(|()| motor_driver.sleep()) {
                error!("motors: shutdown for OTA failed: {:?}", Debug2Format(&e));
            }
            info!("motors: disabled for OTA update");

            while ota_status.changed().await.is_active() {}
            info!("motors: OTA update ended, resuming");
        }

        match select3(
            motor_command.receive(),
            Timer::after(ariel_os::time::Duration::from_secs(3)),
            calibration_interlock.changed(),
        )
        .await
        {
            Either3::First(command) => {
                if calibration_interlock.try_get() == Some(true) {
                    warn!("motors: movement command rejected by calibration interlock");
                    continue;
                }
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
            Either3::Second(()) => {
                warn!("motors: no command received for 3s, stopping");
                if let Err(e) = motor_driver.stop() {
                    error!("motors: stop failed: {:?}", Debug2Format(&e));
                }
            }
            Either3::Third(locked) => {
                if locked {
                    if let Err(e) = motor_driver.stop().and_then(|()| motor_driver.sleep()) {
                        error!(
                            "motors: calibration interlock failed: {:?}",
                            Debug2Format(&e)
                        );
                    } else {
                        forward_duty.send(0.0);
                        let _ = motor_telemetry.try_send(MotorTelemetry::Stopped);
                        calibration_safe.send(()).await;
                        info!("motors: bridge disabled by calibration interlock");
                    }
                } else {
                    info!("motors: calibration interlock released");
                }
                continue;
            }
        };
        match motor_driver.get_status() {
            Ok(MotorStatus::Stopped) => {
                forward_duty.send(0.0);
                motor_telemetry
                    .try_send(MotorTelemetry::Stopped)
                    .unwrap_or_else(|e| {
                        error!("motors: failed to send telemetry: {:?}", Debug2Format(&e))
                    });
            }
            Ok(MotorStatus::Motoring { left, right }) => {
                // The mean of the two signed side duties is the forward component; their difference is
                // the turn, which the pose estimator takes from the gyroscope instead. Reported from
                // `get_status` rather than from the command, so it reflects what the driver's own
                // slew-rate filter actually applied. This is the only forward-motion input the
                // estimator has -- the robots have no wheel encoders.
                forward_duty.send(0.5 * (left + right));
                motor_telemetry
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
