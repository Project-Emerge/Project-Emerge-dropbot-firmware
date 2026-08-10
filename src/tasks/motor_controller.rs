use ariel_os::gpio::Output;
use ariel_os::log::{Debug2Format, debug, error, info, warn};
use ariel_os::time::Timer;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::Receiver as WatchReceiver;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::time::Rate;

use crate::data;
use crate::data::ota::OtaStatus;
use crate::drivers::motor_driver::{DRV8833Driver, types::MotorConfig};
use crate::pins;
use crate::traits::MotorController;

#[ariel_os::task]
pub async fn manage_motor_controller(
    pins: pins::MotorDriverPins,
    motor_telemetry: Sender<'static, CriticalSectionRawMutex, data::telemetry::MotorTelemetry, 2>,
    motor_command: Receiver<'static, CriticalSectionRawMutex, data::commands::DriveCommand, 2>,
    mut ota_status: WatchReceiver<'static, CriticalSectionRawMutex, OtaStatus, 2>,
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

        match select(
            motor_command.receive(),
            Timer::after(ariel_os::time::Duration::from_secs(3)),
        )
        .await
        {
            Either::First(command) => {
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
            Either::Second(()) => {
                warn!("motors: no command received for 3s, stopping");
                if let Err(e) = motor_driver.stop() {
                    error!("motors: stop failed: {:?}", Debug2Format(&e));
                }
            }
        };
        Timer::after(ariel_os::time::Duration::from_millis(10)).await;
    }
}
