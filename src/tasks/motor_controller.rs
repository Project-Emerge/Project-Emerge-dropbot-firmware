use ariel_os::gpio::Output;
use ariel_os::log::{Debug2Format, debug, error, info};
use ariel_os::time::Timer;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::watch::Receiver as WatchReceiver;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::rng;
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
    let mut motor_driver =
        DRV8833Driver::new(ain1, ain2, bin1, bin2, sleep_pin, MotorConfig::default());

    loop {
        // An OTA update is about to overwrite this firmware and reboot into it. Cut the
        // motors and hold them de-energized for its whole duration, so the robot cannot
        // drive off unattended across the reboot. `set_speed` drives SLEEP high again, so
        // a failed update needs no explicit wake-up here.
        if let Some(status) = ota_status.try_changed() {
            if status.is_active() {
                if let Err(e) = motor_driver.stop().and_then(|()| motor_driver.sleep()) {
                    error!("motors: shutdown for OTA failed: {:?}", Debug2Format(&e));
                }
                info!("motors: disabled for OTA update");

                while ota_status.changed().await.is_active() {}
                info!("motors: OTA update ended, resuming");
            }
        }

        motor_driver.set_speed(0.5, 0.5).unwrap();
        motor_telemetry
            .send(data::telemetry::MotorTelemetry {
                left_motor_rpm: 0.5,
                right_motor_rpm: 0.5,
            })
            .await;
        debug!("motors: left=50% right=50%");
        Timer::after(ariel_os::time::Duration::from_millis(10)).await;
    }
}
