use ariel_os::gpio::Output;
use ariel_os::log::debug;
use ariel_os::time::Timer;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::rng;
use esp_hal::time::Rate;

use crate::data;
use crate::drivers::motor_driver::{DRV8833Driver, types::MotorConfig};
use crate::pins;
use crate::traits::MotorController;
use crate::MOTOR_TELEMETRY;

#[ariel_os::task]
pub async fn manage_motor_controller(pins: pins::MotorDriverPins) -> ! {
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
        motor_driver.set_speed(0.5, 0.5).unwrap();
        MOTOR_TELEMETRY
            .send(data::telemetry::MotorTelemetry {
                left_motor_rpm: 0.5,
                right_motor_rpm: 0.5,
            }).await;
        debug!("motors: left=50% right=50%");
        Timer::after(ariel_os::time::Duration::from_millis(10)).await;
    }
}
