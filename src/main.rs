#![no_main]
#![no_std]

use core::fmt::Write as _;

mod drivers;
mod pins;
mod traits;

use ariel_os::{
    asynch::spawner,
    gpio::Output,
    hal,
    i2c::controller::{Kilohertz, highest_freq_in},
    log::{Debug2Format, debug, error, info},
    net,
    time::Timer,
};
use esp_hal::{
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    time::Rate,
};
use heapless::String;

use crate::{
    drivers::motor_driver::{DRV8833Driver, types::MotorConfig},
    pins::I2cBus,
    traits::{DisplayController, MotorController},
};

const DEVICE_ID: &str = match option_env!("DEVICE_ID") {
    Some(device_id) => device_id,
    None => "UNSET",
};

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) -> ! {
    info!(
        "firmware: started on {} device_id={}",
        ariel_os::buildinfo::BOARD,
        DEVICE_ID
    );
    spawner()
        .spawn(manage_btn(peripherals.motor_driver))
        .unwrap();
    spawner().spawn(manage_display(peripherals.i2c)).unwrap();
    spawner().spawn(manage_mqtt_client()).unwrap();
    loop {
        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}

#[ariel_os::task]
async fn manage_mqtt_client() -> ! {
    let stack = net::network_stack().await.unwrap();

    loop {
        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}

#[ariel_os::task]
async fn manage_btn(pins: pins::MotorDriverPins) -> ! {
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
        debug!("motors: left=50% right=50%");
        Timer::after(ariel_os::time::Duration::from_millis(10)).await;
    }
}

#[ariel_os::task]
async fn manage_display(pins: pins::I2cPins) -> ! {
    let mut i2c_config = hal::i2c::controller::Config::default();
    i2c_config.frequency = const { highest_freq_in(Kilohertz::kHz(100)..=Kilohertz::kHz(400)) };
    let bus = I2cBus::new(pins.sda, pins.scl, i2c_config);
    let mut display = drivers::display_driver::SD1306Driver::new(bus, 0x3C);
    match display.init().await {
        Ok(_) => info!("display: initialized"),
        Err(e) => {
            error!("display: initialization failed: {:?}", Debug2Format(&e));
            loop {
                Timer::after(ariel_os::time::Duration::from_secs(1)).await;
            }
        }
    }
    if let Err(e) = display
        .draw_status(DEVICE_ID, "---.---.---.---", None, false)
        .await
    {
        error!("display: initial draw failed: {:?}", Debug2Format(&e));
    }

    // `NetworkStack` is !Send: acquire and keep it inside this task's executor.
    let stack = net::network_stack().await.unwrap();

    loop {
        stack.wait_config_up().await;

        if let Some(config) = stack.config_v4() {
            let mut ip_address: String<15> = String::new();
            let _ = write!(ip_address, "{}", config.address.address());

            if let Err(e) = display
                .draw_status(DEVICE_ID, ip_address.as_str(), None, true)
                .await
            {
                error!("display: network update failed: {:?}", Debug2Format(&e));
            }
        }

        stack.wait_config_down().await;
        if let Err(e) = display
            .draw_status(DEVICE_ID, "---.---.---.---", None, false)
            .await
        {
            error!("display: network update failed: {:?}", Debug2Format(&e));
        }
    }
}
