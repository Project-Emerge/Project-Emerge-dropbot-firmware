use ariel_os::hal::{i2c, peripherals};

#[cfg(context = "esp32c6")]
ariel_os::hal::define_peripherals!(MotorDriverPins {
    pwm_device: MCPWM0,
    bin1: GPIO0,
    bin2: GPIO1,
    ain1: GPIO10,
    ain2: GPIO8,
    sleep: GPIO11,
    btn: GPIO9,
});

#[cfg(context = "esp32c6")]
ariel_os::hal::define_peripherals!(SerialPins {
    tx: GPIO16,
    rx: GPIO17,
});

#[cfg(context = "esp32c6")]
ariel_os::hal::define_peripherals!(PowerManagementPins {
    int: GPIO21,
    kill: GPIO20,
});

#[cfg(context = "esp32c6")]
ariel_os::hal::define_peripherals!(ExpansionPins {
    io18: GPIO18,
    io19: GPIO19,
});

#[cfg(context = "esp32c6")]
pub type I2cBus = i2c::controller::I2C0;
#[cfg(context = "esp32c6")]
ariel_os::hal::define_peripherals!(I2cPins {
    sda: GPIO23,
    scl: GPIO22,
});

#[cfg(context = "esp32c6")]
ariel_os::hal::define_peripherals!(OtaPeripherals { flash: FLASH });

ariel_os::hal::group_peripherals!(Peripherals {
    motor_driver: MotorDriverPins,
    serial: SerialPins,
    power_management: PowerManagementPins,
    expansion: ExpansionPins,
    i2c: I2cPins,
    ota: OtaPeripherals,
});
