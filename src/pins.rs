use ariel_os::hal::peripherals;

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
ariel_os::hal::define_peripherals!(UwbPins {
    rst: GPIO15,
    cs: GPIO7,
    sck: GPIO4,
    mosi: GPIO6,
    miso: GPIO5,
    irq: GPIO2,
    wup: GPIO3,
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
ariel_os::hal::define_peripherals!(I2cPins {
    sda: GPIO23,
    scl: GPIO22,
});

ariel_os::hal::group_peripherals!(Peripherals {
    motor_driver: MotorDriverPins,
    uwb: UwbPins,
    serial: SerialPins,
    power_management: PowerManagementPins,
    expansion: ExpansionPins,
    i2c: I2cPins,
});
