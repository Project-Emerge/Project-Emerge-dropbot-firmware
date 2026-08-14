//! The anchor board's pin map.
//!
//! Grouped the same way the dropbot's `src/pins.rs` is, and for the same reason: each group is
//! handed to exactly one task or driver, so nothing has to reason about which peripherals a
//! sibling might also be holding. The whole file is gated on the chip context so that a build for
//! any other target fails here, on the pin map, rather than three layers down inside a HAL type
//! mismatch.
//!
//! Everything below is the pinout of the pre-ariel-os anchor firmware, unchanged -- these are
//! physical traces on an existing board, not a choice. Note that the doc comments sit *inside* each
//! macro invocation: `define_peripherals!` captures them and attaches them to the struct it
//! generates, whereas a doc comment on the invocation itself is one rustdoc discards with a
//! warning.

use ariel_os::hal::peripherals;

#[cfg(context = "stm32l432kc")]
ariel_os::hal::define_peripherals!(
    /// The DWM3000 module: SPI1 with both DMA channels, plus the three control lines.
    ///
    /// `RSTn` is open-drain and must never be driven high, only released -- see
    /// `crate::drivers::uwb`. SPI1 lives on APB2, which is why the 80 MHz PCLK2 the RCC override in
    /// `main.rs` sets up is what makes 20 MHz SPI reachable.
    UwbPins {
        rst: PA1,
        irq: PA2,
        cs: PA4,
        sck: PA5,
        miso: PA6,
        mosi: PA7,
        spi: SPI1,
        tx_dma: DMA1_CH3,
        rx_dma: DMA1_CH2,
    }
);

#[cfg(context = "stm32l432kc")]
ariel_os::hal::define_peripherals!(
    /// Battery sense: a 10k/20k divider from the cell into PA3, read through ADC1.
    BatteryPins {
        sense: PA3,
        adc: ADC1,
    }
);

#[cfg(context = "stm32l432kc")]
ariel_os::hal::define_peripherals!(
    /// The bicolour indicator LED. PA8 is red and PA9 is green -- the pre-ariel-os firmware had
    /// these swapped at one point, so the mapping is called out rather than left to the field names.
    LedPins {
        red: PA8,
        green: PA9,
    }
);

#[cfg(context = "stm32l432kc")]
ariel_os::hal::define_peripherals!(
    /// BQ24074 charger configuration straps.
    ChargerPins {
        enable: PA10,
        en1: PB0,
        en2: PB1,
    }
);

#[cfg(context = "stm32l432kc")]
ariel_os::hal::define_peripherals!(
    /// STM6600 power-button controller: `PS_HOLD` keeps the rail up, `PWR_INT` reports the press.
    PowerPins {
        ps_hold: PB5,
        power_int: PB7,
    }
);

ariel_os::hal::group_peripherals!(Peripherals {
    uwb: UwbPins,
    battery: BatteryPins,
    led: LedPins,
    charger: ChargerPins,
    power: PowerPins,
});
