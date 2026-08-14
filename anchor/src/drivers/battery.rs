//! Single-cell Li-Ion fuel gauge: ADC1 on PA3 behind a 10k/20k divider.

use ariel_os::hal::peripheral::Peri;
use ariel_os::hal::peripherals::{ADC1, PA3};
use embassy_stm32::adc::{Adc, SampleTime};

/// Length of the moving-average window over raw ADC samples.
///
/// The divider is fed straight off the cell with no filtering capacitor, and the radio's transmit
/// bursts are visible on it, so a single sample swings by more than the 11.5 mV that one percent
/// of charge is worth. Eight samples at the one-second poll interval smooth that out without
/// making the reading lag a real discharge.
const FILTER_LEN: usize = 8;

/// Millivolts at which the cell is treated as full, and as empty.
///
/// A linear interpolation between the two, which is crude for Li-Ion -- the real curve is flat
/// across the middle of the range -- but this only drives a two-colour LED, so the only threshold
/// that has to be roughly right is the low one.
const FULL_MV: u16 = 4150;
const EMPTY_MV: u16 = 3000;

pub struct BatteryMonitor {
    adc: Adc<'static, ADC1>,
    channel: Peri<'static, PA3>,
    window: [u16; FILTER_LEN],
    next: usize,
    filled: usize,
}

impl BatteryMonitor {
    pub fn new(adc: Peri<'static, ADC1>, channel: Peri<'static, PA3>) -> Self {
        Self {
            adc: Adc::new(adc),
            channel,
            window: [0; FILTER_LEN],
            next: 0,
            filled: 0,
        }
    }

    /// Cell voltage in millivolts, averaged over the last [`FILTER_LEN`] samples.
    pub fn read_voltage_mv(&mut self) -> u16 {
        let raw = self.read_filtered();
        // 12-bit conversion against the 3.3 V rail...
        let pin_mv = u32::from(raw) * 3300 / 4095;
        // ...then undo the divider: Vbat -- 10k -- PA3 -- 20k -- GND, so Vpin = Vbat * 2/3.
        (pin_mv * 3 / 2) as u16
    }

    /// State of charge as a percentage, linear between [`EMPTY_MV`] and [`FULL_MV`].
    pub fn read_percentage(&mut self) -> u8 {
        let mv = self.read_voltage_mv();
        if mv >= FULL_MV {
            100
        } else if mv <= EMPTY_MV {
            0
        } else {
            (u32::from(mv - EMPTY_MV) * 100 / u32::from(FULL_MV - EMPTY_MV)) as u8
        }
    }

    fn read_filtered(&mut self) -> u16 {
        // Long sample time: the divider's source impedance is 10k/20k in parallel, i.e. ~6.7k,
        // which needs far more than the default few cycles to charge the sample-and-hold.
        self.adc.set_sample_time(SampleTime::CYCLES92_5);
        let raw = self.adc.blocking_read(&mut self.channel);

        self.window[self.next] = raw;
        self.next = (self.next + 1) % FILTER_LEN;
        self.filled = (self.filled + 1).min(FILTER_LEN);

        let sum: u32 = self.window[..self.filled]
            .iter()
            .copied()
            .map(u32::from)
            .sum();
        (sum / self.filled as u32) as u16
    }
}
