//! Compile-time binding between an anchor identity, one physical STM32 and its DWM3000 delay.

use dw3000_hal::AntennaDelay;

#[derive(Clone, Copy)]
pub struct AnchorHardwareCalibration {
    pub anchor_id: u16,
    pub expected_uid: Option<[u8; 12]>,
    pub rx_ticks: u16,
    pub tx_ticks: u16,
}

include!(concat!(env!("OUT_DIR"), "/anchor_calibration.rs"));

impl AnchorHardwareCalibration {
    pub const fn delay(self) -> AntennaDelay {
        AntennaDelay {
            rx: self.rx_ticks,
            tx: self.tx_ticks,
        }
    }

    pub fn accepts(self, anchor_id: u16, physical_uid: &[u8; 12]) -> bool {
        self.anchor_id == anchor_id && self.expected_uid.as_ref() == Some(physical_uid)
    }
}

pub const fn for_index(index: usize) -> AnchorHardwareCalibration {
    ANCHOR_CALIBRATIONS[index]
}
