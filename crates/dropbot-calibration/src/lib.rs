#![no_std]

//! Persistent wire/storage types shared by the embedded calibration manager and host tests.

use core::cmp::Ordering;

pub const DEVICE_ID_LEN: usize = 6;
pub const RECORD_LEN: usize = 32;
pub const NOMINAL_DELAY_TICKS: u16 = 16_385;
pub const MAX_DELAY_DELTA_TICKS: u16 = 1_024;

const MAGIC: [u8; 4] = *b"UWBC";
const VERSION: u8 = 1;
const CRC_OFFSET: usize = RECORD_LEN - 4;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AntennaDelayRecord {
    pub device_id: [u8; DEVICE_ID_LEN],
    pub phy_fingerprint: u32,
    pub generation: u32,
    pub rx_ticks: u16,
    pub tx_ticks: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecordError {
    Magic,
    Version,
    Crc,
    Device,
    Protocol,
    DelayOutOfRange,
}

impl AntennaDelayRecord {
    pub fn new(
        device_id: [u8; DEVICE_ID_LEN],
        generation: u32,
        rx_ticks: u16,
        tx_ticks: u16,
    ) -> Result<Self, RecordError> {
        validate_delay(rx_ticks)?;
        validate_delay(tx_ticks)?;
        Ok(Self {
            device_id,
            phy_fingerprint: uwb_protocol::PHY_FINGERPRINT,
            generation,
            rx_ticks,
            tx_ticks,
        })
    }

    #[must_use]
    pub fn encode(self) -> [u8; RECORD_LEN] {
        let mut bytes = [0xFF; RECORD_LEN];
        bytes[..4].copy_from_slice(&MAGIC);
        bytes[4] = VERSION;
        bytes[6..12].copy_from_slice(&self.device_id);
        bytes[12..16].copy_from_slice(&self.phy_fingerprint.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.generation.to_le_bytes());
        bytes[20..22].copy_from_slice(&self.rx_ticks.to_le_bytes());
        bytes[22..24].copy_from_slice(&self.tx_ticks.to_le_bytes());
        let crc = crc32(&bytes[..CRC_OFFSET]);
        bytes[CRC_OFFSET..].copy_from_slice(&crc.to_le_bytes());
        bytes
    }

    pub fn decode(
        bytes: &[u8; RECORD_LEN],
        expected_device_id: &[u8; DEVICE_ID_LEN],
        expected_phy_fingerprint: u32,
    ) -> Result<Self, RecordError> {
        if bytes[..4] != MAGIC {
            return Err(RecordError::Magic);
        }
        if bytes[4] != VERSION {
            return Err(RecordError::Version);
        }
        let stored_crc = u32::from_le_bytes(bytes[CRC_OFFSET..].try_into().unwrap());
        if stored_crc != crc32(&bytes[..CRC_OFFSET]) {
            return Err(RecordError::Crc);
        }
        let device_id: [u8; DEVICE_ID_LEN] = bytes[6..12].try_into().unwrap();
        if &device_id != expected_device_id {
            return Err(RecordError::Device);
        }
        let phy_fingerprint = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
        if phy_fingerprint != expected_phy_fingerprint {
            return Err(RecordError::Protocol);
        }
        let rx_ticks = u16::from_le_bytes(bytes[20..22].try_into().unwrap());
        let tx_ticks = u16::from_le_bytes(bytes[22..24].try_into().unwrap());
        validate_delay(rx_ticks)?;
        validate_delay(tx_ticks)?;
        Ok(Self {
            device_id,
            phy_fingerprint,
            generation: u32::from_le_bytes(bytes[16..20].try_into().unwrap()),
            rx_ticks,
            tx_ticks,
        })
    }
}

/// Selects the newest valid journal slot. Equal generations prefer slot B so an interrupted
/// rewrite of A cannot replace a successfully verified B record.
#[must_use]
pub fn select_newest(
    a: Option<AntennaDelayRecord>,
    b: Option<AntennaDelayRecord>,
) -> Option<(usize, AntennaDelayRecord)> {
    match (a, b) {
        (Some(a), Some(b)) => match a.generation.cmp(&b.generation) {
            Ordering::Greater => Some((0, a)),
            Ordering::Less | Ordering::Equal => Some((1, b)),
        },
        (Some(a), None) => Some((0, a)),
        (None, Some(b)) => Some((1, b)),
        (None, None) => None,
    }
}

fn validate_delay(value: u16) -> Result<(), RecordError> {
    if value.abs_diff(NOMINAL_DELAY_TICKS) <= MAX_DELAY_DELTA_TICKS {
        Ok(())
    } else {
        Err(RecordError::DelayOutOfRange)
    }
}

/// CRC-32/ISO-HDLC, small table-free form suitable for a record written only during provisioning.
#[must_use]
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEVICE: [u8; DEVICE_ID_LEN] = *b"A1B2C3";

    fn record(generation: u32) -> AntennaDelayRecord {
        AntennaDelayRecord::new(DEVICE, generation, 16_400, 16_400).unwrap()
    }

    #[test]
    fn record_round_trip_and_crc_failure() {
        let expected = record(7);
        let mut bytes = expected.encode();
        assert_eq!(
            AntennaDelayRecord::decode(&bytes, &DEVICE, uwb_protocol::PHY_FINGERPRINT),
            Ok(expected)
        );
        bytes[20] ^= 1;
        assert_eq!(
            AntennaDelayRecord::decode(&bytes, &DEVICE, uwb_protocol::PHY_FINGERPRINT),
            Err(RecordError::Crc)
        );
    }

    #[test]
    fn rejects_another_robot_or_phy() {
        let bytes = record(1).encode();
        assert_eq!(
            AntennaDelayRecord::decode(&bytes, b"FFFFFF", uwb_protocol::PHY_FINGERPRINT),
            Err(RecordError::Device)
        );
        assert_eq!(
            AntennaDelayRecord::decode(&bytes, &DEVICE, uwb_protocol::PHY_FINGERPRINT ^ 1),
            Err(RecordError::Protocol)
        );
    }

    #[test]
    fn journal_rolls_back_to_the_last_valid_slot() {
        let older = record(4);
        let newer = record(5);
        assert_eq!(select_newest(Some(older), Some(newer)), Some((1, newer)));
        assert_eq!(select_newest(Some(older), None), Some((0, older)));
    }

    #[test]
    fn every_interrupted_write_rolls_back_to_the_previous_sector() {
        let older = record(8);
        let newer_bytes = record(9).encode();
        for bytes_written in 0..RECORD_LEN {
            // A power cut can leave any prefix of the target sector programmed. The inactive
            // sector is independent and must remain the selected commit.
            let mut torn = [0xFF; RECORD_LEN];
            torn[..bytes_written].copy_from_slice(&newer_bytes[..bytes_written]);
            let decoded =
                AntennaDelayRecord::decode(&torn, &DEVICE, uwb_protocol::PHY_FINGERPRINT).ok();
            assert_eq!(select_newest(Some(older), decoded), Some((0, older)));
        }
        let complete =
            AntennaDelayRecord::decode(&newer_bytes, &DEVICE, uwb_protocol::PHY_FINGERPRINT)
                .unwrap();
        assert_eq!(
            select_newest(Some(older), Some(complete)),
            Some((1, complete))
        );
    }

    #[test]
    fn implausible_delay_is_rejected_before_flash() {
        assert_eq!(
            AntennaDelayRecord::new(DEVICE, 1, 10_000, NOMINAL_DELAY_TICKS),
            Err(RecordError::DelayOutOfRange)
        );
    }
}
