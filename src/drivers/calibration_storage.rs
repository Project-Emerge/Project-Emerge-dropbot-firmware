//! Two-sector journal for the robot's hardware antenna delay.

use dropbot_calibration::{
    AntennaDelayRecord, DEVICE_ID_LEN, NOMINAL_DELAY_TICKS, RECORD_LEN, select_newest,
};
use embedded_storage::{ReadStorage, Storage};
use esp_storage::{FlashStorage, FlashStorageError};

/// First two sectors of the `nvs` partition in `partitions.csv`.
const SLOT_OFFSETS: [u32; 2] = [0x9000, 0xA000];

#[derive(Clone, Copy, Debug)]
pub struct LoadedCalibration {
    pub active_slot: Option<usize>,
    pub record: AntennaDelayRecord,
    pub persisted: bool,
}

impl LoadedCalibration {
    pub fn nominal(device_id: [u8; DEVICE_ID_LEN]) -> Self {
        Self {
            active_slot: None,
            record: AntennaDelayRecord::new(device_id, 0, NOMINAL_DELAY_TICKS, NOMINAL_DELAY_TICKS)
                .expect("the nominal delay is always valid"),
            persisted: false,
        }
    }
}

pub fn load(
    flash: &mut FlashStorage<'static>,
    device_id: [u8; DEVICE_ID_LEN],
) -> LoadedCalibration {
    let read_slot = |flash: &mut FlashStorage<'static>, offset: u32| {
        let mut bytes = [0u8; RECORD_LEN];
        flash.read(offset, &mut bytes).ok()?;
        AntennaDelayRecord::decode(&bytes, &device_id, uwb_protocol::PHY_FINGERPRINT).ok()
    };
    let a = read_slot(flash, SLOT_OFFSETS[0]);
    let b = read_slot(flash, SLOT_OFFSETS[1]);
    if let Some((slot, record)) = select_newest(a, b) {
        LoadedCalibration {
            active_slot: Some(slot),
            record,
            persisted: true,
        }
    } else {
        LoadedCalibration::nominal(device_id)
    }
}

pub fn store(
    flash: &mut FlashStorage<'static>,
    current: LoadedCalibration,
    rx_ticks: u16,
    tx_ticks: u16,
) -> Result<LoadedCalibration, FlashStorageError> {
    let target = current.active_slot.map_or(0, |slot| 1 - slot);
    let record = AntennaDelayRecord::new(
        current.record.device_id,
        current.record.generation.saturating_add(1),
        rx_ticks,
        tx_ticks,
    )
    .map_err(|_| FlashStorageError::Other(-1))?;
    let encoded = record.encode();
    flash.write(SLOT_OFFSETS[target], &encoded)?;

    // Read-after-write is the commit point. The old sector is intentionally left intact so a torn
    // write always has a previous valid generation to fall back to.
    let mut verified = [0u8; RECORD_LEN];
    flash.read(SLOT_OFFSETS[target], &mut verified)?;
    AntennaDelayRecord::decode(&verified, &record.device_id, uwb_protocol::PHY_FINGERPRINT)
        .map_err(|_| FlashStorageError::Other(-2))?;
    Ok(LoadedCalibration {
        active_slot: Some(target),
        record,
        persisted: true,
    })
}
