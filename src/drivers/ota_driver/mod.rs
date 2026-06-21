use defmt::{debug, info};
use esp_bootloader_esp_idf::partitions;
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;

use crate::traits::OtaUpdater;

pub struct OtaDriver {
    flash: FlashStorage<'static>,
}

impl OtaDriver {
    pub fn new(flash: FLASH<'static>) -> Self {
        Self {
            flash: FlashStorage::new(flash),
        }
    }
}

impl OtaUpdater for OtaDriver {
    type Error = partitions::Error;

    fn perform_update(&mut self) -> Result<(), Self::Error> {
        let mut buffer = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let partition_table = partitions::read_partition_table(&mut self.flash, &mut buffer)?;
        for partition in partition_table.iter() {
            debug!(
                "Partition: label={}, type={}, subtype={}, offset={}, len={}",
                partition.label_as_str(),
                partition.raw_type(),
                partition.raw_subtype(),
                partition.offset(),
                partition.len()
            );
        }
        debug!("Currenly booted partition: {:?}", &&partition_table.booted_partition()?.map(|p| p.label_as_str()));
        let mut ota = esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(&mut self.flash, &mut buffer)?;
        let current = ota.selected_partition()?;
        info!(
            "current image state {:?} (only relevant if the bootloader was built with auto-rollback support)",
            ota.current_ota_state()
        );
        Ok(())
    }
}
