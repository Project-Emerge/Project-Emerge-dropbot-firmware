use defmt::{debug, info};
use embedded_storage::Storage;
use esp_bootloader_esp_idf::partitions;
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;

use crate::traits::OtaUpdater;

pub struct OtaDriver {
    flash: FlashStorage<'static>,
    chunk_size: usize,
}

impl OtaDriver {
    pub fn new(flash: FLASH<'static>) -> Self {
        Self {
            flash: FlashStorage::new(flash),
            chunk_size: 4096,
        }
    }
}

impl OtaUpdater for OtaDriver {
    type Error = partitions::Error;

    fn perform_update(&mut self, new_image: &[u8]) -> Result<(), Self::Error> {
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
        debug!(
            "Currently booted partition: {:?}",
            &partition_table
                .booted_partition()?
                .map(|p| p.label_as_str())
        );
        let mut ota =
            esp_bootloader_esp_idf::ota_updater::OtaUpdater::new(&mut self.flash, &mut buffer)?;
        let current = ota.selected_partition()?;
        info!(
            "selected OTA partition {:?}; current image state {:?} (only relevant if the bootloader was built with auto-rollback support)",
            current,
            ota.current_ota_state()
        );
        if let Ok(state) = ota.current_ota_state() {
            if state == esp_bootloader_esp_idf::ota::OtaImageState::New
                || state == esp_bootloader_esp_idf::ota::OtaImageState::PendingVerify
            {
                debug!("Changed state to VALID");
                ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::Valid)?;
            }
        }
        let (mut next_app_partition, part_type) = ota.next_partition()?;
        debug!("Flashing image to {:?}", part_type);
        // write to the app partition
        for (sector, chunk) in new_image.chunks(self.chunk_size).enumerate() {
            debug!("Writing sector {}...", sector);
            next_app_partition.write((sector * self.chunk_size) as u32, chunk)?;
        }
        ota.activate_next_partition()?;
        return ota.set_current_ota_state(esp_bootloader_esp_idf::ota::OtaImageState::New);
    }
}
