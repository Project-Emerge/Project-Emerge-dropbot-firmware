use core::fmt::Write as _;

use ariel_os::reexports::static_cell::StaticCell;
use esp_hal::efuse::Efuse;
use heapless::String;

/// Length of the formatted device ID: 3 eFuse bytes, 2 hex characters each.
pub const DEVICE_ID_LEN: usize = 6;

static STORAGE: StaticCell<String<DEVICE_ID_LEN>> = StaticCell::new();

/// Derives the device ID from the ESP32-C6's factory-programmed eFuse base MAC address,
/// formatted as 6 uppercase hex characters. eFuse is burned once at the factory and lives
/// outside flash, so it survives every reflash and OTA update and never needs to be supplied
/// externally.
///
/// Only the 3 low-order bytes are used: the 3 high-order bytes are Espressif's OUI, shared
/// by every device in the fleet, while the low-order bytes are the chip-specific NIC bytes
/// assigned in the factory (the same split ESP-IDF uses for its default `ESP_XXXXXX`
/// hostnames). Dropping the OUI shortens the ID without giving up any real uniqueness.
///
/// Must be called exactly once, at the top of `main` before any task that needs the result
/// is spawned.
pub fn init() -> &'static str {
    let mac = Efuse::read_base_mac_address();
    let mut id = String::new();
    for byte in &mac[3..] {
        let _ = write!(id, "{byte:02X}");
    }
    STORAGE.init(id)
}
