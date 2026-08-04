use serde::Deserialize;

/// Manifest served by the OTA server at `GET /api/firmware/latest`.
///
/// `url` is a path relative to the OTA server's host, e.g. `/firmware/dropbot-1.2.3.bin`.
#[derive(Deserialize)]
pub struct FirmwareManifest<'a> {
    pub version: &'a str,
    pub url: &'a str,
    pub size: u32,
}

/// Progress of an in-flight OTA update.
///
/// Broadcast by `manage_ota` to the tasks that have to react to one: the motor controller
/// cuts power to the motors, and the display swaps the status mask for an update screen.
/// Only published once a new version is known to exist, so the periodic checks that find
/// nothing never disturb either.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OtaStatus {
    /// No update in flight; the device runs normally.
    Idle,
    /// A new version was found and the image transfer is being set up. Its size is not
    /// known to the download loop yet, so there is nothing to derive a percentage from.
    Preparing,
    /// The image is being written into the inactive slot.
    Downloading { written: u32, total: u32 },
    /// The image is complete and activated; the device is about to reboot into it.
    Applying,
}

impl OtaStatus {
    /// Whether an update is in flight, i.e. the device must not be doing anything else.
    pub fn is_active(self) -> bool {
        !matches!(self, OtaStatus::Idle)
    }

    /// Download completion in whole percent, or `None` while no image transfer with a
    /// known size is running -- callers should fall back to an indeterminate indicator.
    pub fn percent(self) -> Option<u8> {
        match self {
            OtaStatus::Downloading { written, total } if total > 0 => {
                Some((u64::from(written) * 100 / u64::from(total)).min(100) as u8)
            }
            OtaStatus::Applying => Some(100),
            _ => None,
        }
    }
}
