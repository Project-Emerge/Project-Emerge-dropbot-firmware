use core::fmt::Write as _;

use ariel_os::config;
use ariel_os::log::{Debug2Format, debug, error, info};
use ariel_os::net;
use ariel_os::reexports::embassy_net::{
    self,
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
use ariel_os::time::{Duration, Timer};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender};
use embedded_io_async::Read as _;
use embedded_storage::Storage as _;
use esp_bootloader_esp_idf::ota::OtaImageState;
use esp_bootloader_esp_idf::ota_updater::OtaUpdater;
use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use esp_storage::FlashStorage;
use reqwless::client::HttpClient;
use reqwless::request::Method;

use crate::data::calibration::{CalibrationWriteRequest, CalibrationWriteResult};
use crate::data::ota::{FirmwareManifest, OtaStatus};
use crate::drivers::calibration_storage::{self, LoadedCalibration};

/// Broadcast channel carrying [`OtaStatus`] to the display and the motor controller.
type OtaStatusSender = WatchSender<'static, CriticalSectionRawMutex, OtaStatus, 2>;

/// Hostname or IP address of the OTA update server, e.g. `192.168.8.1` or `ota.local`.
const OTA_SERVER_HOST: &str = config::str_from_env_or!(
    "OTA_SERVER_HOST",
    "192.168.8.1",
    "hostname or IP address of the OTA update server",
);

const OTA_POLL_INTERVAL: Duration = Duration::from_secs(30 * 60);

const TCP_BUFFER_SIZE: usize = 1024;
const MANIFEST_BUFFER_SIZE: usize = 768;
const DOWNLOAD_HEADER_BUFFER_SIZE: usize = 512;
const DOWNLOAD_CHUNK_SIZE: usize = TCP_BUFFER_SIZE;
const URL_BUFFER_SIZE: usize = 128;
const MAX_CONCURRENT_CONNECTIONS: usize = 1;

// Field contents are only ever surfaced through the derived `Debug` impl (via `Debug2Format`
// in the `error!` logs below), which the dead-code lint doesn't credit as a read.
#[derive(Debug)]
#[allow(dead_code)]
enum OtaError {
    Http(reqwless::Error),
    Json(serde_json::Error),
    Partition(esp_bootloader_esp_idf::partitions::Error),
    UrlTooLong,
    ImageTooLarge,
    SizeMismatch { expected: u32, written: u32 },
}

impl From<reqwless::Error> for OtaError {
    fn from(err: reqwless::Error) -> Self {
        OtaError::Http(err)
    }
}

impl From<serde_json::Error> for OtaError {
    fn from(err: serde_json::Error) -> Self {
        OtaError::Json(err)
    }
}

impl From<esp_bootloader_esp_idf::partitions::Error> for OtaError {
    fn from(err: esp_bootloader_esp_idf::partitions::Error) -> Self {
        OtaError::Partition(err)
    }
}

/// Publishes `status` unless the watch already holds it, so tasks observing it are not
/// woken by the periodic checks that find nothing to install.
fn publish_status(ota_status: &OtaStatusSender, status: OtaStatus) {
    if ota_status.try_get() != Some(status) {
        ota_status.send(status);
    }
}

/// Periodically checks the OTA server for a new firmware version, and installs it if found.
///
/// A check can also be triggered on demand (e.g. from an MQTT command) by signaling
/// `ota_check_request`.
#[ariel_os::task]
pub async fn manage_ota(
    mut network_ready: WatchReceiver<'static, CriticalSectionRawMutex, (), 2>,
    ota_check_request: &'static Signal<CriticalSectionRawMutex, ()>,
    mut flash: FlashStorage<'static>,
    mut antenna_calibration: LoadedCalibration,
    calibration_write_rx: Receiver<'static, CriticalSectionRawMutex, CalibrationWriteRequest, 1>,
    calibration_write_result_tx: Sender<
        'static,
        CriticalSectionRawMutex,
        CalibrationWriteResult,
        1,
    >,
    ota_status: OtaStatusSender,
) -> ! {
    // If we just rebooted into a freshly installed slot, confirm it as working so the
    // bootloader (when built with auto-rollback support) doesn't revert it later.
    {
        let mut partition_table_buf = [0u8; PARTITION_TABLE_MAX_LEN];
        match OtaUpdater::new(&mut flash, &mut partition_table_buf) {
            Ok(mut ota) => {
                if let Ok(state) = ota.current_ota_state()
                    && matches!(state, OtaImageState::New | OtaImageState::PendingVerify)
                    && ota.set_current_ota_state(OtaImageState::Valid).is_ok()
                {
                    info!("ota: confirmed current firmware slot as valid");
                }
            }
            Err(_) => {
                error!(
                    "ota: no OTA-capable partition table found; updates are unavailable until \
                     the device is reflashed with one (see partitions.csv)"
                );
            }
        }
    }

    network_ready.get().await;
    let stack = net::network_stack().await.unwrap();

    loop {
        let ota_trigger = select(Timer::after(OTA_POLL_INTERVAL), ota_check_request.wait());
        match select(ota_trigger, calibration_write_rx.receive()).await {
            Either::First(Either::First(())) => debug!("ota: periodic check"),
            Either::First(Either::Second(())) => info!("ota: manual check requested"),
            Either::Second(request) => {
                let result = match calibration_storage::store(
                    &mut flash,
                    antenna_calibration,
                    request.rx_ticks,
                    request.tx_ticks,
                ) {
                    Ok(saved) => {
                        antenna_calibration = saved;
                        info!(
                            "calibration: persisted robot antenna delay generation {}: RX {}, TX {}",
                            saved.record.generation, saved.record.rx_ticks, saved.record.tx_ticks,
                        );
                        CalibrationWriteResult::Saved {
                            session_id: request.session_id,
                            generation: saved.record.generation,
                            rx_ticks: saved.record.rx_ticks,
                            tx_ticks: saved.record.tx_ticks,
                        }
                    }
                    Err(err) => {
                        error!(
                            "calibration: persisting robot antenna delay failed: {:?}",
                            Debug2Format(&err)
                        );
                        CalibrationWriteResult::Failed {
                            session_id: request.session_id,
                        }
                    }
                };
                calibration_write_result_tx.send(result).await;
                continue;
            }
        }

        match check_and_apply_update(stack, &mut flash, &ota_status).await {
            Ok(true) => {
                info!("ota: update installed, rebooting");
                publish_status(&ota_status, OtaStatus::Applying);
                Timer::after(Duration::from_millis(200)).await;
                esp_hal::system::software_reset();
            }
            Ok(false) => debug!("ota: firmware is up to date"),
            Err(err) => error!("ota: update check failed: {:?}", Debug2Format(&err)),
        }

        // Reached only when no reboot happened, i.e. the firmware was up to date or the
        // update failed: release the tasks that paused for it.
        publish_status(&ota_status, OtaStatus::Idle);
    }
}

/// Returns `Ok(true)` if a new firmware image was downloaded and activated (the caller should
/// reboot), `Ok(false)` if the firmware is already up to date.
async fn check_and_apply_update(
    stack: embassy_net::Stack<'static>,
    flash: &mut FlashStorage<'static>,
    ota_status: &OtaStatusSender,
) -> Result<bool, OtaError> {
    let tcp_client_state =
        TcpClientState::<MAX_CONCURRENT_CONNECTIONS, TCP_BUFFER_SIZE, TCP_BUFFER_SIZE>::new();
    let tcp_client = TcpClient::new(stack, &tcp_client_state);
    let dns_client = DnsSocket::new(stack);
    let mut http_client = HttpClient::new(&tcp_client, &dns_client);

    let mut manifest_url: heapless::String<URL_BUFFER_SIZE> = heapless::String::new();
    write!(manifest_url, "http://{OTA_SERVER_HOST}/api/firmware/latest")
        .map_err(|_| OtaError::UrlTooLong)?;

    // Extract only what we need as owned values, so the manifest response buffer (and the
    // connection borrowed by `http_client`) can be released before we open a second connection
    // to download the firmware image.
    let (firmware_path, expected_size) = {
        let mut manifest_buf = [0u8; MANIFEST_BUFFER_SIZE];
        let mut request = http_client
            .request(Method::GET, manifest_url.as_str())
            .await?;
        let response = request.send(&mut manifest_buf).await?;

        if !response.status.is_successful() {
            info!(
                "ota: manifest endpoint returned status {}",
                response.status.0
            );
            return Ok(false);
        }

        let body = response.body().read_to_end().await?;
        let manifest: FirmwareManifest = serde_json::from_slice(body)?;

        if manifest.version == crate::FIRMWARE_VERSION {
            return Ok(false);
        }

        info!(
            "ota: new firmware available: {} -> {}",
            crate::FIRMWARE_VERSION,
            manifest.version
        );

        // From here on the device is committed to the update: announce it so the motors
        // are cut and the display switches to the update screen.
        publish_status(ota_status, OtaStatus::Preparing);

        let mut path: heapless::String<URL_BUFFER_SIZE> = heapless::String::new();
        path.push_str(manifest.url)
            .map_err(|_| OtaError::UrlTooLong)?;
        (path, manifest.size)
    };

    let mut partition_table_buf = [0u8; PARTITION_TABLE_MAX_LEN];
    let mut ota = OtaUpdater::new(flash, &mut partition_table_buf)?;
    let (mut next_partition, part_type) = ota.next_partition()?;

    if expected_size as usize > next_partition.partition_size() {
        return Err(OtaError::ImageTooLarge);
    }

    let mut firmware_url: heapless::String<URL_BUFFER_SIZE> = heapless::String::new();
    write!(firmware_url, "http://{OTA_SERVER_HOST}{firmware_path}")
        .map_err(|_| OtaError::UrlTooLong)?;

    info!(
        "ota: downloading firmware into {:?} slot ({} bytes)",
        Debug2Format(&part_type),
        expected_size
    );

    let mut download_header_buf = [0u8; DOWNLOAD_HEADER_BUFFER_SIZE];
    let mut download_request = http_client
        .request(Method::GET, firmware_url.as_str())
        .await?;
    let download_response = download_request.send(&mut download_header_buf).await?;

    if !download_response.status.is_successful() {
        info!(
            "ota: firmware download returned status {}",
            download_response.status.0
        );
        return Ok(false);
    }

    let mut body_reader = download_response.body().reader();
    let mut chunk = [0u8; DOWNLOAD_CHUNK_SIZE];
    let mut written: u32 = 0;
    let mut published_percent = None;
    loop {
        let n = body_reader.read(&mut chunk).await?;
        if n == 0 {
            break;
        }
        next_partition.write(written, &chunk[..n])?;
        written += n as u32;

        // Redrawing the display means flushing its whole framebuffer over I2C, which is
        // far slower than a chunk write: only republish when the whole-percent figure
        // actually moves. A zero-sized manifest yields `None` throughout, leaving the
        // display on its indeterminate screen.
        let status = OtaStatus::Downloading {
            written,
            total: expected_size,
        };
        if status.percent() != published_percent {
            published_percent = status.percent();
            ota_status.send(status);
        }
    }

    if written != expected_size {
        error!(
            "ota: downloaded {} bytes, expected {}",
            written, expected_size
        );
        return Err(OtaError::SizeMismatch {
            expected: expected_size,
            written,
        });
    }

    ota.activate_next_partition()?;
    ota.set_current_ota_state(OtaImageState::New)?;

    Ok(true)
}
