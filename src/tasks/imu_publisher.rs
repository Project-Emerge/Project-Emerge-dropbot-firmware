use ariel_os::log::{debug, error};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::Receiver as WatchReceiver;

use crate::data::mqtt::{BrokerStatus, PublishMessage};
use crate::data::telemetry::IMUTelemetry;

/// How often to complain about a stream that is not going out. At the rate this task runs,
/// logging every failure would itself become the bottleneck.
const ERROR_LOG_INTERVAL: u32 = 250;

/// Publishes the high-rate IMU stream on its own topic.
///
/// Separate from `publish_telemetry` because the two carry different contracts. That one
/// sends a once-a-second status summary; this one sends the dead-reckoning input a UWB
/// localization filter integrates between range fixes, at fifty times the rate. Sharing a
/// topic would either starve the fusion consumer or force every subscriber that only wants
/// a battery percentage to parse fifty messages a second.
#[ariel_os::task]
pub async fn publish_imu_stream(
    topic: &'static str,
    mut broker_status: WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 3>,
    imu_stream_rx: Receiver<'static, CriticalSectionRawMutex, IMUTelemetry, 2>,
    mqtt_publish_tx: Sender<'static, CriticalSectionRawMutex, PublishMessage, 2>,
) -> ! {
    while broker_status.get().await != BrokerStatus::Connected {
        broker_status.changed().await;
    }

    let mut dropped: u32 = 0;

    loop {
        let sample = imu_stream_rx.receive().await;

        let Ok(payload) = serde_json::to_vec(&sample) else {
            error!("imu-stream: serialization failed");
            continue;
        };

        let mut message = PublishMessage {
            topic,
            payload: heapless::Vec::new(),
        };
        if message.payload.extend_from_slice(&payload).is_err() {
            error!("imu-stream: payload too large for buffer");
            continue;
        }

        // Never blocks: a stream sample is only worth sending while it is current, so when
        // the broker queue is full the sample is dropped rather than delaying every one
        // behind it. A fusion filter copes with a gap far better than with a late sample
        // carrying a stale timestamp.
        if mqtt_publish_tx.try_send(message).is_err() {
            dropped += 1;
            if dropped.is_multiple_of(ERROR_LOG_INTERVAL) {
                debug!(
                    "imu-stream: {} samples dropped, publish queue full",
                    dropped
                );
            }
        }
    }
}
