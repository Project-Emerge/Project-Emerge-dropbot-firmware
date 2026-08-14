use ariel_os::log::debug;
use ariel_os::log::error;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::Receiver as WatchReceiver;

use crate::data::mqtt::{BrokerStatus, PublishMessage};
use crate::data::telemetry::Telemetry;

#[ariel_os::task]
pub async fn publish_telemetry(
    topic: &'static str,
    mut broker_status: WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 6>,
    aggregated_telemetry_rx: Receiver<'static, CriticalSectionRawMutex, Telemetry, 1>,
    mqtt_publish_tx: Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) -> ! {
    // Hold off until the broker session comes up the first time. Later drops are not
    // waited on: the manager drains its queue on reconnect, so anything queued meanwhile
    // would be stale by the time it could be sent.
    while broker_status.get().await != BrokerStatus::Connected {
        broker_status.changed().await;
    }

    loop {
        let telemetry = aggregated_telemetry_rx.receive().await;

        match serde_json::to_vec(&telemetry) {
            Ok(buffer) => {
                let mut payload = heapless::Vec::<u8, 1024>::new();
                if payload.extend_from_slice(buffer.as_slice()).is_err() {
                    error!("telemetry: payload too large for buffer");
                    continue;
                }

                let payload_len = payload.len();
                let msg = PublishMessage { topic, payload };

                match mqtt_publish_tx.try_send(msg) {
                    Ok(_) => {
                        debug!("telemetry: queued {} bytes for publish", payload_len);
                    }
                    Err(_) => {
                        error!("telemetry: publish queue full or manager disconnected");
                    }
                }
            }
            Err(_) => {
                error!("telemetry: serialization failed");
            }
        }
    }
}
