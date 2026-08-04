use core::fmt::Write;

use ariel_os::log::debug;
use ariel_os::log::error;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::signal::Signal;
use heapless::String;

use crate::data::mqtt::PublishMessage;
use crate::data::telemetry::Telemetry;

#[ariel_os::task]
pub async fn publish_telemetry(
    device_id: &'static str,
    mqtt_connection: &'static Signal<CriticalSectionRawMutex, ()>,
    aggregated_telemetry_rx: Receiver<'static, CriticalSectionRawMutex, Telemetry, 1>,
    mqtt_publish_tx: Sender<'static, CriticalSectionRawMutex, PublishMessage, 2>,
) -> ! {
    mqtt_connection.wait().await;

    loop {
        let telemetry = aggregated_telemetry_rx.receive().await;

        match serde_json::to_vec(&telemetry) {
            Ok(buffer) => {
                let mut topic_buf = String::<64>::new();
                let _ = write!(topic_buf, "/telemetry/{}", device_id);

                let mut payload = heapless::Vec::<u8, 1024>::new();
                if payload.extend_from_slice(buffer.as_slice()).is_err() {
                    error!("telemetry: payload too large for buffer");
                    continue;
                }

                let payload_len = payload.len();
                let msg = PublishMessage {
                    topic: topic_buf,
                    payload,
                };

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
