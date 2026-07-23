use core::fmt::Write;

use ariel_os::log::debug;
use ariel_os::log::error;
use heapless::String;

use crate::data::mqtt::PublishMessage;
use crate::{AGGREGATED_TELEMETRY, DEVICE_ID, MQTT_CONNECTION, MQTT_PUBLISH};

#[ariel_os::task]
pub async fn publish_telemetry() -> ! {
    MQTT_CONNECTION.wait().await;

    loop {
        let telemetry = AGGREGATED_TELEMETRY.receive().await;

        match serde_json::to_vec(&telemetry) {
            Ok(buffer) => {
                let mut topic_buf = String::<64>::new();
                let _ = write!(topic_buf, "/telemetry/{}", DEVICE_ID);

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

                match MQTT_PUBLISH.try_send(msg) {
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
