use ariel_os::log::info;

use crate::MQTT_RECEIVE;

#[ariel_os::task]
pub async fn manage_mqtt_client() -> ! {
    loop {
        let message = MQTT_RECEIVE.receive().await;
        if let Ok(payload_str) = core::str::from_utf8(message.payload.as_slice()) {
            info!("mqtt: topic={}, message={}", message.topic.as_str(), payload_str);
        } else {
            info!("mqtt: topic={}, message=<invalid utf8>", message.topic.as_str());
        }
    }
}
