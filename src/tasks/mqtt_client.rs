use ariel_os::log::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;

use crate::data::mqtt::ReceivedMessage;

#[ariel_os::task]
pub async fn manage_mqtt_client(
    mqtt_receive_rx: Receiver<'static, CriticalSectionRawMutex, ReceivedMessage, 2>,
) -> ! {
    loop {
        let message = mqtt_receive_rx.receive().await;
        if let Ok(payload_str) = core::str::from_utf8(message.payload.as_slice()) {
            info!(
                "mqtt: topic={}, message={}",
                message.topic.as_str(),
                payload_str
            );
        } else {
            info!(
                "mqtt: topic={}, message=<invalid utf8>",
                message.topic.as_str()
            );
        }
    }
}
