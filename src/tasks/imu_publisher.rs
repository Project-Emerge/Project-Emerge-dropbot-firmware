use ariel_os::log::{debug, error};

use crate::data::mqtt::{BrokerStatus, PublishMessage};
use crate::task_sync::{BrokerStatusRx, ImuStreamRx, MqttPublishTx};

/// How often to complain about a stream that is not going out. At the rate this task runs,
/// logging every failure would itself become the bottleneck.
const ERROR_LOG_INTERVAL: u32 = 250;

/// Messaging endpoints owned by the high-rate IMU publisher.
pub struct ImuPublisherPorts {
    pub broker_status: BrokerStatusRx,
    pub imu_stream: ImuStreamRx,
    pub mqtt_publish: MqttPublishTx,
}

/// Publishes the high-rate IMU stream on its own topic.
///
/// Separate from `publish_telemetry` because the two carry different contracts. That one
/// sends a once-a-second status summary; this one sends motion samples at a higher rate.
/// Sharing a topic would force every subscriber that only wants a battery percentage to parse
/// the full sensor stream.
#[ariel_os::task]
pub async fn publish_imu_stream(topic: &'static str, mut ports: ImuPublisherPorts) -> ! {
    while ports.broker_status.get().await != BrokerStatus::Connected {
        ports.broker_status.changed().await;
    }

    let mut dropped: u32 = 0;

    loop {
        let sample = ports.imu_stream.receive().await;

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
        if ports.mqtt_publish.try_send(message).is_err() {
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
