use ariel_os::log::debug;
use ariel_os::log::error;

use crate::data::mqtt::{BrokerStatus, PublishMessage};
use crate::task_sync::{AggregatedTelemetryRx, BrokerStatusRx, MqttPublishTx};

/// Messaging endpoints owned by the aggregated-telemetry publisher.
pub struct TelemetryPublisherPorts {
    pub broker_status: BrokerStatusRx,
    pub aggregated_telemetry: AggregatedTelemetryRx,
    pub mqtt_publish: MqttPublishTx,
}

#[ariel_os::task]
pub async fn publish_telemetry(topic: &'static str, mut ports: TelemetryPublisherPorts) -> ! {
    // Hold off until the broker session comes up the first time. Later drops are not
    // waited on: the manager drains its queue on reconnect, so anything queued meanwhile
    // would be stale by the time it could be sent.
    while ports.broker_status.get().await != BrokerStatus::Connected {
        ports.broker_status.changed().await;
    }

    loop {
        let telemetry = ports.aggregated_telemetry.receive().await;

        match serde_json::to_vec(&telemetry) {
            Ok(buffer) => {
                let mut payload = heapless::Vec::<u8, 1024>::new();
                if payload.extend_from_slice(buffer.as_slice()).is_err() {
                    error!("telemetry: payload too large for buffer");
                    continue;
                }

                let payload_len = payload.len();
                let msg = PublishMessage { topic, payload };

                match ports.mqtt_publish.try_send(msg) {
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
