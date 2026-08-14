use ariel_os::log::debug;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::Receiver as WatchReceiver;

use crate::data::localization::RangeMeasurement;
use crate::data::mqtt::{BrokerStatus, PublishMessage};

/// How often to complain about a stream that is not going out. At the rate this task runs,
/// logging every failure would itself become the bottleneck.
const ERROR_LOG_INTERVAL: u32 = 50;

/// Publishes every accepted UWB range on its own topic.
///
/// One message per measurement -- up to `uwb_protocol::ACTIVE_ANCHOR_COUNT` per superframe --
/// for offline calibration and tuning, and until a later phase folds them into a published pose
/// estimate instead.
#[ariel_os::task]
pub async fn publish_uwb_ranges(
    topic: &'static str,
    mut broker_status: WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 6>,
    ranges_rx: Receiver<'static, CriticalSectionRawMutex, RangeMeasurement, 8>,
    mqtt_publish_tx: Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) -> ! {
    while broker_status.get().await != BrokerStatus::Connected {
        broker_status.changed().await;
    }

    let mut dropped: u32 = 0;

    loop {
        let measurement = ranges_rx.receive().await;

        let Ok(payload) = serde_json::to_vec(&measurement) else {
            debug!("uwb-publish: serialization failed");
            continue;
        };

        let mut message = PublishMessage {
            topic,
            payload: heapless::Vec::new(),
        };
        if message.payload.extend_from_slice(&payload).is_err() {
            debug!("uwb-publish: payload too large for buffer");
            continue;
        }

        // Never blocks: a range is only worth sending while it is current, so when the broker
        // queue is full it is dropped rather than delaying every measurement behind it.
        if mqtt_publish_tx.try_send(message).is_err() {
            dropped += 1;
            if dropped.is_multiple_of(ERROR_LOG_INTERVAL) {
                debug!(
                    "uwb-publish: {} ranges dropped, publish queue full",
                    dropped
                );
            }
        }
    }
}
