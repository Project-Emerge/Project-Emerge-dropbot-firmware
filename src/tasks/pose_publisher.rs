use ariel_os::log::{debug, error};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::watch::Receiver as WatchReceiver;

use crate::data::localization::PoseEstimate;
use crate::data::mqtt::{BrokerStatus, PublishMessage};

/// How often to complain about a stream that is not going out.
const ERROR_LOG_INTERVAL: u32 = 50;

/// Publishes the robot's own pose estimate.
///
/// A `Watch` on the input rather than a channel: a pose is only worth sending while it is current, and
/// if the broker falls behind the right answer is to skip to the latest estimate rather than to send a
/// backlog of stale positions. That also makes this task's cadence the *broker's* rather than the
/// estimator's, so a slow network cannot back-pressure the filter.
#[ariel_os::task]
pub async fn publish_pose(
    topic: &'static str,
    mut broker_status: WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 5>,
    mut pose_rx: WatchReceiver<'static, CriticalSectionRawMutex, PoseEstimate, 2>,
    mqtt_publish_tx: Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) -> ! {
    while broker_status.get().await != BrokerStatus::Connected {
        broker_status.changed().await;
    }

    let mut dropped: u32 = 0;

    loop {
        let pose = pose_rx.changed().await;

        let Ok(payload) = serde_json::to_vec(&pose) else {
            error!("pose-publish: serialization failed");
            continue;
        };

        let mut message = PublishMessage {
            topic,
            payload: heapless::Vec::new(),
        };
        if message.payload.extend_from_slice(&payload).is_err() {
            error!("pose-publish: payload too large for buffer");
            continue;
        }

        if mqtt_publish_tx.try_send(message).is_err() {
            dropped += 1;
            if dropped.is_multiple_of(ERROR_LOG_INTERVAL) {
                debug!(
                    "pose-publish: {} poses dropped, publish queue full",
                    dropped
                );
            }
        }
    }
}
