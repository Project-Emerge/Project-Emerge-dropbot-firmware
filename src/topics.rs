use core::fmt::Write as _;

use ariel_os::reexports::static_cell::StaticCell;
use heapless::String;
use minimq::TopicFilter;

use crate::device_id::DEVICE_ID_LEN;

const TELEMETRY_PREFIX: &str = "/telemetry/";
const IMU_STREAM_PREFIX: &str = "/imu/";
const MOTOR_COMMAND_PREFIX: &str = "/motors/";
const OTA_CHECK_PREFIX: &str = "/ota/check/";

// Each buffer is sized to exactly what it holds: its own prefix plus the device ID.
const TELEMETRY_LEN: usize = TELEMETRY_PREFIX.len() + DEVICE_ID_LEN;
const IMU_STREAM_LEN: usize = IMU_STREAM_PREFIX.len() + DEVICE_ID_LEN;
const MOTOR_COMMAND_LEN: usize = MOTOR_COMMAND_PREFIX.len() + DEVICE_ID_LEN;
const OTA_CHECK_LEN: usize = OTA_CHECK_PREFIX.len() + DEVICE_ID_LEN;

/// Which subscription an inbound message arrived on.
///
/// `mqtt_manager` resolves the wire topic into one of these once, so a `ReceivedMessage`
/// carries a one-byte tag instead of a topic buffer and its consumers match on a variant
/// instead of string-comparing against a topic they would have had to format themselves.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum InboundTopic {
    MotorCommand,
    OtaCheck,
}

impl InboundTopic {
    /// Label for the log line.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::MotorCommand => "motors",
            Self::OtaCheck => "ota-check",
        }
    }
}

/// Every MQTT topic this firmware uses, formatted once from the device ID.
///
/// The fleet shares a broker, so each topic is namespaced by the device ID and none of them
/// is known until the eFuse has been read at boot. Building them all here, into a `'static`
/// the tasks borrow, keeps two things from spreading: the topic strings themselves -- which
/// were being formatted in four tasks, twice each for the two subscriptions -- and the
/// buffers holding them. Publishers and subscription filters take a `&'static str` from this
/// table rather than carrying a `String` apiece, and nothing on the hot path formats a topic
/// per message.
pub struct Topics {
    telemetry: String<TELEMETRY_LEN>,
    imu_stream: String<IMU_STREAM_LEN>,
    motor_command: String<MOTOR_COMMAND_LEN>,
    ota_check: String<OTA_CHECK_LEN>,
}

static STORAGE: StaticCell<Topics> = StaticCell::new();

/// Builds the topic table for this board.
///
/// Must be called exactly once, right after [`crate::device_id::init`] and before any task
/// that publishes or subscribes is spawned.
pub fn init(device_id: &str) -> &'static Topics {
    STORAGE.init(Topics {
        telemetry: build(TELEMETRY_PREFIX, device_id),
        imu_stream: build(IMU_STREAM_PREFIX, device_id),
        motor_command: build(MOTOR_COMMAND_PREFIX, device_id),
        ota_check: build(OTA_CHECK_PREFIX, device_id),
    })
}

fn build<const N: usize>(prefix: &str, device_id: &str) -> String<N> {
    let mut topic = String::new();
    // `N` is this prefix's length plus `DEVICE_ID_LEN`, and `device_id::init` yields exactly
    // that many characters, so there is nothing here that can overflow the buffer.
    let _ = write!(topic, "{prefix}{device_id}");
    topic
}

impl Topics {
    /// Once-a-second aggregated status bundle.
    #[must_use]
    pub fn telemetry(&self) -> &str {
        &self.telemetry
    }

    /// High-rate IMU samples, on their own topic so a subscriber that only wants the status
    /// bundle does not have to read fifty messages a second.
    #[must_use]
    pub fn imu_stream(&self) -> &str {
        &self.imu_stream
    }

    /// The filters to subscribe with. Kept alongside [`Self::resolve`] so the set of topics
    /// the broker is asked for and the set that can be recognised on arrival cannot drift.
    #[must_use]
    pub fn subscriptions(&self) -> [TopicFilter<'_>; 2] {
        [
            TopicFilter::new(&self.motor_command),
            TopicFilter::new(&self.ota_check),
        ]
    }

    /// Maps a topic received off the wire back to the subscription it belongs to, or `None`
    /// if the broker sent something that was never subscribed to.
    #[must_use]
    pub fn resolve(&self, topic: &str) -> Option<InboundTopic> {
        if topic == self.motor_command.as_str() {
            Some(InboundTopic::MotorCommand)
        } else if topic == self.ota_check.as_str() {
            Some(InboundTopic::OtaCheck)
        } else {
            None
        }
    }
}
