use core::fmt::Write as _;

use ariel_os::reexports::static_cell::StaticCell;
use heapless::String;
use minimq::TopicFilter;

use crate::device_id::DEVICE_ID_LEN;

const TELEMETRY_PREFIX: &str = "/telemetry/";
const IMU_STREAM_PREFIX: &str = "/imu/";
const UWB_PREFIX: &str = "/uwb/";
const POSE_PREFIX: &str = "/pose/";
const MOTOR_COMMAND_PREFIX: &str = "/motors/";
const OTA_CHECK_PREFIX: &str = "/ota/check/";

/// Fleet-shared, unlike every topic above: every robot subscribes to and parses the same
/// retained message and picks out its own entry, rather than each robot getting its own
/// namespaced topic. A plain `&'static str` rather than a `Topics` field built with the device
/// ID, since it does not have one. See `data::configurations::TagAssignmentsConfiguration` and
/// `drivers::uwb::tag_id::resolve_from_config`.
pub const TAG_ASSIGNMENTS_TOPIC: &str = "/config/tag-assignments";

/// Fleet-shared for a stronger reason than the tag assignments: the anchors are physically shared
/// hardware, so two robots holding different geometries for the same arena would report poses in two
/// different coordinate frames. One retained message is the only representation where that cannot
/// happen. See `data::configurations::AnchorsConfiguration` and `drivers::uwb::anchors`.
pub const ANCHORS_TOPIC: &str = "/config/anchors";

// Each buffer is sized to exactly what it holds: its own prefix plus the device ID.
const TELEMETRY_LEN: usize = TELEMETRY_PREFIX.len() + DEVICE_ID_LEN;
const IMU_STREAM_LEN: usize = IMU_STREAM_PREFIX.len() + DEVICE_ID_LEN;
const UWB_LEN: usize = UWB_PREFIX.len() + DEVICE_ID_LEN;
const POSE_LEN: usize = POSE_PREFIX.len() + DEVICE_ID_LEN;
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
    TagAssignments,
    Anchors,
}

impl InboundTopic {
    /// Label for the log line.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::MotorCommand => "motors",
            Self::OtaCheck => "ota-check",
            Self::TagAssignments => "tag-assignments",
            Self::Anchors => "anchors",
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
    uwb: String<UWB_LEN>,
    pose: String<POSE_LEN>,
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
        uwb: build(UWB_PREFIX, device_id),
        pose: build(POSE_PREFIX, device_id),
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

    /// Raw UWB range measurements, one message per accepted range.
    ///
    /// Kept alongside [`Self::pose`] rather than replaced by it: these are the input the range-bias
    /// calibration is fitted from (see `uwb_protocol::RangeBias`) and the per-anchor coverage signal a
    /// pose cannot carry. Off by default -- see `tasks::pose_estimator::PUBLISH_RAW_RANGES` for the
    /// bandwidth arithmetic.
    #[must_use]
    pub fn uwb(&self) -> &str {
        &self.uwb
    }

    /// The robot's own pose estimate, one message per superframe.
    ///
    /// This is the product the UWB stack exists to produce; `/uwb/{ID}` is its raw input.
    #[must_use]
    pub fn pose(&self) -> &str {
        &self.pose
    }

    /// The filters to subscribe with. Kept alongside [`Self::resolve`] so the set of topics
    /// the broker is asked for and the set that can be recognised on arrival cannot drift.
    #[must_use]
    pub fn subscriptions(&self) -> [TopicFilter<'_>; 4] {
        [
            TopicFilter::new(&self.motor_command),
            TopicFilter::new(&self.ota_check),
            TopicFilter::new(TAG_ASSIGNMENTS_TOPIC),
            TopicFilter::new(ANCHORS_TOPIC),
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
        } else if topic == TAG_ASSIGNMENTS_TOPIC {
            Some(InboundTopic::TagAssignments)
        } else if topic == ANCHORS_TOPIC {
            Some(InboundTopic::Anchors)
        } else {
            None
        }
    }
}
