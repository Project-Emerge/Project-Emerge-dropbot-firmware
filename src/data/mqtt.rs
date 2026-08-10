use heapless::Vec;

use crate::topics::InboundTopic;

/// Both topics are borrowed from the table in [`crate::topics`] rather than owned: the topic
/// of a message is always one of a handful known at boot, so a queued message carries a
/// pointer instead of a 64-byte buffer it would have to have formatted first.
#[derive(Clone)]
pub struct PublishMessage {
    pub topic: &'static str,
    pub payload: Vec<u8, 1024>,
}

#[derive(Clone)]
pub struct ReceivedMessage {
    pub topic: InboundTopic,
    pub payload: Vec<u8, 1024>,
}

/// Whether the MQTT session is up, broadcast by `mqtt_manager`.
///
/// The manager retries indefinitely, so `Disconnected` also covers "trying to connect": for
/// the display there is no useful difference between the two.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BrokerStatus {
    #[default]
    Disconnected,
    Connected,
}

impl BrokerStatus {
    /// Label for the display.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::Disconnected => "OFFLINE",
            Self::Connected => "ONLINE",
        }
    }
}
