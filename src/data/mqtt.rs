use heapless::String;
use heapless::Vec;

#[derive(Clone)]
pub struct PublishMessage {
    pub topic: String<64>,
    pub payload: Vec<u8, 1024>,
}

#[derive(Clone, Default)]
pub struct ReceivedMessage {
    pub topic: String<64>,
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
