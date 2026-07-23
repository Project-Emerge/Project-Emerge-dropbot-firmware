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
