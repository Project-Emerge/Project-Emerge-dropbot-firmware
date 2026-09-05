use heapless::String;
use serde::{Deserialize, Serialize};

pub const OTA_SERVER_MAX_LEN: usize = 96;

/// Fleet-wide OTA endpoint received from the retained `/config/ota` MQTT topic.
///
/// `server` is the authority portion used in an HTTP URL, for example `192.168.8.1`,
/// `ota.local`, or `192.168.8.1:8787`.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct OtaConfiguration {
    pub server: String<OTA_SERVER_MAX_LEN>,
}

impl OtaConfiguration {
    /// Rejects values that cannot be an HTTP authority before they reach the OTA task.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        !self.server.is_empty()
            && !self
                .server
                .bytes()
                .any(|byte| byte.is_ascii_whitespace() || matches!(byte, b'/' | b'#' | b'?'))
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MotorsConfiguration {
    pub ema_filter_alpha: Option<f32>,
    pub min_duty_cycle: f32,
}
