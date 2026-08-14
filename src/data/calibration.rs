use serde::{Deserialize, Serialize};

/// Per-robot provisioning command received on `/calibration/command/{ID}`.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum CalibrationCommand {
    StartCapture {
        session_id: u32,
        #[serde(default = "default_capture_seconds")]
        duration_s: u16,
    },
    ApplyRobotDelay {
        session_id: u32,
        rx_ticks: u16,
        tx_ticks: u16,
    },
    ClearRobotDelay {
        session_id: u32,
    },
}

const fn default_capture_seconds() -> u16 {
    60
}

/// Internal request sent to the one task that owns flash.
#[derive(Clone, Copy, Debug)]
pub struct CalibrationWriteRequest {
    pub session_id: u32,
    pub rx_ticks: u16,
    pub tx_ticks: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CalibrationWriteResult {
    Saved {
        session_id: u32,
        generation: u32,
        rx_ticks: u16,
        tx_ticks: u16,
    },
    Failed {
        session_id: u32,
    },
}

/// MQTT status payload. `reason` values are stable machine-readable strings used by the CLI.
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(tag = "state", rename_all = "snake_case")]
pub enum CalibrationStatus {
    Armed {
        window_s: u16,
        current_generation: u32,
        current_rx_ticks: u16,
        current_tx_ticks: u16,
    },
    CaptureStarted {
        session_id: u32,
        duration_s: u16,
    },
    CaptureFinished {
        session_id: u32,
    },
    AppliedPendingReboot {
        session_id: u32,
        generation: u32,
        rx_ticks: u16,
        tx_ticks: u16,
    },
    Rejected {
        session_id: u32,
        reason: &'static str,
    },
}

/// Pair result emitted by the temporary DS-TWR fixture.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[cfg_attr(not(feature = "calibration-fixture"), allow(dead_code))]
pub struct CalibrationSample {
    pub session_id: u32,
    pub initiator_id: u16,
    pub responder_id: u16,
    pub sequence: u16,
    pub distance_mm: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CalibrationCapture {
    pub session_id: u32,
    pub expires_at_us: u64,
}
