/// Where the charger is in its charging cycle, from the BQ25887's `CHRG_STAT` field.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum ChargeState {
    /// No charging in progress -- either nothing is plugged in, or charging is disabled.
    #[default]
    NotCharging,
    /// Trickle charge, the pack is below `VBAT_SHORT`.
    Trickle,
    /// Pre-charge, the pack is between `VBAT_UVLO` and `VBAT_LOWV`.
    PreCharge,
    /// Fast charge, constant-current phase.
    FastCharge,
    /// Taper charge, constant-voltage phase.
    TaperCharge,
    /// Top-off timer running after the taper phase.
    TopOff,
    /// Charge terminated: the pack is full.
    Done,
    /// The charger reported the reserved code, i.e. a value this firmware does not know.
    Unknown,
}

impl ChargeState {
    /// Label for the display, kept short enough to sit in a page header.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::NotCharging => "IDLE",
            Self::Trickle => "TRICKLE",
            Self::PreCharge => "PRECHG",
            Self::FastCharge => "CHARGING",
            Self::TaperCharge => "TAPER",
            Self::TopOff => "TOP-OFF",
            Self::Done => "FULL",
            Self::Unknown => "?",
        }
    }

    /// Whether current is currently flowing into the pack.
    #[must_use]
    pub fn is_charging(self) -> bool {
        matches!(
            self,
            Self::Trickle | Self::PreCharge | Self::FastCharge | Self::TaperCharge | Self::TopOff
        )
    }
}

/// A latched fault from the charger's `FAULT_STATUS` register.
///
/// Only one is surfaced at a time; the variants are ordered by how much they matter, and
/// [`ChargerStatus::fault`] reports the first one that is set.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ChargerFault {
    /// The charger shut down because its junction temperature ran away.
    ThermalShutdown,
    /// Input over-voltage on VBUS.
    InputOverVoltage,
    /// The charge safety timer expired before the pack reached termination.
    SafetyTimer,
}

impl ChargerFault {
    /// Label for the display.
    #[must_use]
    pub fn label(self) -> &'static str {
        match self {
            Self::ThermalShutdown => "OVERTEMP",
            Self::InputOverVoltage => "VBUS OVP",
            Self::SafetyTimer => "CHG TIMER",
        }
    }
}

/// Pack thermistor state, from the charger's `TS_STAT` field.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum ThermistorState {
    /// The pack is within its charging temperature window.
    #[default]
    Normal,
    /// Charging continues at a reduced rate or voltage.
    Warm,
    /// Charging continues at a reduced rate or voltage.
    Cool,
    /// Charging is suspended until the pack warms up.
    Cold,
    /// Charging is suspended until the pack cools down.
    Hot,
    /// The charger reported a code this firmware does not know.
    Unknown,
}

impl ThermistorState {
    /// Label for the display, or `None` when the pack temperature is unremarkable and not
    /// worth spending a row on.
    #[must_use]
    pub fn label(self) -> Option<&'static str> {
        match self {
            Self::Normal => None,
            Self::Warm => Some("PACK WARM"),
            Self::Cool => Some("PACK COOL"),
            Self::Cold => Some("PACK COLD"),
            Self::Hot => Some("PACK HOT"),
            Self::Unknown => Some("PACK ?"),
        }
    }
}

/// Everything the BQ25887 reports about the pack and the input, sampled in one pass.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ChargerStatus {
    pub state: ChargeState,
    /// Pack voltage, i.e. both cells in series.
    pub vbat_mv: u16,
    /// Upper cell of the 2S pack.
    pub cell_top_mv: u16,
    /// Lower cell of the 2S pack.
    pub cell_bot_mv: u16,
    /// Input voltage; 0 when nothing is plugged in.
    pub vbus_mv: u16,
    /// Current flowing into the pack.
    pub charge_current_ma: u16,
    /// Current drawn from the input.
    pub input_current_ma: u16,
    /// Charger junction temperature in tenths of a degree Celsius. This is the IC's own
    /// die, not the pack -- the pack only reports the coarse [`ThermistorState`].
    pub die_temp_dc: i16,
    /// Whether the input source is present and usable.
    pub power_good: bool,
    pub thermistor: ThermistorState,
    pub fault: Option<ChargerFault>,
}

/// Resting cell voltage against state of charge for a Li-ion/LiPo cell, in ascending order.
/// Interpolated linearly between points by [`ChargerStatus::state_of_charge`].
const OCV_CURVE: [(u16, u8); 11] = [
    (3000, 0),
    (3300, 3),
    (3400, 8),
    (3500, 15),
    (3600, 25),
    (3700, 37),
    (3800, 50),
    (3900, 62),
    (4000, 75),
    (4100, 87),
    (4200, 100),
];

/// Above this, something is really plugged into VBUS.
///
/// The VBUS ADC does not read a clean zero with nothing connected -- leakage and ADC noise
/// leave it wandering around a few tens of millivolts -- so presence cannot be decided by
/// comparing against zero. Any usable source sits at 5 V nominal and the charger stops
/// accepting one long before this, so the threshold cleanly separates the two.
const VBUS_PRESENT_MV: u16 = 3000;

impl ChargerStatus {
    /// Whether an input source is physically present on VBUS.
    #[must_use]
    pub fn input_present(&self) -> bool {
        self.vbus_mv >= VBUS_PRESENT_MV
    }

    /// Whether an input is plugged in but the charger refuses to draw from it, e.g. a weak
    /// supply or a bad cable. Running on the pack alone is not this: with nothing connected
    /// there is simply no input to complain about.
    #[must_use]
    pub fn input_unusable(&self) -> bool {
        self.input_present() && !self.power_good
    }

    /// Rough state of charge in percent.
    ///
    /// The BQ25887 has no fuel gauge, so this is interpolated off [`OCV_CURVE`] from the
    /// weaker of the two cells -- that cell is what ends the discharge, so it is the honest
    /// one to report. Being voltage-based, it reads low under a heavy motor load and high
    /// while charging; treat it as an indication, not a measurement.
    #[must_use]
    pub fn state_of_charge(&self) -> u8 {
        // A cell reading zero means the ADC has not produced a sample for it yet; fall
        // back to half the pack voltage rather than reporting an empty battery.
        let cell_mv = match (self.cell_top_mv, self.cell_bot_mv) {
            (0, 0) => self.vbat_mv / 2,
            (0, bot) => bot,
            (top, 0) => top,
            (top, bot) => top.min(bot),
        };

        soc_from_cell_mv(cell_mv)
    }
}

fn soc_from_cell_mv(cell_mv: u16) -> u8 {
    let (lowest_mv, empty) = OCV_CURVE[0];
    if cell_mv <= lowest_mv {
        return empty;
    }

    for window in OCV_CURVE.windows(2) {
        let (low_mv, low_percent) = window[0];
        let (high_mv, high_percent) = window[1];
        if cell_mv <= high_mv {
            let span = u32::from(high_mv - low_mv);
            let offset = u32::from(cell_mv - low_mv);
            let rise = u32::from(high_percent - low_percent);
            return (u32::from(low_percent) + offset * rise / span) as u8;
        }
    }

    100
}
