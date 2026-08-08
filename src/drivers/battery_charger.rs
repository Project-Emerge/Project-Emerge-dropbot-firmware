use bq25887::{BQ25887Error, Bq25887Driver, ChrgStat};
use embedded_hal_async::i2c::I2c;

use crate::data::battery::{ChargeState, ChargerFault, ChargerStatus, ThermistorState};
use crate::traits::BatteryCharger;

/// `TS_STAT` occupies the low three bits of the NTC status register. The generated enum for
/// it is private to the `bq25887` crate, so the register is decoded from its raw byte.
const TS_STAT_MASK: u8 = 0b0000_0111;

/// Monitors the pack through a BQ25887 2S charger.
///
/// The charger is read-only as far as this firmware is concerned: charge current, cell
/// voltage limits and the like are set by the hardware's resistors and the chip's defaults,
/// and nothing here writes them. The only registers written are the ADC controls, which
/// have to be enabled before any measurement can be read back.
pub struct BQ25887Charger<I2C: I2c> {
    device: Bq25887Driver<I2C>,
}

impl<I2C: I2c> BQ25887Charger<I2C> {
    /// Wraps the charger at its fixed address, `0x6A`.
    pub fn new(i2c: I2C) -> Self {
        Self {
            device: Bq25887Driver::new(i2c),
        }
    }
}

impl<I2C: I2c> BatteryCharger for BQ25887Charger<I2C> {
    type Error = BQ25887Error<I2C::Error>;

    async fn init(&mut self) -> Result<(), Self::Error> {
        // Reading part information doubles as a presence check: a charger that is not on
        // the bus fails here instead of silently reporting an empty pack forever.
        let _ = self.device.read_part_information().await?;

        // The charger's I2C watchdog resets its registers -- including the ADC enable
        // below -- if the host stops talking to it for long enough. Nothing here relies on
        // that safety net, and leaving it on would mean a delayed poll silently stops the
        // measurements.
        self.device.disable_watchdog().await?;
        self.device.enable_adc_continuous().await?;

        Ok(())
    }

    async fn read_status(&mut self) -> Result<ChargerStatus, Self::Error> {
        let state = charge_state(self.device.get_charge_status().await?);
        let power_good = self.device.is_power_good().await?;
        let vbat_mv = self.device.read_vbat_mv().await?;
        let cell_top_mv = self.device.read_vcell_top_mv().await?;
        let cell_bot_mv = self.device.read_vcell_bot_mv().await?;
        let vbus_mv = self.device.read_vbus_mv().await?;
        let charge_current_ma = self.device.read_ichg_ma().await?;
        let input_current_ma = self.device.read_ibus_ma().await?;
        let die_temp_dc = self.device.read_tdie_decidegrees().await?;

        let faults = self.device.read_fault_status().await?;
        let fault = if faults.tshut_stat() {
            Some(ChargerFault::ThermalShutdown)
        } else if faults.vbus_ovp_stat() {
            Some(ChargerFault::InputOverVoltage)
        } else if faults.tmr_stat() {
            Some(ChargerFault::SafetyTimer)
        } else {
            None
        };

        let ntc: [u8; 1] = self.device.read_ntc_status().await?.into();

        Ok(ChargerStatus {
            state,
            vbat_mv,
            cell_top_mv,
            cell_bot_mv,
            vbus_mv,
            charge_current_ma,
            input_current_ma,
            die_temp_dc,
            power_good,
            thermistor: thermistor_state(ntc[0] & TS_STAT_MASK),
            fault,
        })
    }
}

fn charge_state(status: ChrgStat) -> ChargeState {
    match status {
        ChrgStat::NotCharging => ChargeState::NotCharging,
        ChrgStat::TrickleCharge => ChargeState::Trickle,
        ChrgStat::PreCharge => ChargeState::PreCharge,
        ChrgStat::FastCharge => ChargeState::FastCharge,
        ChrgStat::TaperCharge => ChargeState::TaperCharge,
        ChrgStat::TopoffTimerCharging => ChargeState::TopOff,
        ChrgStat::ChargeTerminationDone => ChargeState::Done,
        ChrgStat::Reserved => ChargeState::Unknown,
    }
}

/// Decodes the `TS_STAT` field. The codes are not contiguous -- 1, 4 and 7 are undefined --
/// hence the explicit table.
fn thermistor_state(code: u8) -> ThermistorState {
    match code {
        0b000 => ThermistorState::Normal,
        0b010 => ThermistorState::Warm,
        0b011 => ThermistorState::Cool,
        0b101 => ThermistorState::Cold,
        0b110 => ThermistorState::Hot,
        _ => ThermistorState::Unknown,
    }
}
