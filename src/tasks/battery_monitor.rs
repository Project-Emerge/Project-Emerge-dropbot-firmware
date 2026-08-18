use ariel_os::log::{Debug2Format, debug, error, info};
use ariel_os::time::{Duration, Timer};

use crate::data::power::ShutdownReason;
use crate::data::telemetry::BatteryTelemetry;
use crate::drivers::battery_charger::BQ25887Charger;
use crate::drivers::shared_i2c::BoardI2cDevice;
use crate::task_sync::{BatteryTelemetryTx, ChargerStatusTx, ShutdownRequestTx};
use crate::traits::BatteryCharger;

/// How often the charger is sampled. The BQ25887's ADC runs continuously and the pack does
/// not move fast, so this is about display responsiveness rather than measurement fidelity.
const POLL_INTERVAL: Duration = Duration::from_secs(2);
/// Backoff before re-probing a charger that stopped answering.
const RETRY_INTERVAL: Duration = Duration::from_secs(5);
/// Low battery threshold: when the pack's state of charge falls below this while it is not
/// charging, the board displays a warning and powers off. The BQ25887's SOC is a rough
/// estimate, so this is set conservatively to avoid brown-out.
const LOW_BATTERY_THRESHOLD: u8 = 15;

/// Messaging endpoints owned by the battery-monitor task.
pub struct BatteryMonitorPorts {
    pub battery_telemetry: BatteryTelemetryTx,
    pub charger_status: ChargerStatusTx,
    pub shutdown_requests: ShutdownRequestTx,
}

/// Polls the BQ25887 charger and publishes what it reports.
///
/// Two consumers: the display, which draws the battery page from [`ChargerStatus`], and the
/// telemetry aggregator, which gets the same data flattened into [`BatteryTelemetry`].
///
/// The charger shares its I2C bus with the display, so this task holds a
/// [`BoardI2cDevice`] handle rather than the bus itself.
#[ariel_os::task]
pub async fn monitor_battery(i2c: BoardI2cDevice, ports: BatteryMonitorPorts) -> ! {
    let mut charger = BQ25887Charger::new(i2c);

    loop {
        // A charger that is not answering is not fatal -- the board runs off the pack
        // either way -- so keep re-probing instead of parking the task. The same path
        // recovers from a charger that resets itself, e.g. after a brown-out on VBUS.
        if let Err(e) = charger.init().await {
            error!("battery: charger init failed: {:?}", Debug2Format(&e));
            Timer::after(RETRY_INTERVAL).await;
            continue;
        }
        info!("battery: charger initialized");

        loop {
            Timer::after(POLL_INTERVAL).await;

            let status = match charger.read_status().await {
                Ok(status) => status,
                Err(e) => {
                    error!("battery: read failed: {:?}", Debug2Format(&e));
                    break;
                }
            };

            debug!(
                "battery: {}% vbat={}mV ichg={}mA vbus={}mV pg={} state={:?}",
                status.state_of_charge(),
                status.vbat_mv,
                status.charge_current_ma,
                status.vbus_mv,
                status.power_good,
                Debug2Format(&status.state)
            );

            ports.charger_status.send(status);

            if status.state_of_charge() < LOW_BATTERY_THRESHOLD && !status.state.is_charging() {
                info!(
                    "battery: {}% and not charging; warning before power-off",
                    status.state_of_charge()
                );
                let _ = ports.shutdown_requests.try_send(ShutdownReason::LowBattery);
            }

            let telemetry = BatteryTelemetry {
                voltage: f32::from(status.vbat_mv) / 1000.0,
                current: f32::from(status.charge_current_ma) / 1000.0,
                temperature: f32::from(status.die_temp_dc) / 10.0,
                is_charging: status.state.is_charging(),
                state_of_charge: status.state_of_charge(),
            };
            // The aggregator samples with `try_receive` and keeps the last value it saw, so
            // a full queue just means it has not caught up: dropping is the right call.
            let _ = ports.battery_telemetry.try_send(telemetry);
        }

        Timer::after(RETRY_INTERVAL).await;
    }
}
