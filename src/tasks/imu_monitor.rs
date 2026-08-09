use ariel_os::log::{Debug2Format, debug, error, info};
use ariel_os::time::{Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;

use crate::data::imu::{FilteredImu, ImuFilter, ImuSample};
use crate::data::telemetry::IMUTelemetry;
use crate::drivers::imu::Imu9Axis;
use crate::drivers::shared_i2c::BoardI2cDevice;
use crate::traits::Imu;

/// How often the sensors are sampled: 100 Hz.
///
/// Set by the dead reckoning, not by the reporting. Between two UWB range fixes the attitude
/// estimate is carried by integrating the gyroscope, and the error of that integration grows
/// with the length of each step; at 100 Hz a robot turning at 200 °/s moves two degrees per
/// step, which is small enough for the first-order integration used here to be honest. It is
/// also half the sensors' own output rate, so no poll ever reads the same sample twice.
///
/// The cost is I2C: each pass moves ~33 bytes across the two chips, which at 400 kHz is
/// under a millisecond, so this holds the bus a tenth of the time and leaves the rest to the
/// display and the charger.
const SAMPLE_INTERVAL: Duration = Duration::from_millis(10);

/// How often a sample goes out on the IMU stream topic: 50 Hz.
///
/// This is the knob to turn if the link is the constraint. A sample serializes to about 560
/// bytes of JSON, so 50 Hz costs roughly 27 kB/s per robot -- unremarkable for one, worth
/// reconsidering for a swarm sharing a broker. Lowering it costs the fusion filter
/// dead-reckoning resolution between UWB fixes; a more compact encoding than JSON would buy
/// most of the same bandwidth back without that cost.
///
/// Note that the stream is decimated from [`SAMPLE_INTERVAL`], so the `raw` axes of a
/// published sample are that sample's own -- the ones sampled in between are folded into the
/// filtered values but never leave the board.
const STREAM_INTERVAL: Duration = Duration::from_millis(20);

/// How often a sample makes it into the aggregated telemetry. The aggregator republishes
/// once a second, so anything faster is dropped on its floor anyway; twice a second means
/// the value it picks up is never more than half a period stale.
const TELEMETRY_INTERVAL: Duration = Duration::from_millis(500);

/// Backoff before re-probing sensors that stopped answering.
const RETRY_INTERVAL: Duration = Duration::from_secs(5);

/// Samples the nine-axis IMU, filters it, and publishes both versions on two paths.
///
/// `imu_stream` carries every published sample at [`STREAM_INTERVAL`] and is what feeds the
/// UWB localization filter; `imu_telemetry` carries a much slower trickle into the
/// once-a-second status payload. They are separate because the two consumers want opposite
/// things: one needs rate, the other needs to not drown the battery and network readings it
/// is bundled with.
///
/// The sensors share their I2C bus with the display and the charger, so this task holds
/// [`BoardI2cDevice`] handles rather than the bus itself -- one per part, since the
/// accelerometer/gyroscope and the magnetometer are separate chips at separate addresses.
#[ariel_os::task]
pub async fn monitor_imu(
    inertial_i2c: BoardI2cDevice,
    magnetic_i2c: BoardI2cDevice,
    imu_stream: Sender<'static, CriticalSectionRawMutex, IMUTelemetry, 2>,
    imu_telemetry: Sender<'static, CriticalSectionRawMutex, IMUTelemetry, 2>,
) -> ! {
    let mut imu = Imu9Axis::new(inertial_i2c, magnetic_i2c);

    loop {
        // Sensors that are not answering are not fatal -- the robot drives either way -- so
        // keep re-probing instead of parking the task. The same path recovers a part that
        // reset itself, e.g. after a brown-out while the motors stalled.
        if let Err(e) = imu.init().await {
            error!("imu: init failed: {:?}", Debug2Format(&e));
            Timer::after(RETRY_INTERVAL).await;
            continue;
        }
        info!("imu: sensors initialized");

        // A fresh filter per session: whatever state the old one held describes a sensor
        // that has since been reset, and its gyroscope bias in particular would be wrong.
        let mut filter = ImuFilter::new();
        let mut sampled_at = Instant::now();
        let mut streamed_at = Instant::now();
        let mut reported_at = Instant::now();

        loop {
            Timer::after(SAMPLE_INTERVAL).await;

            let sample = match imu.read().await {
                Ok(sample) => sample,
                Err(e) => {
                    error!("imu: read failed: {:?}", Debug2Format(&e));
                    break;
                }
            };
            // Stamped here rather than before the transaction: this is the closest the task
            // gets to when the sensors were actually read, and it is the number a fusion
            // filter aligns its UWB ranges against.
            let now = Instant::now();

            // The elapsed time, not `SAMPLE_INTERVAL`: waiting on a bus the display holds
            // for a full framebuffer flush stretches this period well past its nominal
            // value, and the filters integrate over whatever they are told.
            let timestep = (now - sampled_at).as_micros() as f32 / 1_000_000.0;
            sampled_at = now;

            let filtered = filter.update(&sample, timestep);

            if now - streamed_at >= STREAM_INTERVAL {
                streamed_at = now;
                // A full queue means the publisher or the link has not kept up. Dropping the
                // sample is right: the next one is along in 20 ms and is worth more than a
                // stale one delivered late.
                let _ = imu_stream.try_send(telemetry(now, &sample, &filtered));
            }

            if now - reported_at >= TELEMETRY_INTERVAL {
                reported_at = now;

                debug!(
                    "imu: roll={} pitch={} heading={} |a_lin|={} still={} T={}",
                    filtered.roll,
                    filtered.pitch,
                    filtered.heading.unwrap_or(f32::NAN),
                    filtered.linear_acceleration.length(),
                    filtered.is_stationary,
                    sample.temperature.unwrap_or(f32::NAN)
                );

                // The aggregator samples with `try_receive` and keeps the last value it saw,
                // so a full queue just means it has not caught up: dropping is right here too.
                let _ = imu_telemetry.try_send(telemetry(now, &sample, &filtered));
            }
        }

        Timer::after(RETRY_INTERVAL).await;
    }
}

fn telemetry(taken_at: Instant, sample: &ImuSample, filtered: &FilteredImu) -> IMUTelemetry {
    IMUTelemetry {
        timestamp_us: taken_at.as_micros(),
        raw: *sample,
        filtered: *filtered,
    }
}
