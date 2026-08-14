#![no_std]
#![no_main]

mod data;
mod device_id;
#[cfg_attr(feature = "calibration-fixture", allow(dead_code))]
mod drivers;
mod pins;
mod tasks;
mod topics;
mod traits;

use ariel_os::hal;
use ariel_os::i2c::controller::{Kilohertz, highest_freq_in};
use ariel_os::log::info;
use ariel_os::reexports::embassy_net::Ipv4Address;
use ariel_os::reexports::static_cell::StaticCell;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
    watch::Watch,
};

use crate::drivers::shared_i2c::{BoardI2cBus, BoardI2cDevice, SharedI2c};
use crate::pins::I2cBus;
#[cfg(not(feature = "calibration-fixture"))]
use crate::tasks::start_uwb_ranging;
use crate::tasks::{
    aggregate_telemetry, estimate_pose, manage_calibration, manage_display,
    manage_motor_controller, manage_mqtt_client, manage_ota, manage_power_button, monitor_battery,
    monitor_imu, mqtt_manager, network_monitor, publish_imu_stream, publish_pose,
    publish_telemetry, publish_uwb_ranges,
};
#[cfg(feature = "calibration-fixture")]
use crate::tasks::{publish_calibration_samples, start_uwb_fixture};

pub const TCP_BUFFER_SIZE: usize = 1024;
/// Version this firmware build identifies as to the OTA server; taken from `Cargo.toml`'s
/// `package.version`, which Cargo exposes at build time via `CARGO_PKG_VERSION`.
pub const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");

// Wiring between tasks: each primitive lives here, scoped to this module, and is handed out
// as an explicit `spawn(...)` argument below rather than imported ambiently via `crate::FOO`.
// This keeps the task graph readable in one place and lets a `Channel` split into a `Sender`
// (given to the producer) and a `Receiver` (given to the consumer) instead of a shared handle
// both sides could call `.send()`/`.receive()` on.
static NETWORK_STATUS: Signal<CriticalSectionRawMutex, Option<Ipv4Address>> = Signal::new();
// Gates every task that must not run before the network is up. This is a `Watch`, not a
// `Signal`, because it has more than one consumer: a `Signal` holds a single waker and a
// single value, so two waiters replace each other's waker on every poll and wake each other
// forever. That livelock starves lower-priority threads -- including esp-radio's timer
// thread, which Wi-Fi association depends on -- so Wi-Fi never connects.
static NETWORK_READY: Watch<CriticalSectionRawMutex, (), 2> = Watch::new();
static MOTOR_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::MotorTelemetry, 2> =
    Channel::new();
static BATTERY_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::BatteryTelemetry, 2> =
    Channel::new();
static IMU_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::IMUTelemetry, 2> =
    Channel::new();
// The IMU's high-rate path, separate from `IMU_TELEMETRY` above: this one carries every
// sample the fusion filter needs to dead-reckon between UWB range fixes, straight to its own
// MQTT topic, while `IMU_TELEMETRY` trickles the same payload into the once-a-second status
// bundle. Deliberately shallow -- a stream sample is only worth sending while it is current,
// so a backed-up queue should drop samples rather than accumulate stale ones.
static IMU_STREAM: Channel<CriticalSectionRawMutex, data::telemetry::IMUTelemetry, 2> =
    Channel::new();
static NETWORK_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::NetworkTelemetry, 2> =
    Channel::new();
static AGGREGATED_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::Telemetry, 1> =
    Channel::new();
// Every accepted UWB range, from `range_uwb` to `estimate_pose`. Not shallow like `IMU_STREAM`: a
// range is far rarer (four per superframe, not fifty a second), each one is a real measurement rather
// than a sample superseded a few tens of milliseconds later, and they arrive in bursts of four that
// have to be held together long enough to be fused as one snapshot.
static UWB_RANGES: Channel<CriticalSectionRawMutex, data::localization::RangeMeasurement, 8> =
    Channel::new();
// The same ranges again, forwarded by `estimate_pose` to `publish_uwb_ranges` -- and only when
// `/config/estimation.publish_raw_ranges` is on. A second hop rather than a
// second consumer on `UWB_RANGES`, because a `Channel` hands each message to exactly *one* receiver:
// two tasks reading it would each get half the ranges, which would quietly halve the fix rate.
static UWB_RANGES_RAW: Channel<CriticalSectionRawMutex, data::localization::RangeMeasurement, 8> =
    Channel::new();
// Every IMU sample, at the full 100 Hz sampling rate, from `monitor_imu` to `estimate_pose`. A
// `Watch` rather than a channel: the estimator integrates these to carry the pose between range
// fixes, so a sample it did not get in time is worth nothing to it -- widening the next timestep is
// strictly better than delivering a stale one late. Separate from `IMU_STREAM`, which is the
// decimated 10 Hz copy that goes out over MQTT.
static IMU_FUSION: Watch<CriticalSectionRawMutex, data::telemetry::IMUTelemetry, 1> = Watch::new();
// The forward duty cycle the motor driver last applied, from `manage_motor_controller` to
// `estimate_pose`. With no wheel encoders this is the estimator's only forward-motion input.
static COMMANDED_FORWARD_DUTY: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();
// The fused pose. A `Watch` because a pose is only worth having while it is current, and because the
// motor controller and the display are the obvious next consumers after the publisher.
static POSE_ESTIMATE: Watch<CriticalSectionRawMutex, data::localization::PoseEstimate, 2> =
    Watch::new();
// The arena's anchor geometry and range calibration, delivered retained on `topics::ANCHORS_TOPIC`.
// Fleet-shared hardware, so one message for the whole fleet -- see
// `data::configurations::AnchorsConfiguration`.
static ANCHORS_CONFIG: Watch<
    CriticalSectionRawMutex,
    data::configurations::AnchorsConfiguration,
    1,
> = Watch::new();
// Whether `estimate_pose` fuses ranges with the IMU or publishes the raw UWB trilateration,
// delivered retained on `topics::ESTIMATION_TOPIC`. Fleet-shared, and a `Watch` for the same reason
// as `ANCHORS_CONFIG`: the estimator polls it once per pass and needs the last delivered value back
// every time, not just once.
static ESTIMATION_CONFIG: Watch<
    CriticalSectionRawMutex,
    data::configurations::EstimationConfiguration,
    1,
> = Watch::new();
// The fleet-wide tag-slot override delivered on `topics::TAG_ASSIGNMENTS_TOPIC`. Only one
// consumer (`range_uwb`, which falls back to the compiled `drivers::uwb::tag_id::TAG_ASSIGNMENTS`
// while this holds nothing), but still a `Watch` rather than a `Signal`: `range_uwb` calls
// `try_get()` once per superframe for as long as the firmware runs, and needs the same last
// delivered value back every time, not just once -- a `Signal` hands its value to a single
// `.wait()` and is empty again after, which fits a one-shot request, not a value that has to
// stay readable indefinitely.

static TAG_ASSIGNMENTS_CONFIG: Watch<
    CriticalSectionRawMutex,
    data::configurations::TagAssignmentsConfiguration,
    1,
> = Watch::new();
// Whether the MQTT session is up. Production has five consumers: the telemetry, IMU-stream,
// UWB-range and pose publishers plus the display. A fixture image has a sixth consumer for its
// calibration-sample publisher. Capacity must cover the largest feature combination; `receiver()`
// returns `None` once this limit is exhausted.
static BROKER_STATUS: Watch<CriticalSectionRawMutex, data::mqtt::BrokerStatus, 6> = Watch::new();
// Depth 5, not 4: four publishers (telemetry, IMU stream, UWB ranges, pose) share this queue, and
// each of them documents that it would rather drop a stale sample than let it delay everything queued
// behind it -- a queue too shallow to hold one of each producer's next message defeats that by
// serializing them behind whichever fills it first.
static MQTT_PUBLISH: Channel<CriticalSectionRawMutex, data::mqtt::PublishMessage, 5> =
    Channel::new();
static MQTT_RECEIVE: Channel<CriticalSectionRawMutex, data::mqtt::ReceivedMessage, 2> =
    Channel::new();
static OTA_CHECK_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// Progress of an in-flight OTA update. A `Watch` for the same reason as `NETWORK_READY`:
// two consumers, the display (which shows a progress screen) and the motor controller
// (which cuts the motors for the duration of the update).
static OTA_STATUS: Watch<CriticalSectionRawMutex, data::ota::OtaStatus, 2> = Watch::new();
// Presses of the power button, forwarded to the display: short ones page through the menu,
// a long one is the announcement that the board is cutting its own supply.
static BUTTON_EVENTS: Channel<CriticalSectionRawMutex, data::button::ButtonEvent, 2> =
    Channel::new();
// What the battery charger last reported. Only the display consumes it; the telemetry side
// of the same readings goes out over `BATTERY_TELEMETRY`.
static CHARGER_STATUS: Watch<CriticalSectionRawMutex, data::battery::ChargerStatus, 1> =
    Watch::new();
// The board's single I2C bus, built here because it outlives -- and is shared by -- both of
// the tasks that talk on it.

static MOTOR_COMMAND: Channel<CriticalSectionRawMutex, data::commands::DriveCommand, 2> =
    Channel::new();
// This robot's own motor settings -- EMA smoothing and the duty ceiling -- delivered retained on
// `/config/robots/{ID}`. Per-robot rather than fleet-shared, unlike the three `Watch`es above: they
// describe this chassis, not the arena. A `Watch` for the same reason as `ESTIMATION_CONFIG`: the
// motor controller polls it once per pass and needs the last delivered value back every time.
static MOTORS_CONFIG: Watch<CriticalSectionRawMutex, data::configurations::MotorsConfiguration, 1> =
    Watch::new();
// This robot's UWB residual offset and duty-to-speed calibration, carried in the same retained
// `/config/robots/{ID}` payload as `MOTORS_CONFIG` but watched independently by the pose estimator.
static LOCALIZATION_CONFIG: Watch<
    CriticalSectionRawMutex,
    data::configurations::LocalizationConfiguration,
    1,
> = Watch::new();
static CALIBRATION_COMMANDS: Channel<
    CriticalSectionRawMutex,
    data::calibration::CalibrationCommand,
    2,
> = Channel::new();
static CALIBRATION_ARM: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();
// Physical provisioning makes the motor task de-energize nSLEEP. The acknowledgement is separate
// from commanded duty: zero duty says stationary; this says the bridge is actually disabled.
static CALIBRATION_MOTOR_INTERLOCK: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
static CALIBRATION_MOTOR_SAFE: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();
static CALIBRATION_WRITE_REQUESTS: Channel<
    CriticalSectionRawMutex,
    data::calibration::CalibrationWriteRequest,
    1,
> = Channel::new();
static CALIBRATION_WRITE_RESULTS: Channel<
    CriticalSectionRawMutex,
    data::calibration::CalibrationWriteResult,
    1,
> = Channel::new();
static CALIBRATION_CAPTURE: Watch<
    CriticalSectionRawMutex,
    Option<data::calibration::CalibrationCapture>,
    2,
> = Watch::new();
#[cfg(feature = "calibration-fixture")]
static CALIBRATION_SAMPLES: Channel<
    CriticalSectionRawMutex,
    data::calibration::CalibrationSample,
    8,
> = Channel::new();

static I2C_BUS: StaticCell<BoardI2cBus> = StaticCell::new();

#[ariel_os::spawner(autostart, peripherals)]
fn main(spawner: ariel_os::asynch::Spawner, peripherals: pins::Peripherals) {
    let device_id = device_id::init();
    // Every MQTT topic is namespaced by the device ID, so the whole set is built here, once,
    // and handed to the tasks that use it -- see `topics`.
    let topics = topics::init(device_id);
    info!(
        "firmware: started on {} device_id={}",
        ariel_os::buildinfo::BOARD,
        device_id
    );

    // FLASH has one owner for the whole run. Read the antenna-delay journal before ranging starts,
    // then move the same handle into the OTA/flash task for later provisioning writes.
    let device_id_bytes: [u8; dropbot_calibration::DEVICE_ID_LEN] = device_id
        .as_bytes()
        .try_into()
        .expect("device_id always has six ASCII bytes");
    let mut flash = esp_storage::FlashStorage::new(peripherals.ota.flash);
    let antenna_calibration = drivers::calibration_storage::load(&mut flash, device_id_bytes);
    let antenna_delay = dw3000_hal::AntennaDelay {
        rx: antenna_calibration.record.rx_ticks,
        tx: antenna_calibration.record.tx_ticks,
    };
    info!(
        "calibration: robot antenna delay RX {}, TX {}, generation {}, persisted {}",
        antenna_delay.rx,
        antenna_delay.tx,
        antenna_calibration.record.generation,
        antenna_calibration.persisted,
    );

    // The display, the battery charger and both halves of the IMU share one bus, so it is
    // created here and handed to each of them as a `SharedI2c` handle rather than owned by
    // any one of them.
    let mut i2c_config = hal::i2c::controller::Config::default();
    i2c_config.frequency = const { highest_freq_in(Kilohertz::kHz(100)..=Kilohertz::kHz(400)) };
    let i2c_bus: &'static BoardI2cBus = I2C_BUS.init(Mutex::new(I2cBus::new(
        peripherals.i2c.sda,
        peripherals.i2c.scl,
        i2c_config,
    )));

    // Spawned first: it holds the power latch that keeps the board alive once the user lets
    // go of the button.
    spawner
        .spawn(manage_power_button(
            peripherals.power_management,
            BUTTON_EVENTS.sender(),
            CALIBRATION_ARM.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_motor_controller(
            peripherals.motor_driver,
            MOTOR_TELEMETRY.sender(),
            MOTOR_COMMAND.receiver(),
            OTA_STATUS.receiver().unwrap(),
            COMMANDED_FORWARD_DUTY.sender(),
            MOTORS_CONFIG.receiver().unwrap(),
            CALIBRATION_MOTOR_INTERLOCK.receiver().unwrap(),
            CALIBRATION_MOTOR_SAFE.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_display(
            BoardI2cDevice::new(i2c_bus),
            device_id,
            &NETWORK_STATUS,
            OTA_STATUS.receiver().unwrap(),
            BUTTON_EVENTS.receiver(),
            BROKER_STATUS.receiver().unwrap(),
            CHARGER_STATUS.receiver().unwrap(),
        ))
        .unwrap();
    spawner
        .spawn(monitor_battery(
            SharedI2c::new(i2c_bus),
            BATTERY_TELEMETRY.sender(),
            CHARGER_STATUS.sender(),
        ))
        .unwrap();
    spawner
        .spawn(monitor_imu(
            BoardI2cDevice::new(i2c_bus),
            BoardI2cDevice::new(i2c_bus),
            IMU_STREAM.sender(),
            IMU_TELEMETRY.sender(),
            IMU_FUSION.sender(),
        ))
        .unwrap();
    // Not a `spawner.spawn(...)` like every other task: `range_uwb` runs on its own thread and
    // its own executor, not the main one -- see `tasks::uwb_ranging`'s module docs for why, and
    // for why that means handing its dependencies over rather than spawning it directly here.
    #[cfg(not(feature = "calibration-fixture"))]
    start_uwb_ranging(
        peripherals.uwb,
        device_id,
        UWB_RANGES.sender(),
        TAG_ASSIGNMENTS_CONFIG.receiver().unwrap(),
        antenna_delay,
    );
    #[cfg(feature = "calibration-fixture")]
    start_uwb_fixture(
        peripherals.uwb,
        antenna_delay,
        CALIBRATION_SAMPLES.sender(),
        CALIBRATION_CAPTURE.receiver().unwrap(),
    );
    spawner
        .spawn(aggregate_telemetry(
            MOTOR_TELEMETRY.receiver(),
            BATTERY_TELEMETRY.receiver(),
            IMU_TELEMETRY.receiver(),
            NETWORK_TELEMETRY.receiver(),
            AGGREGATED_TELEMETRY.sender(),
        ))
        .unwrap();
    spawner
        .spawn(network_monitor(&NETWORK_STATUS, NETWORK_READY.sender()))
        .unwrap();
    spawner
        .spawn(mqtt_manager(
            device_id,
            topics,
            NETWORK_READY.receiver().unwrap(),
            BROKER_STATUS.sender(),
            MQTT_PUBLISH.receiver(),
            MQTT_RECEIVE.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_mqtt_client(
            MQTT_RECEIVE.receiver(),
            MOTOR_COMMAND.sender(),
            &OTA_CHECK_REQUEST,
            TAG_ASSIGNMENTS_CONFIG.sender(),
            ANCHORS_CONFIG.sender(),
            ESTIMATION_CONFIG.sender(),
            MOTORS_CONFIG.sender(),
            LOCALIZATION_CONFIG.sender(),
            CALIBRATION_COMMANDS.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_telemetry(
            topics.telemetry(),
            BROKER_STATUS.receiver().unwrap(),
            AGGREGATED_TELEMETRY.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_imu_stream(
            topics.imu_stream(),
            BROKER_STATUS.receiver().unwrap(),
            IMU_STREAM.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    // The sole consumer of `UWB_RANGES`, and the producer of everything downstream of it: the pose,
    // and -- only when `/config/estimation.publish_raw_ranges` is on -- the raw ranges forwarded for
    // calibration. Spawned before its publishers so the first pose it produces has somewhere to go.
    spawner
        .spawn(estimate_pose(
            UWB_RANGES.receiver(),
            IMU_FUSION.receiver().unwrap(),
            COMMANDED_FORWARD_DUTY.receiver().unwrap(),
            ANCHORS_CONFIG.receiver().unwrap(),
            ESTIMATION_CONFIG.receiver().unwrap(),
            LOCALIZATION_CONFIG.receiver().unwrap(),
            CALIBRATION_CAPTURE.receiver().unwrap(),
            POSE_ESTIMATE.sender(),
            UWB_RANGES_RAW.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_pose(
            topics.pose(),
            BROKER_STATUS.receiver().unwrap(),
            POSE_ESTIMATE.receiver().unwrap(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_uwb_ranges(
            topics.uwb(),
            BROKER_STATUS.receiver().unwrap(),
            UWB_RANGES_RAW.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    #[cfg(feature = "calibration-fixture")]
    spawner
        .spawn(publish_calibration_samples(
            topics.calibration_samples(),
            BROKER_STATUS.receiver().unwrap(),
            CALIBRATION_SAMPLES.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_calibration(
            topics.calibration_status(),
            antenna_calibration,
            CALIBRATION_COMMANDS.receiver(),
            CALIBRATION_ARM.receiver(),
            CALIBRATION_MOTOR_INTERLOCK.sender(),
            CALIBRATION_MOTOR_SAFE.receiver(),
            CALIBRATION_WRITE_REQUESTS.sender(),
            CALIBRATION_WRITE_RESULTS.receiver(),
            COMMANDED_FORWARD_DUTY.receiver().unwrap(),
            CALIBRATION_CAPTURE.sender(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_ota(
            NETWORK_READY.receiver().unwrap(),
            &OTA_CHECK_REQUEST,
            flash,
            antenna_calibration,
            CALIBRATION_WRITE_REQUESTS.receiver(),
            CALIBRATION_WRITE_RESULTS.sender(),
            OTA_STATUS.sender(),
        ))
        .unwrap();
}
