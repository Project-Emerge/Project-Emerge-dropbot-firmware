#![no_main]
#![no_std]

mod data;
mod device_id;
mod drivers;
mod pins;
mod tasks;
mod traits;

use ariel_os::gpio::{Level, Output};
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
use crate::tasks::{
    aggregate_telemetry, manage_display, manage_motor_controller, manage_mqtt_client, manage_ota,
    manage_power_button, monitor_battery, monitor_imu, mqtt_manager, network_monitor,
    publish_imu_stream, publish_telemetry,
};

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
// Whether the MQTT session is up. A `Watch` because it has three consumers: the telemetry
// publisher and the IMU stream publisher, which both hold off until the first connection,
// and the display, which shows the broker state on its network page.
static BROKER_STATUS: Watch<CriticalSectionRawMutex, data::mqtt::BrokerStatus, 3> = Watch::new();
static MQTT_PUBLISH: Channel<CriticalSectionRawMutex, data::mqtt::PublishMessage, 2> =
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

static MOTOR_COMMAND: Channel<CriticalSectionRawMutex, data::commands::DriveCommand, 2> = Channel::new();

static I2C_BUS: StaticCell<BoardI2cBus> = StaticCell::new();

#[ariel_os::spawner(autostart, peripherals)]
fn main(spawner: ariel_os::asynch::Spawner, peripherals: pins::Peripherals) {
    let device_id = device_id::init();
    info!(
        "firmware: started on {} device_id={}",
        ariel_os::buildinfo::BOARD,
        device_id
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
        ))
        .unwrap();
    spawner
        .spawn(manage_motor_controller(
            peripherals.motor_driver,
            MOTOR_TELEMETRY.sender(),
            MOTOR_COMMAND.receiver(),
            OTA_STATUS.receiver().unwrap(),
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
        ))
        .unwrap();
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
            NETWORK_READY.receiver().unwrap(),
            BROKER_STATUS.sender(),
            MQTT_PUBLISH.receiver(),
            MQTT_RECEIVE.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_telemetry(
            device_id,
            BROKER_STATUS.receiver().unwrap(),
            AGGREGATED_TELEMETRY.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_imu_stream(
            device_id,
            BROKER_STATUS.receiver().unwrap(),
            IMU_STREAM.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_mqtt_client(
            device_id,
            MQTT_RECEIVE.receiver(),
            MOTOR_COMMAND.sender(),
            &OTA_CHECK_REQUEST,
        ))
        .unwrap();
    spawner
        .spawn(manage_ota(
            NETWORK_READY.receiver().unwrap(),
            &OTA_CHECK_REQUEST,
            peripherals.ota,
            OTA_STATUS.sender(),
        ))
        .unwrap();
}
