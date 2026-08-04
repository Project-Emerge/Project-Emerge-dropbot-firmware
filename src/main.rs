#![no_main]
#![no_std]

mod data;
mod drivers;
mod pins;
mod tasks;
mod traits;

use ariel_os::gpio::Output;
use ariel_os::reexports::embassy_net::Ipv4Address;
use ariel_os::{asynch::spawner, log::info, time::Timer};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal, watch::Watch,
};

use crate::tasks::{
    aggregate_telemetry, manage_display, manage_motor_controller, manage_mqtt_client, manage_ota,
    mqtt_manager, network_monitor, publish_telemetry,
};

pub const TCP_BUFFER_SIZE: usize = 1024;
pub const DEVICE_ID: &str = match option_env!("DEVICE_ID") {
    Some(device_id) => device_id,
    None => "UNSET",
};
/// Version this firmware build identifies as to the OTA server; see `.cargo/config.toml`.
pub const FIRMWARE_VERSION: &str = ariel_os::config::str_from_env_or!(
    "FIRMWARE_VERSION",
    "0.0.0",
    "semantic version of this firmware build, compared against the OTA server's manifest",
);

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
static NETWORK_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::NetworkTelemetry, 2> =
    Channel::new();
static AGGREGATED_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::Telemetry, 1> =
    Channel::new();
static MQTT_CONNECTION: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static MQTT_PUBLISH: Channel<CriticalSectionRawMutex, data::mqtt::PublishMessage, 2> =
    Channel::new();
static MQTT_RECEIVE: Channel<CriticalSectionRawMutex, data::mqtt::ReceivedMessage, 2> =
    Channel::new();
static OTA_CHECK_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// Progress of an in-flight OTA update. A `Watch` for the same reason as `NETWORK_READY`:
// two consumers, the display (which shows a progress screen) and the motor controller
// (which cuts the motors for the duration of the update).
static OTA_STATUS: Watch<CriticalSectionRawMutex, data::ota::OtaStatus, 2> = Watch::new();

#[ariel_os::spawner(autostart, peripherals)]
fn main(spawner: ariel_os::asynch::Spawner, peripherals: pins::Peripherals) {
    info!(
        "firmware: started on {} device_id={}",
        ariel_os::buildinfo::BOARD,
        DEVICE_ID
    );

    let mut latch_pin = Output::new(peripherals.power_management.kill, ariel_os::gpio::Level::High);
    latch_pin.set_high();

    spawner
        .spawn(manage_motor_controller(
            peripherals.motor_driver,
            MOTOR_TELEMETRY.sender(),
            OTA_STATUS.receiver().unwrap(),
        ))
        .unwrap();
    spawner
        .spawn(manage_display(
            peripherals.i2c,
            &NETWORK_STATUS,
            OTA_STATUS.receiver().unwrap(),
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
            NETWORK_READY.receiver().unwrap(),
            &MQTT_CONNECTION,
            MQTT_PUBLISH.receiver(),
            MQTT_RECEIVE.sender(),
        ))
        .unwrap();
    spawner
        .spawn(publish_telemetry(
            &MQTT_CONNECTION,
            AGGREGATED_TELEMETRY.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner
        .spawn(manage_mqtt_client(
            MQTT_RECEIVE.receiver(),
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
