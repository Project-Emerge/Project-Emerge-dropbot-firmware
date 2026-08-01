#![no_main]
#![no_std]

mod data;
mod drivers;
mod pins;
mod tasks;
mod traits;

use ariel_os::reexports::embassy_net::Ipv4Address;
use ariel_os::{asynch::spawner, log::info, time::Timer};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};

use crate::tasks::{
    aggregate_telemetry, manage_display, manage_motor_controller, manage_mqtt_client, mqtt_manager,
    network_monitor, publish_telemetry,
};

pub const TCP_BUFFER_SIZE: usize = 1024;
pub const DEVICE_ID: &str = match option_env!("DEVICE_ID") {
    Some(device_id) => device_id,
    None => "UNSET",
};

// Wiring between tasks: each primitive lives here, scoped to this module, and is handed out
// as an explicit `spawn(...)` argument below rather than imported ambiently via `crate::FOO`.
// This keeps the task graph readable in one place and lets a `Channel` split into a `Sender`
// (given to the producer) and a `Receiver` (given to the consumer) instead of a shared handle
// both sides could call `.send()`/`.receive()` on.
static NETWORK_STATUS: Signal<CriticalSectionRawMutex, Option<Ipv4Address>> = Signal::new();
static NETWORK_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
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

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) -> ! {
    info!(
        "firmware: started on {} device_id={}",
        ariel_os::buildinfo::BOARD,
        DEVICE_ID
    );
    spawner()
        .spawn(manage_motor_controller(
            peripherals.motor_driver,
            MOTOR_TELEMETRY.sender(),
        ))
        .unwrap();
    spawner()
        .spawn(manage_display(peripherals.i2c, &NETWORK_STATUS))
        .unwrap();
    spawner()
        .spawn(aggregate_telemetry(
            MOTOR_TELEMETRY.receiver(),
            BATTERY_TELEMETRY.receiver(),
            IMU_TELEMETRY.receiver(),
            NETWORK_TELEMETRY.receiver(),
            AGGREGATED_TELEMETRY.sender(),
        ))
        .unwrap();
    spawner()
        .spawn(network_monitor(&NETWORK_STATUS, &NETWORK_READY))
        .unwrap();
    spawner()
        .spawn(mqtt_manager(
            &NETWORK_READY,
            &MQTT_CONNECTION,
            MQTT_PUBLISH.receiver(),
            MQTT_RECEIVE.sender(),
        ))
        .unwrap();
    spawner()
        .spawn(publish_telemetry(
            &MQTT_CONNECTION,
            AGGREGATED_TELEMETRY.receiver(),
            MQTT_PUBLISH.sender(),
        ))
        .unwrap();
    spawner()
        .spawn(manage_mqtt_client(MQTT_RECEIVE.receiver()))
        .unwrap();
    loop {
        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}
