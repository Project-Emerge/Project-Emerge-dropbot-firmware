#![no_main]
#![no_std]

mod drivers;
mod pins;
mod traits;
mod data;
mod tasks;

use ariel_os::{asynch::spawner, log::info, time::Timer};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal};
use ariel_os::reexports::embassy_net::Ipv4Address;

use crate::tasks::{manage_motor_controller, manage_display, aggregate_telemetry, publish_telemetry, manage_mqtt_client, mqtt_manager, network_monitor};

pub const TCP_BUFFER_SIZE: usize = 1024;
pub const DEVICE_ID: &str = match option_env!("DEVICE_ID") {
    Some(device_id) => device_id,
    None => "UNSET",
};

pub static NETWORK_STATUS: Signal<CriticalSectionRawMutex, Option<Ipv4Address>> = Signal::new();
pub static NETWORK_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static MOTOR_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::MotorTelemetry, 2> = Channel::new();
pub static BATTERY_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::BatteryTelemetry, 2> = Channel::new();
pub static IMU_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::IMUTelemetry, 2> = Channel::new();
pub static NETWORK_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::NetworkTelemetry, 2> = Channel::new();
pub static AGGREGATED_TELEMETRY: Channel<CriticalSectionRawMutex, data::telemetry::Telemetry, 1> = Channel::new();
pub static MQTT_CONNECTION: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static MQTT_PUBLISH: Channel<CriticalSectionRawMutex, data::mqtt::PublishMessage, 2> = Channel::new();
pub static MQTT_RECEIVE: Channel<CriticalSectionRawMutex, data::mqtt::ReceivedMessage, 2> = Channel::new();

#[ariel_os::task(autostart, peripherals)]
async fn main(peripherals: pins::Peripherals) -> ! {
    info!(
        "firmware: started on {} device_id={}",
        ariel_os::buildinfo::BOARD,
        DEVICE_ID
    );
    spawner()
        .spawn(manage_motor_controller(peripherals.motor_driver))
        .unwrap();
    spawner().spawn(manage_display(peripherals.i2c)).unwrap();
    spawner().spawn(aggregate_telemetry()).unwrap();
    spawner().spawn(network_monitor()).unwrap();
    spawner().spawn(mqtt_manager()).unwrap();
    spawner().spawn(publish_telemetry()).unwrap();
    spawner().spawn(manage_mqtt_client()).unwrap();
    loop {
        Timer::after(ariel_os::time::Duration::from_secs(1)).await;
    }
}
