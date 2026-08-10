use core::fmt::Write;
use ariel_os::log::{Debug2Format, error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::signal::Signal;
use heapless::String;
use serde_json::from_str;

use crate::data;
use crate::data::commands::DriveCommand;
use crate::data::mqtt::ReceivedMessage;

#[ariel_os::task]
pub async fn manage_mqtt_client(
    device_id: &'static str,
    mqtt_receive_rx: Receiver<'static, CriticalSectionRawMutex, ReceivedMessage, 2>,
    motor_command_tx: Sender<'static, CriticalSectionRawMutex, data::commands::DriveCommand, 2>,
    ota_check_request: &'static Signal<CriticalSectionRawMutex, ()>,
) -> ! {
    let mut ota_check_request_topic = String::<18>::new();
    write!(ota_check_request_topic, "/ota/check/{}", device_id).unwrap();
    let mut motors_command_topic = String::<16>::new();
    write!(motors_command_topic, "/motors/{}", device_id).unwrap();

    loop {
        let message = mqtt_receive_rx.receive().await;
        info!(
            "mqtt: received message on topic {}",
            message.topic.as_str(),
        );

        if message.topic.as_str() == motors_command_topic {
            // Handle motor commands
            match from_str::<DriveCommand>(String::from_utf8(message.payload).unwrap().as_str()) {
                Ok(command) => {
                    info!("mqtt: received motor command: {:?}", Debug2Format(&command));
                    motor_command_tx.send(command).await;
                }
                Err(e) => {
                    error!("mqtt: failed to parse motor command: {:?}", Debug2Format(&e));
                }
            }
        }

        if message.topic.as_str() == ota_check_request_topic {
            info!("mqtt: ota check requested via mqtt");
            ota_check_request.signal(());
            continue;
        }
    }
}
