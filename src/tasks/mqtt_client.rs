use ariel_os::log::{Debug2Format, error, info};
use heapless::String;
use serde_json::from_str;

use crate::data::commands::DriveCommand;
use crate::task_sync::{MotorCommandTx, MqttReceiveRx, OtaCheckRequestSignal};
use crate::topics::InboundTopic;

/// Messaging endpoints owned by the MQTT command-dispatch task.
pub struct MqttClientPorts {
    pub mqtt_receive: MqttReceiveRx,
    pub motor_commands: MotorCommandTx,
    pub ota_check_request: &'static OtaCheckRequestSignal,
}

#[ariel_os::task]
pub async fn manage_mqtt_client(ports: MqttClientPorts) -> ! {
    loop {
        let message = ports.mqtt_receive.receive().await;
        info!("mqtt: received message on topic {}", message.topic.label());

        // Anything on the broker can put bytes on a subscribed topic, so a non-UTF-8 payload is
        // untrusted input rather than an impossible state. It used to be `.unwrap()`, which panics --
        // and an embedded panic handler halts the whole MCU, so one malformed publish would have
        // stopped the other firmware tasks along with this one.
        let Ok(payload) = String::from_utf8(message.payload) else {
            error!(
                "mqtt: non-UTF-8 payload on topic {}, ignoring",
                message.topic.label()
            );
            continue;
        };

        // `mqtt_manager` has already matched the wire topic against the subscription table,
        // so what arrives here is the topic's meaning rather than its name.
        match message.topic {
            InboundTopic::MotorCommand => match from_str::<DriveCommand>(payload.as_str()) {
                Ok(command) => {
                    info!("mqtt: received motor command: {:?}", Debug2Format(&command));
                    ports.motor_commands.send(command).await;
                }
                Err(e) => {
                    error!(
                        "mqtt: failed to parse motor command: {:?}",
                        Debug2Format(&e)
                    );
                }
            },
            InboundTopic::OtaCheck => {
                info!("mqtt: ota check requested via mqtt");
                ports.ota_check_request.signal(());
            }
        }
    }
}
