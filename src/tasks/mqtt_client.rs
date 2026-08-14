use ariel_os::log::{Debug2Format, error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_sync::watch::Sender as WatchSender;
use heapless::String;
use serde_json::from_str;

use crate::data;
use crate::data::calibration::CalibrationCommand;
use crate::data::commands::DriveCommand;
use crate::data::configurations::{
    AnchorsConfiguration, EstimationConfiguration, LocalizationConfiguration, MotorsConfiguration,
    RobotConfiguration, TagAssignmentsConfiguration,
};
use crate::data::mqtt::ReceivedMessage;
use crate::topics::InboundTopic;

#[ariel_os::task]
pub async fn manage_mqtt_client(
    mqtt_receive_rx: Receiver<'static, CriticalSectionRawMutex, ReceivedMessage, 2>,
    motor_command_tx: Sender<'static, CriticalSectionRawMutex, data::commands::DriveCommand, 2>,
    ota_check_request: &'static Signal<CriticalSectionRawMutex, ()>,
    tag_assignments_tx: WatchSender<
        'static,
        CriticalSectionRawMutex,
        TagAssignmentsConfiguration,
        1,
    >,
    anchors_tx: WatchSender<'static, CriticalSectionRawMutex, AnchorsConfiguration, 1>,
    estimation_tx: WatchSender<'static, CriticalSectionRawMutex, EstimationConfiguration, 1>,
    motors_config_tx: WatchSender<'static, CriticalSectionRawMutex, MotorsConfiguration, 1>,
    localization_config_tx: WatchSender<
        'static,
        CriticalSectionRawMutex,
        LocalizationConfiguration,
        1,
    >,
    calibration_command_tx: Sender<'static, CriticalSectionRawMutex, CalibrationCommand, 2>,
) -> ! {
    loop {
        let message = mqtt_receive_rx.receive().await;
        info!("mqtt: received message on topic {}", message.topic.label());

        // Anything on the broker can put bytes on a subscribed topic, so a non-UTF-8 payload is
        // untrusted input rather than an impossible state. It used to be `.unwrap()`, which panics --
        // and an embedded panic handler halts the whole MCU, so one malformed publish would have
        // stopped the motors, the display and ranging along with this task.
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
                    motor_command_tx.send(command).await;
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
                ota_check_request.signal(());
            }
            InboundTopic::TagAssignments => {
                match from_str::<TagAssignmentsConfiguration>(payload.as_str()) {
                    Ok(config) => {
                        info!(
                            "mqtt: received tag assignments ({} entries)",
                            config.assignments.len()
                        );
                        tag_assignments_tx.send(config);
                    }
                    Err(e) => {
                        error!(
                            "mqtt: failed to parse tag assignments: {:?}",
                            Debug2Format(&e)
                        );
                    }
                }
            }
            InboundTopic::Anchors => match from_str::<AnchorsConfiguration>(payload.as_str()) {
                Ok(config) => {
                    info!(
                        "mqtt: received anchor geometry ({} entries, robot antenna at {} m)",
                        config.anchors.len(),
                        config.robot_antenna_height_m
                    );
                    anchors_tx.send(config);
                }
                Err(e) => {
                    error!(
                        "mqtt: failed to parse anchor geometry: {:?}",
                        Debug2Format(&e)
                    );
                }
            },
            InboundTopic::Estimation => {
                match from_str::<EstimationConfiguration>(payload.as_str()) {
                    Ok(config) => {
                        let config = config.sanitized();
                        info!(
                            "mqtt: estimation config (fusion {}, raw {}, sigma {} m)",
                            if config.fusion_enabled {
                                "enabled"
                            } else {
                                "disabled"
                            },
                            if config.publish_raw_ranges {
                                "enabled"
                            } else {
                                "disabled"
                            },
                            config.filter.range_sigma_m,
                        );
                        estimation_tx.send(config);
                    }
                    Err(e) => {
                        error!(
                            "mqtt: failed to parse estimation config: {:?}",
                            Debug2Format(&e)
                        );
                    }
                }
            }
            InboundTopic::RobotConfig => match from_str::<RobotConfiguration>(payload.as_str()) {
                Ok(config) => {
                    // Sanitized here as well as in the motor controller: this is the log line an
                    // operator reads back after saving from the dashboard, so it should report the
                    // values that will actually be applied rather than the ones that were sent.
                    let motors = config.motors.sanitized();
                    let localization = config.localization.sanitized();
                    info!(
                        "mqtt: robot config (max duty {}, ema {:?}, UWB offset {} mm, full-duty {} m/s)",
                        motors.max_speed,
                        Debug2Format(&motors.ema_filter_alpha),
                        localization.range_offset_mm,
                        localization.full_duty_speed_m_s,
                    );
                    motors_config_tx.send(motors);
                    localization_config_tx.send(localization);
                }
                Err(e) => {
                    error!("mqtt: failed to parse robot config: {:?}", Debug2Format(&e));
                }
            },
            InboundTopic::CalibrationCommand => {
                match from_str::<CalibrationCommand>(payload.as_str()) {
                    Ok(command) => {
                        info!("mqtt: calibration command received");
                        if calibration_command_tx.try_send(command).is_err() {
                            error!("mqtt: calibration command queue full");
                        }
                    }
                    Err(e) => error!(
                        "mqtt: failed to parse calibration command: {:?}",
                        Debug2Format(&e)
                    ),
                }
            }
        }
    }
}
