use ariel_os::log::{error, info};
use ariel_os::time::{Duration, Instant, Timer};
use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender};

use crate::data::calibration::{
    CalibrationCapture, CalibrationCommand, CalibrationStatus, CalibrationWriteRequest,
    CalibrationWriteResult,
};
use crate::data::mqtt::PublishMessage;
use crate::drivers::calibration_storage::LoadedCalibration;

const ARM_WINDOW: Duration = Duration::from_secs(60);
const IDLE_TICK: Duration = Duration::from_secs(1);
const NOMINAL: u16 = dropbot_calibration::NOMINAL_DELAY_TICKS;

#[ariel_os::task]
#[expect(
    clippy::too_many_arguments,
    reason = "explicit calibration task wiring"
)]
pub async fn manage_calibration(
    status_topic: &'static str,
    initial: LoadedCalibration,
    commands: Receiver<'static, CriticalSectionRawMutex, CalibrationCommand, 2>,
    physical_arm: Receiver<'static, CriticalSectionRawMutex, (), 1>,
    motor_interlock: WatchSender<'static, CriticalSectionRawMutex, bool, 1>,
    motor_safe: Receiver<'static, CriticalSectionRawMutex, (), 1>,
    write_requests: Sender<'static, CriticalSectionRawMutex, CalibrationWriteRequest, 1>,
    write_results: Receiver<'static, CriticalSectionRawMutex, CalibrationWriteResult, 1>,
    mut motor_duty: WatchReceiver<'static, CriticalSectionRawMutex, f32, 2>,
    capture: WatchSender<'static, CriticalSectionRawMutex, Option<CalibrationCapture>, 2>,
    mqtt_publish: Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) -> ! {
    let mut armed_until: Option<Instant> = None;
    let mut active_capture: Option<CalibrationCapture> = None;
    capture.send(None);
    motor_interlock.send(false);

    loop {
        let timer_at = match (armed_until, active_capture) {
            (Some(armed), Some(capture)) => {
                let capture_at = Instant::from_micros(capture.expires_at_us);
                if armed < capture_at {
                    armed
                } else {
                    capture_at
                }
            }
            (Some(armed), None) => armed,
            (None, Some(capture)) => Instant::from_micros(capture.expires_at_us),
            (None, None) => Instant::now() + IDLE_TICK,
        };

        match select3(
            commands.receive(),
            physical_arm.receive(),
            Timer::at(timer_at),
        )
        .await
        {
            Either3::Second(()) => {
                // Do not call the window armed until the motor task has lowered DRV8833 nSLEEP.
                motor_interlock.send(true);
                motor_safe.receive().await;
                armed_until = Some(Instant::now() + ARM_WINDOW);
                info!("calibration: physical provisioning window armed for 60 seconds");
                publish_status(
                    status_topic,
                    CalibrationStatus::Armed {
                        window_s: 60,
                        current_generation: initial.record.generation,
                        current_rx_ticks: initial.record.rx_ticks,
                        current_tx_ticks: initial.record.tx_ticks,
                    },
                    &mqtt_publish,
                )
                .await;
            }
            Either3::Third(()) => {
                let now = Instant::now();
                if armed_until.is_some_and(|deadline| now >= deadline) {
                    armed_until = None;
                    motor_interlock.send(false);
                    info!("calibration: physical provisioning window expired");
                }
                if active_capture.is_some_and(|window| now.as_micros() >= window.expires_at_us) {
                    let session_id = active_capture.take().unwrap().session_id;
                    capture.send(None);
                    info!("calibration: capture session {} finished", session_id);
                    publish_status(
                        status_topic,
                        CalibrationStatus::CaptureFinished { session_id },
                        &mqtt_publish,
                    )
                    .await;
                }
            }
            Either3::First(command) => match command {
                CalibrationCommand::StartCapture {
                    session_id,
                    duration_s,
                } => {
                    if !motors_stopped(&mut motor_duty) {
                        reject(status_topic, session_id, "motors_active", &mqtt_publish).await;
                        continue;
                    }
                    let duration_s = duration_s.clamp(5, 600);
                    let window = CalibrationCapture {
                        session_id,
                        expires_at_us: (Instant::now()
                            + Duration::from_secs(u64::from(duration_s)))
                        .as_micros(),
                    };
                    active_capture = Some(window);
                    capture.send(Some(window));
                    info!(
                        "calibration: capture session {} started for {} seconds",
                        session_id, duration_s
                    );
                    publish_status(
                        status_topic,
                        CalibrationStatus::CaptureStarted {
                            session_id,
                            duration_s,
                        },
                        &mqtt_publish,
                    )
                    .await;
                }
                CalibrationCommand::ApplyRobotDelay {
                    session_id,
                    rx_ticks,
                    tx_ticks,
                } => {
                    if !is_armed(armed_until) {
                        reject(
                            status_topic,
                            session_id,
                            "physical_confirmation_required",
                            &mqtt_publish,
                        )
                        .await;
                        continue;
                    }
                    if !motors_stopped(&mut motor_duty) {
                        reject(status_topic, session_id, "motors_active", &mqtt_publish).await;
                        continue;
                    }
                    if dropbot_calibration::AntennaDelayRecord::new(
                        initial.record.device_id,
                        initial.record.generation.saturating_add(1),
                        rx_ticks,
                        tx_ticks,
                    )
                    .is_err()
                    {
                        reject(
                            status_topic,
                            session_id,
                            "delay_out_of_range",
                            &mqtt_publish,
                        )
                        .await;
                        continue;
                    }
                    apply(
                        status_topic,
                        CalibrationWriteRequest {
                            session_id,
                            rx_ticks,
                            tx_ticks,
                        },
                        &write_requests,
                        &write_results,
                        &mqtt_publish,
                    )
                    .await;
                }
                CalibrationCommand::ClearRobotDelay { session_id } => {
                    if !is_armed(armed_until) {
                        reject(
                            status_topic,
                            session_id,
                            "physical_confirmation_required",
                            &mqtt_publish,
                        )
                        .await;
                        continue;
                    }
                    if !motors_stopped(&mut motor_duty) {
                        reject(status_topic, session_id, "motors_active", &mqtt_publish).await;
                        continue;
                    }
                    apply(
                        status_topic,
                        CalibrationWriteRequest {
                            session_id,
                            rx_ticks: NOMINAL,
                            tx_ticks: NOMINAL,
                        },
                        &write_requests,
                        &write_results,
                        &mqtt_publish,
                    )
                    .await;
                }
            },
        }
    }
}

fn is_armed(deadline: Option<Instant>) -> bool {
    deadline.is_some_and(|deadline| Instant::now() < deadline)
}

fn motors_stopped(
    motor_duty: &mut WatchReceiver<'static, CriticalSectionRawMutex, f32, 2>,
) -> bool {
    motor_duty.try_get().unwrap_or(0.0).abs() < 0.01
}

async fn apply(
    topic: &'static str,
    request: CalibrationWriteRequest,
    writes: &Sender<'static, CriticalSectionRawMutex, CalibrationWriteRequest, 1>,
    results: &Receiver<'static, CriticalSectionRawMutex, CalibrationWriteResult, 1>,
    mqtt: &Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) {
    writes.send(request).await;
    match results.receive().await {
        CalibrationWriteResult::Saved {
            session_id,
            generation,
            rx_ticks,
            tx_ticks,
        } => {
            publish_status(
                topic,
                CalibrationStatus::AppliedPendingReboot {
                    session_id,
                    generation,
                    rx_ticks,
                    tx_ticks,
                },
                mqtt,
            )
            .await;
            Timer::after(Duration::from_secs(1)).await;
            esp_hal::system::software_reset();
        }
        CalibrationWriteResult::Failed { session_id } => {
            reject(topic, session_id, "flash_write_failed", mqtt).await;
        }
    }
}

async fn reject(
    topic: &'static str,
    session_id: u32,
    reason: &'static str,
    mqtt: &Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) {
    info!("calibration: rejected session {}: {}", session_id, reason);
    publish_status(
        topic,
        CalibrationStatus::Rejected { session_id, reason },
        mqtt,
    )
    .await;
}

async fn publish_status(
    topic: &'static str,
    status: CalibrationStatus,
    mqtt: &Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) {
    let Ok(payload) = serde_json::to_vec(&status) else {
        error!("calibration: status serialization failed");
        return;
    };
    let mut message = PublishMessage {
        topic,
        payload: heapless::Vec::new(),
    };
    if message.payload.extend_from_slice(&payload).is_err() {
        error!("calibration: status payload too large");
        return;
    }
    mqtt.send(message).await;
}
