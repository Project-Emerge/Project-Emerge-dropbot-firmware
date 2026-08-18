//! Typed coordination contracts shared by the firmware tasks.
//!
//! The concrete primitives still live in `main`, where the complete task graph is wired.
//! These aliases keep queue depths, watcher counts and endpoint directions consistent without
//! giving tasks ambient access to synchronization objects they do not own.

use ariel_os::reexports::embassy_net::Ipv4Address;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender, Watch};

use crate::data::battery::ChargerStatus;
use crate::data::commands::DriveCommand;
use crate::data::configurations::OtaConfiguration;
use crate::data::mqtt::{BrokerStatus, PublishMessage, ReceivedMessage};
use crate::data::ota::OtaStatus;
use crate::data::power::{PowerEvent, ShutdownReason};
use crate::data::telemetry::{
    BatteryTelemetry, IMUTelemetry, MotorTelemetry, NetworkTelemetry, Telemetry,
};

type TaskMutex = CriticalSectionRawMutex;

pub type NetworkStatusSignal = Signal<TaskMutex, Option<Ipv4Address>>;

pub type NetworkReadyWatch = Watch<TaskMutex, (), 2>;
pub type NetworkReadyTx = WatchSender<'static, TaskMutex, (), 2>;
pub type NetworkReadyRx = WatchReceiver<'static, TaskMutex, (), 2>;

pub type MotorTelemetryChannel = Channel<TaskMutex, MotorTelemetry, 2>;
pub type MotorTelemetryTx = Sender<'static, TaskMutex, MotorTelemetry, 2>;
pub type MotorTelemetryRx = Receiver<'static, TaskMutex, MotorTelemetry, 2>;

pub type BatteryTelemetryChannel = Channel<TaskMutex, BatteryTelemetry, 2>;
pub type BatteryTelemetryTx = Sender<'static, TaskMutex, BatteryTelemetry, 2>;
pub type BatteryTelemetryRx = Receiver<'static, TaskMutex, BatteryTelemetry, 2>;

pub type ImuTelemetryChannel = Channel<TaskMutex, IMUTelemetry, 2>;
pub type ImuTelemetryTx = Sender<'static, TaskMutex, IMUTelemetry, 2>;
pub type ImuTelemetryRx = Receiver<'static, TaskMutex, IMUTelemetry, 2>;

pub type ImuStreamChannel = Channel<TaskMutex, IMUTelemetry, 2>;
pub type ImuStreamTx = Sender<'static, TaskMutex, IMUTelemetry, 2>;
pub type ImuStreamRx = Receiver<'static, TaskMutex, IMUTelemetry, 2>;

pub type NetworkTelemetryChannel = Channel<TaskMutex, NetworkTelemetry, 2>;
// Reserved for the currently unimplemented producer; the placeholder queue remains part of the
// telemetry graph so this structural refactor does not change the published payload behavior.
#[allow(dead_code)]
pub type NetworkTelemetryTx = Sender<'static, TaskMutex, NetworkTelemetry, 2>;
pub type NetworkTelemetryRx = Receiver<'static, TaskMutex, NetworkTelemetry, 2>;

pub type AggregatedTelemetryChannel = Channel<TaskMutex, Telemetry, 1>;
pub type AggregatedTelemetryTx = Sender<'static, TaskMutex, Telemetry, 1>;
pub type AggregatedTelemetryRx = Receiver<'static, TaskMutex, Telemetry, 1>;

pub type BrokerStatusWatch = Watch<TaskMutex, BrokerStatus, 5>;
pub type BrokerStatusTx = WatchSender<'static, TaskMutex, BrokerStatus, 5>;
pub type BrokerStatusRx = WatchReceiver<'static, TaskMutex, BrokerStatus, 5>;

pub type MqttPublishChannel = Channel<TaskMutex, PublishMessage, 5>;
pub type MqttPublishTx = Sender<'static, TaskMutex, PublishMessage, 5>;
pub type MqttPublishRx = Receiver<'static, TaskMutex, PublishMessage, 5>;

pub type MqttReceiveChannel = Channel<TaskMutex, ReceivedMessage, 2>;
pub type MqttReceiveTx = Sender<'static, TaskMutex, ReceivedMessage, 2>;
pub type MqttReceiveRx = Receiver<'static, TaskMutex, ReceivedMessage, 2>;

pub type OtaCheckRequestSignal = Signal<TaskMutex, ()>;

pub type OtaConfigurationWatch = Watch<TaskMutex, OtaConfiguration, 1>;
pub type OtaConfigurationTx = WatchSender<'static, TaskMutex, OtaConfiguration, 1>;
pub type OtaConfigurationRx = WatchReceiver<'static, TaskMutex, OtaConfiguration, 1>;

pub type OtaStatusWatch = Watch<TaskMutex, OtaStatus, 2>;
pub type OtaStatusTx = WatchSender<'static, TaskMutex, OtaStatus, 2>;
pub type OtaStatusRx = WatchReceiver<'static, TaskMutex, OtaStatus, 2>;

pub type PowerEventChannel = Channel<TaskMutex, PowerEvent, 2>;
pub type PowerEventTx = Sender<'static, TaskMutex, PowerEvent, 2>;
pub type PowerEventRx = Receiver<'static, TaskMutex, PowerEvent, 2>;

pub type ShutdownRequestChannel = Channel<TaskMutex, ShutdownReason, 1>;
pub type ShutdownRequestTx = Sender<'static, TaskMutex, ShutdownReason, 1>;
pub type ShutdownRequestRx = Receiver<'static, TaskMutex, ShutdownReason, 1>;

pub type ChargerStatusWatch = Watch<TaskMutex, ChargerStatus, 1>;
pub type ChargerStatusTx = WatchSender<'static, TaskMutex, ChargerStatus, 1>;
pub type ChargerStatusRx = WatchReceiver<'static, TaskMutex, ChargerStatus, 1>;

pub type MotorCommandChannel = Channel<TaskMutex, DriveCommand, 2>;
pub type MotorCommandTx = Sender<'static, TaskMutex, DriveCommand, 2>;
pub type MotorCommandRx = Receiver<'static, TaskMutex, DriveCommand, 2>;
