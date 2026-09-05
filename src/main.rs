#![no_std]
#![no_main]

mod data;
mod device_id;
mod drivers;
mod pins;
mod task_sync;
mod tasks;
mod topics;
mod traits;

use ariel_os::hal;
use ariel_os::i2c::controller::{Kilohertz, highest_freq_in};
use ariel_os::log::info;
use ariel_os::reexports::static_cell::StaticCell;
use embassy_sync::mutex::Mutex;

use crate::drivers::shared_i2c::{BoardI2cBus, BoardI2cDevice, SharedI2c};
use crate::pins::I2cBus;
use crate::task_sync::{
    AggregatedTelemetryChannel, BatteryTelemetryChannel, BrokerStatusWatch, ChargerStatusWatch,
    ImuStreamChannel, ImuTelemetryChannel, MotorCommandChannel, MotorConfigurationChannel,
    MotorTelemetryChannel, MqttPublishChannel, MqttReceiveChannel, NetworkReadyWatch,
    NetworkStatusSignal, NetworkTelemetryChannel, OtaCheckRequestSignal, OtaConfigurationWatch,
    OtaStatusWatch, PowerEventChannel, ShutdownRequestChannel,
};
use crate::tasks::{
    BatteryMonitorPorts, DisplayPorts, ImuMonitorPorts, ImuPublisherPorts, MotorControllerPorts,
    MqttClientPorts, MqttManagerPorts, NetworkMonitorPorts, OtaManagerPorts, PowerManagerPorts,
    TelemetryAggregatorPorts, TelemetryPublisherPorts, aggregate_telemetry, manage_display,
    manage_motor_controller, manage_mqtt_client, manage_ota, manage_power_button, monitor_battery,
    monitor_imu, mqtt_manager, network_monitor, publish_imu_stream, publish_telemetry,
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
static NETWORK_STATUS: NetworkStatusSignal = NetworkStatusSignal::new();
// Gates every task that must not run before the network is up. This is a `Watch`, not a
// `Signal`, because it has more than one consumer: a `Signal` holds a single waker and a
// single value, so two waiters replace each other's waker on every poll and wake each other
// forever. That livelock starves lower-priority threads -- including esp-radio's timer
// thread, which Wi-Fi association depends on -- so Wi-Fi never connects.
static NETWORK_READY: NetworkReadyWatch = NetworkReadyWatch::new();
static MOTOR_TELEMETRY: MotorTelemetryChannel = MotorTelemetryChannel::new();
static BATTERY_TELEMETRY: BatteryTelemetryChannel = BatteryTelemetryChannel::new();
static IMU_TELEMETRY: ImuTelemetryChannel = ImuTelemetryChannel::new();
// The IMU's streaming path, separate from `IMU_TELEMETRY` above: this one carries samples to
// their own MQTT topic, while `IMU_TELEMETRY` trickles the same payload into the once-a-second
// status bundle. Deliberately shallow -- a stream sample is only worth sending while it is
// current, so a backed-up queue should drop samples rather than accumulate stale ones.
static IMU_STREAM: ImuStreamChannel = ImuStreamChannel::new();
static NETWORK_TELEMETRY: NetworkTelemetryChannel = NetworkTelemetryChannel::new();
static AGGREGATED_TELEMETRY: AggregatedTelemetryChannel = AggregatedTelemetryChannel::new();
// Whether the MQTT session is up. A `Watch` because it has several consumers: the telemetry,
// IMU-stream publisher and display, which all hold off until the first connection or show its state.
static BROKER_STATUS: BrokerStatusWatch = BrokerStatusWatch::new();
// The publishers share this queue and drop stale samples rather than delaying newer data behind them.
static MQTT_PUBLISH: MqttPublishChannel = MqttPublishChannel::new();
static MQTT_RECEIVE: MqttReceiveChannel = MqttReceiveChannel::new();
static OTA_CHECK_REQUEST: OtaCheckRequestSignal = OtaCheckRequestSignal::new();
// The retained fleet-wide MQTT configuration is the sole source of the OTA server address.
static OTA_CONFIGURATION: OtaConfigurationWatch = OtaConfigurationWatch::new();
// Progress of an in-flight OTA update. A `Watch` for the same reason as `NETWORK_READY`:
// two consumers, the display (which shows a progress screen) and the motor controller
// (which cuts the motors for the duration of the update).
static OTA_STATUS: OtaStatusWatch = OtaStatusWatch::new();
// User-interface events from the task that owns the power button and latch. Ordinary page
// changes may be dropped, while terminal shutdown events are awaited by their producer.
static POWER_EVENTS: PowerEventChannel = PowerEventChannel::new();
// Requests from monitors that need the power-latch owner to shut the board down. One pending
// request is sufficient because shutdown is terminal.
static SHUTDOWN_REQUESTS: ShutdownRequestChannel = ShutdownRequestChannel::new();
// What the battery charger last reported. Only the display consumes it; the telemetry side
// of the same readings goes out over `BATTERY_TELEMETRY`.
static CHARGER_STATUS: ChargerStatusWatch = ChargerStatusWatch::new();

static MOTOR_COMMAND: MotorCommandChannel = MotorCommandChannel::new();
static MOTOR_CONFIGURATION: MotorConfigurationChannel = MotorConfigurationChannel::new();

// The board's single I2C bus, built here because it outlives -- and is shared by -- the
// display, battery and IMU tasks.
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
            PowerManagerPorts {
                power_events: POWER_EVENTS.sender(),
                shutdown_requests: SHUTDOWN_REQUESTS.receiver(),
            },
        ))
        .unwrap();
    spawner
        .spawn(manage_motor_controller(
            peripherals.motor_driver,
            MotorControllerPorts {
                motor_telemetry: MOTOR_TELEMETRY.sender(),
                motor_commands: MOTOR_COMMAND.receiver(),
                motor_configurations: MOTOR_CONFIGURATION.receiver(),
                ota_status: OTA_STATUS.receiver().unwrap(),
            },
        ))
        .unwrap();
    spawner
        .spawn(manage_display(
            BoardI2cDevice::new(i2c_bus),
            device_id,
            DisplayPorts {
                network_status: &NETWORK_STATUS,
                ota_status: OTA_STATUS.receiver().unwrap(),
                power_events: POWER_EVENTS.receiver(),
                broker_status: BROKER_STATUS.receiver().unwrap(),
                charger_status: CHARGER_STATUS.receiver().unwrap(),
            },
        ))
        .unwrap();
    spawner
        .spawn(monitor_battery(
            SharedI2c::new(i2c_bus),
            BatteryMonitorPorts {
                battery_telemetry: BATTERY_TELEMETRY.sender(),
                charger_status: CHARGER_STATUS.sender(),
                shutdown_requests: SHUTDOWN_REQUESTS.sender(),
            },
        ))
        .unwrap();
    spawner
        .spawn(monitor_imu(
            BoardI2cDevice::new(i2c_bus),
            BoardI2cDevice::new(i2c_bus),
            ImuMonitorPorts {
                imu_stream: IMU_STREAM.sender(),
                imu_telemetry: IMU_TELEMETRY.sender(),
            },
        ))
        .unwrap();
    spawner
        .spawn(aggregate_telemetry(TelemetryAggregatorPorts {
            motor_telemetry: MOTOR_TELEMETRY.receiver(),
            battery_telemetry: BATTERY_TELEMETRY.receiver(),
            imu_telemetry: IMU_TELEMETRY.receiver(),
            network_telemetry: NETWORK_TELEMETRY.receiver(),
            aggregated_telemetry: AGGREGATED_TELEMETRY.sender(),
        }))
        .unwrap();
    spawner
        .spawn(network_monitor(NetworkMonitorPorts {
            network_status: &NETWORK_STATUS,
            network_ready: NETWORK_READY.sender(),
        }))
        .unwrap();
    spawner
        .spawn(mqtt_manager(
            device_id,
            topics,
            MqttManagerPorts {
                network_ready: NETWORK_READY.receiver().unwrap(),
                broker_status: BROKER_STATUS.sender(),
                mqtt_publish: MQTT_PUBLISH.receiver(),
                mqtt_receive: MQTT_RECEIVE.sender(),
            },
        ))
        .unwrap();
    spawner
        .spawn(manage_mqtt_client(MqttClientPorts {
            mqtt_receive: MQTT_RECEIVE.receiver(),
            motor_commands: MOTOR_COMMAND.sender(),
            motor_configurations: MOTOR_CONFIGURATION.sender(),
            ota_check_request: &OTA_CHECK_REQUEST,
            ota_configuration: OTA_CONFIGURATION.sender(),
        }))
        .unwrap();
    spawner
        .spawn(publish_telemetry(
            topics.telemetry(),
            TelemetryPublisherPorts {
                broker_status: BROKER_STATUS.receiver().unwrap(),
                aggregated_telemetry: AGGREGATED_TELEMETRY.receiver(),
                mqtt_publish: MQTT_PUBLISH.sender(),
            },
        ))
        .unwrap();
    spawner
        .spawn(publish_imu_stream(
            topics.imu_stream(),
            ImuPublisherPorts {
                broker_status: BROKER_STATUS.receiver().unwrap(),
                imu_stream: IMU_STREAM.receiver(),
                mqtt_publish: MQTT_PUBLISH.sender(),
            },
        ))
        .unwrap();
    spawner
        .spawn(manage_ota(
            peripherals.ota,
            OtaManagerPorts {
                network_ready: NETWORK_READY.receiver().unwrap(),
                ota_check_request: &OTA_CHECK_REQUEST,
                ota_configuration: OTA_CONFIGURATION.receiver().unwrap(),
                ota_status: OTA_STATUS.sender(),
            },
        ))
        .unwrap();
}
