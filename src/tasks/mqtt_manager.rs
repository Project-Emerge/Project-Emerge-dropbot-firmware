use core::fmt::Write;
use core::str::FromStr;

use ariel_os::log::{debug, error, info};
use ariel_os::{config, net};
use ariel_os::reexports::embassy_net::{Ipv4Address, tcp::TcpSocket};
use ariel_os::time::Timer;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::watch::{Receiver as WatchReceiver, Sender as WatchSender};
use minimq::{Buffers, ConfigBuilder, ConnectEvent, Error, Publication, Session, TopicFilter};

use crate::TCP_BUFFER_SIZE;
use crate::data::mqtt::{BrokerStatus, PublishMessage, ReceivedMessage};

const MQTT_SERVER_HOST: &str = config::str_from_env_or!(
    "MQTT_SERVER_HOST",
    "192.168.8.1",
    "hostname or IP address of the MQTT server",
);

#[ariel_os::task]
pub async fn mqtt_manager(
    mut network_ready: WatchReceiver<'static, CriticalSectionRawMutex, (), 2>,
    broker_status: WatchSender<'static, CriticalSectionRawMutex, BrokerStatus, 2>,
    mqtt_publish_rx: Receiver<'static, CriticalSectionRawMutex, PublishMessage, 2>,
    mqtt_receive_tx: Sender<'static, CriticalSectionRawMutex, ReceivedMessage, 2>,
) -> ! {
    broker_status.send(BrokerStatus::Disconnected);

    // Waits on `network_ready` rather than `stack.wait_config_up()` directly: the latter
    // registers a single waker on the network stack, and `network_monitor` already owns
    // that role. `network_ready` is a dedicated watch it fills once the stack is up.
    network_ready.get().await;
    let stack = net::network_stack().await.unwrap();

    let mut tcp_rx_buffer = [0u8; TCP_BUFFER_SIZE];
    let mut tcp_tx_buffer = [0u8; TCP_BUFFER_SIZE];
    let mut tcp_socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);

    let broker = Ipv4Address::from_str(MQTT_SERVER_HOST).unwrap();
    let rx = &mut [0u8; 256];
    let tx = &mut [0u8; 768];
    let mut session = Session::new(
        ConfigBuilder::new(Buffers::new(rx, tx))
            .client_id("dropbot")
            .unwrap(),
    );

    loop {
        tcp_socket.abort();
        tcp_socket.flush().await.ok();

        if let Err(err) = tcp_socket.connect((broker, 1883)).await {
            error!("mqtt: tcp connect failed: {}", err);
            Timer::after(ariel_os::time::Duration::from_secs(1)).await;
            continue;
        }

        let mut conn = match session.connect(&mut tcp_socket).await {
            Ok(conn) => conn,
            Err(err) => {
                error!("mqtt: failed to connect to broker: {}", err);
                Timer::after(ariel_os::time::Duration::from_secs(1)).await;
                continue;
            }
        };

        match conn.connect_event() {
            ConnectEvent::Connected => {
                info!("mqtt: connected to broker");
            }
            ConnectEvent::Reconnected => {
                info!("mqtt: resumed broker session");
            }
        }

        if let Err(err) = conn
            .subscribe(
                &[TopicFilter::new("dio/cane"), TopicFilter::new("dio/ota/check")],
                &[],
            )
            .await
        {
            error!("mqtt: subscribe failed: {}", err);
            continue;
        }

        broker_status.send(BrokerStatus::Connected);

        // Drop any telemetry queued while we were disconnected/reconnecting so we publish fresh data first.
        while mqtt_publish_rx.try_receive().is_ok() {}

        loop {
            match select(conn.recv(), mqtt_publish_rx.receive()).await {
                Either::First(received) => match received {
                    Ok(message) => {
                        let mut msg = ReceivedMessage::default();
                        if write!(msg.topic, "{}", message.topic()).is_err() {
                            error!("mqtt: received topic too long, dropping message");
                            continue;
                        }
                        if msg.payload.extend_from_slice(message.payload()).is_err() {
                            error!("mqtt: received payload too large, dropping message");
                            continue;
                        }
                        if mqtt_receive_tx.try_send(msg).is_err() {
                            error!("mqtt: receive queue full, dropping message");
                        }
                    }
                    Err(Error::Disconnected) => {
                        error!("mqtt: disconnected from broker");
                        break;
                    }
                    Err(err) => {
                        error!("mqtt: recv error: {}", err);
                        break;
                    }
                },
                Either::Second(msg) => {
                    let publication = Publication::new(msg.topic.as_str(), msg.payload.as_slice());
                    match conn.publish(publication).await {
                        Ok(_) => {
                            debug!("mqtt: published {} bytes", msg.payload.len());
                        }
                        Err(_) => {
                            error!("mqtt: publish or disconnection failed");
                            break;
                        }
                    }
                }
            }
        }

        // Any of the `break`s above means the session is gone and the outer loop is about
        // to reconnect.
        broker_status.send(BrokerStatus::Disconnected);
    }
}
