use core::str::FromStr;

use ariel_os::log::{debug, error, info};
use ariel_os::reexports::embassy_net::{Ipv4Address, tcp::TcpSocket};
use ariel_os::time::Timer;
use ariel_os::{config, net};
use embassy_futures::select::{Either, select};
use minimq::{Buffers, ConfigBuilder, ConnectEvent, Error, Publication, Session};

use crate::TCP_BUFFER_SIZE;
use crate::data::mqtt::{BrokerStatus, ReceivedMessage};
use crate::task_sync::{BrokerStatusTx, MqttPublishRx, MqttReceiveTx, NetworkReadyRx};
use crate::topics::Topics;

const MQTT_SERVER_HOST: &str = config::str_from_env_or!(
    "MQTT_SERVER_HOST",
    "192.168.8.1",
    "hostname or IP address of the MQTT server",
);

/// Messaging endpoints owned by the MQTT connection task.
pub struct MqttManagerPorts {
    pub network_ready: NetworkReadyRx,
    pub broker_status: BrokerStatusTx,
    pub mqtt_publish: MqttPublishRx,
    pub mqtt_receive: MqttReceiveTx,
}

#[ariel_os::task]
pub async fn mqtt_manager(
    device_id: &'static str,
    topics: &'static Topics,
    mut ports: MqttManagerPorts,
) -> ! {
    ports.broker_status.send(BrokerStatus::Disconnected);

    // Waits on `network_ready` rather than `stack.wait_config_up()` directly: the latter
    // registers a single waker on the network stack, and `network_monitor` already owns
    // that role. `network_ready` is a dedicated watch it fills once the stack is up.
    ports.network_ready.get().await;
    let stack = net::network_stack().await.unwrap();

    let mut tcp_rx_buffer = [0u8; TCP_BUFFER_SIZE];
    let mut tcp_tx_buffer = [0u8; TCP_BUFFER_SIZE];
    let mut tcp_socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);

    let broker = Ipv4Address::from_str(MQTT_SERVER_HOST).unwrap();
    // Like `tx` below, this has to hold a whole *incoming* packet -- MQTT fixed header, topic
    // and payload together, not just the payload -- and minimq errors the whole session out
    // (`Error::MalformedPacket`) if a packet does not fit rather than truncating it.
    let rx = &mut [0u8; 1536];
    // The transmit buffer has to hold a whole outgoing packet, and the largest one this
    // firmware sends by a wide margin is the telemetry payload -- which carries the IMU's
    // nine axes twice over, raw and filtered.
    let tx = &mut [0u8; 1536];
    let mut session = Session::new(
        ConfigBuilder::new(Buffers::new(rx, tx))
            .client_id(device_id)
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

        if let Err(err) = conn.subscribe(&topics.subscriptions(), &[]).await {
            error!("mqtt: subscribe failed: {}", err);
            continue;
        }

        ports.broker_status.send(BrokerStatus::Connected);

        // Drop any telemetry queued while we were disconnected/reconnecting so we publish fresh data first.
        while ports.mqtt_publish.try_receive().is_ok() {}

        loop {
            match select(conn.recv(), ports.mqtt_publish.receive()).await {
                Either::First(received) => match received {
                    Ok(message) => {
                        // The wire topic is matched against the table here, once, so it never
                        // has to be carried any further than this.
                        let Some(topic) = topics.resolve(message.topic()) else {
                            error!("mqtt: message on unsubscribed topic, dropping message");
                            continue;
                        };
                        let mut payload = heapless::Vec::new();
                        if payload.extend_from_slice(message.payload()).is_err() {
                            error!("mqtt: received payload too large, dropping message");
                            continue;
                        }
                        if ports
                            .mqtt_receive
                            .try_send(ReceivedMessage { topic, payload })
                            .is_err()
                        {
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
                    let publication = Publication::new(msg.topic, msg.payload.as_slice());
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
        ports.broker_status.send(BrokerStatus::Disconnected);
    }
}
