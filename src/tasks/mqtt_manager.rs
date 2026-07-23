use core::fmt::Write;
use core::str::FromStr;

use ariel_os::log::{debug, error, info};
use ariel_os::net;
use ariel_os::reexports::embassy_net::{Ipv4Address, tcp::TcpSocket};
use ariel_os::time::Timer;
use embassy_futures::select::{Either, select};
use minimq::{Buffers, ConfigBuilder, ConnectEvent, Error, Publication, Session, TopicFilter};

use crate::data::mqtt::ReceivedMessage;
use crate::{MQTT_CONNECTION, MQTT_PUBLISH, MQTT_RECEIVE, NETWORK_READY, TCP_BUFFER_SIZE};

#[ariel_os::task]
pub async fn mqtt_manager() -> ! {
    // Waits on `NETWORK_READY` rather than `stack.wait_config_up()` directly: the latter
    // registers a single waker on the network stack, and `network_monitor` already owns
    // that role. `NETWORK_READY` is a dedicated signal it fills once the stack is up.
    NETWORK_READY.wait().await;
    let stack = net::network_stack().await.unwrap();

    let mut tcp_rx_buffer = [0u8; TCP_BUFFER_SIZE];
    let mut tcp_tx_buffer = [0u8; TCP_BUFFER_SIZE];
    let mut tcp_socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);

    let broker = Ipv4Address::from_str("192.168.8.1").unwrap();
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

        if let Err(err) = conn.subscribe(&[TopicFilter::new("dio/cane")], &[]).await {
            error!("mqtt: subscribe failed: {}", err);
            continue;
        }

        MQTT_CONNECTION.signal(());

        // Drop any telemetry queued while we were disconnected/reconnecting so we publish fresh data first.
        while MQTT_PUBLISH.try_receive().is_ok() {}

        loop {
            match select(conn.recv(), MQTT_PUBLISH.receive()).await {
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
                        if MQTT_RECEIVE.try_send(msg).is_err() {
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
    }
}
