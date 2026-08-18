use ariel_os::net;

use crate::task_sync::{NetworkReadyTx, NetworkStatusSignal};

/// Messaging endpoints owned by the network-state task.
pub struct NetworkMonitorPorts {
    pub network_status: &'static NetworkStatusSignal,
    pub network_ready: NetworkReadyTx,
}

/// Sole task allowed to wait on the network stack's config up/down state:
/// `Stack::wait_config_up`/`wait_config_down` register a single waker, so a second
/// concurrent waiter (e.g. another task also calling them) would silently starve
/// whichever one registered first and never get woken.
#[ariel_os::task]
pub async fn network_monitor(ports: NetworkMonitorPorts) -> ! {
    let stack = net::network_stack().await.unwrap();

    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            ports.network_status.signal(Some(config.address.address()));
        }
        ports.network_ready.send(());

        stack.wait_config_down().await;
        ports.network_status.signal(None);
    }
}
