use ariel_os::net;

use crate::{NETWORK_READY, NETWORK_STATUS};

/// Sole task allowed to wait on the network stack's config up/down state:
/// `Stack::wait_config_up`/`wait_config_down` register a single waker, so a second
/// concurrent waiter (e.g. another task also calling them) would silently starve
/// whichever one registered first and never get woken.
#[ariel_os::task]
pub async fn network_monitor() -> ! {
    let stack = net::network_stack().await.unwrap();

    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            NETWORK_STATUS.signal(Some(config.address.address()));
        }
        NETWORK_READY.signal(());

        stack.wait_config_down().await;
        NETWORK_STATUS.signal(None);
    }
}
