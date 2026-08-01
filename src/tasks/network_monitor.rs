use ariel_os::net;
use ariel_os::reexports::embassy_net::Ipv4Address;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Sole task allowed to wait on the network stack's config up/down state:
/// `Stack::wait_config_up`/`wait_config_down` register a single waker, so a second
/// concurrent waiter (e.g. another task also calling them) would silently starve
/// whichever one registered first and never get woken.
#[ariel_os::task]
pub async fn network_monitor(
    network_status: &'static Signal<CriticalSectionRawMutex, Option<Ipv4Address>>,
    network_ready: &'static Signal<CriticalSectionRawMutex, ()>,
) -> ! {
    let stack = net::network_stack().await.unwrap();

    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            network_status.signal(Some(config.address.address()));
        }
        network_ready.signal(());

        stack.wait_config_down().await;
        network_status.signal(None);
    }
}
