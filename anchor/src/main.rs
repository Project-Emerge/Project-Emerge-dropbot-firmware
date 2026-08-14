//! DWM3000 indoor-positioning anchor.
//!
//! One of four fixed nodes that a robot ranges against to locate itself on the floor plane. Anchor
//! index 0 is the TDMA timing master; every anchor owns one deterministic slot per provisioned
//! robot and polls it. See `ranging` for the protocol and `crates/uwb-protocol` for the wire format
//! and timing tables, both shared verbatim with the robot firmware at the repo root.
//!
//! The board has no network interface of any kind -- no UART header, no USB connector, no radio
//! other than the DW3000 -- so RTT over SWD is the only way anything gets out of it. That is a
//! deliberate constraint of the architecture, not an oversight: the anchors stay dumb, stateless
//! beacons and the robots do the position solving, which is also where the answer is needed.

#![no_std]
#![no_main]

mod drivers;
mod pins;
mod ranging;

use ariel_os::log::info;
use ariel_os::time::Timer;

use crate::drivers::battery::BatteryMonitor;
use crate::drivers::led::IndicatorLed;
use crate::drivers::power::PowerLatch;

/// Exactly one `anchor-N` Cargo feature must be enabled. Checked here rather than left to the
/// `cfg` chain below so the error names the actual mistake: an anchor flashed with the wrong -- or
/// with no -- identity transmits in another anchor's slot, and a slot collision on air looks like
/// poor RF rather than like a misconfiguration.
const SELECTED_IDENTITIES: usize = cfg!(feature = "anchor-1") as usize
    + cfg!(feature = "anchor-2") as usize
    + cfg!(feature = "anchor-3") as usize
    + cfg!(feature = "anchor-4") as usize;
const _: () = assert!(
    SELECTED_IDENTITIES == 1,
    "exactly one of the anchor-1..anchor-4 Cargo features must be enabled \
     (laze: -s anchor-1 .. -s anchor-4)"
);

/// This build's index into [`uwb_protocol::ANCHOR_IDS`].
///
/// The final `else` is only reachable with `anchor-4` enabled, which the assertion above
/// guarantees.
const ANCHOR_INDEX: usize = if cfg!(feature = "anchor-1") {
    0
} else if cfg!(feature = "anchor-2") {
    1
} else if cfg!(feature = "anchor-3") {
    2
} else {
    3
};
const _: () = assert!(
    ANCHOR_INDEX < uwb_protocol::ACTIVE_ANCHOR_COUNT,
    "this build's anchor index is outside uwb_protocol::ANCHOR_IDS"
);

/// How often the battery is sampled and the indicator LED updated.
const BATTERY_INTERVAL_SECS: u64 = 1;

/// The 80 MHz clock tree, handed to ariel-os in place of its own per-chip defaults (which have no
/// branch for the STM32L432KC and would leave the part on its 4 MHz MSI).
///
/// HSI16 -> PLL x10 -> /2 = 80 MHz, the STM32L432's maximum. This is not about compute: a ranging
/// slot is dominated by how fast a reply can be *programmed* after a frame arrives, and most of that
/// turnaround is MCU-side work -- DMA setup, the driver's busy-wait loops -- rather than SPI bits,
/// so the core clock matters as much as the SPI clock. It also lifts PCLK2, and therefore SPI1, to
/// where `drivers::uwb::RUNNING_FREQUENCY` becomes reachable.
///
/// `adcsel = SYS` is load-bearing too: without a kernel clock the ADC the battery monitor reads
/// simply never completes a conversion.
#[unsafe(no_mangle)]
extern "Rust" fn __ariel_os_rcc_config() -> embassy_stm32::rcc::Config {
    use embassy_stm32::rcc::{
        Hsi48Config, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk, mux,
    };

    let mut rcc = embassy_stm32::rcc::Config {
        hsi: true,
        hsi48: Some(Hsi48Config {
            sync_from_usb: true,
        }),
        pll: Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL10,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        }),
        sys: Sysclk::PLL1_R,
        ..Default::default()
    };
    // Nested fields, so these stay reassignments rather than folding into the literal above.
    rcc.mux.clk48sel = mux::Clk48sel::HSI48;
    rcc.mux.adcsel = mux::Adcsel::SYS;
    rcc
}

#[ariel_os::spawner(autostart, peripherals)]
fn main(spawner: ariel_os::asynch::Spawner, peripherals: pins::Peripherals) {
    // First thing, before anything that could take time or fail: assert PS_HOLD so the STM6600
    // keeps the rail up, and strap the charger. `PowerLatch` owns those pins for the rest of the
    // program -- see its docs.
    let latch = PowerLatch::new(
        peripherals.power.ps_hold,
        peripherals.power.power_int,
        peripherals.charger.enable,
        peripherals.charger.en1,
        peripherals.charger.en2,
    );

    info!(
        "anchor: started on {} as index {} (id {:04x})",
        ariel_os::buildinfo::BOARD,
        ANCHOR_INDEX,
        uwb_protocol::ANCHOR_IDS[ANCHOR_INDEX],
    );

    spawner.spawn(hold_power(latch)).unwrap();
    spawner
        .spawn(monitor_battery(peripherals.battery, peripherals.led))
        .unwrap();
    spawner.spawn(range(peripherals.uwb)).unwrap();
}

/// Confirms the power-button press that booted this board, then parks forever holding the latch.
///
/// Parks rather than returns because dropping [`PowerLatch`] would release `PS_HOLD` and the
/// charger straps. Only the first press is acted on: the STM6600 also reports a second press as a
/// power-off request, which this firmware deliberately ignores -- an anchor should not be able to
/// take the network's timing down by being leaned against.
#[ariel_os::task]
async fn hold_power(mut latch: PowerLatch) -> ! {
    latch.confirm_power_button().await;
    info!("anchor: STM6600 power latch confirmed");
    core::future::pending().await
}

/// Samples the cell once a second and colours the indicator LED accordingly.
#[ariel_os::task]
async fn monitor_battery(battery: pins::BatteryPins, led: pins::LedPins) -> ! {
    use ariel_os::gpio::{Level, Output};

    let mut monitor = BatteryMonitor::new(battery.adc, battery.sense);
    let mut indicator = IndicatorLed::new(
        Output::new(led.red, Level::Low),
        Output::new(led.green, Level::Low),
    );

    loop {
        let percentage = monitor.read_percentage();
        ariel_os::log::debug!("anchor: battery {}%", percentage);
        indicator.show_battery_percentage(percentage);
        Timer::after_secs(BATTERY_INTERVAL_SECS).await;
    }
}

/// Runs this anchor's half of the ranging protocol.
#[ariel_os::task]
async fn range(uwb: pins::UwbPins) -> ! {
    ranging::run(uwb, ANCHOR_INDEX).await
}
