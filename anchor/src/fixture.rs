//! STM32 adapter for the isolated three-node DS-TWR factory fixture.

use ariel_os::gpio::IntEnabledInput;
use ariel_os::log::{Debug2Format, error, info};
use ariel_os::time::{Duration, Instant, Timer};
use dw3000_ng::hl::SendTime;
use dw3000_ng::time::Instant as DwInstant;
use uwb_fixture::{FixtureEvent, FixtureIo};
use uwb_protocol::CalibrationPacket;

use crate::drivers::uwb::{self, Radio, UwbSpiDevice};
use crate::pins;

const SELECTED_NODES: usize = cfg!(feature = "fixture-node-1") as usize
    + cfg!(feature = "fixture-node-2") as usize
    + cfg!(feature = "fixture-node-3") as usize;
const _: () = assert!(
    SELECTED_NODES == 1,
    "calibration-fixture requires exactly one fixture-node-1..3 feature"
);
const NODE_INDEX: usize = if cfg!(feature = "fixture-node-1") {
    0
} else if cfg!(feature = "fixture-node-2") {
    1
} else {
    2
};

pub async fn run(pins: pins::UwbPins, delay: dw3000_hal::AntennaDelay) -> ! {
    let mut irq = uwb::irq(pins.irq);
    let mut peripherals = Some(uwb::UwbPeripherals {
        rst: pins.rst,
        cs: pins.cs,
        sck: pins.sck,
        miso: pins.miso,
        mosi: pins.mosi,
        spi: pins.spi,
        tx_dma: pins.tx_dma,
        rx_dma: pins.rx_dma,
    });
    let mut sequence = 0u16;
    loop {
        let owned = peripherals
            .take()
            .unwrap_or_else(|| unsafe { uwb::UwbPeripherals::steal_for_retry() });
        let (_reset, device) = uwb::build(owned).await;
        let radio = match uwb::bring_up(device, uwb_fixture::NODE_IDS[NODE_INDEX], delay).await {
            Ok(radio) => radio,
            Err(error) => {
                error!("fixture: bring-up failed: {:?}", Debug2Format(&error));
                Timer::after_secs(2).await;
                continue;
            }
        };
        info!(
            "fixture: isolated STM32 DS-TWR node {} id {:04x}; schedule rev {}, turnaround {} us, report {} us; production frames rejected",
            NODE_INDEX,
            uwb_fixture::NODE_IDS[NODE_INDEX],
            uwb_fixture::SCHEDULE_REVISION,
            uwb_fixture::TURNAROUND_US,
            uwb_fixture::REPORT_DELAY_US
        );
        let mut io = AnchorIo {
            radio: Some(radio),
            irq: &mut irq,
            config: uwb::radio_config(),
            delay,
            origin: Instant::now(),
            synchronized_frames: 0,
            diagnostic_failures: 0,
            sample_counts: [0; uwb_fixture::DIRECTED_PAIRS.len()],
        };
        loop {
            match uwb_fixture::run_superframe(&mut io, sequence).await {
                Ok(true) => sequence = sequence.wrapping_add(1),
                Ok(false) => {}
                Err(error) => {
                    error!("fixture: radio reset: {:?}", Debug2Format(&error));
                    break;
                }
            }
        }
        Timer::after_secs(2).await;
    }
}

struct AnchorIo<'a> {
    radio: Option<Radio>,
    irq: &'a mut IntEnabledInput<'static>,
    config: dw3000_ng::Config,
    delay: dw3000_hal::AntennaDelay,
    origin: Instant,
    synchronized_frames: u32,
    diagnostic_failures: u32,
    sample_counts: [u32; uwb_fixture::DIRECTED_PAIRS.len()],
}

impl AnchorIo<'_> {
    fn take(&mut self) -> Radio {
        self.radio
            .take()
            .expect("fixture radio restored after every operation")
    }

    fn put(&mut self, radio: Radio) {
        self.radio = Some(radio);
    }
}

impl FixtureIo for AnchorIo<'_> {
    type Error = dw3000_ng::Error<UwbSpiDevice>;

    fn node_index(&self) -> usize {
        NODE_INDEX
    }

    fn tx_antenna_delay(&self) -> u16 {
        self.delay.tx
    }

    async fn synchronize(&mut self, sequence: u16) -> Result<Option<(u16, u64)>, Self::Error> {
        if NODE_INDEX == 0 {
            let mut radio = self.take();
            let now = dw3000_hal::sys_time_ticks(&mut radio).await?;
            self.put(radio);
            let scheduled = uwb_protocol::delayed_after(now, 1_500);
            let result = self
                .send_at(CalibrationPacket::Sync { sequence }, scheduled)
                .await?;
            if result.is_some() {
                self.origin = Instant::now();
            }
            Ok(result.map(|timestamp| (sequence, timestamp)))
        } else {
            match self.receive(uwb_fixture::SUPERFRAME_US).await? {
                Some((CalibrationPacket::Sync { sequence: received }, timestamp)) => {
                    self.origin = Instant::now();
                    Ok(Some((received, timestamp)))
                }
                _ => Ok(None),
            }
        }
    }

    async fn wait_until_offset(&mut self, offset_us: u32) {
        Timer::at(self.origin + Duration::from_micros(u64::from(offset_us))).await;
    }

    async fn send_at(
        &mut self,
        packet: CalibrationPacket,
        scheduled_ticks: u64,
    ) -> Result<Option<u64>, Self::Error> {
        let mut bytes = [0u8; CalibrationPacket::MAX_LEN];
        let len = packet.encode(&mut bytes);
        let sent = dw3000_hal::send_packet(
            self.take(),
            self.irq,
            &bytes[..len],
            SendTime::Delayed(
                DwInstant::new(scheduled_ticks).expect("fixture timestamp is 40-bit masked"),
            ),
            self.config,
            Duration::from_micros(u64::from(uwb_fixture::TX_WAIT_US)),
        )
        .await?;
        let result = sent.outcome.ok();
        self.put(sent.radio);
        Ok(result)
    }

    async fn receive(
        &mut self,
        timeout_us: u32,
    ) -> Result<Option<(CalibrationPacket, u64)>, Self::Error> {
        let received = dw3000_hal::receive_decoded(
            self.take(),
            self.irq,
            self.config,
            Duration::from_micros(u64::from(timeout_us)),
            |payload| CalibrationPacket::decode(payload).ok(),
        )
        .await?;
        let result = received.outcome.ok();
        self.put(received.radio);
        Ok(result)
    }

    fn emit(&mut self, report: CalibrationPacket) {
        if let CalibrationPacket::Report {
            initiator_id,
            responder_id,
            sequence,
            distance_mm,
        } = report
        {
            let pair = uwb_fixture::DIRECTED_PAIRS.iter().position(|&(a, b)| {
                uwb_fixture::NODE_IDS[a] == initiator_id && uwb_fixture::NODE_IDS[b] == responder_id
            });
            if let Some(pair) = pair {
                self.sample_counts[pair] = self.sample_counts[pair].saturating_add(1);
                let count = self.sample_counts[pair];
                if count == 1 || count.is_multiple_of(100) {
                    info!(
                        "fixture-sample: {} samples for {:04x}->{:04x}; latest sequence {} distance {} mm",
                        count, initiator_id, responder_id, sequence, distance_mm
                    );
                }
            }
        }
    }

    fn note(&mut self, event: FixtureEvent) {
        if let FixtureEvent::Synchronized { sequence } = event {
            self.synchronized_frames = self.synchronized_frames.saturating_add(1);
            if self.synchronized_frames == 1 || self.synchronized_frames.is_multiple_of(100) {
                info!(
                    "fixture: node {:04x} synchronized at sequence {} ({} superframes)",
                    uwb_fixture::NODE_IDS[NODE_INDEX],
                    sequence,
                    self.synchronized_frames
                );
            }
            return;
        }

        self.diagnostic_failures = self.diagnostic_failures.saturating_add(1);
        if self.diagnostic_failures <= 18 || self.diagnostic_failures.is_multiple_of(500) {
            ariel_os::log::warn!(
                "fixture: DS-TWR stage failure {:?} ({} total)",
                Debug2Format(&event),
                self.diagnostic_failures
            );
        }
    }
}
