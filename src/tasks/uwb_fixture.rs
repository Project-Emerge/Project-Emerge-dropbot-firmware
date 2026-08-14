//! Dedicated factory image for the three-node DS-TWR antenna-delay fixture.

use ariel_os::gpio::IntEnabledInput;
use ariel_os::log::{Debug2Format, error, info, warn};
use ariel_os::reexports::static_cell::StaticCell;
use ariel_os::time::{Duration, Instant, Timer};
use ariel_os_embassy::thread_executor::Executor as ThreadExecutor;
use dw3000_ng::hl::SendTime;
use dw3000_ng::time::Instant as DwInstant;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::watch::Receiver as WatchReceiver;
use uwb_fixture::{FixtureEvent, FixtureIo};
use uwb_protocol::CalibrationPacket;

use crate::data::calibration::{CalibrationCapture, CalibrationSample};
use crate::data::mqtt::{BrokerStatus, PublishMessage};
use crate::drivers::uwb::spi::UwbSpiDevice;
use crate::drivers::uwb::{self, Radio, spi};
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

const THREAD_STACKSIZE: usize = 8192;
const THREAD_PRIORITY: u8 = 9;
const RETRY: Duration = Duration::from_secs(2);

struct Args {
    pins: pins::UwbPins,
    delay: dw3000_hal::AntennaDelay,
    samples: Sender<'static, CriticalSectionRawMutex, CalibrationSample, 8>,
    capture: WatchReceiver<'static, CriticalSectionRawMutex, Option<CalibrationCapture>, 2>,
}

static HANDOFF: Channel<CriticalSectionRawMutex, Args, 1> = Channel::new();

pub fn start(
    pins: pins::UwbPins,
    delay: dw3000_hal::AntennaDelay,
    samples: Sender<'static, CriticalSectionRawMutex, CalibrationSample, 8>,
    capture: WatchReceiver<'static, CriticalSectionRawMutex, Option<CalibrationCapture>, 2>,
) {
    HANDOFF
        .try_send(Args {
            pins,
            delay,
            samples,
            capture,
        })
        .unwrap_or_else(|_| unreachable!("fixture handoff occurs once"));
}

#[ariel_os::thread(autostart, stacksize = THREAD_STACKSIZE, priority = THREAD_PRIORITY)]
fn fixture_thread() {
    let args = ariel_os::thread::block_on(HANDOFF.receive());
    static EXECUTOR: StaticCell<ThreadExecutor> = StaticCell::new();
    EXECUTOR
        .init_with(ThreadExecutor::new)
        .run(|spawner| spawner.must_spawn(run_fixture(args)));
}

#[ariel_os::task]
async fn run_fixture(args: Args) -> ! {
    let Args {
        pins,
        delay,
        samples,
        capture,
    } = args;
    let _wup_guard = spi::hold_wakeup_inactive(pins.wup);
    let mut irq = spi::irq(pins.irq);
    let mut peripherals = Some(spi::UwbPeripherals {
        rst: pins.rst,
        sck: pins.sck,
        mosi: pins.mosi,
        miso: pins.miso,
        cs: pins.cs,
        spi: pins.spi,
    });
    let mut capture = capture;
    let mut sequence = 0u16;
    loop {
        let owned = peripherals
            .take()
            .unwrap_or_else(|| unsafe { spi::UwbPeripherals::steal_for_retry() });
        let (_reset, device) = spi::build(owned).await;
        let radio = match uwb::bring_up(device, uwb_fixture::NODE_IDS[NODE_INDEX], delay).await {
            Ok(radio) => radio,
            Err(error) => {
                error!("fixture: bring-up failed: {:?}", Debug2Format(&error));
                Timer::after(RETRY).await;
                continue;
            }
        };
        info!(
            "fixture: isolated DS-TWR node {} id {:04x}; schedule rev {}, turnaround {} us, report {} us; production frames are rejected",
            NODE_INDEX,
            uwb_fixture::NODE_IDS[NODE_INDEX],
            uwb_fixture::SCHEDULE_REVISION,
            uwb_fixture::TURNAROUND_US,
            uwb_fixture::REPORT_DELAY_US
        );
        let mut io = RobotIo {
            radio: Some(radio),
            irq: &mut irq,
            config: uwb::radio_config(),
            delay,
            origin: Instant::now(),
            samples: &samples,
            capture,
            sample_session: None,
            emitted_samples: 0,
            dropped_samples: 0,
            synchronized_frames: 0,
            diagnostic_failures: 0,
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
        capture = io.capture;
        Timer::after(RETRY).await;
    }
}

struct RobotIo<'a> {
    radio: Option<Radio>,
    irq: &'a mut IntEnabledInput<'static>,
    config: dw3000_ng::Config,
    delay: dw3000_hal::AntennaDelay,
    origin: Instant,
    samples: &'a Sender<'static, CriticalSectionRawMutex, CalibrationSample, 8>,
    capture: WatchReceiver<'static, CriticalSectionRawMutex, Option<CalibrationCapture>, 2>,
    sample_session: Option<u32>,
    emitted_samples: u32,
    dropped_samples: u32,
    synchronized_frames: u32,
    diagnostic_failures: u32,
}

impl RobotIo<'_> {
    fn take(&mut self) -> Radio {
        self.radio
            .take()
            .expect("fixture radio restored after every operation")
    }

    fn put(&mut self, radio: Radio) {
        self.radio = Some(radio);
    }
}

impl FixtureIo for RobotIo<'_> {
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
            let result = self.receive(uwb_fixture::SUPERFRAME_US).await?;
            match result {
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
                DwInstant::new(scheduled_ticks).expect("fixture timestamps are masked"),
            ),
            self.config,
            Duration::from_micros(u64::from(uwb_fixture::TX_WAIT_US)),
        )
        .await?;
        let outcome = sent.outcome.ok();
        self.put(sent.radio);
        Ok(outcome)
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
        let outcome = received.outcome.ok();
        self.put(received.radio);
        Ok(outcome)
    }

    fn emit(&mut self, report: CalibrationPacket) {
        let Some(capture) = self.capture.try_get().flatten() else {
            return;
        };
        if Instant::now().as_micros() >= capture.expires_at_us {
            return;
        }
        if self.sample_session != Some(capture.session_id) {
            self.sample_session = Some(capture.session_id);
            self.emitted_samples = 0;
            self.dropped_samples = 0;
            info!(
                "fixture: first radio report for capture session {}",
                capture.session_id
            );
        }
        if let CalibrationPacket::Report {
            initiator_id,
            responder_id,
            sequence,
            distance_mm,
        } = report
        {
            let sample = CalibrationSample {
                session_id: capture.session_id,
                initiator_id,
                responder_id,
                sequence,
                distance_mm,
            };
            if self.samples.try_send(sample).is_ok() {
                self.emitted_samples = self.emitted_samples.saturating_add(1);
                if self.emitted_samples.is_multiple_of(100) {
                    info!(
                        "fixture: capture {} emitted {} samples ({} queue drops)",
                        capture.session_id, self.emitted_samples, self.dropped_samples
                    );
                }
            } else {
                self.dropped_samples = self.dropped_samples.saturating_add(1);
                if self.dropped_samples == 1 || self.dropped_samples.is_multiple_of(100) {
                    warn!(
                        "fixture: capture {} sample queue full ({} drops)",
                        capture.session_id, self.dropped_samples
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
            warn!(
                "fixture: DS-TWR stage failure {:?} ({} total)",
                Debug2Format(&event),
                self.diagnostic_failures
            );
        }
    }
}

#[ariel_os::task]
pub async fn publish_calibration_samples(
    topic: &'static str,
    mut broker: WatchReceiver<'static, CriticalSectionRawMutex, BrokerStatus, 6>,
    samples: Receiver<'static, CriticalSectionRawMutex, CalibrationSample, 8>,
    mqtt: Sender<'static, CriticalSectionRawMutex, PublishMessage, 5>,
) -> ! {
    while broker.get().await != BrokerStatus::Connected {
        broker.changed().await;
    }
    info!("fixture: MQTT sample publisher active on {}", topic);
    loop {
        let sample = samples.receive().await;
        let Ok(encoded) = serde_json::to_vec(&sample) else {
            error!("fixture: calibration sample serialization failed");
            continue;
        };
        let mut message = PublishMessage {
            topic,
            payload: heapless::Vec::new(),
        };
        if message.payload.extend_from_slice(&encoded).is_ok() {
            // This task is separate from the timing-critical radio thread. Backpressure here is
            // preferable to silently dropping whenever another publisher briefly fills MQTT.
            mqtt.send(message).await;
        } else {
            error!("fixture: calibration sample payload too large");
        }
    }
}
