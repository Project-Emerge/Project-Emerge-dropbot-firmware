#![no_std]

//! Hardware-independent schedule for the isolated three-node DS-TWR calibration fixture.

use uwb_protocol::CalibrationPacket;

pub const NODE_IDS: [u16; 3] = [0xC001, 0xC002, 0xC003];
/// Printed at boot so mixed fixture images can be identified without inspecting binaries.
pub const SCHEDULE_REVISION: u8 = 2;
pub const DIRECTED_PAIRS: [(usize, usize); 6] = [(0, 1), (1, 0), (0, 2), (2, 0), (1, 2), (2, 1)];
// Factory calibration values deliberately favour deterministic completion over update rate. The
// ESP32 gateway also services Wi-Fi/MQTT; hardware showed that a 1.2 ms RX-to-delayed-TX deadline
// was missed systematically while it acted as responder. Five milliseconds leaves room for that
// preemption without changing the production ranging schedule.
pub const SYNC_GUARD_US: u32 = 6_000;
pub const SLOT_US: u32 = 18_000;
pub const TURNAROUND_US: u32 = 5_000;
// A Report is prepared after the responder has decoded Final and calculated the distance. Keep
// this comfortably above MCU/SPI jitter; in particular, the STM32 diagnostic path must never make
// a valid DS-TWR exchange miss its delayed Report transmission.
pub const REPORT_DELAY_US: u32 = 3_000;
pub const RX_WINDOW_US: u32 = 7_000;
pub const PROGRAM_GUARD_US: u32 = 2_000;
/// Covers the longest delayed operation (`TURNAROUND_US`) plus radio airtime/IRQ margin.
pub const TX_WAIT_US: u32 = 8_000;
pub const SUPERFRAME_US: u32 = SYNC_GUARD_US + DIRECTED_PAIRS.len() as u32 * SLOT_US + 2_000;
const _: () = {
    assert!(SYNC_GUARD_US > PROGRAM_GUARD_US);
    assert!(RX_WINDOW_US > TURNAROUND_US);
    assert!(TX_WAIT_US > TURNAROUND_US);
    assert!(SLOT_US > 2 * TURNAROUND_US + REPORT_DELAY_US + PROGRAM_GUARD_US);
};

/// Low-rate diagnostics emitted by platform adapters while commissioning the physical fixture.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FixtureEvent {
    Synchronized {
        sequence: u16,
    },
    PollTxMiss {
        initiator: u16,
        responder: u16,
    },
    PollMiss {
        initiator: u16,
        responder: u16,
    },
    ResponseTxMiss {
        initiator: u16,
        responder: u16,
    },
    ResponseMiss {
        initiator: u16,
        responder: u16,
    },
    FinalTxMiss {
        initiator: u16,
        responder: u16,
    },
    FinalMiss {
        initiator: u16,
        responder: u16,
    },
    DistanceInvalid {
        initiator: u16,
        responder: u16,
    },
    ReportTxMiss {
        initiator: u16,
        responder: u16,
    },
    ReportMiss {
        initiator: u16,
        responder: u16,
    },
    ObserverReportMiss {
        initiator: u16,
        responder: u16,
        frames_seen: u8,
    },
}

/// Platform adapter. It owns the DW3000 typestate internally, allowing the schedule to stay shared
/// by ESP32 robot and STM32 anchor fixture images.
#[allow(async_fn_in_trait)]
pub trait FixtureIo {
    type Error;

    fn node_index(&self) -> usize;
    fn tx_antenna_delay(&self) -> u16;
    /// Send (node 0) or receive (nodes 1/2) the fixture Sync and return its local radio timestamp.
    async fn synchronize(&mut self, sequence: u16) -> Result<Option<(u16, u64)>, Self::Error>;
    async fn wait_until_offset(&mut self, offset_us: u32);
    async fn send_at(
        &mut self,
        packet: CalibrationPacket,
        scheduled_ticks: u64,
    ) -> Result<Option<u64>, Self::Error>;
    async fn receive(
        &mut self,
        timeout_us: u32,
    ) -> Result<Option<(CalibrationPacket, u64)>, Self::Error>;
    fn emit(&mut self, report: CalibrationPacket);
    fn note(&mut self, _event: FixtureEvent) {}
}

/// Executes all six directed pair exchanges once. Both directions are intentional: they expose a
/// role-dependent implementation/timestamp error even though the robust delay fit pools them.
pub async fn run_superframe<I: FixtureIo>(io: &mut I, sequence: u16) -> Result<bool, I::Error> {
    let Some((sequence, base_ticks)) = io.synchronize(sequence).await? else {
        return Ok(false);
    };
    io.note(FixtureEvent::Synchronized { sequence });
    let node = io.node_index();
    for (slot, &(initiator, responder)) in DIRECTED_PAIRS.iter().enumerate() {
        let offset = SYNC_GUARD_US + slot as u32 * SLOT_US;
        io.wait_until_offset(offset.saturating_sub(PROGRAM_GUARD_US))
            .await;
        if node == initiator {
            initiate(io, base_ticks, offset, initiator, responder, sequence).await?;
        } else if node == responder {
            respond(io, initiator, responder, sequence).await?;
        } else if node == 0 {
            // The coordinator is not part of the C002<->C003 exchange, but still forwards its
            // Report to MQTT. Listen for the whole exchange instead of predicting the Report from
            // the MCU clock: the ESP32 and STM32 establish `origin` after different SPI/IRQ paths,
            // so their software-clock origins can differ even though their DW3000 timestamps are
            // synchronized. Poll, Response and Final are intentionally consumed and skipped.
            let expected_initiator = NODE_IDS[initiator];
            let expected_responder = NODE_IDS[responder];
            let mut report_received = false;
            let mut frames_seen = 0u8;
            for _ in 0..4 {
                match io.receive(RX_WINDOW_US).await? {
                    Some((report, _))
                        if report_matches(
                            &report,
                            expected_initiator,
                            expected_responder,
                            sequence,
                        ) =>
                    {
                        io.emit(report);
                        report_received = true;
                        break;
                    }
                    Some(_) => frames_seen = frames_seen.saturating_add(1),
                    None => break,
                }
            }
            if !report_received {
                io.note(FixtureEvent::ObserverReportMiss {
                    initiator: NODE_IDS[initiator],
                    responder: NODE_IDS[responder],
                    frames_seen,
                });
            }
        }
    }
    Ok(true)
}

fn report_matches(
    packet: &CalibrationPacket,
    initiator_id: u16,
    responder_id: u16,
    sequence: u16,
) -> bool {
    matches!(
        packet,
        CalibrationPacket::Report {
            initiator_id: a,
            responder_id: b,
            sequence: s,
            ..
        } if *a == initiator_id && *b == responder_id && *s == sequence
    )
}

async fn initiate<I: FixtureIo>(
    io: &mut I,
    base_ticks: u64,
    offset_us: u32,
    initiator: usize,
    responder: usize,
    sequence: u16,
) -> Result<(), I::Error> {
    let scheduled = uwb_protocol::delayed_after(base_ticks, offset_us);
    let poll_tx = uwb_protocol::delayed_tx_timestamp(scheduled, io.tx_antenna_delay());
    let poll = CalibrationPacket::Poll {
        initiator_id: NODE_IDS[initiator],
        responder_id: NODE_IDS[responder],
        sequence,
        poll_tx,
    };
    if io.send_at(poll, scheduled).await?.is_none() {
        io.note(FixtureEvent::PollTxMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    }
    let Some((
        CalibrationPacket::Response {
            initiator_id,
            responder_id,
            sequence: rx_sequence,
            ..
        },
        response_rx,
    )) = io.receive(RX_WINDOW_US).await?
    else {
        io.note(FixtureEvent::ResponseMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    };
    if initiator_id != NODE_IDS[initiator]
        || responder_id != NODE_IDS[responder]
        || rx_sequence != sequence
    {
        return Ok(());
    }
    let final_scheduled = uwb_protocol::delayed_after(response_rx, TURNAROUND_US);
    let final_tx = uwb_protocol::delayed_tx_timestamp(final_scheduled, io.tx_antenna_delay());
    let final_packet = CalibrationPacket::Final {
        initiator_id,
        responder_id,
        sequence,
        poll_tx,
        response_rx,
        final_tx,
    };
    if io.send_at(final_packet, final_scheduled).await?.is_none() {
        io.note(FixtureEvent::FinalTxMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    }
    if let Some((
        report @ CalibrationPacket::Report {
            initiator_id: a,
            responder_id: b,
            sequence: s,
            ..
        },
        _,
    )) = io.receive(RX_WINDOW_US).await?
        && a == initiator_id
        && b == responder_id
        && s == sequence
    {
        io.emit(report);
    } else {
        io.note(FixtureEvent::ReportMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
    }
    Ok(())
}

async fn respond<I: FixtureIo>(
    io: &mut I,
    initiator: usize,
    responder: usize,
    sequence: u16,
) -> Result<(), I::Error> {
    let Some((
        CalibrationPacket::Poll {
            initiator_id,
            responder_id,
            sequence: rx_sequence,
            poll_tx,
        },
        poll_rx,
    )) = io.receive(RX_WINDOW_US).await?
    else {
        io.note(FixtureEvent::PollMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    };
    if initiator_id != NODE_IDS[initiator]
        || responder_id != NODE_IDS[responder]
        || rx_sequence != sequence
    {
        return Ok(());
    }
    let scheduled = uwb_protocol::delayed_after(poll_rx, TURNAROUND_US);
    let response_tx = uwb_protocol::delayed_tx_timestamp(scheduled, io.tx_antenna_delay());
    let response = CalibrationPacket::Response {
        initiator_id,
        responder_id,
        sequence,
        poll_rx,
        response_tx,
    };
    if io.send_at(response, scheduled).await?.is_none() {
        io.note(FixtureEvent::ResponseTxMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    }
    let Some((
        CalibrationPacket::Final {
            initiator_id: a,
            responder_id: b,
            sequence: s,
            poll_tx: ptx,
            response_rx,
            final_tx,
        },
        final_rx,
    )) = io.receive(RX_WINDOW_US).await?
    else {
        io.note(FixtureEvent::FinalMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    };
    if a != initiator_id || b != responder_id || s != sequence || ptx != poll_tx {
        return Ok(());
    }
    let Some(distance_mm) = uwb_protocol::distance_mm_ds_twr(
        poll_tx,
        poll_rx,
        response_tx,
        response_rx,
        final_tx,
        final_rx,
    ) else {
        io.note(FixtureEvent::DistanceInvalid {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
        return Ok(());
    };
    let report = CalibrationPacket::Report {
        initiator_id,
        responder_id,
        sequence,
        distance_mm,
    };
    let report_scheduled = uwb_protocol::delayed_after(final_rx, REPORT_DELAY_US);
    if io.send_at(report, report_scheduled).await?.is_none() {
        io.note(FixtureEvent::ReportTxMiss {
            initiator: NODE_IDS[initiator],
            responder: NODE_IDS[responder],
        });
    }
    // Keep reporting, queueing and serial logging off the delayed-TX critical path. In fixture
    // images `emit` may ultimately format a log line or wake the MQTT publisher.
    io.emit(report);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schedule_contains_every_pair_in_both_directions_once() {
        for a in 0..3 {
            for b in 0..3 {
                if a != b {
                    assert_eq!(
                        DIRECTED_PAIRS
                            .iter()
                            .filter(|&&(i, r)| i == a && r == b)
                            .count(),
                        1
                    );
                }
            }
        }
        assert!(SYNC_GUARD_US + DIRECTED_PAIRS.len() as u32 * SLOT_US < SUPERFRAME_US);
    }

    #[test]
    fn observer_accepts_only_the_expected_report() {
        let expected = CalibrationPacket::Report {
            initiator_id: NODE_IDS[1],
            responder_id: NODE_IDS[2],
            sequence: 42,
            distance_mm: 2_000,
        };
        let wrong_direction = CalibrationPacket::Report {
            initiator_id: NODE_IDS[2],
            responder_id: NODE_IDS[1],
            sequence: 42,
            distance_mm: 2_000,
        };
        let final_packet = CalibrationPacket::Final {
            initiator_id: NODE_IDS[1],
            responder_id: NODE_IDS[2],
            sequence: 42,
            poll_tx: 1,
            response_rx: 2,
            final_tx: 3,
        };

        assert!(report_matches(&expected, NODE_IDS[1], NODE_IDS[2], 42));
        assert!(!report_matches(
            &wrong_direction,
            NODE_IDS[1],
            NODE_IDS[2],
            42
        ));
        assert!(!report_matches(&final_packet, NODE_IDS[1], NODE_IDS[2], 42));
    }
}
