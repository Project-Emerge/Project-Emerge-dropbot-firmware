//! DW3000 wire protocol, TDMA timing and ranging maths shared by the robot and anchor firmwares.
//!
//! Radio-touching code lives in `dw3000-hal`; this crate is deliberately dependency-free, and every
//! value that crosses its boundary is a raw `u64`/`u32`/`u16`, never a `dw3000_ng::time::Instant`.
//! That, plus the empty `[dependencies]` in this crate's manifest, is what lets `cargo test` run on
//! the host, and it is the only reason the arithmetic below has regression tests at all.
//!
//! # The network
//!
//! Four anchors at fixed, surveyed points; up to twelve robots on a floor plane. One anchor -- index
//! 0, [`MASTER_ANCHOR_ID`] -- is the timing master and broadcasts a [`Packet::Sync`] once per
//! superframe. Every node derives the superframe's time base from that frame: the master from its
//! own transmit timestamp, everyone else from their receive timestamp.
//!
//! Each robot then owns one [`robot_slot`], in which it broadcasts a single [`Packet::Poll`] that
//! **all four anchors** answer, each in its own sub-slot ([`response_offset`]). The robot ends up
//! with four ranges measured from one transmission, i.e. a genuine simultaneous snapshot rather than
//! four measurements smeared across a superframe.
//!
//! ```text
//! SUPERFRAME = SYNC_GUARD_US + 12 x ROBOT_SLOT_US + SYNC_TAIL_GUARD_US
//!
//! robot slot k:
//!   t=0        robot k --Poll(broadcast)-->  A1 A2 A3 A4
//!   t=R+0*S    A1 --Response(poll_rx, response_tx, master_offset)-->
//!   t=R+1*S    A2 --Response-->
//!   t=R+2*S    A3 --Response-->
//!   t=R+3*S    A4 --Response-->
//! ```
//!
//! # Why the robot initiates, and what that costs
//!
//! The previous revision of this protocol was anchor-initiated three-message DS-TWR with one slot
//! per *(anchor, robot)* pair: 4 x 12 = 48 slots of 5.5 ms, a 274 ms superframe, **3.65 Hz** per
//! robot. Airtime was never the problem -- three frames are about 420 us of a 5500 us slot -- the
//! other 92% was receive-to-transmit turnaround, and the cost scaled as `anchors x robots`.
//!
//! Turning it around drops that to one slot per robot and five frames instead of twelve, for
//! roughly **14 Hz**. It also moves every tight deadline onto the anchors: the robot's only
//! transmission is a delayed send scheduled from the `Sync`, so it never has to turn a reception
//! around inside a budget. That matters because the robot is the *slow* node -- an ESP32-C6 also
//! running Wi-Fi, MQTT, a display and an IMU -- while the anchors are 80 MHz STM32s doing nothing
//! else.
//!
//! The cost is that a two-message exchange is single-sided, so it no longer cancels the two nodes'
//! clock-rate difference: the error is `k/2 * t_reply`, which at an untrimmed 20 ppm and a 3.2 ms
//! reply delay is **9.6 metres**. [`ClockRatioTracker`] is what pays that cost back, at zero
//! airtime; [`distance_mm_ss_twr`] will not compile a caller that forgets to pass its result.

#![no_std]

pub const PAN_ID: u16 = 0x0D57;
pub const MASTER_ANCHOR_ID: u16 = ANCHOR_IDS[0];
/// Sized to the anchors actually installed. Unlike the tag table below, every entry here costs a
/// sub-slot in *every* robot's slot, so an anchor listed but not installed costs
/// `ACTIVE_TAG_COUNT * SUBSLOT_US` per superframe. Keep at least three for a 2D fix; four is what
/// the deployment has, and gives two redundant ranges against two unknowns.
pub const ANCHOR_IDS: [u16; 4] = [0xA001, 0xA002, 0xA003, 0xA004];
pub const TAG_IDS: [u16; 12] = [
    0xB001, 0xB002, 0xB003, 0xB004, 0xB005, 0xB006, 0xB007, 0xB008, 0xB009, 0xB00A, 0xB00B, 0xB00C,
];

pub const ACTIVE_ANCHOR_COUNT: usize = ANCHOR_IDS.len();
pub const ACTIVE_TAG_COUNT: usize = TAG_IDS.len();

/// Fingerprint of the physical-layer settings antenna delay was calibrated against.
///
/// Ordered fields: channel 5, 64 MHz PRF, 128-symbol preamble, 6.8 Mbit/s, preamble code 9,
/// IEEE short SFD, STS disabled, PAC 8. `dw3000-hal::radio_config` is the authoritative mapping to
/// driver enums. Any change there must change this list, which deliberately invalidates robot NVS
/// calibration records rather than applying a delay measured under another PHY.
pub const PHY_FINGERPRINT: u32 = fingerprint_words(&[5, 64, 128, 6_800, 9, 0, 0, 8]);

const fn fingerprint_words(words: &[u32]) -> u32 {
    let mut hash = 0x811C_9DC5u32;
    let mut i = 0;
    while i < words.len() {
        let bytes = words[i].to_le_bytes();
        let mut b = 0;
        while b < bytes.len() {
            hash ^= bytes[b] as u32;
            hash = hash.wrapping_mul(0x0100_0193);
            b += 1;
        }
        i += 1;
    }
    hash
}

/// Air time of the longest frame in the protocol, rounded up.
///
/// Channel 5, 64 MHz PRF, 128-symbol preamble, 6.8 Mbps: 128 preamble symbols at 1017.63 ns
/// (130.3 us), an 8-symbol IEEE short SFD (8.1 us), a 21.0 us PHR, and the data phase. The longest
/// frame is a [`Packet::Response`]: 24 payload bytes, plus the 9-byte 802.15.4 header `dw3000-ng`
/// wraps everything in, plus its footer and pad bytes, plus the two-octet CRC -- about 37 bytes,
/// which at 6.8 Mbps with Reed-Solomon parity is roughly 50 us. That totals ~210 us; 280 us leaves
/// room for the inter-frame spacing the radio inserts and for a longer payload later.
///
/// Up from 200 us with the preamble, which `dw3000_hal::radio_config` explains the reasoning for.
/// Note what this does *not* have to drag with it: [`SUBSLOT_US`] is sized by a frame's air time
/// plus the robot's frame readout, and the readout is the larger term and is unchanged -- the
/// payload is the same 24 bytes however long the preamble in front of it is.
pub const MAX_FRAME_AIRTIME_US: u32 = 280;

/// How far ahead of the current radio time the master programs its first `Sync`.
///
/// Only the *first* one: every subsequent `Sync` is scheduled a whole [`SUPERFRAME_US`] after the
/// previous one's transmit timestamp, so the grid never accumulates the master's software latency.
/// This value therefore only has to cover the one-off cost of programming a delayed send from a
/// freshly read `SYS_TIME`.
pub const SYNC_SCHEDULE_GUARD_US: u32 = 1_500;

/// From the `Sync` frame's transmit timestamp to the first robot slot.
///
/// Must cover the `Sync`'s own air time plus the time every slave anchor needs to read the frame
/// out, work out its sub-slot instants and arm its receiver for robot slot 0. Down from 5000 us in
/// the anchor-initiated revision, where it also had to absorb a much coarser grid.
///
/// Also has to clear `tasks::uwb_ranging::POLL_PROGRAM_LEAD_US` with room to spare: `robot_slot(0)`
/// *is* this constant, so whatever margin it leaves above that lead is the only pre-send scheduling
/// slack tag_index 0 gets before its delayed `Poll` deadline. At the old 2000 us the two were
/// exactly equal, robot_slot(0) - LEAD == 0, so tag_index 0 alone got no explicit wait at all and
/// leaned on `Timer::at` quietly absorbing an already-elapsed deadline instead of getting the same
/// sleep-then-send schedule every other robot gets -- which is exactly what made that one robot's
/// `poll_tx_failed` rate far higher than its fleetmates'.
pub const SYNC_GUARD_US: u32 = 3_000;

/// From the end of the last robot slot to the next `Sync`.
///
/// Exists so the master has somewhere to program the next `Sync`'s delayed send after finishing its
/// own last sub-slot; a slot boundary that ran right up to the next `Sync` would make the grid
/// depend on the master's software latency again.
pub const SYNC_TAIL_GUARD_US: u32 = 2_000;

/// Turnaround budget between a robot's `Poll` arriving and the first anchor's `Response` going out.
///
/// This is the one genuinely tight deadline left in the protocol, and it sits on the anchors, which
/// is the point of the robot-initiated design. The measurement that sizes it is the anchor's
/// `max_response_setup_us`: `dw3000-ng` moves frame buffers as 129-byte block transfers and brackets
/// each exchange with a dozen smaller register accesses and two busy-wait loops, which on an 80 MHz
/// core with 20 MHz SPI is expected to land around 400-600 us. 800 us is sized against a pessimistic
/// version of that; tighten it once the anchors report their real figure.
pub const RESPONSE_GUARD_US: u32 = 800;

/// Spacing between consecutive anchors' `Response` frames.
///
/// Bounded below by the frame's own air time plus how long the *robot* takes to read one frame out
/// before the next arrives, which is the same `dw3000-ng` block-transfer cost seen from the other
/// side. Note that only the anchor in the first sub-slot has to meet [`RESPONSE_GUARD_US`]; the
/// others get that plus their sub-slot offset, which is why [`response_offset`] rotates the order.
///
/// Up from 700 us, where every superframe capped at two responses however many anchors answered.
///
/// That was originally read as the robot's arm-decode-rearm cycle -- reading a frame out over SPI,
/// not just its air time -- overrunning 700 us on this RV32IMAC core and eating into the next
/// sub-slot before the receiver could rearm. `LOG_RECEPTION_TIMING` has since measured that cycle
/// directly at **100-330 us**, comfortably inside even the old 700, so a slow readout cannot have
/// been the whole story; a hard cap at exactly two also fits it badly, since a merely slow loop
/// would lose a varying number.
///
/// What fits both observations is the cascade [`RESPONSE_BLOCK_US`] describes: lose the alignment
/// between attempt `n` and sub-slot `n` once, and with fixed per-attempt timeouts every later
/// attempt stays a sub-slot late for the rest of the superframe. Tighter sub-slots make that first
/// slip likelier, which is why widening them helped. The deadline-derived timeouts now address the
/// cascade itself, so this may well be reducible again -- but only against a fresh measurement, and
/// there is no rate problem today that would pay for the risk.
pub const SUBSLOT_US: u32 = 1_000;

/// Margin between one robot's last sub-slot and the next robot's `Poll`.
pub const INTER_SLOT_GUARD_US: u32 = 400;

/// How long a receiver stays armed for a frame it expects imminently.
pub const RX_TIMEOUT_US: u32 = 3_000;

/// DW3000 clock ticks per microsecond: the counter runs at 499.2 MHz * 128.
pub const TICKS_PER_US: u64 = 63_898;

/// One robot's whole exchange: its `Poll`, the anchor turnaround, the four `Response` sub-slots, and
/// a margin before the next robot.
pub const ROBOT_SLOT_US: u32 = MAX_FRAME_AIRTIME_US
    + RESPONSE_GUARD_US
    + ACTIVE_ANCHOR_COUNT as u32 * SUBSLOT_US
    + INTER_SLOT_GUARD_US;

/// The position update rate, and now a function of the *robot* count alone -- the anchor count only
/// affects the width of one slot, not how many there are.
///
/// At the constants above: 3000 + 12 x 5400 + 2000 = 69_800 us, i.e. **~14.3 Hz** per robot, with all
/// four ranges of a fix taken from a single transmission. Shrinking [`TAG_IDS`] to the robots
/// actually deployed still helps proportionally (6 robots -> 37.4 ms -> ~27 Hz), but no longer has
/// to: twelve provisioned robots already clear the rate the fusion estimator needs.
///
/// If the tables grow enough that a superframe would overflow `u32`, this const-evaluates to a build
/// error instead of silently wrapping.
pub const SUPERFRAME_US: u32 =
    SYNC_GUARD_US + ACTIVE_TAG_COUNT as u32 * ROBOT_SLOT_US + SYNC_TAIL_GUARD_US;

// The tail guard has to be wide enough to program the next Sync in, and the whole grid has to fit
// inside the superframe. Both are arithmetic consequences of the constants above, so they are
// checked here rather than left to be discovered as a missed deadline on air.
const _: () = assert!(
    SYNC_TAIL_GUARD_US >= MAX_FRAME_AIRTIME_US,
    "the tail guard must at least cover the Sync frame's own air time"
);
const _: () = assert!(
    ROBOT_SLOT_US > MAX_FRAME_AIRTIME_US + RESPONSE_GUARD_US,
    "a robot slot must outlast its own Poll and the anchor turnaround"
);

/// 40-bit DW3000 timestamp wraparound mask, i.e. `dw3000_ng::time::TIME_MAX`. Copied rather than
/// imported -- see the module docs on why this crate has no dependencies.
const TIME_MASK: u64 = 0xFF_FFFF_FFFF;

const MAGIC: u8 = 0xD3;
/// Bumped to 2 for the robot-initiated schedule: `Request` is gone, `Poll` no longer carries an
/// anchor ID, and `Response` carries `master_offset`. A v1 node and a v2 node reject each other's
/// frames at the header rather than misreading them.
const VERSION: u8 = 2;
/// Separate header for the temporary three-node DS-TWR calibration fixture. Production nodes reject
/// these frames before reading a kind byte, so a fixture accidentally powered near an arena cannot
/// be mistaken for a normal TDMA participant.
const CALIBRATION_MAGIC: u8 = 0xC4;
const CALIBRATION_VERSION: u8 = 1;
const TIME_BYTES: usize = 5;
/// Nanometres of flight per DW3000 clock tick: the counter runs at 499.2 MHz * 128, so one tick is
/// 15.65 ps, i.e. 4.691764 mm at the speed of light. The numerator is therefore in *nanometres* and
/// the denominator must scale it to millimetres -- dividing by 1e9 instead yields metres from a
/// function named `distance_mm`, under-reporting every distance by 1000x.
const MM_PER_DWT_NUM: i128 = 4_691_764;
const MM_PER_DWT_DEN: i128 = 1_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum PacketKind {
    Sync = 1,
    Poll = 2,
    Response = 3,
}

impl TryFrom<u8> for PacketKind {
    type Error = PacketError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Sync),
            2 => Ok(Self::Poll),
            3 => Ok(Self::Response),
            _ => Err(PacketError::UnknownKind),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PacketError {
    InvalidHeader,
    InvalidLength,
    UnknownKind,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Packet {
    /// Master anchor -> everyone: starts a superframe.
    Sync {
        sequence: u16,
        /// The master's own transmit timestamp for this frame, in the master's clock.
        ///
        /// Carried in the frame rather than left implicit because the `Sync` is a *delayed* send, so
        /// the master knows the instant before transmitting it. That is what lets any listener
        /// relate its own clock to the master's, which is what
        /// [`Packet::Response::master_offset`](Packet::Response) is built from.
        sync_tx: u64,
    },
    /// Robot -> all anchors, broadcast: opens this robot's slot.
    ///
    /// No `anchor_id`: one `Poll` is answered by every anchor, which is the whole point of the
    /// robot-initiated schedule.
    Poll {
        tag_id: u16,
        sequence: u16,
        /// The robot's own transmit timestamp for this frame, in the robot's clock.
        ///
        /// Included so a passive listener can reconstruct the exchange; the robot itself uses the
        /// hardware timestamp its own `s_wait` returned, which is the same value.
        poll_tx: u64,
    },
    /// Anchor -> robot: the two timestamps only this anchor can measure, plus its clock offset.
    Response {
        anchor_id: u16,
        tag_id: u16,
        sequence: u16,
        /// When this anchor received the `Poll`, in this anchor's clock.
        poll_rx: u64,
        /// When this anchor will transmit this frame, in this anchor's clock.
        ///
        /// Predicted, not measured: a frame cannot carry a timestamp read after its own
        /// transmission. See [`delayed_tx_timestamp`].
        response_tx: u64,
        /// This anchor's tick for the `Sync` epoch minus the master's tick for the same epoch, i.e.
        /// `sync_rx - sync_tx`, masked into the 40-bit range.
        ///
        /// Zero on the master by definition. Note this is the clock offset **plus** the master-to-
        /// anchor flight time (up to ~2500 ticks across a 12 m room); a consumer that has the anchor
        /// survey can subtract the latter, and one that does not should treat it as a bound.
        ///
        /// Nothing in the current schedule reads this. It is here so that the passive-listening
        /// extension -- a robot reconstructing the anchor clocks from *another* robot's slot, which
        /// turns the same frames into a much higher-rate TDoA stream at zero extra air time -- is a
        /// purely additive change rather than another wire-format break.
        master_offset: u64,
    },
}

impl Packet {
    pub const MAX_LEN: usize = 24;

    pub fn encode(self, out: &mut [u8; Self::MAX_LEN]) -> usize {
        out.fill(0);
        out[0] = MAGIC;
        out[1] = VERSION;

        match self {
            Self::Sync { sequence, sync_tx } => {
                out[2] = PacketKind::Sync as u8;
                write_u16(&mut out[3..5], sequence);
                write_time(&mut out[5..10], sync_tx);
                10
            }
            Self::Poll {
                tag_id,
                sequence,
                poll_tx,
            } => {
                out[2] = PacketKind::Poll as u8;
                write_u16(&mut out[3..5], tag_id);
                write_u16(&mut out[5..7], sequence);
                write_time(&mut out[7..12], poll_tx);
                12
            }
            Self::Response {
                anchor_id,
                tag_id,
                sequence,
                poll_rx,
                response_tx,
                master_offset,
            } => {
                out[2] = PacketKind::Response as u8;
                write_u16(&mut out[3..5], anchor_id);
                write_u16(&mut out[5..7], tag_id);
                write_u16(&mut out[7..9], sequence);
                write_time(&mut out[9..14], poll_rx);
                write_time(&mut out[14..19], response_tx);
                write_time(&mut out[19..24], master_offset);
                24
            }
        }
    }

    /// Decodes a packet from a received frame payload.
    ///
    /// Lengths are checked as lower bounds, not exact matches: `dw3000-ng` appends a footer byte and
    /// a pad byte to every frame it sends, and its `r_wait` reports `RXFLEN` (which includes the
    /// two-octet CRC) without trimming it, so `Ieee802154Frame::payload()` always hands back four
    /// bytes more than were encoded. Every field sits at a fixed offset, so ignoring the trailing
    /// bytes is safe -- requiring an exact length instead rejected every frame on every node.
    pub fn decode(data: &[u8]) -> Result<Self, PacketError> {
        if data.len() < 3 || data[0] != MAGIC || data[1] != VERSION {
            return Err(PacketError::InvalidHeader);
        }

        match PacketKind::try_from(data[2])? {
            PacketKind::Sync if data.len() >= 10 => Ok(Self::Sync {
                sequence: read_u16(&data[3..5]),
                sync_tx: read_time(&data[5..10]),
            }),
            PacketKind::Poll if data.len() >= 12 => Ok(Self::Poll {
                tag_id: read_u16(&data[3..5]),
                sequence: read_u16(&data[5..7]),
                poll_tx: read_time(&data[7..12]),
            }),
            PacketKind::Response if data.len() >= 24 => Ok(Self::Response {
                anchor_id: read_u16(&data[3..5]),
                tag_id: read_u16(&data[5..7]),
                sequence: read_u16(&data[7..9]),
                poll_rx: read_time(&data[9..14]),
                response_tx: read_time(&data[14..19]),
                master_offset: read_time(&data[19..24]),
            }),
            _ => Err(PacketError::InvalidLength),
        }
    }
}

/// Frame kind used only by the three-node antenna-delay calibration fixture.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CalibrationPacketKind {
    Sync = 1,
    Poll = 2,
    Response = 3,
    Final = 4,
    Report = 5,
}

impl TryFrom<u8> for CalibrationPacketKind {
    type Error = PacketError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Sync),
            2 => Ok(Self::Poll),
            3 => Ok(Self::Response),
            4 => Ok(Self::Final),
            5 => Ok(Self::Report),
            _ => Err(PacketError::UnknownKind),
        }
    }
}

/// Clock-offset-free DS-TWR exchange used to create and validate antenna-delay references.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CalibrationPacket {
    Sync {
        sequence: u16,
    },
    Poll {
        initiator_id: u16,
        responder_id: u16,
        sequence: u16,
        poll_tx: u64,
    },
    Response {
        initiator_id: u16,
        responder_id: u16,
        sequence: u16,
        poll_rx: u64,
        response_tx: u64,
    },
    Final {
        initiator_id: u16,
        responder_id: u16,
        sequence: u16,
        poll_tx: u64,
        response_rx: u64,
        final_tx: u64,
    },
    Report {
        initiator_id: u16,
        responder_id: u16,
        sequence: u16,
        distance_mm: u32,
    },
}

impl CalibrationPacket {
    pub const MAX_LEN: usize = 24;

    pub fn encode(self, out: &mut [u8; Self::MAX_LEN]) -> usize {
        out.fill(0);
        out[0] = CALIBRATION_MAGIC;
        out[1] = CALIBRATION_VERSION;
        match self {
            Self::Sync { sequence } => {
                out[2] = CalibrationPacketKind::Sync as u8;
                write_u16(&mut out[3..5], sequence);
                5
            }
            Self::Poll {
                initiator_id,
                responder_id,
                sequence,
                poll_tx,
            } => {
                out[2] = CalibrationPacketKind::Poll as u8;
                write_u16(&mut out[3..5], initiator_id);
                write_u16(&mut out[5..7], responder_id);
                write_u16(&mut out[7..9], sequence);
                write_time(&mut out[9..14], poll_tx);
                14
            }
            Self::Response {
                initiator_id,
                responder_id,
                sequence,
                poll_rx,
                response_tx,
            } => {
                out[2] = CalibrationPacketKind::Response as u8;
                write_u16(&mut out[3..5], initiator_id);
                write_u16(&mut out[5..7], responder_id);
                write_u16(&mut out[7..9], sequence);
                write_time(&mut out[9..14], poll_rx);
                write_time(&mut out[14..19], response_tx);
                19
            }
            Self::Final {
                initiator_id,
                responder_id,
                sequence,
                poll_tx,
                response_rx,
                final_tx,
            } => {
                out[2] = CalibrationPacketKind::Final as u8;
                write_u16(&mut out[3..5], initiator_id);
                write_u16(&mut out[5..7], responder_id);
                write_u16(&mut out[7..9], sequence);
                write_time(&mut out[9..14], poll_tx);
                write_time(&mut out[14..19], response_rx);
                write_time(&mut out[19..24], final_tx);
                24
            }
            Self::Report {
                initiator_id,
                responder_id,
                sequence,
                distance_mm,
            } => {
                out[2] = CalibrationPacketKind::Report as u8;
                write_u16(&mut out[3..5], initiator_id);
                write_u16(&mut out[5..7], responder_id);
                write_u16(&mut out[7..9], sequence);
                write_u32(&mut out[9..13], distance_mm);
                13
            }
        }
    }

    pub fn decode(data: &[u8]) -> Result<Self, PacketError> {
        if data.len() < 3 || data[0] != CALIBRATION_MAGIC || data[1] != CALIBRATION_VERSION {
            return Err(PacketError::InvalidHeader);
        }
        match CalibrationPacketKind::try_from(data[2])? {
            CalibrationPacketKind::Sync if data.len() >= 5 => Ok(Self::Sync {
                sequence: read_u16(&data[3..5]),
            }),
            CalibrationPacketKind::Poll if data.len() >= 14 => Ok(Self::Poll {
                initiator_id: read_u16(&data[3..5]),
                responder_id: read_u16(&data[5..7]),
                sequence: read_u16(&data[7..9]),
                poll_tx: read_time(&data[9..14]),
            }),
            CalibrationPacketKind::Response if data.len() >= 19 => Ok(Self::Response {
                initiator_id: read_u16(&data[3..5]),
                responder_id: read_u16(&data[5..7]),
                sequence: read_u16(&data[7..9]),
                poll_rx: read_time(&data[9..14]),
                response_tx: read_time(&data[14..19]),
            }),
            CalibrationPacketKind::Final if data.len() >= 24 => Ok(Self::Final {
                initiator_id: read_u16(&data[3..5]),
                responder_id: read_u16(&data[5..7]),
                sequence: read_u16(&data[7..9]),
                poll_tx: read_time(&data[9..14]),
                response_rx: read_time(&data[14..19]),
                final_tx: read_time(&data[19..24]),
            }),
            CalibrationPacketKind::Report if data.len() >= 13 => Ok(Self::Report {
                initiator_id: read_u16(&data[3..5]),
                responder_id: read_u16(&data[5..7]),
                sequence: read_u16(&data[7..9]),
                distance_mm: read_u32(&data[9..13]),
            }),
            _ => Err(PacketError::InvalidLength),
        }
    }
}

/// When `tag_index`'s slot opens, in microseconds after the `Sync` frame's hardware timestamp.
pub const fn robot_slot(tag_index: usize) -> u32 {
    SYNC_GUARD_US + (tag_index as u32 * ROBOT_SLOT_US)
}

/// When `anchor_index` transmits its `Response`, in microseconds after the robot slot opened.
///
/// The sub-slot order is **rotated by `sequence`**. Because `response_tx` is derived from the
/// schedule rather than from `poll_rx`, the anchor in the first sub-slot has only
/// [`RESPONSE_GUARD_US`] to read the `Poll` out and program its reply, while each later one gets an
/// extra [`SUBSLOT_US`] of slack. Rotating means no single anchor is permanently the one on the
/// tightest deadline, so a marginal turnaround shows up as an evenly spread low failure rate across
/// all four -- which is diagnosable -- rather than as one anchor that always looks broken.
///
/// The rotation is `(anchor_index + sequence) mod ACTIVE_ANCHOR_COUNT`, a bijection for any fixed
/// `sequence`, so two anchors can never land in the same sub-slot.
pub const fn response_offset(anchor_index: usize, sequence: u16) -> u32 {
    let subslot = response_subslot(anchor_index, sequence);
    MAX_FRAME_AIRTIME_US + RESPONSE_GUARD_US + (subslot as u32 * SUBSLOT_US)
}

/// Which sub-slot `anchor_index` answers in during the superframe numbered `sequence`.
///
/// The rotation itself, split out of [`response_offset`] because it is also a *diagnostic*. An
/// anchor's reply delay -- the `response_tx - poll_rx` term the single-sided exchange has to correct
/// for -- is [`RESPONSE_GUARD_US`] plus this index times [`SUBSLOT_US`], so it varies more than
/// fourfold across four consecutive superframes for one and the same anchor.
///
/// That matters because the residual clock-rate error left by [`ClockRatioTracker`] enters
/// [`distance_mm_ss_twr`] scaled by exactly that delay: `error = ratio_error * t_reply / 2 * c`,
/// which at today's constants is about 0.13 mm per ppb in the first sub-slot and 0.58 mm per ppb in
/// the last. A range whose value tracks this index is therefore a clock-correction problem, not an
/// RF one -- an anchor that is genuinely being heard through a reflection has no reason to care
/// which sub-slot it was asked to answer in. Logging the two together is what separates the two
/// explanations, so the robot needs the index rather than only the offset it feeds.
pub const fn response_subslot(anchor_index: usize, sequence: u16) -> usize {
    (anchor_index + sequence as usize) % ACTIVE_ANCHOR_COUNT
}

/// How long a robot should keep its receiver armed for one `Response` sub-slot.
///
/// One sub-slot plus a frame's air time: the frame starts at the sub-slot boundary and ends
/// [`MAX_FRAME_AIRTIME_US`] later, and the receive timestamp is only reported once it has fully
/// arrived.
pub const SUBSLOT_RX_TIMEOUT_US: u32 = SUBSLOT_US + MAX_FRAME_AIRTIME_US;

/// How long after its own `Poll` clears the air a robot can still expect a `Response`.
///
/// The anchor turnaround, every sub-slot, and the last frame's own air time. This is the *absolute*
/// end of one robot's reply window, and it is what a receive timeout should be derived from --
/// rather than giving each of the four attempts a fixed [`SUBSLOT_RX_TIMEOUT_US`].
///
/// The two agree exactly as long as the robot keeps pace, and differ only when it does not, which
/// is the case that matters. A fixed per-attempt duration silently assumes attempt `n` is waiting
/// for sub-slot `n`; lose that alignment once -- one preemption between reading the clock and the
/// receiver actually arming is enough -- and every later attempt is armed a sub-slot late, so the
/// robot keeps listening after the block has ended and drops a range it could have had. Measured on
/// hardware: a single ~900 us stall cost one anchor and then left the loop a full sub-slot behind
/// for the rest of the superframe. Deriving every timeout from this deadline instead makes a stall
/// cost only the frames that physically passed during it, because a late attempt simply gets a
/// shorter window aimed at what is left.
///
/// Nothing is lost by the change: a robot attributes each `Response` by the anchor ID it carries,
/// never by which attempt caught it, so attempt `n` never had to correspond to sub-slot `n`.
pub const RESPONSE_BLOCK_US: u32 =
    RESPONSE_GUARD_US + ACTIVE_ANCHOR_COUNT as u32 * SUBSLOT_US + MAX_FRAME_AIRTIME_US;

/// The tick the DW3000 will report for a delayed transmission programmed at `scheduled_ticks`, given
/// `tx_antenna_delay` (in DW3000 ticks).
///
/// `DX_TIME` says when the frame leaves the digital transmitter, but the timestamp the radio reports
/// -- the one ranging must use -- is that instant plus the configured TX antenna delay. This matters
/// only where a timestamp has to be written into the frame being sent, since the true value cannot
/// be read back until after transmission; everywhere else, prefer the hardware timestamp the
/// driver's `s_wait` returns.
///
/// `tx_antenna_delay` is a parameter rather than a crate constant because each board needs its own
/// calibrated value: the robot's DWM3000 sits next to two motors, a battery and a Wi-Fi antenna, the
/// anchors' do not.
///
/// Feeding the programmed instant into the ranging maths instead skews the reply term and, at
/// millisecond reply delays, inflates the result by tens of metres at nominal antenna delays.
pub fn delayed_tx_timestamp(scheduled_ticks: u64, tx_antenna_delay: u16) -> u64 {
    (scheduled_ticks + tx_antenna_delay as u64) & TIME_MASK
}

/// `base_ticks` plus `delay_us`, rounded down to the DW3000's `DX_TIME` granularity: the register
/// ignores the low 9 bits of the programmed instant, so a caller that doesn't mask them off would be
/// a few ticks earlier than what the radio actually schedules.
pub fn delayed_after(base_ticks: u64, delay_us: u32) -> u64 {
    let raw = (base_ticks + ticks_from_us(delay_us)) & TIME_MASK;
    raw & !0x1ff
}

/// Converts microseconds to DW3000 ticks with the same fixed-point arithmetic as
/// `dw3000_ng::time::Duration::from_nanos` (ticks-per-nanosecond = 638976 / 10000 = 63.8976, i.e.
/// 499.2 MHz * 128), so a caller mixing this crate's tick math with the driver's own `Duration`
/// conversions doesn't accumulate a rounding mismatch between the two.
pub fn ticks_from_us(us: u32) -> u64 {
    let nanos = (us as u64).saturating_mul(1_000);
    (nanos * 638_976 + 5_000) / 10_000
}

/// The ratio between two nodes' clock rates, as a signed offset from 1 in parts per billion.
///
/// `local_rate / remote_rate = 1 + offset_ppb * 1e-9`. Parts per billion rather than a float because
/// this multiplies a 40-bit tick count and the whole point is not to lose the low bits: the useful
/// resolution here is well under a part per million, and [`ClockRatioTracker`] routinely reaches
/// single-digit ppb.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ClockRatio {
    pub offset_ppb: i32,
}

impl ClockRatio {
    /// Two clocks assumed to run at exactly the same rate.
    ///
    /// Only correct as a starting point, before any baseline exists. Two untrimmed crystals differ
    /// by up to ~20 ppm, which at a 3.2 ms reply delay is 9.6 m of range error, so a fix computed
    /// with this should be treated as unusable rather than merely imprecise --
    /// [`ClockRatioTracker::is_converged`] is how a caller tells the difference.
    pub const IDEAL: Self = Self { offset_ppb: 0 };

    /// Converts a duration measured in the remote node's ticks into this node's ticks.
    fn scale(self, remote_ticks: u64) -> i128 {
        let ticks = remote_ticks as i128;
        ticks + (ticks * self.offset_ppb as i128) / 1_000_000_000
    }
}

/// Estimates one remote node's clock rate relative to the local one, from timestamps the protocol
/// already carries.
///
/// # Why this works with no extra frames
///
/// Every `Response` pairs a remote timestamp (`response_tx`, in the anchor's clock) with a local one
/// (its receive timestamp, in the robot's clock). Two such pairs a while apart give the rate ratio
/// directly: `(d_local / d_remote)`. Nothing has to be transmitted for it, and the estimate improves
/// with the baseline rather than with the sample count.
///
/// The noise floor is the robot's own motion, since the pair differs by the flight time as well as
/// by clock rate. Over a 0.5 s baseline at 0.5 m/s that is at most 25 cm, i.e. 0.83 ns, i.e. ~53
/// ticks against a `d_remote` of ~3.2e7 -- under **2 ppb**. Compare the 20 ppm the correction exists
/// to remove: there are four orders of magnitude of headroom, which is why this is worth doing
/// instead of adding a second `Poll` per slot to measure the ratio explicitly.
#[derive(Clone, Copy, Debug, Default)]
pub struct ClockRatioTracker {
    baseline: Option<(u64, u64)>,
    ratio: ClockRatio,
    estimates: u32,
}

impl ClockRatioTracker {
    /// Shortest baseline that produces a usable ratio, in microseconds.
    ///
    /// At 15.65 ps of timestamp resolution, a 50 ms baseline resolves the ratio to about 0.3 ppb in
    /// the absence of motion; the motion term above dominates well before this matters. Roughly one
    /// superframe, so the first estimate lands on the second fix rather than seconds later.
    const MIN_BASELINE_US: u32 = 50_000;

    /// Longest baseline before the reference pair is moved forward, in microseconds.
    ///
    /// Crystal drift is thermal and slow, but not zero, so the baseline has to be refreshed or the
    /// estimate ends up describing an average over a stale window. Also keeps the span far short of
    /// the 40-bit counter's 17.2 s wrap, where `elapsed` would silently alias.
    const MAX_BASELINE_US: u32 = 1_000_000;

    /// Ratios beyond this are rejected as bad samples rather than believed.
    ///
    /// Two untrimmed crystals differ by tens of ppm; 200 ppm is far outside that, so a computed
    /// ratio this large means the input pair is not what it claims -- a missed superframe that
    /// aliased the wrap, a timestamp from a different anchor, a corrupted frame that still passed
    /// CRC. Believing it would poison every subsequent range.
    const MAX_ABS_PPB: i64 = 200_000;

    /// Weight of a new estimate against the running one, as a divisor.
    ///
    /// Each estimate is already averaged over its whole baseline, so this is only smoothing out the
    /// motion noise between baselines, not building the estimate from scratch.
    const BLEND_DIVISOR: i64 = 4;

    /// Folds one `(remote, local)` timestamp pair in and returns the current best ratio.
    ///
    /// Call once per received `Response` with that frame's `response_tx` and its local receive
    /// timestamp, both in raw DW3000 ticks. One tracker per anchor: the four anchors have four
    /// separate crystals, and the `Sync` aligns their epochs, not their rates.
    pub fn update(&mut self, remote_ticks: u64, local_ticks: u64) -> ClockRatio {
        let Some((base_remote, base_local)) = self.baseline else {
            self.baseline = Some((remote_ticks, local_ticks));
            return self.ratio;
        };

        let d_remote = elapsed(remote_ticks, base_remote);
        let d_local = elapsed(local_ticks, base_local);
        if d_remote < ticks_from_us(Self::MIN_BASELINE_US) {
            return self.ratio;
        }

        // (d_local - d_remote) / d_remote, in parts per billion. `d_remote` is at least one
        // superframe of ticks here, so the division cannot be by zero.
        let drift = d_local as i128 - d_remote as i128;
        let ppb = (drift * 1_000_000_000) / d_remote as i128;

        if ppb.unsigned_abs() > Self::MAX_ABS_PPB as u128 {
            // Not a plausible pair. Drop the baseline as well as the sample: whichever of the two is
            // wrong, keeping it would keep producing rejected estimates.
            self.baseline = None;
            return self.ratio;
        }

        let ppb = ppb as i64;
        self.ratio.offset_ppb = if self.estimates == 0 {
            ppb as i32
        } else {
            let previous = self.ratio.offset_ppb as i64;
            (previous + (ppb - previous) / Self::BLEND_DIVISOR) as i32
        };
        self.estimates = self.estimates.saturating_add(1);

        if d_remote >= ticks_from_us(Self::MAX_BASELINE_US) {
            self.baseline = Some((remote_ticks, local_ticks));
        }
        self.ratio
    }

    /// The current ratio, without folding in a new sample.
    pub fn ratio(&self) -> ClockRatio {
        self.ratio
    }

    /// Whether at least one baseline has produced an estimate.
    ///
    /// Until this is true the ratio is [`ClockRatio::IDEAL`], and a range computed from it can be
    /// metres out -- see that constant's docs. A caller should either withhold such a range or mark
    /// it, rather than feeding it to an estimator as if it were a normal measurement.
    pub fn is_converged(&self) -> bool {
        self.estimates > 0
    }

    /// Forgets everything. Call when the radio is re-initialized: the DW3000's counter restarts, so
    /// the stored baseline no longer refers to the same epoch.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Computes the robot-side single-sided two-way-ranging result.
///
/// `poll_tx` and `response_rx` are in the robot's clock, `poll_rx` and `response_tx` in the anchor's.
/// `ratio` converts the latter pair's *duration* into the former's units and is what makes this
/// usable at all: without it the error is `k/2 * t_reply`, or 9.6 m at 20 ppm and a 3.2 ms reply
/// delay. Pass [`ClockRatioTracker::ratio`]; see [`ClockRatio::IDEAL`] for why passing that instead
/// is not a neutral default.
///
/// `None` for a timestamp set that cannot describe a real flight: a non-positive round trip, a reply
/// longer than the round trip, or a result that does not fit `u32` millimetres. These are the
/// sanity gates that stand in for the outlier rejection the fusion estimator does properly.
pub fn distance_mm_ss_twr(
    poll_tx: u64,
    poll_rx: u64,
    response_tx: u64,
    response_rx: u64,
    ratio: ClockRatio,
) -> Option<u32> {
    let round_trip = elapsed(response_rx, poll_tx) as i128;
    let reply = ratio.scale(elapsed(response_tx, poll_rx));
    if round_trip <= 0 || reply < 0 {
        return None;
    }

    // Two flights and one turnaround make up the round trip, so halving what is left after removing
    // the turnaround gives one flight.
    let tof = (round_trip - reply) / 2;
    if tof < 0 {
        return None;
    }
    u32::try_from(tof * MM_PER_DWT_NUM / MM_PER_DWT_DEN).ok()
}

/// A per-anchor range correction: a constant offset plus a distance-proportional term.
///
/// # Why both terms
///
/// The constant term is the antenna-delay residual. `AntennaDelay::NOMINAL` is Qorvo's nominal value
/// for this PHY configuration, not a measurement, and it leaves a fixed bias of up to a few tens of
/// centimetres -- on its own enough to miss a 10-20 cm accuracy target. It is per anchor because each
/// anchor is a separate board with its own antenna and its own trace lengths.
///
/// The proportional term catches what the constant one cannot: the DW3000's reported first-path index
/// walks slightly with received power, so the residual bias after removing the offset is not flat with
/// distance. Fitting `offset + scale * range` over a few surveyed distances removes most of what is
/// left.
///
/// Both are *what to subtract*, so a positive `offset_mm` means the anchor reads long.
///
/// # Calibration procedure
///
/// Two knobs, applied at different ends, and it matters which:
///
/// 1. **Antenna delay** lives in the radio and applies to both ends of an exchange, so changing it by
///    N ticks moves the reported distance by roughly 2N ticks, i.e. ~9.4N mm at 4.69 mm per tick. Set
///    it once per board so a range at a mid-arena distance reads about right.
/// 2. **This correction** lives on the robot and is fitted afterwards, from raw ranges published on
///    `/uwb/{ID}` at several surveyed distances. Least-squares a line through
///    `(true_range, measured - true_range)`: the intercept is `offset_mm`, the slope is `scale_ppm`.
///
/// Doing it in the other order does not converge: the antenna delay is what makes the residual close
/// to a straight line in the first place.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RangeBias {
    pub offset_mm: i32,
    /// Parts per million of the measured range to subtract.
    pub scale_ppm: i32,
}

impl RangeBias {
    /// No correction. What an uncalibrated deployment runs with, and what the accuracy target is
    /// **not** reachable under -- see the type's docs.
    pub const NONE: Self = Self {
        offset_mm: 0,
        scale_ppm: 0,
    };

    /// Applies the correction, saturating at zero: a correction larger than the measurement itself
    /// means the calibration is wrong, and a wrapped `u32` would turn that into a range of thousands
    /// of kilometres rather than an obviously bogus zero.
    pub fn correct(self, raw_mm: u32) -> u32 {
        let raw = i64::from(raw_mm);
        let corrected =
            raw - i64::from(self.offset_mm) - (raw * i64::from(self.scale_ppm)) / 1_000_000;
        u32::try_from(corrected.max(0)).unwrap_or(u32::MAX)
    }
}

/// Computes the asymmetric double-sided two-way-ranging result, from all six timestamps of a
/// three-message exchange.
///
/// Not used by the current schedule, which is two-message and therefore single-sided. It is kept
/// because it is the *clock-offset-free* formulation: it cancels the two nodes' rate difference
/// algebraically instead of correcting for it, so it is the reference [`distance_mm_ss_twr`] is
/// validated against in this crate's tests. A disagreement between the two over the same synthetic
/// geometry means the clock correction is wrong, which is exactly the failure that is invisible on
/// hardware -- it looks like a distance, just the wrong one.
pub fn distance_mm_ds_twr(
    poll_tx: u64,
    poll_rx: u64,
    request_tx: u64,
    request_rx: u64,
    response_tx: u64,
    response_rx: u64,
) -> Option<u32> {
    let round_anchor = elapsed(request_rx, poll_tx) as i128;
    let reply_anchor = elapsed(response_tx, request_rx) as i128;
    let round_tag = elapsed(response_rx, request_tx) as i128;
    let reply_tag = elapsed(request_tx, poll_rx) as i128;
    let denominator = round_anchor + reply_anchor + round_tag + reply_tag;
    if denominator <= 0 {
        return None;
    }

    let tof = (round_anchor * round_tag - reply_anchor * reply_tag) / denominator;
    if tof < 0 {
        return None;
    }
    u32::try_from(tof * MM_PER_DWT_NUM / MM_PER_DWT_DEN).ok()
}

/// Converts a signed range bias into the symmetric RX/TX register adjustment for one node.
///
/// With RX and TX assigned the same per-path value, that node contributes one DW timestamp tick to
/// a pair's measured range for every tick of register error. Positive range bias therefore means the
/// configured antenna delay must be increased. The calibration workflow validates the candidate at
/// a second distance rather than relying on this conversion alone.
pub fn symmetric_delay_delta_ticks(range_bias_mm: i32) -> i32 {
    let numerator = i64::from(range_bias_mm) * MM_PER_DWT_DEN as i64;
    let denominator = MM_PER_DWT_NUM as i64;
    let rounded = if numerator >= 0 {
        (numerator + denominator / 2) / denominator
    } else {
        (numerator - denominator / 2) / denominator
    };
    i32::try_from(rounded).unwrap_or(if rounded < 0 { i32::MIN } else { i32::MAX })
}

fn elapsed(later: u64, earlier: u64) -> u64 {
    later.wrapping_sub(earlier) & TIME_MASK
}

fn write_u16(out: &mut [u8], value: u16) {
    out.copy_from_slice(&value.to_le_bytes());
}

fn read_u16(input: &[u8]) -> u16 {
    u16::from_le_bytes([input[0], input[1]])
}

fn write_u32(out: &mut [u8], value: u32) {
    out.copy_from_slice(&value.to_le_bytes());
}

fn read_u32(input: &[u8]) -> u32 {
    u32::from_le_bytes([input[0], input[1], input[2], input[3]])
}

fn write_time(out: &mut [u8], value: u64) {
    out[..TIME_BYTES].copy_from_slice(&(value & TIME_MASK).to_le_bytes()[..TIME_BYTES]);
}

fn read_time(input: &[u8]) -> u64 {
    let mut bytes = [0u8; 8];
    bytes[..TIME_BYTES].copy_from_slice(&input[..TIME_BYTES]);
    u64::from_le_bytes(bytes)
}

/// A cheap fleet-wide fingerprint of the values that make up the TDMA "ABI": the timing constants
/// and the ID tables.
///
/// [`Packet`]'s `VERSION` byte only catches a wire-layout change; it does not catch a `SUBSLOT_US`
/// or `TAG_IDS` change, which desyncs the TDMA grid silently across the whole fleet -- a robot and
/// an anchor built from different revisions of this crate would still decode each other's frames
/// just fine while listening in the wrong windows. Log this at boot and publish it alongside range
/// measurements so a mismatched anchor or robot shows up as a visible fingerprint difference instead
/// of an unexplained rate drop.
pub const PROTOCOL_FINGERPRINT: u32 = const_fingerprint();

const fn const_fingerprint() -> u32 {
    // FNV-1a over the timing constants and ID tables, folded to 32 bits. The exact algorithm doesn't
    // matter -- this only needs to change whenever the two firmwares could silently disagree about
    // the TDMA grid.
    let mut hash: u32 = 0x811c_9dc5;
    hash = fnv_u32(hash, PAN_ID as u32);
    hash = fnv_u32(hash, VERSION as u32);
    hash = fnv_u32(hash, MAX_FRAME_AIRTIME_US);
    hash = fnv_u32(hash, SYNC_GUARD_US);
    hash = fnv_u32(hash, SYNC_TAIL_GUARD_US);
    hash = fnv_u32(hash, RESPONSE_GUARD_US);
    hash = fnv_u32(hash, SUBSLOT_US);
    hash = fnv_u32(hash, INTER_SLOT_GUARD_US);
    hash = fnv_u32(hash, RX_TIMEOUT_US);
    hash = fnv_u32(hash, ACTIVE_ANCHOR_COUNT as u32);
    hash = fnv_u32(hash, ACTIVE_TAG_COUNT as u32);
    let mut i = 0;
    while i < ANCHOR_IDS.len() {
        hash = fnv_u32(hash, ANCHOR_IDS[i] as u32);
        i += 1;
    }
    let mut i = 0;
    while i < TAG_IDS.len() {
        hash = fnv_u32(hash, TAG_IDS[i] as u32);
        i += 1;
    }
    hash
}

const fn fnv_u32(mut hash: u32, value: u32) -> u32 {
    let bytes = value.to_le_bytes();
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u32;
        hash = hash.wrapping_mul(0x0100_0193);
        i += 1;
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The four extra bytes `dw3000-ng` appends to every frame -- see [`Packet::decode`].
    const DRIVER_EXCESS: usize = 4;

    fn round_trip(packet: Packet) -> Packet {
        let mut buffer = [0u8; Packet::MAX_LEN];
        let len = packet.encode(&mut buffer);
        Packet::decode(&buffer[..len]).expect("a freshly encoded packet must decode")
    }

    #[test]
    fn protocol_round_trip() {
        for packet in [
            Packet::Sync {
                sequence: 0x1234,
                sync_tx: 0x12_3456_789A,
            },
            Packet::Poll {
                tag_id: 0xB003,
                sequence: 7,
                poll_tx: 0xFF_FFFF_FFFF,
            },
            Packet::Response {
                anchor_id: 0xA002,
                tag_id: 0xB00C,
                sequence: 0xFFFF,
                poll_rx: 1,
                response_tx: 0x80_0000_0000,
                master_offset: 0x00_0000_2710,
            },
        ] {
            assert_eq!(round_trip(packet), packet);
        }
    }

    fn calibration_round_trip(packet: CalibrationPacket) -> CalibrationPacket {
        let mut buffer = [0u8; CalibrationPacket::MAX_LEN];
        let len = packet.encode(&mut buffer);
        CalibrationPacket::decode(&buffer[..len]).expect("fixture packet must round-trip")
    }

    #[test]
    fn calibration_protocol_round_trip_and_isolation() {
        for packet in [
            CalibrationPacket::Sync { sequence: 9 },
            CalibrationPacket::Poll {
                initiator_id: 0xC001,
                responder_id: 0xC002,
                sequence: 10,
                poll_tx: 0x11_2233_4455,
            },
            CalibrationPacket::Response {
                initiator_id: 0xC001,
                responder_id: 0xC002,
                sequence: 10,
                poll_rx: 1,
                response_tx: 2,
            },
            CalibrationPacket::Final {
                initiator_id: 0xC001,
                responder_id: 0xC002,
                sequence: 10,
                poll_tx: 1,
                response_rx: 2,
                final_tx: 3,
            },
            CalibrationPacket::Report {
                initiator_id: 0xC001,
                responder_id: 0xC002,
                sequence: 10,
                distance_mm: 2_345,
            },
        ] {
            assert_eq!(calibration_round_trip(packet), packet);
            let mut buffer = [0u8; CalibrationPacket::MAX_LEN];
            let len = packet.encode(&mut buffer);
            assert_eq!(
                Packet::decode(&buffer[..len]),
                Err(PacketError::InvalidHeader)
            );
        }

        let production = Packet::Poll {
            tag_id: 0xB001,
            sequence: 1,
            poll_tx: 2,
        };
        let mut buffer = [0u8; Packet::MAX_LEN];
        let len = production.encode(&mut buffer);
        assert_eq!(
            CalibrationPacket::decode(&buffer[..len]),
            Err(PacketError::InvalidHeader)
        );
    }

    #[test]
    fn symmetric_delay_conversion_uses_one_tick_per_node_contribution() {
        assert_eq!(symmetric_delay_delta_ticks(0), 0);
        assert_eq!(symmetric_delay_delta_ticks(47), 10);
        assert_eq!(symmetric_delay_delta_ticks(-47), -10);
        assert_eq!(symmetric_delay_delta_ticks(469), 100);
    }

    #[test]
    fn decode_ignores_driver_trailing_bytes() {
        let packet = Packet::Response {
            anchor_id: 0xA001,
            tag_id: 0xB001,
            sequence: 42,
            poll_rx: 0x11_2233_4455,
            response_tx: 0x66_7788_99AA,
            master_offset: 0,
        };
        let mut buffer = [0u8; Packet::MAX_LEN];
        let len = packet.encode(&mut buffer);

        // What `Ieee802154Frame::payload()` actually hands back: the encoded frame plus the driver's
        // footer, pad and untrimmed CRC bytes, whatever they happen to contain.
        let mut framed = [0xAAu8; Packet::MAX_LEN + DRIVER_EXCESS];
        framed[..len].copy_from_slice(&buffer[..len]);
        assert_eq!(
            Packet::decode(&framed[..len + DRIVER_EXCESS]),
            Ok(packet),
            "a longer-than-encoded payload must still decode; requiring an exact length \
             rejected every frame on every node"
        );
    }

    #[test]
    fn decode_still_rejects_truncated_frames() {
        let mut buffer = [0u8; Packet::MAX_LEN];
        let len = Packet::Response {
            anchor_id: 0xA001,
            tag_id: 0xB001,
            sequence: 1,
            poll_rx: 1,
            response_tx: 2,
            master_offset: 3,
        }
        .encode(&mut buffer);
        assert_eq!(
            Packet::decode(&buffer[..len - 1]),
            Err(PacketError::InvalidLength)
        );
    }

    #[test]
    fn decode_rejects_the_previous_protocol_version() {
        // A v1 frame: same magic, same `Poll` discriminant, version byte 1.
        let frame = [
            MAGIC,
            1,
            PacketKind::Poll as u8,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        assert_eq!(
            Packet::decode(&frame),
            Err(PacketError::InvalidHeader),
            "a v1 node's frames must be rejected at the header, not misread as v2"
        );
    }

    #[test]
    fn robot_slots_are_contiguous_and_fit_the_superframe() {
        for tag_index in 0..ACTIVE_TAG_COUNT {
            let start = robot_slot(tag_index);
            assert_eq!(start, SYNC_GUARD_US + tag_index as u32 * ROBOT_SLOT_US);
            if tag_index > 0 {
                assert_eq!(start - robot_slot(tag_index - 1), ROBOT_SLOT_US);
            }
        }
        let last_end = robot_slot(ACTIVE_TAG_COUNT - 1) + ROBOT_SLOT_US;
        assert_eq!(
            SUPERFRAME_US - last_end,
            SYNC_TAIL_GUARD_US,
            "the master programs the next Sync in exactly this gap"
        );
    }

    #[test]
    fn response_subslots_never_collide_and_rotate() {
        for sequence in 0..=8u16 {
            let mut offsets = [0u32; ACTIVE_ANCHOR_COUNT];
            for (anchor_index, offset) in offsets.iter_mut().enumerate() {
                *offset = response_offset(anchor_index, sequence);
            }
            let mut sorted = offsets;
            sorted.sort_unstable();
            for pair in sorted.windows(2) {
                assert_eq!(
                    pair[1] - pair[0],
                    SUBSLOT_US,
                    "every anchor must get its own sub-slot, sequence {sequence}"
                );
            }
            // The last sub-slot's frame must still finish inside the robot's slot.
            assert!(
                sorted[ACTIVE_ANCHOR_COUNT - 1] + MAX_FRAME_AIRTIME_US <= ROBOT_SLOT_US,
                "sub-slots overflow the robot slot at sequence {sequence}"
            );
        }

        // Rotation actually rotates: anchor 0 is not always first.
        assert_ne!(response_offset(0, 0), response_offset(0, 1));
        assert_eq!(
            response_offset(0, 0),
            response_offset(0, ACTIVE_ANCHOR_COUNT as u16),
            "the rotation has period ACTIVE_ANCHOR_COUNT"
        );
    }

    #[test]
    fn superframe_matches_the_documented_rate() {
        assert_eq!(ROBOT_SLOT_US, 280 + 800 + 4 * 1_000 + 400);
        assert_eq!(SUPERFRAME_US, 3_000 + 12 * ROBOT_SLOT_US + 2_000);
        assert_eq!(SUPERFRAME_US, 70_760);
        // ~14.1 Hz, against the 3.65 Hz of the anchor-initiated revision this replaced. The 128-symbol
        // preamble that bought a quieter first-path timestamp cost 1 ms of superframe to get here.
        assert_eq!(1_000_000 / SUPERFRAME_US, 14);
    }

    #[test]
    fn the_response_block_outlasts_the_last_sub_slot() {
        // The deadline every receive timeout is derived from has to leave the frame in the *last*
        // sub-slot room to arrive and be timestamped; a block that ended on the sub-slot boundary
        // would cut off the anchor that answers last, which is a different anchor every superframe.
        assert_eq!(
            response_subslot(0, 3),
            3,
            "anchor 0 answers last at sequence 3"
        );

        // Both sides in slot-start terms. The block is measured from the robot's `poll_sent_at`,
        // which is the `Poll`'s transmit-done and so never earlier than the slot start; taking it
        // *at* the slot start is therefore the tightest the deadline can ever be.
        let last_frame_ends = response_offset(0, 3) + MAX_FRAME_AIRTIME_US;
        assert!(
            RESPONSE_BLOCK_US >= last_frame_ends,
            "the block ({RESPONSE_BLOCK_US} us) must outlast the last sub-slot's frame \
             ({last_frame_ends} us)"
        );
        assert_eq!(RESPONSE_BLOCK_US, 800 + 4 * 1_000 + 280);
    }

    #[test]
    fn ticks_from_us_matches_driver_scale() {
        // 1 us at 499.2 MHz * 128.
        assert_eq!(ticks_from_us(1), 63_898);
        assert_eq!(ticks_from_us(1_000), 63_897_600);
    }

    #[test]
    fn delayed_after_rounds_to_dx_time_granularity() {
        let base = 0x1234_5678u64;
        let scheduled = delayed_after(base, 1_000);
        assert_eq!(scheduled & 0x1ff, 0, "DX_TIME ignores the low 9 bits");
        assert!(scheduled <= base + ticks_from_us(1_000));
    }

    #[test]
    fn delayed_tx_timestamp_adds_antenna_delay_and_wraps() {
        assert_eq!(delayed_tx_timestamp(1_000, 16_385), 17_385);
        assert_eq!(delayed_tx_timestamp(TIME_MASK, 1), 0);
    }

    /// A flight time of 1 m, in DW3000 ticks: 3.336 ns at c, i.e. 213 ticks.
    const ONE_METRE_TICKS: u64 = 213;

    /// Scales a real duration -- expressed in the robot's ticks, taken as the reference -- into the
    /// number of ticks an anchor whose counter runs `anchor_ppb` faster would report for it.
    ///
    /// A *faster* remote counter reports *more* ticks for the same real duration, so
    /// `ClockRatio::offset_ppb` (which converts the other way) is the negation of `anchor_ppb`.
    fn in_anchor_ticks(robot_ticks: u64, anchor_ppb: i64) -> u64 {
        let t = robot_ticks as i128;
        (t + (t * anchor_ppb as i128) / 1_000_000_000) as u64
    }

    /// Builds a synthetic v2 exchange: robot polls, anchor replies `reply_us` later.
    ///
    /// Returns `(poll_tx, poll_rx, response_tx, response_rx)` exactly as the four values reach
    /// [`distance_mm_ss_twr`] -- the robot's pair in robot ticks, the anchor's pair in anchor ticks,
    /// with unrelated epochs, which is what actually arrives on the wire.
    fn synth_ss_twr(tof_ticks: u64, reply_us: u32, anchor_ppb: i64) -> (u64, u64, u64, u64) {
        let reply = ticks_from_us(reply_us);
        // Deliberately unrelated epochs: only durations are meaningful across the two clocks.
        let poll_tx = 1_000_000u64;
        let poll_rx = 500_000_000u64;
        let response_tx = poll_rx + in_anchor_ticks(reply, anchor_ppb);
        let response_rx = poll_tx + tof_ticks + reply + tof_ticks;
        (poll_tx, poll_rx, response_tx, response_rx)
    }

    /// Builds a synthetic three-message DS-TWR exchange over the same physical geometry.
    ///
    /// This is the v1 role assignment -- anchor initiates, robot replies, anchor responds -- because
    /// that is the exchange [`distance_mm_ds_twr`]'s argument order describes: `poll_tx`,
    /// `request_rx` and `response_tx` are anchor ticks, `poll_rx`, `request_tx` and `response_rx` are
    /// robot ticks.
    fn synth_ds_twr(
        tof_ticks: u64,
        robot_reply_us: u32,
        anchor_reply_us: u32,
        anchor_ppb: i64,
    ) -> (u64, u64, u64, u64, u64, u64) {
        let robot_reply = ticks_from_us(robot_reply_us);
        let anchor_reply = ticks_from_us(anchor_reply_us);
        let anchor_epoch = 900_000_000u64;
        let robot_epoch = 2_000_000u64;

        // Real time, in robot ticks, measured from the anchor's transmission of the Poll.
        let t_poll_rx = tof_ticks;
        let t_request_tx = t_poll_rx + robot_reply;
        let t_request_rx = t_request_tx + tof_ticks;
        let t_response_tx = t_request_rx + anchor_reply;
        let t_response_rx = t_response_tx + tof_ticks;

        (
            anchor_epoch,
            robot_epoch + t_poll_rx,
            robot_epoch + t_request_tx,
            anchor_epoch + in_anchor_ticks(t_request_rx, anchor_ppb),
            anchor_epoch + in_anchor_ticks(t_response_tx, anchor_ppb),
            robot_epoch + t_response_rx,
        )
    }

    #[test]
    fn one_metre_of_flight_reads_about_one_metre() {
        let (poll_tx, poll_rx, response_tx, response_rx) = synth_ss_twr(ONE_METRE_TICKS, 800, 0);
        let mm = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            response_tx,
            response_rx,
            ClockRatio::IDEAL,
        )
        .expect("a well-formed exchange must range");
        assert!(
            (990..=1_010).contains(&mm),
            "expected about 1000 mm, got {mm}"
        );
    }

    #[test]
    fn uncorrected_clock_offset_costs_metres() {
        // The failure this whole correction exists to prevent. 20 ppm is two untrimmed crystals;
        // 3.2 ms is the reply delay of the last anchor sub-slot, i.e. the worst case in the schedule.
        //
        // An anchor running *slow* reports a shorter reply than really elapsed, so the uncorrected
        // result is a plausible-looking number that happens to be metres too large. That direction is
        // the dangerous one: nothing downstream can tell it from a real range.
        let (poll_tx, poll_rx, response_tx, response_rx) =
            synth_ss_twr(ONE_METRE_TICKS, 3_200, -20_000);
        let uncorrected = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            response_tx,
            response_rx,
            ClockRatio::IDEAL,
        )
        .expect("still produces a number, which is exactly the problem");
        assert!(
            uncorrected.abs_diff(1_000) > 5_000,
            "expected metres of error from an uncorrected 20 ppm offset, got {uncorrected} mm"
        );

        // With the ratio the tracker measures, the same exchange ranges correctly.
        let corrected = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            response_tx,
            response_rx,
            ClockRatio { offset_ppb: 20_000 },
        )
        .expect("a well-formed exchange must range");
        assert!(
            corrected.abs_diff(1_000) < 20,
            "expected within 20 mm after correction, got {corrected} mm"
        );
    }

    #[test]
    fn uncorrected_clock_offset_in_the_other_direction_is_rejected() {
        // An anchor running *fast* reports a longer reply than really elapsed, which makes the
        // computed flight negative and trips the sanity gate. Worth pinning: it means half of the
        // uncorrected failures are silent and half are visible, so a deployment that "works" with
        // no correction is only telling you which way its crystals happen to lean.
        let (poll_tx, poll_rx, response_tx, response_rx) =
            synth_ss_twr(ONE_METRE_TICKS, 3_200, 20_000);
        assert_eq!(
            distance_mm_ss_twr(
                poll_tx,
                poll_rx,
                response_tx,
                response_rx,
                ClockRatio::IDEAL
            ),
            None,
        );
        let corrected = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            response_tx,
            response_rx,
            ClockRatio {
                offset_ppb: -20_000,
            },
        )
        .expect("a well-formed exchange must range once corrected");
        assert!(
            corrected.abs_diff(1_000) < 20,
            "expected within 20 mm after correction, got {corrected} mm"
        );
    }

    #[test]
    fn ss_twr_agrees_with_the_clock_offset_free_reference() {
        // Same physical geometry through both formulations. DS-TWR cancels the rate difference
        // algebraically; corrected SS-TWR has to arrive at the same answer, or the correction is
        // wrong -- a failure that on hardware looks like a distance, just the wrong one.
        let tof = 3 * ONE_METRE_TICKS;
        let anchor_ppb = 15_000i64;

        let (poll_tx, poll_rx, response_tx, response_rx) = synth_ss_twr(tof, 2_000, anchor_ppb);
        let ss = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            response_tx,
            response_rx,
            ClockRatio {
                offset_ppb: -anchor_ppb as i32,
            },
        )
        .expect("SS-TWR must range");

        let (ds_poll_tx, ds_poll_rx, ds_request_tx, ds_request_rx, ds_response_tx, ds_response_rx) =
            synth_ds_twr(tof, 1_500, 2_000, anchor_ppb);
        let ds = distance_mm_ds_twr(
            ds_poll_tx,
            ds_poll_rx,
            ds_request_tx,
            ds_request_rx,
            ds_response_tx,
            ds_response_rx,
        )
        .expect("DS-TWR must range");

        assert!(
            ss.abs_diff(ds) < 20,
            "corrected SS-TWR ({ss} mm) must agree with DS-TWR ({ds} mm)"
        );
        assert!(ds.abs_diff(3_000) < 20, "expected ~3 m, got {ds} mm");
    }

    #[test]
    fn programmed_transmit_times_would_inflate_distance() {
        // The regression this crate exists to keep: the anchor writes its *predicted* transmit
        // timestamp into the frame, which is the programmed instant plus the TX antenna delay.
        // Writing the programmed instant instead shortens the measured reply and inflates the range.
        let tof = 213u64;
        let antenna_delay = 16_385u16;
        let poll_tx = 1_000_000u64;
        let poll_rx = 500_000_000u64;
        let scheduled = delayed_after(poll_rx, 2_000);
        let response_rx = poll_tx + tof + (scheduled - poll_rx) + u64::from(antenna_delay) + tof;

        let correct = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            delayed_tx_timestamp(scheduled, antenna_delay),
            response_rx,
            ClockRatio::IDEAL,
        )
        .expect("must range");
        let wrong = distance_mm_ss_twr(poll_tx, poll_rx, scheduled, response_rx, ClockRatio::IDEAL)
            .expect("must range");

        assert!(correct.abs_diff(1_000) < 20, "expected ~1 m, got {correct}");
        // Half the antenna delay of extra flight, i.e. ~8200 ticks, i.e. ~38 m.
        assert!(
            wrong > 30_000,
            "expected tens of metres of inflation, got {wrong} mm"
        );
    }

    #[test]
    fn distance_handles_timestamp_wrap() {
        // The robot's poll goes out just before the 40-bit counter wraps and the response arrives
        // after it.
        let tof = 213u64;
        let reply = ticks_from_us(800);
        let poll_tx = TIME_MASK - 100;
        let poll_rx = 500_000_000u64;
        let response_tx = poll_rx + reply;
        let response_rx = (poll_tx + tof + reply + tof) & TIME_MASK;
        let mm = distance_mm_ss_twr(
            poll_tx,
            poll_rx,
            response_tx,
            response_rx,
            ClockRatio::IDEAL,
        )
        .expect("a wrapped exchange must still range");
        assert!((990..=1_010).contains(&mm), "expected ~1 m, got {mm}");
    }

    #[test]
    fn clock_ratio_tracker_converges_on_a_synthetic_offset() {
        // An anchor clock running 20 ppm fast, sampled once per superframe with the flight time
        // wobbling as a moving robot's would.
        let anchor_ppb = 20_000i64;
        let mut tracker = ClockRatioTracker::default();
        assert!(!tracker.is_converged());
        assert_eq!(tracker.ratio(), ClockRatio::IDEAL);

        let mut remote = 1_000_000u64;
        let mut local = 7_000_000u64;
        let step_local = ticks_from_us(SUPERFRAME_US);
        for i in 0..40u64 {
            // The remote clock advances faster than the local one by `anchor_ppb`.
            let step_remote = (step_local as i128
                + (step_local as i128 * anchor_ppb as i128) / 1_000_000_000)
                as u64;
            remote += step_remote;
            // Flight-time wobble: +/- 53 ticks, the 25 cm of motion the docs bound this by.
            local += step_local + if i % 2 == 0 { 53 } else { 0 };
            tracker.update(remote, local);
        }

        assert!(tracker.is_converged());
        let measured = tracker.ratio().offset_ppb;
        // `local_rate / remote_rate - 1` is -20 ppm for a remote running 20 ppm fast. Within 1 ppm,
        // which at a 3.2 ms reply delay is 1.6 ns, i.e. under 25 cm -- and in practice far better,
        // since the motion wobble here is deliberately worst-case and one-sided.
        assert!(
            (measured + 20_000).abs() < 1_000,
            "expected about -20000 ppb, got {measured}"
        );
    }

    #[test]
    fn clock_ratio_tracker_rejects_implausible_pairs() {
        let mut tracker = ClockRatioTracker::default();
        let step = ticks_from_us(SUPERFRAME_US);
        // Converge on a clean 0 ppb first.
        let mut remote = 1_000_000u64;
        let mut local = 1_000_000u64;
        for _ in 0..10 {
            remote += step;
            local += step;
            tracker.update(remote, local);
        }
        let good = tracker.ratio();
        assert!(tracker.is_converged());

        // Now a pair that implies a 1000 ppm difference: far outside two crystals, so it must be
        // dropped rather than blended in.
        tracker.update(remote + step, local + step * 2);
        assert_eq!(
            tracker.ratio(),
            good,
            "an implausible pair must not move the estimate"
        );
    }

    #[test]
    fn clock_ratio_tracker_needs_a_baseline_before_estimating() {
        let mut tracker = ClockRatioTracker::default();
        // Two samples a single sub-slot apart: far too short a baseline to say anything.
        tracker.update(1_000_000, 1_000_000);
        tracker.update(1_000_000 + ticks_from_us(SUBSLOT_US), 1_000_000);
        assert!(
            !tracker.is_converged(),
            "a sub-millisecond baseline must not produce an estimate"
        );
        assert_eq!(tracker.ratio(), ClockRatio::IDEAL);
    }

    #[test]
    fn range_bias_removes_both_terms() {
        // An anchor reading 120 mm long with a further 2000 ppm of range-proportional drift, i.e.
        // 2 mm per metre.
        let bias = RangeBias {
            offset_mm: 120,
            scale_ppm: 2_000,
        };
        // A true 5 m range would be measured as 5000 + 120 + 10 = 5130.
        assert_eq!(bias.correct(5_130), 5_000);
        // And a true 1 m range as 1000 + 120 + 2 = 1122.
        assert_eq!(bias.correct(1_122), 1_000);
        assert_eq!(RangeBias::NONE.correct(1_234), 1_234);
    }

    #[test]
    fn range_bias_saturates_instead_of_wrapping() {
        // A correction bigger than the measurement means the calibration is wrong. Zero is obviously
        // bogus; a wrapped `u32` would be 4000 km and look almost plausible next to a real range.
        let bias = RangeBias {
            offset_mm: 5_000,
            scale_ppm: 0,
        };
        assert_eq!(bias.correct(100), 0);
    }

    #[test]
    fn protocol_fingerprint_is_stable_and_nonzero() {
        assert_ne!(PROTOCOL_FINGERPRINT, 0);
        assert_eq!(PROTOCOL_FINGERPRINT, const_fingerprint());
    }
}
