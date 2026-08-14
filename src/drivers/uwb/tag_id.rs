//! Maps this robot's device ID to its TDMA tag index.
//!
//! One firmware image serves every robot in the fleet. [`TAG_ASSIGNMENTS`] is the compiled
//! default -- edited and reflashed only for the robot being provisioned, so it does not force a
//! fleet-wide reflash -- and gets ranging going immediately at boot, without depending on the
//! network. [`resolve_from_config`] lets an MQTT-delivered
//! `data::configurations::TagAssignmentsConfiguration` override it later, to reprovision the
//! fleet without touching firmware at all; see `tasks::uwb_ranging` for how the two are
//! combined.
//!
//! Either source is as much a part of the network's "ABI" as `uwb_protocol::PROTOCOL_FINGERPRINT`
//! and the anchors' own tables: two robots sharing a tag index would reply in the same slot and
//! corrupt each other's ranging. An unassigned `device_id` deliberately gets no tag index at all
//! rather than falling back to index 0 -- see [`tag_index`].

use crate::data::configurations::TagAssignment;

/// `(device_id, tag_index)` pairs. `device_id` is the 6 uppercase hex characters
/// `crate::device_id::init` derives from the board's eFuse MAC (also shown on the OLED and in
/// every `laze … attach` boot log), and `tag_index` an index into `uwb_protocol::TAG_IDS`.
///
/// Empty until the fleet is provisioned. Checked for duplicates at compile time -- see the
/// bottom of this file -- so a copy-paste mistake fails the build instead of two robots
/// corrupting each other's ranging on the bench.
pub const TAG_ASSIGNMENTS: &[(&str, usize)] = &[
    // ("A1B2C3", 0),
    ("508040", 0),
];

/// Looks up `device_id`'s tag index in the compiled [`TAG_ASSIGNMENTS`].
///
/// Callers must treat `None` as "do not range" rather than defaulting to an index: two robots
/// answering in the same slot would collide and corrupt each other's measurements as well as
/// their own, which is worse than one robot not ranging at all.
#[must_use]
pub fn tag_index(device_id: &str) -> Option<usize> {
    TAG_ASSIGNMENTS
        .iter()
        .find(|(id, _)| *id == device_id)
        .map(|(_, index)| *index)
}

/// Looks up `device_id`'s tag index in a fleet-wide assignment list delivered over MQTT.
///
/// Rejects the *entire* list -- not just this device's entry -- if it contains a duplicate
/// `device_id` or `tag_index` anywhere: a single malformed publish must not let two robots start
/// answering in the same slot on the strength of a partially-valid list. Returns `None` on
/// rejection or on no entry for `device_id`, either of which should leave the compiled
/// [`TAG_ASSIGNMENTS`] default in effect -- see `tasks::uwb_ranging::resolve_tag_index`.
#[must_use]
pub fn resolve_from_config(device_id: &str, assignments: &[TagAssignment]) -> Option<usize> {
    for (i, a) in assignments.iter().enumerate() {
        for b in &assignments[i + 1..] {
            if a.device_id == b.device_id || a.tag_index == b.tag_index {
                return None;
            }
        }
    }
    assignments
        .iter()
        .find(|a| a.device_id.as_str() == device_id)
        .map(|a| a.tag_index as usize)
}

const fn str_eq(a: &str, b: &str) -> bool {
    let a = a.as_bytes();
    let b = b.as_bytes();
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

// Fails the build, rather than the fleet, on a duplicate `device_id` or `tag_index` in
// `TAG_ASSIGNMENTS`: hand-edited rows are exactly the kind of table a copy-paste slips through,
// and the failure mode on hardware -- two robots answering in the same slot -- is silent
// corruption of both robots' ranging, not an error message.
const _: () = {
    let n = TAG_ASSIGNMENTS.len();
    let mut i = 0;
    while i < n {
        let mut j = i + 1;
        while j < n {
            assert!(
                !str_eq(TAG_ASSIGNMENTS[i].0, TAG_ASSIGNMENTS[j].0),
                "TAG_ASSIGNMENTS: duplicate device_id"
            );
            assert!(
                TAG_ASSIGNMENTS[i].1 != TAG_ASSIGNMENTS[j].1,
                "TAG_ASSIGNMENTS: duplicate tag_index"
            );
            j += 1;
        }
        i += 1;
    }
};
