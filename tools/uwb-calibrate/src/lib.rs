//! Robust host-side solvers used by the DWM3000 calibration CLI.

use std::collections::{BTreeMap, BTreeSet};

use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};

pub const NOMINAL_DELAY_TICKS: i32 = 16_385;
const DWT_TICK_MM: f64 = 4.691_764;
const HUBER_K: f64 = 1.5;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FixtureSample {
    pub session_id: u32,
    pub initiator_id: String,
    pub responder_id: String,
    pub true_distance_m: f64,
    pub measured_distance_m: f64,
    #[serde(default)]
    pub direction: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NodeDelayEstimate {
    pub node_id: String,
    /// Contribution of this node to measured minus true range.
    pub range_bias_m: f64,
    pub delay_delta_ticks: i32,
    pub rx_ticks: u16,
    pub tx_ticks: u16,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ErrorMetrics {
    pub samples: usize,
    pub median_abs_m: f64,
    pub p95_abs_m: f64,
    pub standard_deviation_m: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ResidualPoint {
    pub group: String,
    pub x: f64,
    pub residual_m: f64,
    pub validation: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HardwareReport {
    pub schema_version: u8,
    pub session_id: u32,
    pub nodes: Vec<NodeDelayEstimate>,
    pub training: ErrorMetrics,
    pub validation: ErrorMetrics,
    pub accepted: bool,
    pub residuals: Vec<ResidualPoint>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HardwareValidationReport {
    pub schema_version: u8,
    pub session_id: u32,
    /// Metrics of raw measured-minus-known distance after the candidate delays were flashed.
    pub raw_error: ErrorMetrics,
    pub accepted: bool,
    pub residuals: Vec<ResidualPoint>,
}

/// Scores a post-flash capture without fitting another set of delays.
///
/// This is deliberately separate from [`solve_hardware`]: fitting can make a repeat capture look
/// excellent even when the new registers were never flashed. Validation must score the raw error.
pub fn validate_hardware(
    samples: &[FixtureSample],
    min_samples_per_pair: usize,
) -> Result<HardwareValidationReport> {
    ensure!(!samples.is_empty(), "fixture CSV contains no samples");
    let session_id = samples[0].session_id;
    ensure!(
        samples.iter().all(|sample| sample.session_id == session_id),
        "fixture CSV mixes session IDs"
    );
    let nodes: BTreeSet<_> = samples
        .iter()
        .flat_map(|sample| [&sample.initiator_id, &sample.responder_id])
        .collect();
    ensure!(nodes.len() == 3, "validation requires exactly three nodes");
    let mut pair_counts = BTreeMap::<(&str, &str), usize>::new();
    let mut errors = Vec::with_capacity(samples.len());
    let mut residuals = Vec::with_capacity(samples.len());
    for sample in samples {
        ensure!(
            sample.true_distance_m.is_finite()
                && sample.true_distance_m > 0.0
                && sample.measured_distance_m.is_finite()
                && sample.measured_distance_m > 0.0,
            "invalid validation distance"
        );
        let pair = if sample.initiator_id < sample.responder_id {
            (sample.initiator_id.as_str(), sample.responder_id.as_str())
        } else {
            (sample.responder_id.as_str(), sample.initiator_id.as_str())
        };
        *pair_counts.entry(pair).or_default() += 1;
        let error = sample.measured_distance_m - sample.true_distance_m;
        errors.push(error);
        residuals.push(ResidualPoint {
            group: format!("{}→{}", sample.initiator_id, sample.responder_id),
            x: sample.true_distance_m,
            residual_m: error,
            validation: true,
        });
    }
    ensure!(
        pair_counts.len() == 3,
        "validation capture is missing a fixture pair"
    );
    ensure!(
        pair_counts
            .values()
            .all(|count| *count >= min_samples_per_pair),
        "one or more validation pairs have fewer than {min_samples_per_pair} samples"
    );
    let raw_error = metrics(&errors);
    let accepted = raw_error.median_abs_m <= 0.02
        && raw_error.p95_abs_m <= 0.05
        && raw_error.standard_deviation_m <= 0.05;
    Ok(HardwareValidationReport {
        schema_version: 1,
        session_id,
        raw_error,
        accepted,
        residuals,
    })
}

pub fn solve_hardware(
    samples: &[FixtureSample],
    min_samples_per_pair: usize,
) -> Result<HardwareReport> {
    ensure!(!samples.is_empty(), "fixture CSV contains no samples");
    let session_id = samples[0].session_id;
    ensure!(
        samples.iter().all(|s| s.session_id == session_id),
        "fixture CSV mixes session IDs"
    );

    let nodes: Vec<_> = samples
        .iter()
        .flat_map(|s| [&s.initiator_id, &s.responder_id])
        .cloned()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    ensure!(
        nodes.len() == 3,
        "APS014 fixture requires exactly three distinct nodes, found {}",
        nodes.len()
    );
    let indices: BTreeMap<_, _> = nodes.iter().cloned().zip(0usize..).collect();

    let mut pair_counts = BTreeMap::<(usize, usize), usize>::new();
    let mut rows = Vec::with_capacity(samples.len());
    for sample in samples {
        ensure!(
            sample.true_distance_m.is_finite() && sample.true_distance_m > 0.0,
            "invalid true distance"
        );
        ensure!(
            sample.measured_distance_m.is_finite() && sample.measured_distance_m > 0.0,
            "invalid measured distance"
        );
        let a = indices[&sample.initiator_id];
        let b = indices[&sample.responder_id];
        ensure!(a != b, "fixture sample ranges a node to itself");
        let pair = if a < b { (a, b) } else { (b, a) };
        *pair_counts.entry(pair).or_default() += 1;
        let mut design = vec![0.0; 3];
        design[a] = 1.0;
        design[b] = 1.0;
        rows.push((design, sample.measured_distance_m - sample.true_distance_m));
    }
    for pair in [(0, 1), (0, 2), (1, 2)] {
        ensure!(
            pair_counts.get(&pair).copied().unwrap_or(0) >= min_samples_per_pair,
            "pair {}-{} has fewer than {} valid samples",
            nodes[pair.0],
            nodes[pair.1],
            min_samples_per_pair
        );
    }

    // Deterministic 80/20 hold-out, stratified naturally by the fixture's repeated pair cycle.
    let training_rows: Vec<_> = rows
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 5 != 0)
        .map(|(_, r)| r.clone())
        .collect();
    let beta = irls(&training_rows, 3, HUBER_K, 30)?;
    let mut training_residuals = Vec::new();
    let mut validation_residuals = Vec::new();
    let mut residuals = Vec::with_capacity(samples.len());
    for (index, (sample, (design, observed))) in samples.iter().zip(rows.iter()).enumerate() {
        let residual = observed - dot(design, &beta);
        let validation = index % 5 == 0;
        if validation {
            validation_residuals.push(residual);
        } else {
            training_residuals.push(residual);
        }
        residuals.push(ResidualPoint {
            group: format!("{}→{}", sample.initiator_id, sample.responder_id),
            x: sample.true_distance_m,
            residual_m: residual,
            validation,
        });
    }

    let estimates = nodes
        .iter()
        .zip(beta.iter())
        .map(|(node_id, bias)| {
            let delta = (bias * 1000.0 / DWT_TICK_MM).round() as i32;
            let delay = (NOMINAL_DELAY_TICKS + delta).clamp(0, i32::from(u16::MAX)) as u16;
            NodeDelayEstimate {
                node_id: node_id.clone(),
                range_bias_m: *bias,
                delay_delta_ticks: delta,
                // DS-TWR observes only the aggregate device contribution. Applying the same value
                // to TX and RX is the explicit symmetric split; the data cannot identify two paths.
                rx_ticks: delay,
                tx_ticks: delay,
            }
        })
        .collect();
    let training = metrics(&training_residuals);
    let validation = metrics(&validation_residuals);
    let accepted = validation.median_abs_m <= 0.02
        && validation.p95_abs_m <= 0.05
        && validation.standard_deviation_m <= 0.05;
    Ok(HardwareReport {
        schema_version: 1,
        session_id,
        nodes: estimates,
        training,
        validation,
        accepted,
        residuals,
    })
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SiteSample {
    pub session_id: u32,
    pub robot_id: String,
    pub anchor_id: u16,
    pub robot_x_m: f64,
    pub robot_y_m: f64,
    pub heading_deg: f64,
    pub robot_antenna_height_m: f64,
    pub antenna_offset_x_m: f64,
    pub antenna_offset_y_m: f64,
    pub anchor_x_m: f64,
    pub anchor_y_m: f64,
    pub anchor_z_m: f64,
    pub distance_mm: u32,
    pub response_subslot: u8,
    #[serde(default = "yes")]
    pub clock_ratio_converged: bool,
}

const fn yes() -> bool {
    true
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SiteAnchorEstimate {
    pub anchor_id: u16,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub offset_mm: i32,
    pub scale_ppm: i32,
    pub range_sigma_m: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OrientationDiagnostic {
    pub heading_bucket_deg: i32,
    pub median_residual_m: f64,
    pub samples: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SubslotDiagnostic {
    pub response_subslot: u8,
    pub median_residual_m: f64,
    pub samples: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SiteReport {
    pub schema_version: u8,
    pub session_id: u32,
    pub robot_id: String,
    /// Millimetres the firmware subtracts from all this robot's ranges.
    pub robot_range_offset_mm: i32,
    pub antenna_offset_x_m: f64,
    pub antenna_offset_y_m: f64,
    pub robot_antenna_height_m: f64,
    pub anchors: Vec<SiteAnchorEstimate>,
    pub training: ErrorMetrics,
    pub validation: ErrorMetrics,
    pub accepted: bool,
    pub orientation: Vec<OrientationDiagnostic>,
    pub subslots: Vec<SubslotDiagnostic>,
    pub residuals: Vec<ResidualPoint>,
}

pub fn solve_site(samples: &[SiteSample], min_samples_per_pose: usize) -> Result<SiteReport> {
    let valid: Vec<_> = samples.iter().filter(|s| s.clock_ratio_converged).collect();
    ensure!(
        !valid.is_empty(),
        "site CSV contains no clock-converged samples"
    );
    let first = valid[0];
    ensure!(
        valid.iter().all(|s| s.session_id == first.session_id),
        "site CSV mixes session IDs"
    );
    ensure!(
        valid.iter().all(|s| s.robot_id == first.robot_id),
        "site CSV mixes robots"
    );
    ensure!(
        valid.iter().all(
            |s| (s.antenna_offset_x_m - first.antenna_offset_x_m).abs() < 1e-9
                && (s.antenna_offset_y_m - first.antenna_offset_y_m).abs() < 1e-9
        ),
        "antenna offset changes within the capture"
    );

    let anchor_ids: Vec<u16> = valid
        .iter()
        .map(|s| s.anchor_id)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    ensure!(
        anchor_ids.len() == 4,
        "site fit requires exactly four anchors, found {}",
        anchor_ids.len()
    );
    let anchor_index: BTreeMap<_, _> = anchor_ids.iter().copied().zip(0usize..).collect();
    let mut pose_counts = BTreeMap::<(i64, i64, i32), usize>::new();
    for s in &valid {
        let key = (
            (s.robot_x_m * 1000.0).round() as i64,
            (s.robot_y_m * 1000.0).round() as i64,
            (normalize_degrees(s.heading_deg) / 45.0).round() as i32,
        );
        *pose_counts.entry(key).or_default() += 1;
    }
    ensure!(
        pose_counts
            .values()
            .all(|count| *count >= min_samples_per_pose),
        "one or more site poses have fewer than {min_samples_per_pose} valid samples"
    );

    // beta = robot common, anchor offsets 0..2 (fourth is minus their sum), four scale terms.
    let parameters = 8;
    let mut rows = Vec::with_capacity(valid.len());
    let mut truths = Vec::with_capacity(valid.len());
    for s in &valid {
        let true_range = true_antenna_range(s);
        let measured = f64::from(s.distance_mm) * 1e-3;
        let ai = anchor_index[&s.anchor_id];
        let mut design = vec![0.0; parameters];
        design[0] = 1.0;
        if ai < 3 {
            design[1 + ai] = 1.0;
        } else {
            design[1] = -1.0;
            design[2] = -1.0;
            design[3] = -1.0;
        }
        design[4 + ai] = measured * 1.0e-6;
        rows.push((design, measured - true_range));
        truths.push(true_range);
    }
    // Hold out entire deterministic samples; grid/orientation coverage remains spread across both sets.
    let training_rows: Vec<_> = rows
        .iter()
        .enumerate()
        .filter(|(i, _)| i % 5 != 0)
        .map(|(_, r)| r.clone())
        .collect();
    let beta = irls(&training_rows, parameters, HUBER_K, 40)?;

    let mut all_residuals = Vec::new();
    let mut training_residuals = Vec::new();
    let mut validation_residuals = Vec::new();
    let mut per_anchor: BTreeMap<u16, Vec<f64>> = BTreeMap::new();
    let mut per_heading: BTreeMap<i32, Vec<f64>> = BTreeMap::new();
    let mut per_subslot: BTreeMap<u8, Vec<f64>> = BTreeMap::new();
    for (index, (sample, (design, observed))) in valid.iter().zip(rows.iter()).enumerate() {
        let residual = observed - dot(design, &beta);
        let validation = index % 5 == 0;
        if validation {
            validation_residuals.push(residual);
        } else {
            training_residuals.push(residual);
        }
        per_anchor
            .entry(sample.anchor_id)
            .or_default()
            .push(residual);
        let heading =
            ((normalize_degrees(sample.heading_deg) / 45.0).round() as i32 * 45).rem_euclid(360);
        per_heading.entry(heading).or_default().push(residual);
        per_subslot
            .entry(sample.response_subslot)
            .or_default()
            .push(residual);
        all_residuals.push(ResidualPoint {
            group: format!("{:04X}", sample.anchor_id),
            x: truths[index],
            residual_m: residual,
            validation,
        });
    }

    let mut geometry = BTreeMap::new();
    for s in &valid {
        geometry
            .entry(s.anchor_id)
            .or_insert((s.anchor_x_m, s.anchor_y_m, s.anchor_z_m));
    }
    let offsets = [beta[1], beta[2], beta[3], -beta[1] - beta[2] - beta[3]];
    let anchors = anchor_ids
        .iter()
        .enumerate()
        .map(|(i, id)| {
            let (x, y, z) = geometry[id];
            SiteAnchorEstimate {
                anchor_id: *id,
                x,
                y,
                z,
                offset_mm: (offsets[i] * 1000.0).round() as i32,
                scale_ppm: beta[4 + i].round() as i32,
                range_sigma_m: robust_sigma(&per_anchor[id]).max(0.02),
            }
        })
        .collect();
    let orientation = per_heading
        .into_iter()
        .map(|(heading_bucket_deg, mut values)| {
            let samples = values.len();
            OrientationDiagnostic {
                heading_bucket_deg,
                median_residual_m: median(&mut values),
                samples,
            }
        })
        .collect();
    let subslots = per_subslot
        .into_iter()
        .map(|(response_subslot, mut values)| {
            let samples = values.len();
            SubslotDiagnostic {
                response_subslot,
                median_residual_m: median(&mut values),
                samples,
            }
        })
        .collect();
    let training = metrics(&training_residuals);
    let validation = metrics(&validation_residuals);
    let accepted = validation.median_abs_m <= 0.05 && validation.p95_abs_m <= 0.15;
    Ok(SiteReport {
        schema_version: 1,
        session_id: first.session_id,
        robot_id: first.robot_id.clone(),
        robot_range_offset_mm: (beta[0] * 1000.0).round() as i32,
        antenna_offset_x_m: first.antenna_offset_x_m,
        antenna_offset_y_m: first.antenna_offset_y_m,
        robot_antenna_height_m: first.robot_antenna_height_m,
        anchors,
        training,
        validation,
        accepted,
        orientation,
        subslots,
        residuals: all_residuals,
    })
}

pub fn true_antenna_range(s: &SiteSample) -> f64 {
    let heading = s.heading_deg.to_radians();
    let antenna_x =
        s.robot_x_m + heading.cos() * s.antenna_offset_x_m - heading.sin() * s.antenna_offset_y_m;
    let antenna_y =
        s.robot_y_m + heading.sin() * s.antenna_offset_x_m + heading.cos() * s.antenna_offset_y_m;
    let dx = antenna_x - s.anchor_x_m;
    let dy = antenna_y - s.anchor_y_m;
    let dz = s.robot_antenna_height_m - s.anchor_z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn irls(
    rows: &[(Vec<f64>, f64)],
    parameters: usize,
    huber_k: f64,
    iterations: usize,
) -> Result<Vec<f64>> {
    ensure!(
        rows.len() >= parameters,
        "not enough observations for {parameters} fit parameters"
    );
    let mut beta = weighted_least_squares(rows, &vec![1.0; rows.len()], parameters)?;
    for _ in 0..iterations {
        let residuals: Vec<_> = rows.iter().map(|(x, y)| y - dot(x, &beta)).collect();
        let sigma = robust_sigma(&residuals).max(1e-6);
        let cutoff = huber_k * sigma;
        let weights: Vec<_> = residuals
            .iter()
            .map(|r| {
                if r.abs() <= cutoff {
                    1.0
                } else {
                    cutoff / r.abs()
                }
            })
            .collect();
        let next = weighted_least_squares(rows, &weights, parameters)?;
        let delta = next
            .iter()
            .zip(&beta)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        beta = next;
        if delta < 1e-10 {
            break;
        }
    }
    Ok(beta)
}

fn weighted_least_squares(rows: &[(Vec<f64>, f64)], weights: &[f64], n: usize) -> Result<Vec<f64>> {
    let mut normal = vec![vec![0.0; n + 1]; n];
    for ((x, y), w) in rows.iter().zip(weights) {
        for i in 0..n {
            for j in 0..n {
                normal[i][j] += w * x[i] * x[j];
            }
            normal[i][n] += w * x[i] * y;
        }
    }
    gaussian_solve(normal)
}

fn gaussian_solve(mut a: Vec<Vec<f64>>) -> Result<Vec<f64>> {
    let n = a.len();
    for column in 0..n {
        let pivot = (column..n)
            .max_by(|&r1, &r2| a[r1][column].abs().total_cmp(&a[r2][column].abs()))
            .unwrap();
        ensure!(
            a[pivot][column].abs() > 1e-14,
            "calibration matrix is singular; geometry/data are not observable"
        );
        a.swap(column, pivot);
        let divisor = a[column][column];
        for value in a[column].iter_mut().take(n + 1).skip(column) {
            *value /= divisor;
        }
        let normalized_pivot = a[column][column..=n].to_vec();
        for (row_index, row) in a.iter_mut().enumerate() {
            if row_index == column {
                continue;
            }
            let factor = row[column];
            for (value, pivot_value) in row[column..=n].iter_mut().zip(&normalized_pivot) {
                *value -= factor * pivot_value;
            }
        }
    }
    Ok(a.into_iter().map(|row| row[n]).collect())
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn metrics(values: &[f64]) -> ErrorMetrics {
    if values.is_empty() {
        return ErrorMetrics {
            samples: 0,
            median_abs_m: f64::NAN,
            p95_abs_m: f64::NAN,
            standard_deviation_m: f64::NAN,
        };
    }
    let mut absolute: Vec<_> = values.iter().map(|v| v.abs()).collect();
    absolute.sort_by(f64::total_cmp);
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64;
    ErrorMetrics {
        samples: values.len(),
        median_abs_m: percentile_sorted(&absolute, 0.5),
        p95_abs_m: percentile_sorted(&absolute, 0.95),
        standard_deviation_m: variance.sqrt(),
    }
}

fn robust_sigma(values: &[f64]) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    let mut copy = values.to_vec();
    let center = median(&mut copy);
    let mut deviations: Vec<_> = values.iter().map(|v| (v - center).abs()).collect();
    1.4826 * median(&mut deviations)
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    percentile_sorted(values, 0.5)
}

fn percentile_sorted(values: &[f64], fraction: f64) -> f64 {
    let index = ((values.len() - 1) as f64 * fraction).round() as usize;
    values[index]
}

fn normalize_degrees(value: f64) -> f64 {
    value.rem_euclid(360.0)
}

pub fn read_fixture_csv(path: &std::path::Path) -> Result<Vec<FixtureSample>> {
    csv::Reader::from_path(path)
        .with_context(|| format!("open {}", path.display()))?
        .deserialize()
        .collect::<Result<_, _>>()
        .context("decode fixture CSV")
}

pub fn read_site_csv(path: &std::path::Path) -> Result<Vec<SiteSample>> {
    csv::Reader::from_path(path)
        .with_context(|| format!("open {}", path.display()))?
        .deserialize()
        .collect::<Result<_, _>>()
        .context("decode site CSV")
}

pub fn require_accepted(accepted: bool) -> Result<()> {
    if accepted {
        Ok(())
    } else {
        bail!("report does not meet acceptance thresholds")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_three_node_delays_with_outliers_and_missing_directions() {
        let biases = [0.021, -0.012, 0.034];
        let ids = ["REF1", "REF2", "DUT1"];
        let mut samples = Vec::new();
        for pair in [(0, 1), (0, 2), (1, 2)] {
            for i in 0..650 {
                if pair == (1, 2) && i % 7 == 0 {
                    continue;
                }
                let noise = ((i * 17 % 23) as f64 - 11.0) * 0.0007;
                let outlier = if i % 113 == 0 { 0.35 } else { 0.0 };
                samples.push(FixtureSample {
                    session_id: 7,
                    initiator_id: ids[pair.0].into(),
                    responder_id: ids[pair.1].into(),
                    true_distance_m: 2.0 + pair.0 as f64 * 0.3,
                    measured_distance_m: 2.0
                        + pair.0 as f64 * 0.3
                        + biases[pair.0]
                        + biases[pair.1]
                        + noise
                        + outlier,
                    direction: "forward".into(),
                });
            }
        }
        let report = solve_hardware(&samples, 500).unwrap();
        for estimate in &report.nodes {
            let expected = biases[ids.iter().position(|id| *id == estimate.node_id).unwrap()];
            assert!((estimate.range_bias_m - expected).abs() < 0.003);
        }
        assert!(report.validation.p95_abs_m < 0.02);
    }

    #[test]
    fn post_flash_validation_scores_raw_error_without_refitting_it() {
        let mut samples = Vec::new();
        for (a, b) in [("C001", "C002"), ("C001", "C003"), ("C002", "C003")] {
            for index in 0..20 {
                samples.push(FixtureSample {
                    session_id: 11,
                    initiator_id: a.into(),
                    responder_id: b.into(),
                    true_distance_m: 2.5,
                    measured_distance_m: 2.5 + if index == 0 { 0.04 } else { 0.006 },
                    direction: "validation".into(),
                });
            }
        }
        let accepted = validate_hardware(&samples, 20).unwrap();
        assert!(accepted.accepted);
        let mut biased = samples;
        for sample in &mut biased {
            sample.measured_distance_m += 0.08;
        }
        assert!(!validate_hardware(&biased, 20).unwrap().accepted);
    }

    #[test]
    fn site_solver_recovers_constrained_offsets_scale_and_rejects_spikes() {
        let anchors = [
            (0xA001, 0.0, 0.0, 0.5),
            (0xA002, 4.0, 0.0, 0.5),
            (0xA003, 4.0, 4.0, 0.5),
            (0xA004, 0.0, 4.0, 0.5),
        ];
        let offsets = [0.015, -0.010, 0.005, -0.010];
        let scales = [800.0, -400.0, 200.0, -600.0];
        let common = 0.018;
        let mut samples = Vec::new();
        let mut n = 0usize;
        for (x, y) in [(2.0, 2.0), (0.4, 0.4), (3.6, 0.4), (3.6, 3.6), (0.4, 3.6)] {
            for heading in (0..360).step_by(45) {
                for repeat in 0..52 {
                    for (ai, &(id, ax, ay, az)) in anchors.iter().enumerate() {
                        let mut sample = SiteSample {
                            session_id: 9,
                            robot_id: "ABC123".into(),
                            anchor_id: id,
                            robot_x_m: x,
                            robot_y_m: y,
                            heading_deg: heading as f64,
                            robot_antenna_height_m: 0.07,
                            antenna_offset_x_m: 0.025,
                            antenna_offset_y_m: -0.008,
                            anchor_x_m: ax,
                            anchor_y_m: ay,
                            anchor_z_m: az,
                            distance_mm: 0,
                            response_subslot: (ai + repeat) as u8 % 4,
                            clock_ratio_converged: true,
                        };
                        let truth = true_antenna_range(&sample);
                        let measured = (truth + common + offsets[ai]) / (1.0 - scales[ai] * 1e-6);
                        let noise = ((n * 31 % 19) as f64 - 9.0) * 0.0008;
                        let spike = if n.is_multiple_of(997) { 0.30 } else { 0.0 };
                        sample.distance_mm = ((measured + noise + spike) * 1000.0).round() as u32;
                        samples.push(sample);
                        n += 1;
                    }
                }
            }
        }
        let report = solve_site(&samples, 200).unwrap();
        assert!((report.robot_range_offset_mm - 18).abs() <= 3);
        for (fit, expected) in report.anchors.iter().zip(offsets) {
            assert!((fit.offset_mm as f64 - expected * 1000.0).abs() < 4.0);
        }
        assert!(report.validation.p95_abs_m < 0.02);
    }
}
