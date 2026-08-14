use std::{
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result, bail, ensure};
use clap::{Args, Parser, Subcommand};
use plotters::prelude::*;
use rumqttc::{AsyncClient, Event, EventLoop, MqttOptions, Packet, QoS};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::time::{Instant, timeout_at};
use toml_edit::{DocumentMut, value};
use uwb_calibrate::{
    FixtureSample, HardwareReport, ResidualPoint, SiteReport, SiteSample, read_fixture_csv,
    read_site_csv, require_accepted, solve_hardware, solve_site, validate_hardware,
};

#[derive(Parser)]
#[command(about = "DWM3000 hardware/site calibration with explicit preview before writes")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Fit three-node APS014 fixture data and create JSON/CSV/SVG reports.
    FitHardware {
        #[arg(long)]
        input: PathBuf,
        #[arg(long)]
        output: PathBuf,
        #[arg(long, default_value_t = 500)]
        min_samples_per_pair: usize,
    },
    /// Score a post-flash capture as raw error; does not refit and hide a wrong delay.
    ValidateHardware {
        #[arg(long)]
        input: PathBuf,
        #[arg(long)]
        output: PathBuf,
        #[arg(long, default_value_t = 500)]
        min_samples_per_pair: usize,
    },
    /// Fit arena residuals, per-anchor scale/sigma and orientation diagnostics.
    FitSite {
        #[arg(long)]
        input: PathBuf,
        #[arg(long)]
        output: PathBuf,
        #[arg(long, default_value_t = 200)]
        min_samples_per_pose: usize,
    },
    /// Capture one surveyed site pose from the robot's raw UWB MQTT stream.
    CaptureSite(CaptureSiteArgs),
    /// Capture DS-TWR reports emitted by a robot running the temporary fixture image.
    CaptureFixture(CaptureFixtureArgs),
    /// Write one fitted node delay into a robot's journaled NVS after its physical confirmation.
    ApplyRobotDelay {
        #[arg(long)]
        report: PathBuf,
        /// C001/C002/C003 label used by this device in the fixture report.
        #[arg(long)]
        node_id: String,
        /// Six-character ESP32 robot device ID used in MQTT topics.
        #[arg(long)]
        robot_id: String,
        #[command(flatten)]
        mqtt: MqttArgs,
        #[arg(long)]
        yes: bool,
    },
    /// Replace a robot's hardware delay with the nominal value after physical confirmation.
    ClearRobotDelay {
        #[arg(long)]
        robot_id: String,
        #[command(flatten)]
        mqtt: MqttArgs,
        #[arg(long)]
        yes: bool,
    },
    /// Update retained robot and anchor site configuration from a successful report.
    ApplySite {
        #[arg(long)]
        report: PathBuf,
        /// Existing full robot configuration to preserve motor/speed tuning.
        #[arg(long)]
        current_robot_config: Option<PathBuf>,
        /// Publish only this robot's configuration, preserving the shared anchor configuration.
        #[arg(long)]
        robot_only: bool,
        #[command(flatten)]
        mqtt: MqttArgs,
        #[arg(long)]
        yes: bool,
    },
    /// Bind an anchor ID to a physical STM32 UID and fitted delay in the compiled manifest.
    ProvisionAnchor {
        #[arg(long)]
        report: PathBuf,
        #[arg(long)]
        node_id: String,
        /// A001, A002, A003 or A004.
        #[arg(long)]
        anchor_id: String,
        /// 96-bit UID printed by the anchor boot log, as 24 hex digits.
        #[arg(long)]
        uid: String,
        #[arg(long, default_value = "anchor/calibration.toml")]
        manifest: PathBuf,
        #[arg(long)]
        yes: bool,
    },
}

#[derive(Clone, Args)]
struct MqttArgs {
    #[arg(long, default_value = "127.0.0.1")]
    broker: String,
    #[arg(long, default_value_t = 1883)]
    port: u16,
    #[arg(long)]
    username: Option<String>,
    #[arg(long)]
    password: Option<String>,
}

#[derive(Args)]
struct CaptureSiteArgs {
    #[arg(long)]
    robot_id: String,
    #[arg(long)]
    anchors: PathBuf,
    #[arg(long)]
    output: PathBuf,
    #[arg(long)]
    x_m: f64,
    #[arg(long)]
    y_m: f64,
    #[arg(long)]
    heading_deg: f64,
    #[arg(long)]
    antenna_offset_x_m: f64,
    #[arg(long)]
    antenna_offset_y_m: f64,
    #[arg(long)]
    robot_antenna_height_m: Option<f64>,
    #[arg(long, default_value_t = 60)]
    duration_s: u16,
    #[arg(long)]
    append: bool,
    /// Campaign ID. When appending, defaults to the first row's existing session ID.
    #[arg(long)]
    session_id: Option<u32>,
    #[command(flatten)]
    mqtt: MqttArgs,
}

#[derive(Args)]
struct CaptureFixtureArgs {
    #[arg(long)]
    gateway_id: String,
    #[arg(long)]
    output: PathBuf,
    /// Known pair as INITIATOR,RESPONDER,DISTANCE_M; repeat three times.
    #[arg(long = "pair", required = true)]
    pairs: Vec<String>,
    #[arg(long, default_value_t = 180)]
    duration_s: u16,
    #[arg(long)]
    session_id: Option<u32>,
    #[command(flatten)]
    mqtt: MqttArgs,
}

#[derive(Deserialize)]
struct RawRange {
    anchor_id: u16,
    distance_mm: u32,
    response_subslot: u8,
    clock_ratio_converged: bool,
}

#[derive(Deserialize)]
struct FixtureWireSample {
    session_id: u32,
    initiator_id: u16,
    responder_id: u16,
    distance_mm: u32,
}

#[derive(Clone, Deserialize)]
struct ArenaConfig {
    robot_antenna_height_m: f64,
    anchors: Vec<ArenaAnchor>,
}

#[derive(Clone, Deserialize)]
struct ArenaAnchor {
    anchor_id: u16,
    x: f64,
    y: f64,
    z: f64,
}

#[tokio::main]
async fn main() -> Result<()> {
    match Cli::parse().command {
        Command::FitHardware {
            input,
            output,
            min_samples_per_pair,
        } => {
            let report = solve_hardware(&read_fixture_csv(&input)?, min_samples_per_pair)?;
            print_hardware_preview(&report);
            write_report_bundle(&output, &report, &report.residuals)?;
            require_accepted(report.accepted)
        }
        Command::ValidateHardware {
            input,
            output,
            min_samples_per_pair,
        } => {
            let report = validate_hardware(&read_fixture_csv(&input)?, min_samples_per_pair)?;
            println!(
                "Post-flash raw validation: median {:.1} mm, P95 {:.1} mm, sigma {:.1} mm — {}",
                report.raw_error.median_abs_m * 1000.0,
                report.raw_error.p95_abs_m * 1000.0,
                report.raw_error.standard_deviation_m * 1000.0,
                if report.accepted {
                    "ACCEPTED"
                } else {
                    "REJECTED"
                }
            );
            write_report_bundle(&output, &report, &report.residuals)?;
            require_accepted(report.accepted)
        }
        Command::FitSite {
            input,
            output,
            min_samples_per_pose,
        } => {
            let report = solve_site(&read_site_csv(&input)?, min_samples_per_pose)?;
            print_site_preview(&report);
            write_report_bundle(&output, &report, &report.residuals)?;
            require_accepted(report.accepted)
        }
        Command::CaptureSite(args) => capture_site(args).await,
        Command::CaptureFixture(args) => capture_fixture(args).await,
        Command::ApplyRobotDelay {
            report,
            node_id,
            robot_id,
            mqtt,
            yes,
        } => {
            let report: HardwareReport = read_json(&report)?;
            require_accepted(report.accepted)?;
            let node = report
                .nodes
                .iter()
                .find(|n| n.node_id == node_id)
                .with_context(|| format!("node {node_id} is absent from report"))?;
            let preview = json!({"action":"apply_robot_delay", "session_id":report.session_id,
                "rx_ticks":node.rx_ticks, "tx_ticks":node.tx_ticks});
            println!(
                "PREVIEW /calibration/command/{robot_id} (fixture node {node_id})\n{}",
                serde_json::to_string_pretty(&preview)?
            );
            confirm(yes)?;
            apply_robot_delay(&mqtt, &robot_id, &preview).await
        }
        Command::ClearRobotDelay {
            robot_id,
            mqtt,
            yes,
        } => {
            let preview = json!({"action":"clear_robot_delay", "session_id":session_id()});
            println!(
                "PREVIEW /calibration/command/{robot_id}\n{}",
                serde_json::to_string_pretty(&preview)?
            );
            confirm(yes)?;
            apply_robot_delay(&mqtt, &robot_id, &preview).await
        }
        Command::ApplySite {
            report,
            current_robot_config,
            robot_only,
            mqtt,
            yes,
        } => {
            let report: SiteReport = read_json(&report)?;
            require_accepted(report.accepted)?;
            let (robot, anchors) = site_payloads(&report, current_robot_config.as_deref())?;
            println!(
                "PREVIEW retained /config/robots/{}\n{}",
                report.robot_id,
                serde_json::to_string_pretty(&robot)?
            );
            if !robot_only {
                println!(
                    "PREVIEW retained /config/anchors\n{}",
                    serde_json::to_string_pretty(&anchors)?
                );
            } else {
                println!("PREVIEW: --robot-only leaves retained /config/anchors unchanged");
            }
            confirm(yes)?;
            let (client, mut eventloop) = mqtt_connect(&mqtt, "site-apply");
            client
                .publish(
                    format!("/config/robots/{}", report.robot_id),
                    QoS::AtLeastOnce,
                    true,
                    serde_json::to_vec(&robot)?,
                )
                .await?;
            if !robot_only {
                client
                    .publish(
                        "/config/anchors",
                        QoS::AtLeastOnce,
                        true,
                        serde_json::to_vec(&anchors)?,
                    )
                    .await?;
            }
            flush_mqtt(&mut eventloop, Duration::from_secs(3)).await
        }
        Command::ProvisionAnchor {
            report,
            node_id,
            anchor_id,
            uid,
            manifest,
            yes,
        } => {
            let report: HardwareReport = read_json(&report)?;
            require_accepted(report.accepted)?;
            let node = report
                .nodes
                .iter()
                .find(|n| n.node_id == node_id)
                .with_context(|| format!("node {node_id} is absent from report"))?;
            let normalized_uid = uid.to_ascii_uppercase();
            ensure!(
                normalized_uid.len() == 24 && normalized_uid.bytes().all(|b| b.is_ascii_hexdigit()),
                "UID must contain exactly 24 hexadecimal digits"
            );
            ensure!(
                ["A001", "A002", "A003", "A004"].contains(&anchor_id.as_str()),
                "unknown anchor ID {anchor_id}"
            );
            println!(
                "PREVIEW {}: expected_uid={}, RX={}, TX={} (source node {})",
                anchor_id, normalized_uid, node.rx_ticks, node.tx_ticks, node_id
            );
            confirm(yes)?;
            provision_manifest(
                &manifest,
                &anchor_id,
                &normalized_uid,
                node.rx_ticks,
                node.tx_ticks,
            )
        }
    }
}

fn print_hardware_preview(report: &HardwareReport) {
    println!(
        "Hardware fit session {}: validation median {:.1} mm, P95 {:.1} mm, sigma {:.1} mm — {}",
        report.session_id,
        report.validation.median_abs_m * 1000.0,
        report.validation.p95_abs_m * 1000.0,
        report.validation.standard_deviation_m * 1000.0,
        if report.accepted {
            "ACCEPTED"
        } else {
            "REJECTED"
        }
    );
    for node in &report.nodes {
        println!(
            "  {}: bias {:+.1} mm -> RX/TX {} (delta {:+} ticks)",
            node.node_id,
            node.range_bias_m * 1000.0,
            node.rx_ticks,
            node.delay_delta_ticks
        );
    }
}

fn print_site_preview(report: &SiteReport) {
    println!(
        "Site fit {} session {}: robot offset {:+} mm; validation median {:.1} mm, P95 {:.1} mm — {}",
        report.robot_id,
        report.session_id,
        report.robot_range_offset_mm,
        report.validation.median_abs_m * 1000.0,
        report.validation.p95_abs_m * 1000.0,
        if report.accepted {
            "ACCEPTED"
        } else {
            "REJECTED"
        }
    );
    for anchor in &report.anchors {
        println!(
            "  {:04X}: offset {:+} mm scale {:+} ppm sigma {:.1} mm",
            anchor.anchor_id,
            anchor.offset_mm,
            anchor.scale_ppm,
            anchor.range_sigma_m * 1000.0
        );
    }
}

fn write_report_bundle<T: Serialize>(
    base: &Path,
    report: &T,
    residuals: &[ResidualPoint],
) -> Result<()> {
    let json_path = base.with_extension("json");
    let csv_path = suffix(base, "-residuals.csv");
    let svg_path = suffix(base, "-residuals.svg");
    fs::write(&json_path, serde_json::to_vec_pretty(report)?)?;
    let mut writer = csv::Writer::from_path(&csv_path)?;
    for row in residuals {
        writer.serialize(row)?;
    }
    writer.flush()?;
    plot_residuals(&svg_path, residuals)?;
    println!(
        "Wrote {}, {} and {}",
        json_path.display(),
        csv_path.display(),
        svg_path.display()
    );
    Ok(())
}

fn suffix(base: &Path, suffix: &str) -> PathBuf {
    let parent = base.parent().unwrap_or_else(|| Path::new(""));
    let stem = base.file_stem().unwrap_or_default().to_string_lossy();
    parent.join(format!("{stem}{suffix}"))
}

fn plot_residuals(path: &Path, residuals: &[ResidualPoint]) -> Result<()> {
    ensure!(!residuals.is_empty(), "cannot plot an empty report");
    let x_min = residuals.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let x_max = residuals
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let y_limit = residuals
        .iter()
        .map(|p| p.residual_m.abs())
        .fold(0.05, f64::max)
        .min(0.5)
        * 1.1;
    let root = SVGBackend::new(path, (1200, 700)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption("UWB calibration residuals", ("sans-serif", 28))
        .margin(20)
        .x_label_area_size(45)
        .y_label_area_size(60)
        .build_cartesian_2d((x_min - 0.05)..(x_max + 0.05), -y_limit..y_limit)?;
    chart
        .configure_mesh()
        .x_desc("true range (m)")
        .y_desc("residual (m)")
        .draw()?;
    chart.draw_series(residuals.iter().map(|p| {
        Circle::new(
            (p.x, p.residual_m),
            2,
            if p.validation {
                RED.filled()
            } else {
                BLUE.mix(0.35).filled()
            },
        )
    }))?;
    root.present()?;
    Ok(())
}

async fn capture_site(args: CaptureSiteArgs) -> Result<()> {
    let arena: ArenaConfig = read_json(&args.anchors)?;
    let session_id = args
        .session_id
        .or_else(|| {
            if args.append && args.output.exists() {
                read_site_csv(&args.output)
                    .ok()
                    .and_then(|rows| rows.first().map(|row| row.session_id))
            } else {
                None
            }
        })
        .unwrap_or_else(session_id);
    let (client, mut eventloop) = mqtt_connect(&args.mqtt, "site-capture");
    let range_topic = format!("/uwb/{}", args.robot_id);
    client.subscribe(&range_topic, QoS::AtLeastOnce).await?;
    let command =
        json!({"action":"start_capture", "session_id":session_id, "duration_s":args.duration_s});
    client
        .publish(
            format!("/calibration/command/{}", args.robot_id),
            QoS::AtLeastOnce,
            false,
            serde_json::to_vec(&command)?,
        )
        .await?;
    println!(
        "Capturing stationary pose ({:.3}, {:.3}) m at {:.1}° for {} s; keep motors disabled.",
        args.x_m, args.y_m, args.heading_deg, args.duration_s
    );

    let mut rows = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(u64::from(args.duration_s) + 3);
    while let Ok(event) = timeout_at(deadline, eventloop.poll()).await {
        let Event::Incoming(Packet::Publish(message)) = event? else {
            continue;
        };
        if message.topic != range_topic {
            continue;
        }
        let raw: RawRange = serde_json::from_slice(&message.payload)?;
        let anchor = arena
            .anchors
            .iter()
            .find(|a| a.anchor_id == raw.anchor_id)
            .with_context(|| format!("range from unknown anchor {:04X}", raw.anchor_id))?;
        rows.push(SiteSample {
            session_id,
            robot_id: args.robot_id.clone(),
            anchor_id: raw.anchor_id,
            robot_x_m: args.x_m,
            robot_y_m: args.y_m,
            heading_deg: args.heading_deg,
            robot_antenna_height_m: args
                .robot_antenna_height_m
                .unwrap_or(arena.robot_antenna_height_m),
            antenna_offset_x_m: args.antenna_offset_x_m,
            antenna_offset_y_m: args.antenna_offset_y_m,
            anchor_x_m: anchor.x,
            anchor_y_m: anchor.y,
            anchor_z_m: anchor.z,
            distance_mm: raw.distance_mm,
            response_subslot: raw.response_subslot,
            clock_ratio_converged: raw.clock_ratio_converged,
        });
    }
    write_csv(&args.output, &rows, args.append)?;
    println!(
        "Captured {} ranges into {} (session {}).",
        rows.len(),
        args.output.display(),
        session_id
    );
    Ok(())
}

async fn capture_fixture(args: CaptureFixtureArgs) -> Result<()> {
    let known = parse_pairs(&args.pairs)?;
    let session_id = args.session_id.unwrap_or_else(session_id);
    let (client, mut eventloop) = mqtt_connect(&args.mqtt, "fixture-capture");
    let sample_topic = format!("/calibration/samples/{}", args.gateway_id);
    client.subscribe(&sample_topic, QoS::AtLeastOnce).await?;
    let command =
        json!({"action":"start_capture", "session_id":session_id, "duration_s":args.duration_s});
    client
        .publish(
            format!("/calibration/command/{}", args.gateway_id),
            QoS::AtLeastOnce,
            false,
            serde_json::to_vec(&command)?,
        )
        .await?;
    println!(
        "Capturing isolated DS-TWR fixture for {} s on {}.",
        args.duration_s, sample_topic
    );
    let deadline = Instant::now() + Duration::from_secs(u64::from(args.duration_s) + 3);
    let mut rows = Vec::new();
    while let Ok(event) = timeout_at(deadline, eventloop.poll()).await {
        let Event::Incoming(Packet::Publish(message)) = event? else {
            continue;
        };
        if message.topic != sample_topic {
            continue;
        }
        let raw: FixtureWireSample = serde_json::from_slice(&message.payload)?;
        if raw.session_id != session_id {
            continue;
        }
        let pair = ordered_pair(raw.initiator_id, raw.responder_id);
        let true_distance_m = *known
            .get(&pair)
            .with_context(|| format!("unknown fixture pair {:04X}-{:04X}", pair.0, pair.1))?;
        rows.push(FixtureSample {
            session_id,
            initiator_id: format!("{:04X}", raw.initiator_id),
            responder_id: format!("{:04X}", raw.responder_id),
            true_distance_m,
            measured_distance_m: f64::from(raw.distance_mm) * 1e-3,
            direction: format!("{:04X}->{:04X}", raw.initiator_id, raw.responder_id),
        });
    }
    write_csv(&args.output, &rows, false)?;
    println!(
        "Captured {} fixture ranges into {}.",
        rows.len(),
        args.output.display()
    );
    Ok(())
}

fn parse_pairs(raw: &[String]) -> Result<std::collections::BTreeMap<(u16, u16), f64>> {
    ensure!(raw.len() == 3, "exactly three --pair values are required");
    let mut result = std::collections::BTreeMap::new();
    for spec in raw {
        let fields: Vec<_> = spec.split(',').collect();
        ensure!(
            fields.len() == 3,
            "pair must be INITIATOR,RESPONDER,DISTANCE_M"
        );
        let a = u16::from_str_radix(fields[0].trim_start_matches("0x"), 16)?;
        let b = u16::from_str_radix(fields[1].trim_start_matches("0x"), 16)?;
        let distance: f64 = fields[2].parse()?;
        ensure!(distance > 0.0, "pair distance must be positive");
        result.insert(ordered_pair(a, b), distance);
    }
    ensure!(result.len() == 3, "fixture pairs must be distinct");
    Ok(result)
}

fn ordered_pair(a: u16, b: u16) -> (u16, u16) {
    if a < b { (a, b) } else { (b, a) }
}

fn write_csv<T: Serialize>(path: &Path, rows: &[T], append: bool) -> Result<()> {
    let file = fs::OpenOptions::new()
        .create(true)
        .write(true)
        .append(append)
        .truncate(!append)
        .open(path)?;
    let has_content = append && file.metadata()?.len() > 0;
    let mut writer = csv::WriterBuilder::new()
        .has_headers(!has_content)
        .from_writer(file);
    for row in rows {
        writer.serialize(row)?;
    }
    writer.flush()?;
    Ok(())
}

async fn apply_robot_delay(mqtt: &MqttArgs, robot_id: &str, command: &Value) -> Result<()> {
    let (client, mut eventloop) = mqtt_connect(mqtt, "robot-delay-apply");
    let status_topic = format!("/calibration/status/{robot_id}");
    client.subscribe(&status_topic, QoS::AtLeastOnce).await?;
    println!(
        "Hold the robot button for 1.5–3 seconds now; waiting up to 60 seconds for ARMED status…"
    );
    wait_for_state(
        &mut eventloop,
        &status_topic,
        "armed",
        Duration::from_secs(60),
    )
    .await?;
    client
        .publish(
            format!("/calibration/command/{robot_id}"),
            QoS::AtLeastOnce,
            false,
            serde_json::to_vec(command)?,
        )
        .await?;
    let status = wait_for_any_state(
        &mut eventloop,
        &status_topic,
        &["applied_pending_reboot", "rejected"],
        Duration::from_secs(10),
    )
    .await?;
    println!("Robot response: {}", serde_json::to_string_pretty(&status)?);
    ensure!(
        status["state"] == "applied_pending_reboot",
        "robot rejected calibration"
    );
    Ok(())
}

fn site_payloads(report: &SiteReport, current: Option<&Path>) -> Result<(Value, Value)> {
    let mut robot = if let Some(path) = current {
        read_json::<Value>(path)?
    } else {
        json!({"motors":{"ema_filter_alpha":0.1,"max_speed":1.0},
            "localization":{"range_offset_mm":0,"full_duty_speed_m_s":0.5,
                "antenna_offset_x_m":0.0,"antenna_offset_y_m":0.0}})
    };
    let localization = robot
        .get_mut("localization")
        .and_then(Value::as_object_mut)
        .context("current robot config has no localization object")?;
    localization.insert(
        "range_offset_mm".into(),
        report.robot_range_offset_mm.into(),
    );
    localization.insert(
        "antenna_offset_x_m".into(),
        report.antenna_offset_x_m.into(),
    );
    localization.insert(
        "antenna_offset_y_m".into(),
        report.antenna_offset_y_m.into(),
    );
    let anchors = json!({"robot_antenna_height_m":report.robot_antenna_height_m,
        "anchors":report.anchors});
    Ok((robot, anchors))
}

fn provision_manifest(path: &Path, anchor_id: &str, uid: &str, rx: u16, tx: u16) -> Result<()> {
    let source = fs::read_to_string(path)?;
    let mut doc: DocumentMut = source.parse()?;
    let tables = doc["anchors"]
        .as_array_of_tables_mut()
        .context("manifest has no [[anchors]]")?;
    let table = tables
        .iter_mut()
        .find(|table| table["anchor_id"].as_str() == Some(anchor_id))
        .with_context(|| format!("manifest has no {anchor_id}"))?;
    table["expected_uid"] = value(uid);
    table["rx_ticks"] = value(i64::from(rx));
    table["tx_ticks"] = value(i64::from(tx));
    fs::write(path, doc.to_string())?;
    println!(
        "Updated {}; rebuild and flash only the {} image.",
        path.display(),
        anchor_id
    );
    Ok(())
}

fn mqtt_connect(args: &MqttArgs, purpose: &str) -> (AsyncClient, EventLoop) {
    let mut options = MqttOptions::new(
        format!("uwb-calibrate-{purpose}-{}", session_id()),
        &args.broker,
        args.port,
    );
    options.set_keep_alive(Duration::from_secs(10));
    if let Some(username) = &args.username {
        options.set_credentials(username, args.password.as_deref().unwrap_or(""));
    }
    AsyncClient::new(options, 32)
}

async fn wait_for_state(
    eventloop: &mut EventLoop,
    topic: &str,
    state: &str,
    duration: Duration,
) -> Result<Value> {
    wait_for_any_state(eventloop, topic, &[state], duration).await
}

async fn wait_for_any_state(
    eventloop: &mut EventLoop,
    topic: &str,
    states: &[&str],
    duration: Duration,
) -> Result<Value> {
    let deadline = Instant::now() + duration;
    loop {
        let event = timeout_at(deadline, eventloop.poll())
            .await
            .context("MQTT status timeout")??;
        let Event::Incoming(Packet::Publish(message)) = event else {
            continue;
        };
        if message.topic != topic {
            continue;
        }
        let value: Value = serde_json::from_slice(&message.payload)?;
        if value["state"]
            .as_str()
            .is_some_and(|state| states.contains(&state))
        {
            return Ok(value);
        }
    }
}

async fn flush_mqtt(eventloop: &mut EventLoop, duration: Duration) -> Result<()> {
    let deadline = Instant::now() + duration;
    while let Ok(event) = timeout_at(deadline, eventloop.poll()).await {
        event?;
    }
    Ok(())
}

fn confirm(yes: bool) -> Result<()> {
    if yes {
        println!("Application authorized by --yes (preview shown above).");
        return Ok(());
    }
    print!("Type APPLY to continue: ");
    io::stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    if input.trim() == "APPLY" {
        Ok(())
    } else {
        bail!("application cancelled")
    }
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T> {
    serde_json::from_slice(&fs::read(path).with_context(|| format!("read {}", path.display()))?)
        .with_context(|| format!("parse {}", path.display()))
}

fn session_id() -> u32 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u32
}
