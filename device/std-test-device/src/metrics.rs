//! Mission pass/fail metrics for PID sweep testing.
//!
//! A sim run produces a `MissionReport` (a time-series of position, attitude,
//! and phase markers). `MissionReport::evaluate` maps this into a `PassFailReport`
//! with explicit hard checks (safety + mission goals) and soft quality metrics
//! (rise time, overshoot, settling time, steady-state error).
//!
//! Hard checks must all pass for a gain point to count as "stable" in a sweep.
//! Quality metrics are logged so the user can compare stable points and pick a
//! working setpoint far from the stability boundary.

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{LazyLock, RwLock};

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MissionPhase {
    Preflight = 0,
    Ramp = 1,
    Climb = 2,
    Hover = 3,
    Descend = 4,
    Landed = 5,
}

impl MissionPhase {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Ramp,
            2 => Self::Climb,
            3 => Self::Hover,
            4 => Self::Descend,
            5 => Self::Landed,
            _ => Self::Preflight,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Preflight => "preflight",
            Self::Ramp => "ramp",
            Self::Climb => "climb",
            Self::Hover => "hover",
            Self::Descend => "descend",
            Self::Landed => "landed",
        }
    }
}

/// Shared mission-phase marker. The mission task writes; the sim collector reads.
static MISSION_PHASE: AtomicU8 = AtomicU8::new(0);

pub fn set_phase(p: MissionPhase) {
    MISSION_PHASE.store(p as u8, Ordering::Relaxed);
}

pub fn get_phase() -> MissionPhase {
    MissionPhase::from_u8(MISSION_PHASE.load(Ordering::Relaxed))
}

/// One ground-truth sample from the sim, at ~50 Hz.
#[derive(Debug, Clone)]
pub struct Sample {
    /// Seconds since the recorder started (wallclock-from-boot is fine; we only
    /// use differences).
    pub t: f32,
    pub phase: MissionPhase,
    /// World-frame position NED, metres. `pos[2] < 0` means above takeoff.
    pub pos: [f32; 3],
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    /// Angle between body-z axis and world-z axis (radians).
    /// Zero = perfectly level. `PI/2` = sideways. `PI` = upside down.
    pub tilt_from_vertical: f32,
}

#[derive(Debug, Default)]
pub struct MissionReport {
    pub samples: Vec<Sample>,
}

/// Thresholds that define pass/fail for a mission run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PassFailSpec {
    pub target_altitude_m: f32,
    pub max_tilt_deg: f32,
    pub peak_altitude_min_m: f32,
    pub peak_altitude_max_m: f32,
    pub landing_distance_max_m: f32,
    pub max_hover_oscillations: usize,
    /// A cycle counts only if its peak-to-trough amplitude exceeds this.
    /// Keeps noise from inflating the cycle count.
    pub oscillation_amplitude_thresh_rad: f32,
    /// Settling band as a percentage of target altitude (e.g. 5.0 = +/-5%).
    pub settling_band_pct: f32,
    /// Rise-time percentile (e.g. 90.0 = time to reach 90% of target).
    pub rise_time_pct: f32,
    /// Count oscillations only in the last `oscillation_tail_s` seconds of
    /// hover. Matches "consecutive oscillations that don't damp out".
    pub oscillation_tail_s: f32,
}

impl Default for PassFailSpec {
    fn default() -> Self {
        Self {
            target_altitude_m: 1.0,
            max_tilt_deg: 30.0,
            peak_altitude_min_m: 0.9,
            peak_altitude_max_m: 1.10,
            landing_distance_max_m: 0.30,
            max_hover_oscillations: 2,
            // Peak-to-trough amplitude that qualifies as an "oscillation cycle".
            // At 0.15 rad (~8.6 deg p2p, ~4.3 deg about the mean) this filters
            // out normal PID activity but still catches marginally-stable
            // limit cycles.
            oscillation_amplitude_thresh_rad: 0.15,
            settling_band_pct: 5.0,
            rise_time_pct: 90.0,
            oscillation_tail_s: 2.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CheckOutcome {
    pub name: String,
    pub passed: bool,
    pub detail: String,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct PassFailReport {
    /// Hard criteria; ALL must pass for the sweep point to count as stable.
    pub hard: Vec<CheckOutcome>,
    /// Quality metrics reported for comparison. (name, value, unit).
    /// `None` means the metric was not computable (e.g. never settled).
    pub quality: Vec<(String, Option<f32>, String)>,
}

impl PassFailReport {
    pub fn all_hard_passed(&self) -> bool {
        self.hard.iter().all(|o| o.passed)
    }

    pub fn hard_failures(&self) -> Vec<&CheckOutcome> {
        self.hard.iter().filter(|o| !o.passed).collect()
    }

    pub fn print_summary(&self) {
        eprintln!("=== Hard checks ===");
        for o in &self.hard {
            let mark = if o.passed { "PASS" } else { "FAIL" };
            eprintln!("  [{}] {:28} {}", mark, o.name, o.detail);
        }
        eprintln!("=== Quality metrics ===");
        for (name, v, unit) in &self.quality {
            match v {
                Some(val) => eprintln!("        {:28} {:.3} {}", name, val, unit),
                None => eprintln!("        {:28} n/a", name),
            }
        }
    }
}

impl MissionReport {
    pub fn push(&mut self, s: Sample) {
        self.samples.push(s);
    }

    fn in_phase<'a>(&'a self, p: MissionPhase) -> impl Iterator<Item = &'a Sample> + 'a {
        self.samples.iter().filter(move |s| s.phase == p)
    }

    /// Worst tilt observed across the whole mission, in degrees.
    pub fn max_tilt_deg(&self) -> f32 {
        self.samples
            .iter()
            .map(|s| s.tilt_from_vertical)
            .fold(0.0_f32, f32::max)
            .to_degrees()
    }

    /// Peak altitude above takeoff reached during the mission (m).
    /// Altitude is `-pos[2]` because NED has z pointing down.
    pub fn peak_altitude_m(&self) -> f32 {
        self.samples
            .iter()
            .map(|s| -s.pos[2])
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// Takeoff origin: mean of preflight samples (falls back to first sample if
    /// the mission started before any preflight samples were recorded).
    pub fn takeoff_origin(&self) -> [f32; 2] {
        let pre: Vec<&Sample> = self.in_phase(MissionPhase::Preflight).collect();
        let pool: &[&Sample] = if !pre.is_empty() {
            &pre
        } else {
            return self
                .samples
                .first()
                .map(|s| [s.pos[0], s.pos[1]])
                .unwrap_or([0.0, 0.0]);
        };
        let n = pool.len() as f32;
        let sx = pool.iter().map(|s| s.pos[0]).sum::<f32>() / n;
        let sy = pool.iter().map(|s| s.pos[1]).sum::<f32>() / n;
        [sx, sy]
    }

    /// Final XY distance from takeoff origin (m).
    pub fn landing_distance_m(&self, origin: [f32; 2]) -> f32 {
        match self.samples.last() {
            Some(s) => {
                let dx = s.pos[0] - origin[0];
                let dy = s.pos[1] - origin[1];
                (dx * dx + dy * dy).sqrt()
            }
            None => 0.0,
        }
    }

    /// Steady-state altitude error: mean altitude over the last 2s of hover minus target.
    pub fn hover_steady_state_error_m(&self, target: f32) -> Option<f32> {
        let hover: Vec<&Sample> = self.in_phase(MissionPhase::Hover).collect();
        if hover.is_empty() {
            return None;
        }
        let t_end = hover.last().unwrap().t;
        let tail: Vec<&&Sample> = hover.iter().filter(|s| s.t >= t_end - 2.0).collect();
        if tail.is_empty() {
            return None;
        }
        let n = tail.len() as f32;
        let mean = tail.iter().map(|s| -s.pos[2]).sum::<f32>() / n;
        Some(mean - target)
    }

    /// Altitude overshoot above target during climb + hover (m). Positive = overshoot.
    pub fn altitude_overshoot_m(&self, target: f32) -> Option<f32> {
        let peak = self
            .samples
            .iter()
            .filter(|s| matches!(s.phase, MissionPhase::Climb | MissionPhase::Hover))
            .map(|s| -s.pos[2])
            .fold(f32::NEG_INFINITY, f32::max);
        if peak.is_finite() {
            Some(peak - target)
        } else {
            None
        }
    }

    /// Time (seconds, measured from climb start) to reach `pct`% of target altitude.
    pub fn altitude_rise_time_s(&self, target: f32, pct: f32) -> Option<f32> {
        let threshold = target * pct / 100.0;
        let climb_start = self.in_phase(MissionPhase::Climb).next()?.t;
        self.samples
            .iter()
            .filter(|s| matches!(s.phase, MissionPhase::Climb | MissionPhase::Hover))
            .find(|s| -s.pos[2] >= threshold)
            .map(|s| s.t - climb_start)
    }

    /// Time (seconds, from climb start) after which altitude stays within
    /// `band_pct`% of target through the rest of hover. Returns `None` if the
    /// drone never settles inside the band.
    pub fn altitude_settling_time_s(&self, target: f32, band_pct: f32) -> Option<f32> {
        let band = target * band_pct / 100.0;
        let climb_start = self.in_phase(MissionPhase::Climb).next()?.t;
        let hover_end = self.in_phase(MissionPhase::Hover).last()?.t;

        let mut last_exit: Option<f32> = None;
        for s in self
            .samples
            .iter()
            .filter(|s| matches!(s.phase, MissionPhase::Climb | MissionPhase::Hover))
        {
            let z = -s.pos[2];
            if (z - target).abs() > band {
                last_exit = Some(s.t);
            }
        }

        match last_exit {
            None => Some(0.0),
            Some(t) if t >= hover_end - 0.05 => None,
            Some(t) => Some(t - climb_start),
        }
    }

    /// Count attitude-axis oscillation cycles in the last `tail_s` seconds of
    /// hover. A cycle is one trough-to-peak or peak-to-trough excursion whose
    /// amplitude exceeds `amplitude_thresh_rad`. Restricting to the tail
    /// captures "consecutive oscillations that don't damp out" rather than
    /// transient ringing at the start of hover.
    pub fn hover_tail_oscillation_cycles(
        &self,
        axis_idx: usize,
        amplitude_thresh_rad: f32,
        tail_s: f32,
    ) -> usize {
        let hover_all: Vec<&Sample> = self.in_phase(MissionPhase::Hover).collect();
        if hover_all.len() < 4 {
            return 0;
        }
        let t_end = hover_all.last().unwrap().t;
        let hover: Vec<&Sample> = hover_all
            .into_iter()
            .filter(|s| s.t >= t_end - tail_s)
            .collect();
        if hover.len() < 4 {
            return 0;
        }
        let axis = |s: &Sample| match axis_idx {
            0 => s.roll,
            1 => s.pitch,
            _ => s.yaw,
        };

        let n = hover.len() as f32;
        let mean: f32 = hover.iter().map(|s| axis(s)).sum::<f32>() / n;
        let dev: Vec<f32> = hover.iter().map(|s| axis(s) - mean).collect();

        let mut cycles = 0usize;
        let mut last_dir: i8 = 0;
        let mut last_extremum: Option<f32> = None;
        for pair in dev.windows(2) {
            let diff = pair[1] - pair[0];
            let dir = if diff > 0.0 {
                1
            } else if diff < 0.0 {
                -1
            } else {
                0
            };
            if dir != 0 && last_dir != 0 && dir != last_dir {
                let extremum = pair[0];
                if let Some(prev) = last_extremum {
                    if (extremum - prev).abs() > amplitude_thresh_rad {
                        cycles += 1;
                    }
                }
                last_extremum = Some(extremum);
            }
            if dir != 0 {
                last_dir = dir;
            }
        }
        cycles / 2 // each full cycle = trough + peak = 2 half-swings
    }

    pub fn evaluate(&self, spec: &PassFailSpec) -> PassFailReport {
        let origin = self.takeoff_origin();
        let max_tilt_deg = self.max_tilt_deg();
        let peak_alt = self.peak_altitude_m();
        let landing = self.landing_distance_m(origin);
        let osc_roll = self.hover_tail_oscillation_cycles(0, spec.oscillation_amplitude_thresh_rad, spec.oscillation_tail_s);
        let osc_pitch = self.hover_tail_oscillation_cycles(1, spec.oscillation_amplitude_thresh_rad, spec.oscillation_tail_s);
        let osc_yaw = self.hover_tail_oscillation_cycles(2, spec.oscillation_amplitude_thresh_rad, spec.oscillation_tail_s);

        let mut hard = Vec::new();
        hard.push(CheckOutcome {
            name: "max_tilt_deg".into(),
            passed: max_tilt_deg <= spec.max_tilt_deg,
            detail: format!("{:.1} deg <= {:.1} deg", max_tilt_deg, spec.max_tilt_deg),
        });
        hard.push(CheckOutcome {
            name: "peak_altitude_in_band".into(),
            passed: peak_alt >= spec.peak_altitude_min_m && peak_alt <= spec.peak_altitude_max_m,
            detail: format!(
                "{:.3} m in [{:.2}, {:.2}]",
                peak_alt, spec.peak_altitude_min_m, spec.peak_altitude_max_m
            ),
        });
        hard.push(CheckOutcome {
            name: "landing_distance_m".into(),
            passed: landing <= spec.landing_distance_max_m,
            detail: format!("{:.3} m <= {:.3} m", landing, spec.landing_distance_max_m),
        });
        hard.push(CheckOutcome {
            name: "hover_oscillations_roll".into(),
            passed: osc_roll <= spec.max_hover_oscillations,
            detail: format!("{} cycles <= {}", osc_roll, spec.max_hover_oscillations),
        });
        hard.push(CheckOutcome {
            name: "hover_oscillations_pitch".into(),
            passed: osc_pitch <= spec.max_hover_oscillations,
            detail: format!("{} cycles <= {}", osc_pitch, spec.max_hover_oscillations),
        });
        hard.push(CheckOutcome {
            name: "hover_oscillations_yaw".into(),
            passed: osc_yaw <= spec.max_hover_oscillations,
            detail: format!("{} cycles <= {}", osc_yaw, spec.max_hover_oscillations),
        });

        let mut quality: Vec<(String, Option<f32>, String)> = Vec::new();
        quality.push((
            "rise_time_90pct".into(),
            self.altitude_rise_time_s(spec.target_altitude_m, spec.rise_time_pct),
            "s".into(),
        ));
        quality.push((
            "overshoot_pct".into(),
            self.altitude_overshoot_m(spec.target_altitude_m)
                .map(|v| 100.0 * v / spec.target_altitude_m),
            "%".into(),
        ));
        quality.push((
            "settling_time_5pct".into(),
            self.altitude_settling_time_s(spec.target_altitude_m, spec.settling_band_pct),
            "s".into(),
        ));
        quality.push((
            "steady_state_error".into(),
            self.hover_steady_state_error_m(spec.target_altitude_m),
            "m".into(),
        ));

        PassFailReport { hard, quality }
    }
}

// ---------------------------------------------------------------------------
// Run inputs: every tunable parameter that feeds a test run.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

/// All inputs to a test run. A sweep harness constructs one of these, installs
/// it via `set_run_inputs`, then runs the test. The same struct is also
/// embedded in the `TestReport` so the artifact is self-describing.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunInputs {
    pub rate_roll: Pid,
    pub rate_pitch: Pid,
    pub rate_yaw: Pid,
    pub angle_roll_ki: f32,
    pub angle_pitch_ki: f32,
    pub alt: Pid,
    pub alt_int_clamp: f32,
    pub alt_min_thrust_frac: f32,
    pub alt_max_thrust_frac: f32,
    pub flow_kp_pos: f32,
    pub flow_kd_vel: f32,
    pub flow_max_tilt_rad: f32,
    pub flow_scale: f32,
    pub flow_noise_sigma: f32,
    /// Seed for the MTF-01 flow-noise RNG. Fixing this makes the sim fully
    /// deterministic across runs with identical RunInputs, which is a
    /// prerequisite for trusting the sweep results.
    pub flow_noise_seed: u64,
    /// Overrides `vehicle.pipeline_latency_steps` from the sim config so the
    /// sweep harness can measure how much end-to-end lag the controller can
    /// tolerate. Each step = 1 ms at the default sim rate (1 kHz).
    pub pipeline_latency_steps: usize,
    pub mission_ramp_s: u64,
    pub mission_climb_s: u64,
    pub mission_hover_s: u64,
    pub mission_descend_s: u64,
    pub target_altitude_m: f32,
    pub sim_config: String,
}

impl Default for RunInputs {
    fn default() -> Self {
        Self {
            // Centred in the middle of each axis's observed stable band under
            // the hardened sim (2 ms pipeline latency, 5% vibration, asymmetric
            // motor lag, battery sag). With latency the stable kp ranges are
            // ~4x narrower than in the clean sim, and kd moves into the
            // 0.01-0.08 range to supply phase lead. ki stays at 0 because the
            // rate-PID integrator is architecturally gated below CALM_THRESHOLD.
            rate_roll: Pid { kp: 0.0040, ki: 0.0, kd: 0.020 },
            rate_pitch: Pid { kp: 0.0045, ki: 0.0, kd: 0.015 },
            rate_yaw: Pid { kp: 0.0035, ki: 0.0, kd: 0.005 },
            angle_roll_ki: 0.0,
            angle_pitch_ki: 0.0,
            alt: Pid { kp: 2.0, ki: 0.5, kd: 1.5 },
            alt_int_clamp: 0.5,
            alt_min_thrust_frac: 0.3,
            alt_max_thrust_frac: 1.5,
            flow_kp_pos: 0.30,
            flow_kd_vel: 0.25,
            flow_max_tilt_rad: 0.17,
            flow_scale: 0.25,
            flow_noise_sigma: 0.7,
            flow_noise_seed: 0xabcd_ef01,
            pipeline_latency_steps: 2,
            mission_ramp_s: 10,
            mission_climb_s: 5,
            mission_hover_s: 5,
            mission_descend_s: 5,
            target_altitude_m: 1.0,
            sim_config: "sim_config_h743v2.toml".into(),
        }
    }
}

static RUN_INPUTS: LazyLock<RwLock<RunInputs>> =
    LazyLock::new(|| RwLock::new(RunInputs::default()));

pub fn set_run_inputs(inputs: RunInputs) {
    *RUN_INPUTS.write().unwrap() = inputs;
}

pub fn get_run_inputs() -> RunInputs {
    RUN_INPUTS.read().unwrap().clone()
}

// ---------------------------------------------------------------------------
// Test report: combines inputs + spec + measured summary + pass/fail outcomes
// into a single serializable artifact.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Summary {
    pub sample_count: usize,
    pub max_tilt_deg: f32,
    pub peak_altitude_m: f32,
    pub landing_distance_m: f32,
    pub hover_tail_oscillations: [usize; 3], // roll, pitch, yaw
}

/// Optional extras for per-run markdown output (plots, rerun command).
/// `None` fields are simply omitted from the rendered markdown.
#[derive(Debug, Clone, Default)]
pub struct ReportExtras {
    /// Markdown-relative path to the top-down trajectory SVG.
    pub top_svg: Option<String>,
    /// Markdown-relative path to the side-view trajectory SVG.
    pub side_svg: Option<String>,
    /// Shell command that re-runs this exact point with Rerun viewer.
    pub rerun_command: Option<String>,
    /// Human hint about where to run the command from (CWD).
    pub rerun_cwd_hint: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestReport {
    pub test_name: String,
    pub generated_unix_s: u64,
    pub overall_passed: bool,
    pub inputs: RunInputs,
    pub spec: PassFailSpec,
    pub summary: Summary,
    pub hard_checks: Vec<CheckOutcome>,
    pub quality_metrics: Vec<(String, Option<f32>, String)>,
}

impl TestReport {
    pub fn build(
        test_name: &str,
        inputs: RunInputs,
        spec: PassFailSpec,
        mission: &MissionReport,
        pf: PassFailReport,
    ) -> Self {
        let origin = mission.takeoff_origin();
        let oscillations = [
            mission.hover_tail_oscillation_cycles(0, spec.oscillation_amplitude_thresh_rad, spec.oscillation_tail_s),
            mission.hover_tail_oscillation_cycles(1, spec.oscillation_amplitude_thresh_rad, spec.oscillation_tail_s),
            mission.hover_tail_oscillation_cycles(2, spec.oscillation_amplitude_thresh_rad, spec.oscillation_tail_s),
        ];
        let summary = Summary {
            sample_count: mission.samples.len(),
            max_tilt_deg: mission.max_tilt_deg(),
            peak_altitude_m: mission.peak_altitude_m(),
            landing_distance_m: mission.landing_distance_m(origin),
            hover_tail_oscillations: oscillations,
        };
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        Self {
            test_name: test_name.into(),
            generated_unix_s: now,
            overall_passed: pf.all_hard_passed(),
            inputs,
            spec,
            summary,
            hard_checks: pf.hard,
            quality_metrics: pf.quality,
        }
    }

    pub fn write_json(&self, path: &std::path::Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let s = serde_json::to_string_pretty(self).map_err(std::io::Error::other)?;
        std::fs::write(path, s)
    }

    pub fn write_markdown(&self, path: &std::path::Path) -> std::io::Result<()> {
        self.write_markdown_with(path, &ReportExtras::default())
    }

    pub fn write_markdown_with(
        &self,
        path: &std::path::Path,
        extras: &ReportExtras,
    ) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, self.render_markdown_with(extras))
    }

    pub fn render_markdown(&self) -> String {
        self.render_markdown_with(&ReportExtras::default())
    }

    pub fn render_markdown_with(&self, extras: &ReportExtras) -> String {
        use std::fmt::Write;
        let mut s = String::new();
        let overall = if self.overall_passed { "PASS" } else { "FAIL" };
        let _ = writeln!(s, "# Test report: {}", self.test_name);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "- Generated (unix s): {}", self.generated_unix_s);
        let _ = writeln!(s, "- Overall result: **{}**", overall);
        let _ = writeln!(s, "- Samples recorded: {}", self.summary.sample_count);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "## At-a-glance");
        let _ = writeln!(s, "- Max tilt: {:.2} deg", self.summary.max_tilt_deg);
        let _ = writeln!(s, "- Peak altitude: {:.3} m", self.summary.peak_altitude_m);
        let _ = writeln!(s, "- Landing distance: {:.3} m", self.summary.landing_distance_m);
        let _ = writeln!(
            s,
            "- Hover-tail oscillation cycles (roll/pitch/yaw): {}/{}/{}",
            self.summary.hover_tail_oscillations[0],
            self.summary.hover_tail_oscillations[1],
            self.summary.hover_tail_oscillations[2]
        );
        let _ = writeln!(s, "");

        if extras.top_svg.is_some() || extras.side_svg.is_some() {
            let _ = writeln!(s, "## Trajectory");
            let _ = writeln!(s, "");
            if let Some(top) = &extras.top_svg {
                let _ = writeln!(s, "![top-down trajectory]({})", top);
            }
            if let Some(side) = &extras.side_svg {
                let _ = writeln!(s, "![side view]({})", side);
            }
            let _ = writeln!(s, "");
            let _ = writeln!(s, "_Green dot = start, red dot = end._");
            let _ = writeln!(s, "");
        }

        if let Some(cmd) = &extras.rerun_command {
            let _ = writeln!(s, "## Re-run with Rerun visualization");
            let _ = writeln!(s, "");
            if let Some(from) = &extras.rerun_cwd_hint {
                let _ = writeln!(s, "Run from `{}`:", from);
                let _ = writeln!(s, "");
            }
            let _ = writeln!(s, "```bash");
            let _ = writeln!(s, "{}", cmd);
            let _ = writeln!(s, "```");
            let _ = writeln!(s, "");
        }

        let _ = writeln!(s, "## Input parameters");
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Rate PID");
        let _ = writeln!(s, "| Axis  | Kp       | Ki       | Kd       |");
        let _ = writeln!(s, "|-------|----------|----------|----------|");
        for (name, p) in [
            ("roll", self.inputs.rate_roll),
            ("pitch", self.inputs.rate_pitch),
            ("yaw", self.inputs.rate_yaw),
        ] {
            let _ = writeln!(s, "| {:5} | {:<8.4} | {:<8.4} | {:<8.4} |", name, p.kp, p.ki, p.kd);
        }
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Angle controller (cascade trims)");
        let _ = writeln!(s, "- roll ki: {}", self.inputs.angle_roll_ki);
        let _ = writeln!(s, "- pitch ki: {}", self.inputs.angle_pitch_ki);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Altitude PID");
        let _ = writeln!(s, "- kp={}, ki={}, kd={}", self.inputs.alt.kp, self.inputs.alt.ki, self.inputs.alt.kd);
        let _ = writeln!(s, "- integral clamp: +/- {} m.s", self.inputs.alt_int_clamp);
        let _ = writeln!(s, "- thrust clamp: [{}, {}] x hover_thrust",
            self.inputs.alt_min_thrust_frac, self.inputs.alt_max_thrust_frac);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Flow hold (PD position)");
        let _ = writeln!(s, "- kp_pos: {} rad/m", self.inputs.flow_kp_pos);
        let _ = writeln!(s, "- kd_vel: {} rad/(m/s)", self.inputs.flow_kd_vel);
        let _ = writeln!(s, "- max tilt: {} rad", self.inputs.flow_max_tilt_rad);
        let _ = writeln!(s, "- flow_scale: {}, noise sigma: {} counts",
            self.inputs.flow_scale, self.inputs.flow_noise_sigma);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Mission timing");
        let _ = writeln!(s, "- ramp: {} s", self.inputs.mission_ramp_s);
        let _ = writeln!(s, "- climb: {} s", self.inputs.mission_climb_s);
        let _ = writeln!(s, "- hover: {} s", self.inputs.mission_hover_s);
        let _ = writeln!(s, "- descend: {} s", self.inputs.mission_descend_s);
        let _ = writeln!(s, "- target altitude: {} m", self.inputs.target_altitude_m);
        let _ = writeln!(s, "- sim config: `{}`", self.inputs.sim_config);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "## Pass/fail thresholds");
        let _ = writeln!(s, "| Criterion                        | Threshold               |");
        let _ = writeln!(s, "|----------------------------------|-------------------------|");
        let _ = writeln!(s, "| max_tilt_deg                     | <= {:.1} deg             |", self.spec.max_tilt_deg);
        let _ = writeln!(s, "| peak_altitude_in_band            | [{:.2}, {:.2}] m           |", self.spec.peak_altitude_min_m, self.spec.peak_altitude_max_m);
        let _ = writeln!(s, "| landing_distance_m               | <= {:.3} m               |", self.spec.landing_distance_max_m);
        let _ = writeln!(s, "| max_hover_oscillations (per axis)| <= {} cycles             |", self.spec.max_hover_oscillations);
        let _ = writeln!(s, "| oscillation amplitude threshold  | {:.3} rad (peak-to-trough)|", self.spec.oscillation_amplitude_thresh_rad);
        let _ = writeln!(s, "| oscillation tail window          | last {:.1} s of hover    |", self.spec.oscillation_tail_s);
        let _ = writeln!(s, "| settling band                    | +/- {:.1}% of target     |", self.spec.settling_band_pct);
        let _ = writeln!(s, "| rise time                        | to {:.0}% of target      |", self.spec.rise_time_pct);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "## Results");
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Summary");
        let _ = writeln!(s, "- Max tilt: {:.2} deg", self.summary.max_tilt_deg);
        let _ = writeln!(s, "- Peak altitude: {:.3} m", self.summary.peak_altitude_m);
        let _ = writeln!(s, "- Landing distance: {:.3} m", self.summary.landing_distance_m);
        let _ = writeln!(s, "- Oscillation cycles (roll/pitch/yaw): {} / {} / {}",
            self.summary.hover_tail_oscillations[0],
            self.summary.hover_tail_oscillations[1],
            self.summary.hover_tail_oscillations[2]);
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Hard checks");
        let _ = writeln!(s, "| Check                     | Result | Detail                   |");
        let _ = writeln!(s, "|---------------------------|--------|--------------------------|");
        for o in &self.hard_checks {
            let mark = if o.passed { "PASS" } else { "FAIL" };
            let _ = writeln!(s, "| {:<25} | {:<6} | {} |", o.name, mark, o.detail);
        }
        let _ = writeln!(s, "");
        let _ = writeln!(s, "### Quality metrics");
        let _ = writeln!(s, "| Name                 | Value   | Unit |");
        let _ = writeln!(s, "|----------------------|---------|------|");
        for (name, v, unit) in &self.quality_metrics {
            match v {
                Some(val) => { let _ = writeln!(s, "| {:<20} | {:<7.3} | {:<4} |", name, val, unit); }
                None => { let _ = writeln!(s, "| {:<20} | n/a     | {:<4} |", name, unit); }
            }
        }
        s
    }

    pub fn write_html(&self, path: &std::path::Path) -> std::io::Result<()> {
        self.write_html_with(path, &ReportExtras::default())
    }

    pub fn write_html_with(
        &self,
        path: &std::path::Path,
        extras: &ReportExtras,
    ) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, self.render_html_with(extras))
    }

    pub fn render_html(&self) -> String {
        self.render_html_with(&ReportExtras::default())
    }

    pub fn render_html_with(&self, extras: &ReportExtras) -> String {
        use std::fmt::Write;
        let mut s = String::new();
        let overall_cls = if self.overall_passed { "pass" } else { "fail" };
        let overall_txt = if self.overall_passed { "PASS" } else { "FAIL" };

        let _ = write!(s,
            "<!doctype html><html><head><meta charset=\"utf-8\"><title>{}</title><style>{}</style></head><body>",
            html_escape(&self.test_name), REPORT_CSS
        );

        let _ = writeln!(s, "<h1>Test report: {}</h1>", html_escape(&self.test_name));
        let _ = writeln!(s,
            "<p>Generated (unix s): {}<br>Overall result: <span class=\"badge {}\">{}</span><br>Samples: {}</p>",
            self.generated_unix_s, overall_cls, overall_txt, self.summary.sample_count
        );

        let _ = writeln!(s, "<h2>At-a-glance</h2>");
        let _ = writeln!(s, "<ul>");
        let _ = writeln!(s, "<li>Max tilt: {:.2} deg</li>", self.summary.max_tilt_deg);
        let _ = writeln!(s, "<li>Peak altitude: {:.3} m</li>", self.summary.peak_altitude_m);
        let _ = writeln!(s, "<li>Landing distance: {:.3} m</li>", self.summary.landing_distance_m);
        let _ = writeln!(s, "<li>Hover-tail oscillation cycles (roll/pitch/yaw): {}/{}/{}</li>",
            self.summary.hover_tail_oscillations[0],
            self.summary.hover_tail_oscillations[1],
            self.summary.hover_tail_oscillations[2]);
        let _ = writeln!(s, "</ul>");

        if extras.top_svg.is_some() || extras.side_svg.is_some() {
            let _ = writeln!(s, "<h2>Trajectory</h2>");
            let _ = writeln!(s, "<div class=\"plots\">");
            if let Some(top) = &extras.top_svg {
                let _ = writeln!(s, "<img src=\"{}\" alt=\"top-down\">", html_escape(top));
            }
            if let Some(side) = &extras.side_svg {
                let _ = writeln!(s, "<img src=\"{}\" alt=\"side\">", html_escape(side));
            }
            let _ = writeln!(s, "</div>");
            let _ = writeln!(s, "<p><em>Green dot = start, red dot = end.</em></p>");
        }

        if let Some(cmd) = &extras.rerun_command {
            let _ = writeln!(s, "<h2>Re-run with Rerun visualization</h2>");
            if let Some(from) = &extras.rerun_cwd_hint {
                let _ = writeln!(s, "<p>Run from <code>{}</code>:</p>", html_escape(from));
            }
            let _ = writeln!(s, "<pre><code>{}</code></pre>", html_escape(cmd));
        }

        let _ = writeln!(s, "<h2>Input parameters</h2>");
        let _ = writeln!(s, "<h3>Rate PID</h3>");
        let _ = writeln!(s, "<table><thead><tr><th>Axis</th><th>Kp</th><th>Ki</th><th>Kd</th></tr></thead><tbody>");
        for (name, p) in [
            ("roll", self.inputs.rate_roll),
            ("pitch", self.inputs.rate_pitch),
            ("yaw", self.inputs.rate_yaw),
        ] {
            let _ = writeln!(s,
                "<tr><td>{}</td><td>{:.4}</td><td>{:.4}</td><td>{:.4}</td></tr>",
                name, p.kp, p.ki, p.kd
            );
        }
        let _ = writeln!(s, "</tbody></table>");

        let _ = writeln!(s, "<h3>Other controllers + mission</h3>");
        let _ = writeln!(s, "<ul>");
        let _ = writeln!(s, "<li>Angle ki (roll/pitch): {} / {}</li>", self.inputs.angle_roll_ki, self.inputs.angle_pitch_ki);
        let _ = writeln!(s, "<li>Altitude PID: kp={}, ki={}, kd={} (integral clamp +/- {} m.s; thrust clamp [{}, {}] x hover)</li>",
            self.inputs.alt.kp, self.inputs.alt.ki, self.inputs.alt.kd,
            self.inputs.alt_int_clamp, self.inputs.alt_min_thrust_frac, self.inputs.alt_max_thrust_frac);
        let _ = writeln!(s, "<li>Flow hold: kp_pos={}, kd_vel={}, max tilt={} rad</li>",
            self.inputs.flow_kp_pos, self.inputs.flow_kd_vel, self.inputs.flow_max_tilt_rad);
        let _ = writeln!(s, "<li>MTF-01 flow model: scale={}, noise sigma={} counts</li>",
            self.inputs.flow_scale, self.inputs.flow_noise_sigma);
        let _ = writeln!(s, "<li>Mission (s): ramp {}, climb {}, hover {}, descend {}; target altitude {} m</li>",
            self.inputs.mission_ramp_s, self.inputs.mission_climb_s, self.inputs.mission_hover_s,
            self.inputs.mission_descend_s, self.inputs.target_altitude_m);
        let _ = writeln!(s, "<li>Sim config: <code>{}</code></li>", html_escape(&self.inputs.sim_config));
        let _ = writeln!(s, "</ul>");

        let _ = writeln!(s, "<h2>Pass/fail thresholds</h2>");
        let _ = writeln!(s, "<table><thead><tr><th>Criterion</th><th>Threshold</th></tr></thead><tbody>");
        let rows = [
            ("max_tilt_deg", format!("<= {:.1} deg", self.spec.max_tilt_deg)),
            ("peak_altitude_in_band", format!("[{:.2}, {:.2}] m", self.spec.peak_altitude_min_m, self.spec.peak_altitude_max_m)),
            ("landing_distance_m", format!("<= {:.3} m", self.spec.landing_distance_max_m)),
            ("max_hover_oscillations (per axis)", format!("<= {} cycles", self.spec.max_hover_oscillations)),
            ("oscillation amplitude threshold", format!("{:.3} rad (peak-to-trough)", self.spec.oscillation_amplitude_thresh_rad)),
            ("oscillation tail window", format!("last {:.1} s of hover", self.spec.oscillation_tail_s)),
            ("settling band", format!("+/- {:.1}% of target", self.spec.settling_band_pct)),
            ("rise time", format!("to {:.0}% of target", self.spec.rise_time_pct)),
        ];
        for (name, thresh) in rows {
            let _ = writeln!(s, "<tr><td>{}</td><td>{}</td></tr>", html_escape(name), html_escape(&thresh));
        }
        let _ = writeln!(s, "</tbody></table>");

        let _ = writeln!(s, "<h2>Results</h2>");
        let _ = writeln!(s, "<h3>Hard checks</h3>");
        let _ = writeln!(s, "<table class=\"results\"><thead><tr><th>Check</th><th>Result</th><th>Detail</th></tr></thead><tbody>");
        for o in &self.hard_checks {
            let cls = if o.passed { "pass" } else { "fail" };
            let mark = if o.passed { "PASS" } else { "FAIL" };
            let _ = writeln!(s,
                "<tr class=\"{}\"><td>{}</td><td><span class=\"badge {}\">{}</span></td><td>{}</td></tr>",
                cls, html_escape(&o.name), cls, mark, html_escape(&o.detail)
            );
        }
        let _ = writeln!(s, "</tbody></table>");

        let _ = writeln!(s, "<h3>Quality metrics</h3>");
        let _ = writeln!(s, "<table><thead><tr><th>Name</th><th>Value</th><th>Unit</th></tr></thead><tbody>");
        for (name, v, unit) in &self.quality_metrics {
            let val_txt = match v {
                Some(x) => format!("{:.3}", x),
                None => "n/a".into(),
            };
            let _ = writeln!(s,
                "<tr><td>{}</td><td>{}</td><td>{}</td></tr>",
                html_escape(name), val_txt, html_escape(unit)
            );
        }
        let _ = writeln!(s, "</tbody></table>");

        let _ = writeln!(s, "<script>{}</script>", COPY_JS);
        let _ = writeln!(s, "</body></html>");
        s
    }
}

pub fn html_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}

pub const REPORT_CSS: &str = "
body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; max-width: 980px; margin: 2em auto; padding: 0 1em; color: #222; background: #fff; line-height: 1.45; }
h1, h2, h3 { border-bottom: 1px solid #e1e1e1; padding-bottom: 0.2em; }
h1 { font-size: 22px; }
h2 { font-size: 18px; margin-top: 1.6em; }
h3 { font-size: 15px; }
table { border-collapse: collapse; margin: 0.6em 0 1.2em; }
th, td { padding: 4px 10px; border: 1px solid #ccc; font-size: 13px; vertical-align: top; font-variant-numeric: tabular-nums; }
th { background: #efefef; text-align: left; }
tr.pass td  { background: #d6f2de; }
tr.fail td  { background: #f9d7da; }
tr.error td { background: #fff3cd; }
td.violate  { background: #ef8383 !important; font-weight: 600; color: #4a0000; }
.badge { display: inline-block; padding: 1px 8px; border-radius: 3px; font-weight: 600; font-size: 12px; }
.badge.pass { background: #28a745; color: #fff; }
.badge.fail { background: #dc3545; color: #fff; }
pre { background: #f6f6f6; padding: 10px 12px; border-radius: 4px; overflow-x: auto; position: relative; font-size: 13px; }
pre code { font-family: ui-monospace, Menlo, Consolas, monospace; }
img { max-width: 100%; height: auto; border: 1px solid #eee; margin: 4px 0; }
.plots { display: flex; flex-wrap: wrap; gap: 8px; }
.plots img { flex: 1 1 300px; }
.copy-btn { position: absolute; top: 6px; right: 6px; padding: 3px 10px; cursor: pointer; border: 1px solid #aaa; background: #fafafa; border-radius: 3px; font-size: 12px; }
.copy-btn:hover { background: #eaeaea; }
details summary { cursor: pointer; font-weight: bold; margin: 0.5em 0; }
details > pre { margin-top: 0.3em; }
";

pub const COPY_JS: &str = "
document.querySelectorAll('pre').forEach(p => {
  const btn = document.createElement('button');
  btn.textContent = 'copy';
  btn.className = 'copy-btn';
  btn.onclick = () => {
    const t = p.querySelector('code')?.textContent || p.textContent;
    navigator.clipboard.writeText(t.replace(/^copy\\n/, '').trim());
    btn.textContent = 'copied';
    setTimeout(() => btn.textContent = 'copy', 1200);
  };
  p.appendChild(btn);
});
";

