//! The DaRUS-3600 cable actuator test rig (CATi) as a headless scene: rope runs off a test
//! winch drum onto a load winch drum at constant tension and speed. The observable is the one
//! the rig's laser surface velocimeter (LSV) gives: rope travel in the free span per drum
//! travel, as a function of tension.
//!
//! Fixtures:
//!   - `tests/fixtures/reference_data/darus3600_runs.csv` (348 passes, `summarize_3600.py`).
//!   - `tests/fixtures/reference_data/darus3600_replay_pair.csv` (measured 125 N, 2 m/s unwind
//!     and rewind motor trajectories, `extract_replay_traces.py`).
//!
//! Note on rope SKU: DaRUS-3600 uses 6 mm LIROS D-Pro 01505-0600 (SK78), whereas DaRUS-5171
//! identifies parameters on 6 mm LIROS D-Pro XTR (SK99). Cross-dataset transfer tests how well
//! the 2-parameter spool law transfers across closely related 6 mm 12-strand LIROS D-Pro braids.
mod common;
use common::*;

const SPOOL_END: f64 = 3.0;
/// Rig drum radius (nominal drum diameter 100.36 mm).
const RADIUS: f64 = 0.05018;
/// SW-G3.1 90° carriage-pulley force-sensor conversion (`F_true = sqrt(2) * f_logged`).
/// Raw CSV fixtures preserve logged sensor values untouched; conversion to physical Newtons
/// happens explicitly here.
const WINCH_FORCE_SCALE: f64 = std::f64::consts::SQRT_2;
const SPEEDS: [f64; 3] = [0.5, 2.0, 5.0];

/// Rope travel in the span per drum travel over a 10 m pass at `force` and `speed`.
///
/// The drums are flywheels whose angular velocity is reset every step, as a perfect speed
/// servo. The spool's initial storage tension is explicitly set to `force` (matching the rig's
/// pre-wound test state).
fn rope_per_drum(
    force: f64,
    speed: f64,
    ea: f64,
    radial_flattening: f64,
    strained_spool: bool,
) -> f64 {
    let span = 2.0;
    let test = wheel(0, [-span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
    let load = wheel(1, [span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: span / (1.0 + force / ea),
        pa: [-span / 2.0, RADIUS, 0.0],
        pb: [span / 2.0, RADIUS, 0.0],
    }];
    // Test drum pays out, load drum winds in.
    let links = [(SPOOL_END, false, 15.0), (SPOOL_END, true, 15.0)];
    let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
    w.options[1] = 0.0;
    // Explicitly declare that the pre-wound rope on the CATi rig was wound at `force`.
    w.set_cable_spool_params(0, radial_flattening, strained_spool, force);
    let dt = 0.001;
    let step = |w: &mut PhysicsWorld| {
        for id in 0..2 {
            w.entities[id][72..75].copy_from_slice(&[0.0, 0.0, -speed / RADIUS]);
        }
        w.step(dt).unwrap();
    };
    for _ in 0..50 {
        step(&mut w);
    }
    let (stored0, angle0) = (stored(&mut w, 0, 0), w.entities[0][66]);
    let (mut rope, mut tensions) = (0.0, 0.0);
    let steps = ((5.0 / speed) / dt).round() as usize;
    for _ in 0..steps {
        let before = stored(&mut w, 0, 0);
        step(&mut w);
        let t = tension(&mut w, 0, 0);
        rope += (before - stored(&mut w, 0, 0)) * (1.0 + t / ea);
        tensions += t;
    }
    let drum = (w.entities[0][66] - angle0).abs() * RADIUS;
    let mean = tensions / steps as f64;
    assert!((mean - force).abs() < 0.01 * force, "tension {mean} vs {force}");
    if radial_flattening == 0.0 && !strained_spool {
        assert!((stored0 - stored(&mut w, 0, 0) - drum).abs() < 1e-6 * drum, "rest flow per drum arc");
    }
    rope / drum
}

/// Simulated ratio slope over the rig's physical force range, in ppm of drum travel per N.
fn simulated_slope(speed: f64, ea: f64, radial_flattening: f64, strained_spool: bool) -> f64 {
    let forces = [70.0 * WINCH_FORCE_SCALE, 125.0 * WINCH_FORCE_SCALE, 180.0 * WINCH_FORCE_SCALE];
    let ratios: Vec<f64> = forces
        .iter()
        .map(|&f| rope_per_drum(f, speed, ea, radial_flattening, strained_spool))
        .collect();
    fit(&forces, &ratios) * 1e6
}

fn fit(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let (mx, my) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
    let sxy: f64 = x.iter().zip(y).map(|(a, b)| (a - mx) * (b - my)).sum();
    let sxx: f64 = x.iter().map(|a| (a - mx) * (a - mx)).sum();
    sxy / sxx
}

/// Fit LIROS D-Pro `(EA, k_r)` directly from `darus5171_fdp_fc_map.csv` so cross-dataset tests
/// cannot drift from the identification fixture.
fn identify_dpro_from_5171() -> (f64, f64) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus5171_fdp_fc_map.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|&h| h == name).unwrap();
    let (mut a, mut b) = ([[0.0; 3]; 3], [0.0; 3]);
    for line in lines.filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split(',').collect();
        if f[col("rope")] != "LIROS_D-Pro" {
            continue;
        }
        let r = [
            1.0,
            f[col("f_wind")].parse::<f64>().unwrap() * WINCH_FORCE_SCALE,
            f[col("f_unwind")].parse::<f64>().unwrap() * WINCH_FORCE_SCALE,
        ];
        let y = (f[col("lsv_ratio")].parse::<f64>().unwrap() - 1.0) * 1e6;
        for i in 0..3 {
            b[i] += r[i] * y;
            for j in 0..3 {
                a[i][j] += r[i] * r[j];
            }
        }
    }
    let mut m = [[0.0; 4]; 3];
    for i in 0..3 {
        m[i][..3].copy_from_slice(&a[i]);
        m[i][3] = b[i];
    }
    for c in 0..3 {
        let p = (c..3).max_by(|&i, &j| m[i][c].abs().total_cmp(&m[j][c].abs())).unwrap();
        m.swap(c, p);
        let d = m[c][c];
        for k in c..4 {
            m[c][k] /= d;
        }
        for r in 0..3 {
            if r != c {
                let s = m[r][c];
                for k in c..4 {
                    m[r][k] -= s * m[c][k];
                }
            }
        }
    }
    let ea = 1e6 / m[2][3];
    let k_r = -(m[1][3] + m[2][3]) * 1e-6 * RADIUS;
    (ea, k_r)
}

/// Measured LSV/drum slope in ppm/N: regression on force within each control x direction x
/// speed cell (cell means removed), pooled over all runs. Also returns per-cell `(key, slope, residual_sd_ppm)`.
fn measured_cells() -> (f64, Vec<(String, f64, f64)>) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus3600_runs.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|&h| h == name).unwrap();
    let (control, direction, speed, force, lsv) =
        (col("control"), col("direction"), col("speed"), col("force_test"), col("lsv_ratio"));
    let mut cells: std::collections::BTreeMap<String, (Vec<f64>, Vec<f64>)> = Default::default();
    for line in lines.filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split(',').collect();
        let cell = cells.entry(format!("{} {} {}", f[control], f[direction], f[speed])).or_default();
        cell.0.push(f[force].parse::<f64>().unwrap() * WINCH_FORCE_SCALE);
        cell.1.push(f[lsv].parse().unwrap());
    }
    let (mut sxy, mut sxx, mut out) = (0.0, 0.0, vec![]);
    for (k, (x, y)) in &cells {
        let n = x.len() as f64;
        let (mx, my) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
        let cxy: f64 = x.iter().zip(y).map(|(a, b)| (a - mx) * (b - my)).sum();
        let cxx: f64 = x.iter().map(|a| (a - mx) * (a - mx)).sum();
        sxy += cxy;
        sxx += cxx;
        let slope = cxy / cxx;
        let resid_ppm = (x
            .iter()
            .zip(y)
            .map(|(a, b)| ((b - my) - slope * (a - mx)).powi(2))
            .sum::<f64>()
            / n)
            .sqrt()
            * 1e6;
        out.push((k.clone(), slope * 1e6, resid_ppm));
    }
    (sxy / sxx * 1e6, out)
}

/// Legacy mode (k_r = 0, strained = false): spooled rope is stored as rest length at fixed radius,
/// so at constant tension span rope outruns the drum surface by the strain: +1e6/EA ppm/N.
#[test]
fn spool_pays_out_rest_length() {
    for ea in [1e5, 3e5] {
        let slope = simulated_slope(2.0, ea, 0.0, false);
        eprintln!("EA {ea:.0e} N: simulated slope {slope:+.3} ppm/N, 1e6/EA {:+.3}", 1e6 / ea);
        assert!((slope - 1e6 / ea).abs() < 0.02 * 1e6 / ea);
    }
}

/// Pure strained spool storage (no radial flattening) cancels the span strain at constant
/// tension (F_S == F_L), giving zero slope regardless of EA.
#[test]
fn strained_spool_cancels_constant_tension_strain() {
    let (ea_dpro, _) = identify_dpro_from_5171();
    let slope = simulated_slope(2.0, ea_dpro, 0.0, true);
    assert!(slope.abs() < 0.1, "expected ~0 ppm/N, got {slope:+.3}");
}

#[test]
fn rig_data_slope_is_negative_in_every_cell() {
    let (pooled, cells) = measured_cells();
    eprintln!("measured pooled slope {pooled:+.2} ppm/N across {} cells:", cells.len());
    for (k, s, r) in &cells {
        eprintln!("  {k:26}: slope={s:+.2} ppm/N, within-cell repeat RMS={r:.1} ppm");
    }
    assert_eq!(cells.len(), 12);
    assert!(cells.iter().all(|(_, s, _)| (-9.0..-3.5).contains(s)));
}

/// Blind cross-dataset prediction across all 3 experimental rig speeds (0.5, 2.0, 5.0 m/s):
/// LIROS D-Pro XTR parameters identified live from DaRUS-5171 `FDP-FC-MAP` (in physical Newtons)
/// predict DaRUS-3600's 348-run pooled slope (-6.15 ppm/N) and per-speed cells without fitting on 3600.
#[test]
fn rig_matches_measured_running_radius_across_speeds() {
    let (ea_dpro, k_r_dpro) = identify_dpro_from_5171();
    assert!((ea_dpro - 118_500.0).abs() < 250.0, "5171 D-Pro EA drifted: {ea_dpro:.1}");
    assert!((k_r_dpro * 1e6 - 0.299).abs() < 0.005, "5171 D-Pro k_r drifted: {:.4}", k_r_dpro * 1e6);

    let (measured_pooled, cells) = measured_cells();
    for speed in SPEEDS {
        let simulated = simulated_slope(speed, ea_dpro, k_r_dpro, true);
        let speed_tag = format!(" {speed:.1}");
        let speed_cells: Vec<&(String, f64, f64)> =
            cells.iter().filter(|(k, _, _)| k.ends_with(&speed_tag)).collect();
        assert_eq!(speed_cells.len(), 4, "expected 4 cells for speed {speed:.1}");
        let meas_speed = speed_cells.iter().map(|c| c.1).sum::<f64>() / speed_cells.len() as f64;
        eprintln!(
            "[3600 v={speed:.1} m/s] sim={simulated:+.2} ppm/N vs meas_mean={meas_speed:+.2} ppm/N (pooled={measured_pooled:+.2} ppm/N)"
        );
        assert!(
            (simulated - measured_pooled).abs() < 1.0,
            "v={speed}: simulated {simulated:+.2} vs pooled measured {measured_pooled:+.2} ppm/N"
        );
        assert!(
            (simulated - meas_speed).abs() < 2.0,
            "v={speed}: simulated {simulated:+.2} vs speed-cell mean {meas_speed:+.2} ppm/N"
        );
    }
}

/// Review Item 11: Measured motor-trajectory replay of DaRUS-3600 (`darus3600_replay_pair.csv`,
/// 125 N, 2 m/s, both unwind and rewind) with ZERO constitutive feedforward.
/// Drives both drums directly from the recorded encoder positions `x_t(t), x_l(t)` and compares
/// predicted span travel `d_lsv_sim / d_xT` against measured `d_lsv_meas / d_xT` for legacy vs upgraded.
#[test]
fn measured_motor_trajectory_replay_3600() {
    let (ea_dpro, k_r_dpro) = identify_dpro_from_5171();
    let path_csv = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus3600_replay_pair.csv");
    let text = std::fs::read_to_string(path_csv).unwrap();
    let mut phases: std::collections::BTreeMap<String, Vec<[f64; 6]>> = Default::default();
    for line in text.lines().skip(1).filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split(',').collect();
        phases.entry(f[0].to_string()).or_default().push([
            f[1].parse().unwrap(),                                   // t
            f[2].parse().unwrap(),                                   // x_t
            f[3].parse().unwrap(),                                   // x_l
            f[4].parse::<f64>().unwrap() * WINCH_FORCE_SCALE,        // f_t (physical N)
            f[5].parse::<f64>().unwrap() * WINCH_FORCE_SCALE,        // f_l (physical N)
            f[6].parse().unwrap(),                                   // x_lsv
        ]);
    }
    let mut upgraded_ratios = Vec::new();
    let mut legacy_ratios = Vec::new();
    for (phase, rows) in &phases {
        let f0 = rows[0][3];
        let replay = |k_r: f64, strained: bool| {
            let span = 2.0;
            let test = wheel(0, [-span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
            let load = wheel(1, [span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
            let segs = [Seg {
                a: 0,
                b: 1,
                rest: span / (1.0 + f0 / ea_dpro),
                pa: [-span / 2.0, RADIUS, 0.0],
                pb: [span / 2.0, RADIUS, 0.0],
            }];
            let links = [(SPOOL_END, false, 15.0), (SPOOL_END, true, 15.0)];
            let mut w = world(vec![test, load], vec![path(&links, &segs, ea_dpro, 0.0, 4.0)]);
            w.options[1] = 0.0;
            w.set_cable_spool_params(0, k_r, strained, f0);

            let sub = 10;
            let mut rope = 0.0;
            for k in 1..rows.len() {
                let dt_tot = rows[k][0] - rows[k - 1][0];
                let dt = dt_tot / sub as f64;
                let omega_t = -(rows[k][1] - rows[k - 1][1]) / (RADIUS * dt_tot);
                let omega_l = (rows[k][2] - rows[k - 1][2]) / (RADIUS * dt_tot);
                for _ in 0..sub {
                    let before = stored(&mut w, 0, 0);
                    w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, omega_t]);
                    w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, omega_l]);
                    w.step(dt).unwrap();
                    let t = tension(&mut w, 0, 0);
                    rope += (before - stored(&mut w, 0, 0)) * (1.0 + t / ea_dpro);
                }
            }
            let d_xt = rows.last().unwrap()[1] - rows[0][1];
            rope / d_xt
        };

        let ratio_legacy = replay(0.0, false);
        let ratio_upgraded = replay(k_r_dpro, true);
        let d_xt = rows.last().unwrap()[1] - rows[0][1];
        let d_lsv = rows.last().unwrap()[5] - rows[0][5];
        let ratio_meas = d_lsv / d_xt;
        let err_legacy_ppm = (ratio_legacy - ratio_meas).abs() * 1e6;
        let err_upgraded_ppm = (ratio_upgraded - ratio_meas).abs() * 1e6;
        eprintln!(
            "[3600 REPLAY {phase:6}] meas={ratio_meas:.6} | legacy={ratio_legacy:.6} (err={err_legacy_ppm:.0} ppm) | upgraded={ratio_upgraded:.6} (err={err_upgraded_ppm:.0} ppm)"
        );
        // Upgraded beats legacy on both phases and stays within the LSV's +/-1200 ppm absolute scale band.
        assert!(err_upgraded_ppm < err_legacy_ppm, "{phase}: upgraded must beat legacy");
        assert!(err_upgraded_ppm < 1200.0, "{phase}: upgraded replay error {err_upgraded_ppm:.0} ppm too high");
        upgraded_ratios.push(ratio_upgraded);
        legacy_ratios.push(ratio_legacy);
    }
    // Measured unwind vs rewind symmetry is 72 ppm (0.999768 vs 0.999840).
    // Upgraded reproduces this unwind/rewind symmetry to <100 ppm (vs legacy >1500 ppm asymmetry!).
    let upg_asym_ppm = (upgraded_ratios[0] - upgraded_ratios[1]).abs() * 1e6;
    let leg_asym_ppm = (legacy_ratios[0] - legacy_ratios[1]).abs() * 1e6;
    eprintln!("[3600 REPLAY symmetry] legacy asym={leg_asym_ppm:.0} ppm vs upgraded asym={upg_asym_ppm:.0} ppm");
    assert!(upg_asym_ppm < 100.0, "upgraded unwind/rewind asymmetry {upg_asym_ppm:.0} ppm too high");
    assert!(leg_asym_ppm > 1000.0, "expected large legacy unwind/rewind asymmetry");
}
