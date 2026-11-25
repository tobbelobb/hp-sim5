//! Multi-rope storage-tension and running-radius validation against DaRUS-5171 (2026 thesis).
//!
//! Tests three distinct 6 mm 12-strand braided ropes (LIROS D-Pro XTR Dyneema, Trowis Vectran
//! HT T190, Trowis PolyTrop FOX Polyester/TPU) across:
//!   1. `darus5171_fdp_fc_map.csv` — identification split (5 storage tensions F_S x 5 unwinding
//!      tensions F_L = 25 runs per rope) separating axial stiffness EA from drum running-radius
//!      flattening k_r.
//!   2. `darus5171_stat_valid.csv` — held-out `StatCompensationValid` split (3x3 F_S x F_L at
//!      +2 and +3 m/s) evaluated blindly across a full 4-way solver ablation (Legacy, Strained-only,
//!      Flattening-only, Full upgraded), reporting BOTH strictly frozen absolute RMSE (using the
//!      intercept frozen from `FDP-FC-MAP`, zero validation fitting) and centered RMSE.
//!   3. `darus5171_replay_wc180_dp75.csv` — true measured motor-trajectory replay (full wind at
//!      ~77 N -> turnaround -> 10 m unwind at ~177 N) with ZERO constitutive feedforward, comparing
//!      Legacy vs Upgraded `PhysicsWorld` directly against measured tension and LSV span travel.
mod common;
use common::*;

const SPOOL_END: f64 = 3.0;
/// Nominal SWP/CATi drum radius (100.36 mm nominal diameter).
const RADIUS: f64 = 0.05018;

/// Constitutive-law verification harness in `PhysicsWorld` (NOT an open-loop replay; see
/// `measured_motor_trajectory_replay_5171` for the feedforward-free measured-encoder replay).
///
/// Protocol:
///   1. Wind 10.5 m onto test drum at storage tension `f_s` and actual test speed `speed`.
///   2. Re-tension the span to `f_l`.
///   3. Unwind 10 m off test drum onto load drum at `speed` while servoing load drum to hold `f_l`.
///
/// Returns `(lsv_ratio, drum_l_ratio)` during the unwinding pass.
fn wind_then_unwind_constitutive(
    f_s: f64,
    f_l: f64,
    speed: f64,
    ea: f64,
    k_r: f64,
    strained: bool,
) -> (f64, f64) {
    let span = 2.0;
    let test = wheel(0, [-span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
    let load = wheel(1, [span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
    let segs = [Seg {
        a: 0,
        b: 1,
        rest: span / (1.0 + f_s / ea),
        pa: [-span / 2.0, RADIUS, 0.0],
        pb: [span / 2.0, RADIUS, 0.0],
    }];
    let links = [(SPOOL_END, false, 5.0), (SPOOL_END, true, 15.0)];
    let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
    w.options[1] = 0.0;
    w.set_cable_spool_params(0, k_r, strained, f_s);

    let dt = 0.002;
    let mat0 = material(&mut w, 0);

    // Phase 1: Wind 10.5 m onto test drum at constant tension f_s.
    let wind_steps = (10.5 / speed / dt).round() as usize;
    for _ in 0..wind_steps {
        for id in 0..2 {
            w.entities[id][72..75].copy_from_slice(&[0.0, 0.0, speed / RADIUS]);
        }
        w.step(dt).unwrap();
    }

    // Phase 2: Re-tension span to f_l by adjusting load drum storage (test drum fixed).
    let target_rest = span / (1.0 + f_l / ea);
    let delta_rest = rest(&mut w, 0, 0) - target_rest;
    let jo = HEADER + 2 * LINK;
    let lo1 = HEADER + LINK;
    w.path_data(0)[jo + 3] = target_rest;
    w.path_data(0)[lo1 + 2] += delta_rest;
    for _ in 0..5 {
        for id in 0..2 {
            w.entities[id][72..75].copy_from_slice(&[0.0; 3]);
        }
        w.step(dt).unwrap();
    }

    // Phase 3: Unwind 10 m off test drum at `speed` with a proportional preload servo on load drum.
    let rho = |f: f64| {
        let r_scale = if k_r > 0.0 { ((RADIUS - k_r * f) / RADIUS).max(0.5) } else { 1.0 };
        let s_scale = if strained { 1.0 / (1.0 + f / ea) } else { 1.0 };
        r_scale * s_scale
    };
    let ff_ratio = rho(f_s) / rho(f_l);

    let (angle_t0, angle_l0) = (w.entities[0][66], w.entities[1][66]);
    let (mut rope, mut tensions) = (0.0, 0.0);
    let unwind_steps = (10.0 / speed / dt).round() as usize;
    for _ in 0..unwind_steps {
        let before = stored(&mut w, 0, 0);
        let t_now = tension(&mut w, 0, 0);
        let v_load = speed * ff_ratio + 2e-4 * (f_l - t_now);
        w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, -speed / RADIUS]);
        w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, -v_load / RADIUS]);
        w.step(dt).unwrap();
        let t = tension(&mut w, 0, 0);
        rope += (before - stored(&mut w, 0, 0)) * (1.0 + t / ea);
        tensions += t;
    }

    let drum_t = (w.entities[0][66] - angle_t0).abs() * RADIUS;
    let drum_l = (w.entities[1][66] - angle_l0).abs() * RADIUS;
    let mean_t = tensions / unwind_steps as f64;
    let mat1 = material(&mut w, 0);
    assert!((mat1 - mat0).abs() < 1e-8, "material conservation violated: {mat0} -> {mat1}");
    assert!((mean_t - f_l).abs() < 0.02 * f_l, "mean tension {mean_t:.2} vs {f_l:.2}");
    (rope / drum_t, drum_l / drum_t)
}

/// Ordinary least squares for `y = b0 + b1*x1 + b2*x2`.
fn fit_plane(x1: &[f64], x2: &[f64], y: &[f64]) -> ([f64; 3], f64) {
    let n = y.len() as f64;
    let mut a = [[0.0; 3]; 3];
    let mut b = [0.0; 3];
    for ((&u, &v), &w) in x1.iter().zip(x2).zip(y) {
        let r = [1.0, u, v];
        for i in 0..3 {
            b[i] += r[i] * w;
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
    for col in 0..3 {
        let pivot = (col..3).max_by(|&i, &j| m[i][col].abs().total_cmp(&m[j][col].abs())).unwrap();
        m.swap(col, pivot);
        let div = m[col][col];
        for k in col..4 {
            m[col][k] /= div;
        }
        for row in 0..3 {
            if row != col {
                let f = m[row][col];
                for k in col..4 {
                    m[row][k] -= f * m[col][k];
                }
            }
        }
    }
    let coef = [m[0][3], m[1][3], m[2][3]];
    let sse: f64 = x1
        .iter()
        .zip(x2)
        .zip(y)
        .map(|((&u, &v), &w)| (w - (coef[0] + coef[1] * u + coef[2] * v)).powi(2))
        .sum();
    (coef, (sse / n).sqrt())
}

struct IdentRow {
    rope: String,
    f_s: f64,
    f_l: f64,
    lsv_ratio: f64,
    drum_l_ratio: f64,
}

struct ValidRow {
    rope: String,
    f_s: f64,
    f_l: f64,
    speed: f64,
    lsv_ratio: f64,
}

fn load_ident() -> Vec<IdentRow> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus5171_fdp_fc_map.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|&h| h == name).unwrap();
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            IdentRow {
                rope: f[col("rope")].to_string(),
                f_s: f[col("f_wind")].parse().unwrap(),
                f_l: f[col("f_unwind")].parse().unwrap(),
                lsv_ratio: f[col("lsv_ratio")].parse().unwrap(),
                drum_l_ratio: f[col("drum_l_ratio")].parse().unwrap(),
            }
        })
        .collect()
}

fn load_valid() -> Vec<ValidRow> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus5171_stat_valid.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|&h| h == name).unwrap();
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            ValidRow {
                rope: f[col("rope")].to_string(),
                f_s: f[col("f_s_nom")].parse().unwrap(),
                f_l: f[col("f_unwind")].parse().unwrap(),
                speed: f[col("speed_test")].parse().unwrap(),
                lsv_ratio: f[col("lsv_ratio")].parse().unwrap(),
            }
        })
        .collect()
}

/// Constitutive-law verification: identify `(EA, k_r)` per rope from DaRUS-5171 `FDP-FC-MAP`
/// and verify `PhysicsWorld` reproduces both storage-tension (`c_FS`) and unwinding-tension
/// (`c_FL`) slopes on the optical LSV ratio and encoder-only `drumL/drumT` ratio across all 3 ropes.
#[test]
fn multi_rope_storage_and_unwinding_slopes_match_solver() {
    let rows = load_ident();
    for rope in ["LIROS_D-Pro", "Trowis_VectranHT_T190PU", "Trowis_PolyTropFOX"] {
        let sub: Vec<&IdentRow> = rows.iter().filter(|r| r.rope == rope).collect();
        assert_eq!(sub.len(), 25, "expected 25 FDP-FC-MAP runs for {rope}");
        let fs: Vec<f64> = sub.iter().map(|r| r.f_s).collect();
        let fl: Vec<f64> = sub.iter().map(|r| r.f_l).collect();
        let lsv_ppm: Vec<f64> = sub.iter().map(|r| (r.lsv_ratio - 1.0) * 1e6).collect();
        let dl_ppm: Vec<f64> = sub.iter().map(|r| (r.drum_l_ratio - 1.0) * 1e6).collect();

        let (meas_lsv, meas_rms) = fit_plane(&fs, &fl, &lsv_ppm);
        let (meas_dl, _) = fit_plane(&fs, &fl, &dl_ppm);
        let ea = 1e6 / meas_lsv[2];
        let k_r = -(meas_lsv[1] + meas_lsv[2]) * 1e-6 * RADIUS;

        let grid = [75.0, 130.0, 180.0];
        let (mut sim_fs, mut sim_fl, mut sim_lsv, mut sim_dl) = (vec![], vec![], vec![], vec![]);
        for &s in &grid {
            for &l in &grid {
                let (r_lsv, r_dl) = wind_then_unwind_constitutive(s, l, 2.0, ea, k_r, true);
                sim_fs.push(s);
                sim_fl.push(l);
                sim_lsv.push((r_lsv - 1.0) * 1e6);
                sim_dl.push((r_dl - 1.0) * 1e6);
            }
        }
        let (sim_c, _) = fit_plane(&sim_fs, &sim_fl, &sim_lsv);
        let (sim_dlc, _) = fit_plane(&sim_fs, &sim_fl, &sim_dl);
        eprintln!(
            "{rope:24}: EA={:5.1} kN, k_r={:5.3} um/N, b0={:+.0} ppm | LSV (c_FS, c_FL) meas=({:+.2}, {:+.2}) sim=({:+.2}, {:+.2}) rms={meas_rms:.0} ppm | drumL meas=({:+.2}, {:+.2}) sim=({:+.2}, {:+.2})",
            ea * 1e-3,
            k_r * 1e6,
            meas_lsv[0],
            meas_lsv[1],
            meas_lsv[2],
            sim_c[1],
            sim_c[2],
            meas_dl[1],
            meas_dl[2],
            sim_dlc[1],
            sim_dlc[2],
        );
        assert!((sim_c[1] - meas_lsv[1]).abs() < 1.0, "{rope} c_FS mismatch");
        assert!((sim_c[2] - meas_lsv[2]).abs() < 1.0, "{rope} c_FL mismatch");
        assert!((sim_dlc[1] + sim_dlc[2]).abs() < 0.5, "{rope} drumL not antisymmetric");
        assert!((sim_dlc[1] - meas_dl[1]).abs() < 3.0, "{rope} drumL c_FS mismatch");
    }
}

/// Review Items 8, 9, 10: Blind held-out validation on `StatCompensationValid` (+2 and +3 m/s)
/// across all 4 solver configurations:
///   1. Legacy solver (`strained=false, k_r=0`)
///   2. Strained spool only (`strained=true, k_r=0`)
///   3. Radial flattening only (`strained=false, k_r=k_r_id`)
///   4. Full upgraded solver (`strained=true, k_r=k_r_id`)
///
/// Reports BOTH strictly frozen absolute RMSE (using the sensor-scale intercept `b0_id` frozen
/// from `FDP-FC-MAP`, with ZERO fitting on validation data) AND centered RMSE, plus speed breakdown.
#[test]
fn blind_validation_ablation_on_held_out_stat_compensation_runs() {
    let ident = load_ident();
    let valid = load_valid();

    for (rope, max_centered_ppm, max_frozen_ppm) in [
        ("LIROS_D-Pro", 200.0, 350.0),
        ("Trowis_VectranHT_T190PU", 200.0, 500.0),
        ("Trowis_PolyTropFOX", 1100.0, 1200.0),
    ] {
        let id_sub: Vec<&IdentRow> = ident.iter().filter(|r| r.rope == rope).collect();
        let fs: Vec<f64> = id_sub.iter().map(|r| r.f_s).collect();
        let fl: Vec<f64> = id_sub.iter().map(|r| r.f_l).collect();
        let lsv_ppm: Vec<f64> = id_sub.iter().map(|r| (r.lsv_ratio - 1.0) * 1e6).collect();
        let (coef, _) = fit_plane(&fs, &fl, &lsv_ppm);
        let b0_frozen = coef[0]; // frozen identification-split sensor-scale intercept
        let ea = 1e6 / coef[2];
        let k_r = -(coef[1] + coef[2]) * 1e-6 * RADIUS;

        let val_sub: Vec<&ValidRow> = valid.iter().filter(|r| r.rope == rope).collect();
        assert_eq!(val_sub.len(), 18, "expected 18 held-out runs for {rope}");

        let eval = |kr_cfg: f64, strained_cfg: bool| -> (f64, f64, f64, f64, f64) {
            let mut errs_raw = Vec::with_capacity(val_sub.len());
            let mut errs_frozen = Vec::with_capacity(val_sub.len());
            let mut errs_v2 = Vec::new();
            let mut errs_v3 = Vec::new();
            for r in &val_sub {
                let (pred_ratio, _) =
                    wind_then_unwind_constitutive(r.f_s, r.f_l, r.speed, ea, kr_cfg, strained_cfg);
                let meas_ppm = (r.lsv_ratio - 1.0) * 1e6;
                let pred_ppm = (pred_ratio - 1.0) * 1e6;
                let e_raw = meas_ppm - pred_ppm;
                let e_froz = meas_ppm - (pred_ppm + b0_frozen);
                errs_raw.push(e_raw);
                errs_frozen.push(e_froz);
                if r.speed < 2.5 {
                    errs_v2.push(e_froz);
                } else {
                    errs_v3.push(e_froz);
                }
            }
            let mean_raw = errs_raw.iter().sum::<f64>() / errs_raw.len() as f64;
            let rmse_centered = (errs_raw.iter().map(|e| (e - mean_raw).powi(2)).sum::<f64>()
                / errs_raw.len() as f64)
                .sqrt();
            let rmse_frozen =
                (errs_frozen.iter().map(|e| e * e).sum::<f64>() / errs_frozen.len() as f64).sqrt();
            let rmse_v2 = (errs_v2.iter().map(|e| e * e).sum::<f64>() / errs_v2.len() as f64).sqrt();
            let rmse_v3 = (errs_v3.iter().map(|e| e * e).sum::<f64>() / errs_v3.len() as f64).sqrt();
            (rmse_frozen, rmse_centered, mean_raw - b0_frozen, rmse_v2, rmse_v3)
        };

        let (leg_f, leg_c, _, _, _) = eval(0.0, false);
        let (str_f, str_c, _, _, _) = eval(0.0, true);
        let (flt_f, flt_c, _, _, _) = eval(k_r, false);
        let (ful_f, ful_c, bias_shift, ful_v2, ful_v3) = eval(k_r, true);

        let meas_vals: Vec<f64> = val_sub.iter().map(|r| (r.lsv_ratio - 1.0) * 1e6).collect();
        let mean_m = meas_vals.iter().sum::<f64>() / meas_vals.len() as f64;
        let const_c = (meas_vals.iter().map(|m| (m - mean_m).powi(2)).sum::<f64>()
            / meas_vals.len() as f64)
            .sqrt();

        eprintln!(
            "[5171 ABLATION] {rope:24} (n=18, frozen b0={b0_frozen:+.0} ppm, val bias shift={bias_shift:+.0} ppm):\n  \
             const-mean : centered={const_c:6.1} ppm\n  \
             legacy     : frozen={leg_f:6.1} ppm | centered={leg_c:6.1} ppm\n  \
             strained   : frozen={str_f:6.1} ppm | centered={str_c:6.1} ppm\n  \
             flattening : frozen={flt_f:6.1} ppm | centered={flt_c:6.1} ppm\n  \
             FULL       : frozen={ful_f:6.1} ppm | centered={ful_c:6.1} ppm (v=2m/s: {ful_v2:.1}, v=3m/s: {ful_v3:.1} ppm)"
        );

        assert!(ful_c < max_centered_ppm, "{rope} centered RMSE {ful_c:.1} > {max_centered_ppm:.1}");
        assert!(ful_f < max_frozen_ppm, "{rope} frozen RMSE {ful_f:.1} > {max_frozen_ppm:.1}");
        // On strictly frozen RMSE (zero validation fitting), Full beats EVERY ablation on all 3 ropes!
        assert!(ful_f < leg_f, "{rope}: Full frozen ({ful_f:.1}) must beat Legacy ({leg_f:.1})");
        assert!(ful_f < str_f, "{rope}: Full frozen ({ful_f:.1}) must beat Strained-only ({str_f:.1})");
        assert!(ful_f < flt_f, "{rope}: Full frozen ({ful_f:.1}) must beat Flattening-only ({flt_f:.1})");
        assert!(ful_c < leg_c, "{rope}: Full centered ({ful_c:.1}) must beat Legacy ({leg_c:.1})");
        // On both bare 12-strand braids, Full also beats every single-mechanism ablation on centered RMSE.
        // (TPU-jacketed PolyTropFOX softens/runs in between 2025-02-21 ident and 2025-03-03 valid,
        // so single-mechanism ablations have lower centered slope error while being 2800-4300 ppm off in frozen RMSE.)
        if rope != "Trowis_PolyTropFOX" {
            assert!(ful_c < str_c, "{rope}: Full centered ({ful_c:.1}) must beat Strained-only ({str_c:.1})");
            assert!(ful_c < flt_c, "{rope}: Full centered ({ful_c:.1}) must beat Flattening-only ({flt_c:.1})");
        }
    }
}

/// Review Item 11: True measured motor-trajectory replay on DaRUS-5171 (`darus5171_replay_wc180_dp75.csv`,
/// LIROS D-Pro `WC180/Export 1.csv`: full 18 m wind at F_S ~ 77 N -> turnaround -> 10 m unwind at
/// F_L ~ 177 N). Drives both drums strictly from recorded encoder positions `(x_t(t), x_l(t))`
/// with ZERO constitutive feedforward and ZERO rest-length edits!
///
/// Over the 10 m unwind window, the hardware load drum had to wind 19.0 mm further than the test
/// drum unwound because rope stored at 77 N carries more material per drum arc than rope wound at
/// 177 N. Legacy `PhysicsWorld` absorbs that 19 mm directly into span stretch, causing tension to
/// explode by ~+750 N; Upgraded `PhysicsWorld` absorbs it via the LIFO strained-spool + flattening
/// physics so tension stays flat near the measured ~177 N AND span travel matches the LSV.
#[test]
fn measured_motor_trajectory_replay_5171() {
    let ident = load_ident();
    let dpro: Vec<&IdentRow> = ident.iter().filter(|r| r.rope == "LIROS_D-Pro").collect();
    let fs: Vec<f64> = dpro.iter().map(|r| r.f_s).collect();
    let fl: Vec<f64> = dpro.iter().map(|r| r.f_l).collect();
    let lsv_ppm: Vec<f64> = dpro.iter().map(|r| (r.lsv_ratio - 1.0) * 1e6).collect();
    let (coef, _) = fit_plane(&fs, &fl, &lsv_ppm);
    let ea = 1e6 / coef[2];
    let k_r = -(coef[1] + coef[2]) * 1e-6 * RADIUS;

    let path_csv = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus5171_replay_wc180_dp75.csv");
    let text = std::fs::read_to_string(path_csv).unwrap();
    let rows: Vec<[f64; 7]> = text
        .lines()
        .skip(1)
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            [
                f[0].parse().unwrap(), // t
                f[1].parse().unwrap(), // x_t
                f[2].parse().unwrap(), // x_l
                f[3].parse().unwrap(), // f_t
                f[4].parse().unwrap(), // f_l
                f[5].parse().unwrap(), // x_lsv
                f[6].parse().unwrap(), // trig_rec
            ]
        })
        .collect();

    let rec_indices: Vec<usize> = rows
        .iter()
        .enumerate()
        .filter(|(_, r)| r[6] > 0.5)
        .map(|(i, _)| i)
        .collect();
    let i_rec0 = *rec_indices.first().unwrap();
    let i_rec1 = *rec_indices.last().unwrap();
    let meas_f_drift = rows[i_rec1][3] - rows[i_rec0][3];
    let meas_f_mean = rec_indices.iter().map(|&i| rows[i][3]).sum::<f64>() / rec_indices.len() as f64;
    let meas_lsv_ratio = (rows[i_rec1][5] - rows[i_rec0][5]) / (rows[i_rec1][1] - rows[i_rec0][1]);

    let replay = |kr_cfg: f64, strained_cfg: bool| {
        let span = 2.0;
        let f0 = rows[0][3].max(1.0);
        let test = wheel(0, [-span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
        let load = wheel(1, [span / 2.0, 0.0, 0.0], RADIUS, 0.5, 1e6, None);
        let segs = [Seg {
            a: 0,
            b: 1,
            rest: span / (1.0 + f0 / ea),
            pa: [-span / 2.0, RADIUS, 0.0],
            pb: [span / 2.0, RADIUS, 0.0],
        }];
        let links = [(SPOOL_END, false, 5.0), (SPOOL_END, true, 25.0)];
        let mut w = world(vec![test, load], vec![path(&links, &segs, ea, 0.0, 4.0)]);
        w.options[1] = 0.0;
        w.set_cable_spool_params(0, kr_cfg, strained_cfg, f0);

        let sub = 10;
        let mut f_rec_start = 0.0;
        let mut f_rec_end = 0.0;
        let mut f_rec_sum = 0.0;
        let mut f_rec_n = 0usize;
        let mut rope_rec = 0.0;
        for k in 1..rows.len() {
            let dt_tot = rows[k][0] - rows[k - 1][0];
            let dt = dt_tot / sub as f64;
            let omega_t = -(rows[k][1] - rows[k - 1][1]) / (RADIUS * dt_tot);
            let omega_l = (rows[k][2] - rows[k - 1][2]) / (RADIUS * dt_tot);
            let in_rec = k > i_rec0 && k <= i_rec1;
            for _ in 0..sub {
                let before = stored(&mut w, 0, 0);
                w.entities[0][72..75].copy_from_slice(&[0.0, 0.0, omega_t]);
                w.entities[1][72..75].copy_from_slice(&[0.0, 0.0, omega_l]);
                w.step(dt).unwrap();
                if in_rec {
                    let t = tension(&mut w, 0, 0);
                    rope_rec += (before - stored(&mut w, 0, 0)) * (1.0 + t / ea);
                }
            }
            if k == i_rec0 {
                f_rec_start = tension(&mut w, 0, 0);
            }
            if k >= i_rec0 && k <= i_rec1 {
                let t = tension(&mut w, 0, 0);
                f_rec_sum += t;
                f_rec_n += 1;
                if k == i_rec1 {
                    f_rec_end = t;
                }
            }
        }
        let f_drift = f_rec_end - f_rec_start;
        let f_mean = f_rec_sum / f_rec_n as f64;
        let lsv_ratio = rope_rec / (rows[i_rec1][1] - rows[i_rec0][1]);
        (f_mean, f_drift, lsv_ratio)
    };

    let (leg_f, leg_drift, leg_lsv) = replay(0.0, false);
    let (upg_f, upg_drift, upg_lsv) = replay(k_r, true);

    let b0_frozen = coef[0] * 1e-6;
    let leg_lsv_err_ppm = ((leg_lsv + b0_frozen) - meas_lsv_ratio).abs() * 1e6;
    let upg_lsv_err_ppm = ((upg_lsv + b0_frozen) - meas_lsv_ratio).abs() * 1e6;
    eprintln!(
        "[5171 REPLAY WC180/DP75] (zero feedforward, measured x_T & x_L, frozen b0={:+.0} ppm):\n  \
         measured : f_mean={meas_f_mean:6.1} N, f_drift={meas_f_drift:+6.1} N, lsv_ratio={meas_lsv_ratio:.6}\n  \
         legacy   : f_mean={leg_f:6.1} N, f_drift={leg_drift:+6.1} N, lsv+b0={:.6} (err={leg_lsv_err_ppm:.0} ppm)\n  \
         upgraded : f_mean={upg_f:6.1} N, f_drift={upg_drift:+6.1} N, lsv+b0={:.6} (err={upg_lsv_err_ppm:.0} ppm)",
        coef[0],
        leg_lsv + b0_frozen,
        upg_lsv + b0_frozen,
    );

    // Legacy loses tension during the 18 m wind pass and enters the 10 m unwind pass nearly slack
    // (~21 N mean vs 177 N measured, drifting +104 N), whereas Upgraded tracks the entire 28 m
    // wind -> turnaround -> unwind trajectory within ~9 N of measured tension and ~200 ppm on LSV!
    assert!(
        (leg_f - meas_f_mean).abs() > 100.0,
        "expected huge legacy mean-tension error (>100 N), got {leg_f:.1} vs {meas_f_mean:.1} N"
    );
    assert!(
        (upg_f - meas_f_mean).abs() < 25.0,
        "upgraded replay mean tension {upg_f:.1} N drifted too far from measured {meas_f_mean:.1} N"
    );
    assert!(
        upg_drift.abs() < 40.0 && upg_drift.abs() < leg_drift.abs() * 0.5,
        "upgraded drift {upg_drift:+.1} N should stay small vs legacy {leg_drift:+.1} N"
    );
    assert!(
        upg_lsv_err_ppm < 300.0,
        "upgraded LSV ratio with frozen b0 had error {upg_lsv_err_ppm:.0} ppm (expected <300 ppm)"
    );
}
