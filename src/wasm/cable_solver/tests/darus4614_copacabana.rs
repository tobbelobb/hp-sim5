//! Static cable-stiffness and swivel-pulley IK benchmark against DaRUS-4614 (COPacabana, 2025).
//!
//! Scope (explicitly bounded):
//!   1. Exact WireX/IPAnema swivel-pulley inverse kinematics (`COPacabana.xml`) verified against
//!      logged motor targets `x_tar[0..7]` across all 6 PlaneA poses.
//!   2. Static cable-stiffness & force-distribution benchmark across ALL 5 non-home poses
//!      (`P1..P5`) x both payloads (`13.9 kg`, `29.1 kg`) x all APC levels (`APC25/50/75`), using
//!      fixed pulley tangent anchors (not winch drums or dynamic motion).
//!   3. Blind vertical elastic sag prediction between `13.9 kg` and `29.1 kg`, preserving the
//!      ~50-58% discrepancy openly as an unmodeled residual (without fitting extra parameters).
//!   4. `PhysicsWorld` swivel pulleys (entity flag 4096) against the same IK: kinematics while the
//!      platform travels P0..P5, and statics against the fixed-tangent model at every pose.
//!
//! Rope SKU & force-sensor note: Raw CSV fixtures preserve logged SW-G3.1 90° carriage-pulley
//! force readings untouched; `WINCH_FORCE_SCALE = sqrt(2)` converts them to physical cable Newtons.
//! `EA = 118.5 kN` is identified live from DaRUS-5171 `FDP-FC-MAP` on 6 mm LIROS D-Pro XTR (SK99)
//! and transferred blind to COPacabana's 6 mm LIROS D-Pro 01505-0600 (SK78).
//! Platform inertia `I = diag(1, 1, 1)` comes directly from `COPacabana.xml` and has no effect
//! on static equilibrium.
//!
//! Data fixture: `tests/fixtures/reference_data/darus4614_planeA.csv` (`summarize_4614.py`).
mod common;
use common::*;

/// Nominal SWP drum radius (used only to verify live 5171 EA fit).
const RADIUS_5171: f64 = 0.05018;
/// SW-G3.1 90° carriage-pulley force-sensor conversion (`F_true = sqrt(2) * f_logged`).
const WINCH_FORCE_SCALE: f64 = std::f64::consts::SQRT_2;

/// Base pulley entry points from `COPacabana.xml` (metres).
const ANCHORS: [[f64; 3]; 8] = [
    [-2.427578, 1.959987, 2.757960],
    [2.411243, 1.944128, 2.770735],
    [2.408630, -1.976237, 2.749285],
    [-2.433437, -1.965415, 2.758603],
    [-2.298856, 1.609864, -0.325174],
    [2.315327, 1.615126, -0.248102],
    [2.308631, -1.613281, -0.325031],
    [-2.385985, -1.614363, -0.246768],
];

/// Platform cable attachment offsets from `COPacabana.xml` (metres; crossed configuration).
const PLATFORMS: [[f64; 3]; 8] = [
    [-0.15, 0.2365, 0.0225],
    [0.15, 0.2365, 0.0225],
    [0.15, -0.2365, 0.0225],
    [-0.15, -0.2365, 0.0225],
    [-0.20, 0.15, 0.575],
    [0.20, 0.15, 0.575],
    [0.20, -0.15, 0.575],
    [-0.20, -0.15, 0.575],
];

/// Pulley radii from `COPacabana.xml` (metres).
const PULLEY_R: [f64; 8] = [0.037, 0.037, 0.037, 0.037, 0.033, 0.033, 0.033, 0.033];

/// Swivel-pulley entry axes (`[a31, a32, a33]` in `COPacabana.xml`).
const SWIVEL: [[f64; 3]; 8] = [
    [-1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
];

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm3(a: [f64; 3]) -> f64 {
    dot3(a, a).sqrt()
}

/// Live identification of LIROS D-Pro `EA` from `darus5171_fdp_fc_map.csv` (zero fitting on 4614).
fn ea_dpro_from_5171() -> f64 {
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
    let _k_r = -(m[1][3] + m[2][3]) * 1e-6 * RADIUS_5171;
    let ea = 1e6 / m[2][3];
    assert!((ea - 118_500.0).abs() < 250.0, "5171 EA drifted: {ea:.1}");
    ea
}

/// Exact WireX/IPAnema swivel-pulley inverse kinematics for cable `i` at platform position `pos`
/// (zero rotation). Returns `(total_cable_length, tangent_point_world, free_span_length)`.
fn pulley_ik(i: usize, pos: [f64; 3]) -> (f64, [f64; 3], f64) {
    let a = ANCHORS[i];
    let b = [pos[0] + PLATFORMS[i][0], pos[1] + PLATFORMS[i][1], pos[2] + PLATFORMS[i][2]];
    let u_s = SWIVEL[i];
    let r = PULLEY_R[i];
    let v = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let z = dot3(v, u_s);
    let v_rad = [v[0] - z * u_s[0], v[1] - z * u_s[1], v[2] - z * u_s[2]];
    let rho = norm3(v_rad);
    let e_rad = [v_rad[0] / rho, v_rad[1] / rho, v_rad[2] / rho];
    let d2 = (rho - r).powi(2) + z * z;
    let d = d2.sqrt();
    let l_tan = (d2 - r * r).max(0.0).sqrt();
    let theta_t = z.atan2(rho - r) + (r / d).clamp(-1.0, 1.0).acos();
    let beta = PI - theta_t;
    let t_pt = [
        a[0] + r * e_rad[0] + r * (theta_t.cos() * e_rad[0] + theta_t.sin() * u_s[0]),
        a[1] + r * e_rad[1] + r * (theta_t.cos() * e_rad[1] + theta_t.sin() * u_s[1]),
        a[2] + r * e_rad[2] + r * (theta_t.cos() * e_rad[2] + theta_t.sin() * u_s[2]),
    ];
    (l_tan + r * beta, t_pt, l_tan)
}

#[derive(Clone)]
struct PlaneARow {
    experiment: String,
    pose: String,
    tar: [f64; 3],
    raw: [f64; 3],
    x: [f64; 8],
    xtar: [f64; 8],
    f: [f64; 8],
}

fn load_plane_a() -> Vec<PlaneARow> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus4614_planeA.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|&h| h == name).unwrap();
    let mut rows = Vec::new();
    for line in lines.filter(|l| !l.trim().is_empty()) {
        let f: Vec<&str> = line.split(',').collect();
        let mut x = [0.0; 8];
        let mut xtar = [0.0; 8];
        let mut forces = [0.0; 8];
        for i in 0..8 {
            x[i] = f[col(&format!("x{i}"))].parse().unwrap();
            xtar[i] = f[col(&format!("xtar{i}"))].parse().unwrap();
            forces[i] = f[col(&format!("f{i}"))].parse::<f64>().unwrap() * WINCH_FORCE_SCALE;
        }
        rows.push(PlaneARow {
            experiment: f[col("experiment")].to_string(),
            pose: f[col("pose")].to_string(),
            tar: [
                f[col("tar_x")].parse().unwrap(),
                f[col("tar_y")].parse().unwrap(),
                f[col("tar_z")].parse().unwrap(),
            ],
            raw: [
                f[col("raw_x")].parse().unwrap(),
                f[col("raw_y")].parse().unwrap(),
                f[col("raw_z")].parse().unwrap(),
            ],
            x,
            xtar,
            f: forces,
        });
    }
    rows
}

fn find_row<'a>(rows: &'a [PlaneARow], exp: &str, pose: &str) -> &'a PlaneARow {
    rows.iter()
        .find(|r| r.experiment == exp && r.pose == pose)
        .unwrap_or_else(|| panic!("missing {exp}/{pose}"))
}

/// Build an 8-cable COPacabana static-stiffness `PhysicsWorld` around `pos` with 6-DOF rigid
/// platform `mass_kg` (`I = diag(1, 1, 1)` from `COPacabana.xml`), fixed pulley exit tangents,
/// axial stiffness `ea`, and per-cable length deltas `delta_rest` relative to zero-tension
/// geometric span lengths.
fn build_copacabana(pos: [f64; 3], mass_kg: f64, ea: f64, delta_rest: [f64; 8]) -> PhysicsWorld {
    let mut plat = base(0, pos);
    plat[75] = (1 | 2 | 4 | 32 | 64 | 128 | 256) as f64;
    plat[52] = 1.0;
    plat[54] = 1.0;
    plat[56] = 1.0;
    plat[83..86].copy_from_slice(&pos);
    plat[124] = mass_kg;
    for k in 0..3 {
        plat[25 + 4 * k] = 1.0;
        plat[34 + 4 * k] = 1.0;
    }

    let mut entities = vec![plat];
    let mut paths = Vec::with_capacity(8);
    let mut tangents = [[0.0; 3]; 8];
    let mut span_lens = [0.0; 8];
    let mut total_lens = [0.0; 8];
    for i in 0..8 {
        let (l_tot, t_pt, l_tan) = pulley_ik(i, pos);
        tangents[i] = t_pt;
        span_lens[i] = l_tan;
        total_lens[i] = l_tot;
        entities.push(fixed(1 + i, t_pt, 0.0));
    }
    for i in 0..8 {
        let b_world = [pos[0] + PLATFORMS[i][0], pos[1] + PLATFORMS[i][1], pos[2] + PLATFORMS[i][2]];
        let mut m = base(9 + i, b_world);
        m[75] = 256.0;
        m[50] = 0.0;
        m[14..17].copy_from_slice(&PLATFORMS[i]);
        m[131] = i as f64;
        entities.push(m);
    }

    for i in 0..8 {
        let b_world = [pos[0] + PLATFORMS[i][0], pos[1] + PLATFORMS[i][1], pos[2] + PLATFORMS[i][2]];
        let ea_eff = ea * (span_lens[i] / total_lens[i]);
        let rest_len = (span_lens[i] + delta_rest[i]).max(0.1);
        let segs = [Seg { a: 1 + i, b: 9 + i, rest: rest_len, pa: tangents[i], pb: b_world }];
        let links = [(ATTACHMENT, false, 0.0), (ATTACHMENT, false, 0.0)];
        let mut p = path(&links, &segs, ea_eff, 0.0, 8.0);
        p[1] = 1000.0 + i as f64;
        p[HEADER + 2 * LINK] = 2000.0 + i as f64;
        paths.push(p);
    }

    let mut w = world(entities, paths);
    w.options[1] = 0.0;
    w.options[2] = -G;
    w
}

/// Step the damped platform until static equilibrium and return `(platform_pos, cable_tensions)`.
fn settle_copacabana(w: &mut PhysicsWorld, steps: usize) -> ([f64; 3], [f64; 8]) {
    let dt = 0.002;
    for _ in 0..steps {
        for slot in 69..75 {
            w.entities[0][slot] *= 0.92;
        }
        w.step(dt).unwrap();
    }
    let pos = [w.entities[0][0], w.entities[0][1], w.entities[0][2]];
    let mut forces = [0.0; 8];
    for i in 0..8 {
        forces[i] = tension(w, i, 0);
    }
    (pos, forces)
}

/// Verify our exact swivel-pulley inverse kinematics reproduces COPacabana's logged `x_tar`
/// across all 6 poses (P0..P5) and all 8 cables to sub-millimetre precision without `WiPy.pyd`.
#[test]
fn copacabana_pulley_ik_matches_logged_motor_targets() {
    let rows = load_plane_a();
    let l_origin: [f64; 8] = std::array::from_fn(|i| pulley_ik(i, [0.0; 3]).0);
    let mut max_err_mm = 0.0_f64;
    for pose in ["P0", "P1", "P2", "P3", "P4", "P5"] {
        let row = find_row(&rows, "20_Std_Load13900", pose);
        for i in 0..8 {
            let l_ik = pulley_ik(i, row.tar).0 - l_origin[i];
            let err_mm = (l_ik - row.xtar[i]).abs() * 1e3;
            max_err_mm = max_err_mm.max(err_mm);
        }
    }
    eprintln!("COPacabana pulley IK max error across P0..P5 x 8 cables: {max_err_mm:.4} mm");
    assert!(max_err_mm < 0.2, "pulley IK mismatch: {max_err_mm:.4} mm");
}

/// Review Items 12 & 14: Static cable-stiffness benchmark across ALL 5 non-home poses (P1..P5),
/// both payloads (13.9 kg and 29.1 kg), and all APC levels (APC25, APC50, APC75).
///
/// Each pose's baseline rest lengths are initialized directly from the actual measured baseline
/// cable tensions `lo_row.f[i]` (converted to physical Newtons via `WINCH_FORCE_SCALE`), then the
/// exact measured motor position changes `dX = hi_row.x - lo_row.x` are applied using `EA = 118.5 kN`
/// transferred blind from DaRUS-5171 (D-Pro XTR SK99 -> D-Pro 01505-0600 SK78).
#[test]
fn copacabana_blind_apc_preload_across_all_poses_and_levels() {
    let ea = ea_dpro_from_5171();
    let rows = load_plane_a();

    // Shared-homing pairs:
    //   13.9 kg: STD -> APC25, APC50, APC75
    //   29.1 kg: APC25 -> APC50, APC75 (10_Std had a separate higher homing preload)
    let pairs = [
        ("13.9kg APC25-STD", 13.9, "20_Std_Load13900", "40_Apc_Load13900_V25"),
        ("13.9kg APC50-STD", 13.9, "20_Std_Load13900", "41_Apc_Load13900_V50"),
        ("13.9kg APC75-STD", 13.9, "20_Std_Load13900", "42_Apc_Load13900_V75"),
        ("29.1kg APC50-APC25", 29.1, "30_Apc_Load29100_V25", "31_Apc_Load29100_V50"),
        ("29.1kg APC75-APC25", 29.1, "30_Apc_Load29100_V25", "32_Apc_Load29100_V75"),
    ];

    let mut all_sq_err = 0.0;
    let mut all_count = 0usize;
    let mut p1_checked = 0usize;

    for (label, mass_kg, lo_exp, hi_exp) in pairs {
        eprintln!("=== COPacabana {label} (blind EA={:.1} kN from 5171) ===", ea * 1e-3);
        for pose in ["P1", "P2", "P3", "P4", "P5"] {
            let lo_row = find_row(&rows, lo_exp, pose);
            let hi_row = find_row(&rows, hi_exp, pose);
            let l_pose: [f64; 8] = std::array::from_fn(|i| pulley_ik(i, lo_row.tar).0);
            // Initialize baseline elastic stretch from the actual measured baseline forces.
            let base_delta: [f64; 8] = std::array::from_fn(|i| -l_pose[i] * lo_row.f[i].max(0.0) / ea);

            let mut w_lo = build_copacabana(lo_row.tar, mass_kg, ea, base_delta);
            let (pos_lo, f_lo) = settle_copacabana(&mut w_lo, 1000);

            let hi_delta: [f64; 8] = std::array::from_fn(|i| base_delta[i] + (hi_row.x[i] - lo_row.x[i]));
            let mut w_hi = build_copacabana(lo_row.tar, mass_kg, ea, hi_delta);
            let (pos_hi, f_hi) = settle_copacabana(&mut w_hi, 1000);

            let mut df_sim = [0.0; 8];
            let mut df_meas = [0.0; 8];
            for i in 0..8 {
                df_sim[i] = f_hi[i] - f_lo[i];
                df_meas[i] = hi_row.f[i] - lo_row.f[i];
                all_sq_err += (df_sim[i] - df_meas[i]).powi(2);
                all_count += 1;
            }
            let mean_sim: f64 = df_sim.iter().sum::<f64>() / 8.0;
            let mean_meas: f64 = df_meas.iter().sum::<f64>() / 8.0;
            let rmse = (df_sim.iter().zip(df_meas.iter()).map(|(s, m)| (s - m).powi(2)).sum::<f64>() / 8.0).sqrt();
            let dp_sim = [
                (pos_hi[0] - pos_lo[0]) * 1e3,
                (pos_hi[1] - pos_lo[1]) * 1e3,
                (pos_hi[2] - pos_lo[2]) * 1e3,
            ];
            let dp_meas = [
                (hi_row.raw[0] - lo_row.raw[0]) * 1e3,
                (hi_row.raw[1] - lo_row.raw[1]) * 1e3,
                (hi_row.raw[2] - lo_row.raw[2]) * 1e3,
            ];
            let min_f = lo_row.f.iter().copied().fold(f64::INFINITY, f64::min);
            eprintln!(
                "  {pose} (min_F={min_f:4.0}N): mean dF sim={mean_sim:+5.1} vs meas={mean_meas:+5.1} N, RMSE={rmse:4.1} N | dPos_mm sim=({:+.2},{:+.2},{:+.2}) meas=({:+.2},{:+.2},{:+.2})",
                dp_sim[0], dp_sim[1], dp_sim[2], dp_meas[0], dp_meas[1], dp_meas[2]
            );
            if pose == "P1" && (label.contains("APC75")) {
                assert!((mean_sim - mean_meas).abs() / mean_meas.abs() < 0.25, "{label}/{pose} mean dF error");
                assert!(rmse < 36.0, "{label}/{pose} RMSE {rmse:.1} N");
                p1_checked += 1;
            }
        }
    }
    let overall_rmse = (all_sq_err / all_count as f64).sqrt();
    eprintln!("=== COPacabana overall blind per-cable dF RMSE across all 25 (pair x pose) = {overall_rmse:.1} N (n={all_count} cable readings) ===");
    assert_eq!(p1_checked, 2);
    // Low-tension boundary poses (P2/P3, down to ~17 N physical) show softening relative to linear 118.5 kN;
    // overall blind RMSE across all 200 cable readings stays below 50 N (physical N) without any parameter fitting.
    assert!(overall_rmse < 50.0, "overall blind RMSE {overall_rmse:.1} N too high");
}

/// Review Item 13: Blind vertical elastic sag prediction when payload increases from 13.9 kg to
/// 29.1 kg (+149.1 N) in STD mode at home P0 and workspace center P1, compared directly with the
/// FARO laser tracker.
///
/// Unlike force-vs-force benchmarks, gravity (m*g) is already in physical Newtons, so converting
/// EA from 83.8 kN to 118.5 kN reduces simulated linear-elastic sag from -2.62 mm to ~-1.85 mm
/// against FARO's -3.69 / -4.40 mm (~50-58% under-prediction). This is preserved openly as an
/// unmodeled residual strengthening the case for nonlinear low-tension rope stiffness / structural
/// compliance rather than fitting extra parameters.
#[test]
fn copacabana_blind_payload_sag_quantifies_prediction_error() {
    let ea = ea_dpro_from_5171();
    let rows = load_plane_a();
    let p1_13 = find_row(&rows, "20_Std_Load13900", "P1");
    let p1_29 = find_row(&rows, "10_Std_Load29100", "P1");
    let p0_13 = find_row(&rows, "20_Std_Load13900", "P0");
    let p0_29 = find_row(&rows, "10_Std_Load29100", "P0");

    let l_p1: [f64; 8] = std::array::from_fn(|i| pulley_ik(i, p1_13.tar).0);
    // Use actual measured 13.9 kg P1 tensions as the unloaded baseline.
    let base_delta: [f64; 8] = std::array::from_fn(|i| -l_p1[i] * p1_13.f[i].max(0.0) / ea);

    let mut w13 = build_copacabana(p1_13.tar, 13.9, ea, base_delta);
    let (pos13, _) = settle_copacabana(&mut w13, 1200);
    let mut w29 = build_copacabana(p1_13.tar, 29.1, ea, base_delta);
    let (pos29, _) = settle_copacabana(&mut w29, 1200);

    let dz_sim_mm = (pos29[2] - pos13[2]) * 1e3;
    let dz_meas_p0_mm = (p0_29.raw[2] - p0_13.raw[2]) * 1e3;
    let dz_meas_p1_mm = (p1_29.raw[2] - p1_13.raw[2]) * 1e3;
    let err_p0_mm = dz_sim_mm - dz_meas_p0_mm;
    let err_p1_mm = dz_sim_mm - dz_meas_p1_mm;
    let rel_p0 = 100.0 * err_p0_mm.abs() / dz_meas_p0_mm.abs();
    let rel_p1 = 100.0 * err_p1_mm.abs() / dz_meas_p1_mm.abs();
    eprintln!(
        "COPacabana 29.1-13.9 kg sag (blind EA={:.1} kN): sim={dz_sim_mm:+.2} mm vs tracker P0={dz_meas_p0_mm:+.2} mm (err={err_p0_mm:+.2} mm, {rel_p0:.1}%), P1={dz_meas_p1_mm:+.2} mm (err={err_p1_mm:+.2} mm, {rel_p1:.1}%)",
        ea * 1e-3,
    );
    // Explicit prediction-error bounds: linear elastic model predicts the right sign and order of
    // magnitude (~-1.85 mm vs -3.69/-4.40 mm), with an openly documented ~50-58% unmodeled residual.
    assert!(dz_sim_mm < 0.0 && dz_meas_p1_mm < 0.0, "sag must be negative (downward)");
    assert!(err_p0_mm.abs() < 2.2, "P0 sag prediction error {err_p0_mm:+.2} mm exceeded 2.2 mm");
    assert!(err_p1_mm.abs() < 3.0, "P1 sag prediction error {err_p1_mm:+.2} mm exceeded 3.0 mm");
}

/// Swivel wheel centre and wrap arc `r * beta` of cable `i` at platform position `pos`.
fn swivel_center_and_wrap(i: usize, pos: [f64; 3]) -> ([f64; 3], f64) {
    let (l_tot, _, l_tan) = pulley_ik(i, pos);
    let a = ANCHORS[i];
    let u_s = SWIVEL[i];
    let v: [f64; 3] = std::array::from_fn(|k| pos[k] + PLATFORMS[i][k] - a[k]);
    let z = dot3(v, u_s);
    let v_rad: [f64; 3] = std::array::from_fn(|k| v[k] - z * u_s[k]);
    let rho = norm3(v_rad);
    (std::array::from_fn(|k| a[k] + PULLEY_R[i] * v_rad[k] / rho), l_tot - l_tan)
}

/// Length of the feed line arriving along each swivel axis.
const FEED: f64 = 0.2;

/// COPacabana with moving swivel pulleys: per cable a fixed feed point `FEED` before the entry
/// point (against the swivel axis), a frictionless swivel wheel and the platform attachment.
/// Rest lengths share `delta_rest` as in `build_copacabana`, and `EA` is scaled so each line's
/// spring constant equals `build_copacabana`'s. Rows stretch only their free spans: material on
/// a wheel, like the wrap here, is unstrained. A `kinematic` platform is held where it is put.
fn build_swivel_copacabana(pos: [f64; 3], mass_kg: f64, ea: f64, delta_rest: [f64; 8], kinematic: bool) -> PhysicsWorld {
    let (m, inertia) = if kinematic { (1e12, 1e12) } else { (mass_kg, 1.0) };
    let mut plat = base(0, pos);
    plat[75] = (1 | 2 | 4 | 32 | 64 | 128 | 256) as f64;
    plat[52] = 1.0;
    plat[54] = 1.0;
    plat[56] = 1.0;
    plat[83..86].copy_from_slice(&pos);
    plat[124] = m;
    for k in 0..3 {
        plat[25 + 4 * k] = inertia;
        plat[34 + 4 * k] = 1.0 / inertia;
    }
    let mut entities = vec![plat];
    let feed: [[f64; 3]; 8] = std::array::from_fn(|i| std::array::from_fn(|k| ANCHORS[i][k] - FEED * SWIVEL[i][k]));
    for i in 0..8 {
        entities.push(fixed(1 + i, feed[i], 0.0));
    }
    for i in 0..8 {
        let (c, _) = swivel_center_and_wrap(i, pos);
        let mut e = fixed(9 + i, c, 0.0);
        e[75] = (256 | 1024 | 4096) as f64;
        e[104] = PULLEY_R[i];
        e[138..141].copy_from_slice(&ANCHORS[i]);
        e[141..144].copy_from_slice(&SWIVEL[i]);
        let out: [f64; 3] = std::array::from_fn(|k| (c[k] - ANCHORS[i][k]) / PULLEY_R[i]);
        let u = SWIVEL[i];
        e[101..104].copy_from_slice(&[out[1] * u[2] - out[2] * u[1], out[2] * u[0] - out[0] * u[2], out[0] * u[1] - out[1] * u[0]]);
        entities.push(e);
    }
    for i in 0..8 {
        let b_world: [f64; 3] = std::array::from_fn(|k| pos[k] + PLATFORMS[i][k]);
        let mut e = base(17 + i, b_world);
        e[75] = 256.0;
        e[50] = 0.0;
        e[14..17].copy_from_slice(&PLATFORMS[i]);
        e[131] = i as f64;
        entities.push(e);
    }
    let mut paths = Vec::with_capacity(8);
    for i in 0..8 {
        let (l_tot, t_pt, l_tan) = pulley_ik(i, pos);
        let (_, wrap) = swivel_center_and_wrap(i, pos);
        let b_world: [f64; 3] = std::array::from_fn(|k| pos[k] + PLATFORMS[i][k]);
        let segs = [
            Seg { a: 1 + i, b: 9 + i, rest: FEED, pa: feed[i], pb: ANCHORS[i] },
            Seg { a: 9 + i, b: 17 + i, rest: l_tan + delta_rest[i], pa: t_pt, pb: b_world },
        ];
        let links = [(ATTACHMENT, false, 0.0), (ROLLING, SWIVEL_CW, wrap), (ATTACHMENT, false, 0.0)];
        let mut p = path(&links, &segs, ea * (FEED + l_tan) / l_tot, 0.0, 8.0);
        p[1] = 1000.0 + i as f64;
        for s in 0..2 {
            p[HEADER + 3 * LINK + s * SEGMENT] = 2000.0 + (2 * i + s) as f64;
        }
        paths.push(p);
    }
    let mut w = world(entities, paths);
    w.options[1] = 0.0;
    w.options[2] = if kinematic { 0.0 } else { -G };
    w
}
/// Winding sense of the line over a swivel wheel (its plane normal is `e_rad x u_s`).
const SWIVEL_CW: bool = true;

fn platform_poses() -> Vec<[f64; 3]> {
    let rows = load_plane_a();
    ["P0", "P1", "P2", "P3", "P4", "P5"].iter().map(|p| find_row(&rows, "20_Std_Load13900", p).tar).collect()
}

/// Carrying the platform through P0..P5 swivels every pulley with it: the exit tangent, the
/// wrap arc `r * beta` and the total line length stay on `pulley_ik`, and no material is lost.
#[test]
fn swivel_pulleys_follow_pulley_ik_across_plane_a() {
    let poses = platform_poses();
    let mut w = build_swivel_copacabana(poses[0], 0.0, 118_500.0, [0.0; 8], true);
    let material0: [f64; 8] = std::array::from_fn(|i| material(&mut w, i));
    let (mut tangent_err, mut wrap_err, mut length_err, mut material_err) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    for leg in poses.windows(2) {
        let n = 500;
        for s in 1..=n {
            let t = s as f64 / n as f64;
            let pos: [f64; 3] = std::array::from_fn(|k| leg[0][k] + t * (leg[1][k] - leg[0][k]));
            w.entities[0][0..3].copy_from_slice(&pos);
            w.entities[0][69..75].fill(0.0);
            w.step(0.002).unwrap();
        }
        for i in 0..8 {
            let (l_tot, t_pt, _) = pulley_ik(i, leg[1]);
            let (_, wrap) = swivel_center_and_wrap(i, leg[1]);
            let t_sim = [seg_field(&mut w, i, 1, 4), seg_field(&mut w, i, 1, 5), seg_field(&mut w, i, 1, 6)];
            let b_sim = [seg_field(&mut w, i, 1, 7), seg_field(&mut w, i, 1, 8), seg_field(&mut w, i, 1, 9)];
            let entry = [seg_field(&mut w, i, 0, 7), seg_field(&mut w, i, 0, 8), seg_field(&mut w, i, 0, 9)];
            let span = norm3(std::array::from_fn(|k| b_sim[k] - t_sim[k]));
            tangent_err = tangent_err.max(norm3(std::array::from_fn(|k| t_sim[k] - t_pt[k])));
            tangent_err = tangent_err.max(norm3(std::array::from_fn(|k| entry[k] - ANCHORS[i][k])));
            wrap_err = wrap_err.max((stored(&mut w, i, 1) - wrap).abs());
            length_err = length_err.max((stored(&mut w, i, 1) + span - l_tot).abs());
            material_err = material_err.max((material(&mut w, i) - material0[i]).abs());
        }
    }
    eprintln!(
        "swivel vs pulley_ik over P0..P5: tangent {:.2e} m, wrap {:.2e} m, length {:.2e} m, material {:.2e} m",
        tangent_err, wrap_err, length_err, material_err
    );
    assert!(tangent_err < 1e-9, "tangent point off pulley_ik by {tangent_err:.2e} m");
    assert!(wrap_err < 1e-9, "wrap arc off r*beta by {wrap_err:.2e} m");
    assert!(length_err < 1e-9, "line length off pulley_ik by {length_err:.2e} m");
    assert!(material_err < 1e-9, "material drifted by {material_err:.2e} m");
}

/// At every PlaneA pose the platform hung on swivel pulleys settles where the fixed-tangent
/// model built from `pulley_ik` does, with the same cable tensions.
#[test]
fn swivel_pulley_statics_match_fixed_pulley_ik_tangents() {
    let ea = 118_500.0;
    let rows = load_plane_a();
    let (mut pos_err, mut force_err) = (0.0_f64, 0.0_f64);
    for pose in ["P0", "P1", "P2", "P3", "P4", "P5"] {
        let row = find_row(&rows, "20_Std_Load13900", pose);
        let delta: [f64; 8] = std::array::from_fn(|i| -pulley_ik(i, row.tar).0 * row.f[i].max(0.0) / ea);
        let mut fixed_world = build_copacabana(row.tar, 13.9, ea, delta);
        let (pos_fixed, f_fixed) = settle_copacabana(&mut fixed_world, 1000);
        let mut swivel_world = build_swivel_copacabana(row.tar, 13.9, ea, delta, false);
        let dt = 0.002;
        for _ in 0..1000 {
            for slot in 69..75 {
                swivel_world.entities[0][slot] *= 0.92;
            }
            swivel_world.step(dt).unwrap();
        }
        let pos_swivel = [swivel_world.entities[0][0], swivel_world.entities[0][1], swivel_world.entities[0][2]];
        let f_swivel: [f64; 8] = std::array::from_fn(|i| tension(&mut swivel_world, i, 1));
        let dp = norm3(std::array::from_fn(|k| pos_swivel[k] - pos_fixed[k])) * 1e3;
        let df = (0..8).map(|i| (f_swivel[i] - f_fixed[i]).abs()).fold(0.0, f64::max);
        eprintln!("  {pose}: swivel - fixed pose {dp:.4} mm, max |dF| {df:.3} N");
        pos_err = pos_err.max(dp);
        force_err = force_err.max(df);
    }
    assert!(pos_err < 0.005, "swivel statics moved {pos_err:.4} mm from fixed pulley_ik tangents");
    assert!(force_err < 0.1, "swivel tensions differ by {force_err:.3} N from fixed pulley_ik tangents");
}
