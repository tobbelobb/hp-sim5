//! The DaRUS-3600 cable actuator test rig (CATi) as a headless scene: rope runs off a test
//! winch drum onto a load winch drum at constant tension and speed. The observable is the one
//! the rig's laser surface velocimeter (LSV) gives: rope travel in the free span per drum
//! travel, as a function of tension. Data: `tests/fixtures/reference_data/darus3600_runs.csv`
//! (one row per measured 10 m pass, see `scripts/reference_data/summarize_3600.py`).
mod common;
use common::*;

const SPOOL_END: f64 = 3.0;
/// Rig drum radius (nominal drum diameter 100.36 mm) and the rig's middle speed.
const RADIUS: f64 = 0.050;
const SPEED: f64 = 2.0;

/// Rope travel in the span per drum travel over a 10 m pass at `force`.
///
/// The drums are flywheels whose angular velocity is reset every step, as a perfect speed
/// servo. Span rope moves at the rest-length flow off the test drum times (1 + strain).
fn rope_per_drum(force: f64, ea: f64) -> f64 {
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
    let dt = 0.001;
    let step = |w: &mut PhysicsWorld| {
        for id in 0..2 {
            w.entities[id][72..75].copy_from_slice(&[0.0, 0.0, -SPEED / RADIUS]);
        }
        w.step(dt).unwrap();
    };
    for _ in 0..100 {
        step(&mut w);
    }
    let (stored0, angle0) = (stored(&mut w, 0, 0), w.entities[0][66]);
    let (mut rope, mut tensions) = (0.0, 0.0);
    let steps = (10.0 / SPEED / dt) as usize;
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
    assert!((stored0 - stored(&mut w, 0, 0) - drum).abs() < 1e-6 * drum, "rest flow per drum arc");
    rope / drum
}

/// Simulated ratio slope over the rig's force range, in ppm of drum travel per N.
fn simulated_slope(ea: f64) -> f64 {
    let forces = [70.0, 125.0, 180.0];
    let ratios: Vec<f64> = forces.iter().map(|&f| rope_per_drum(f, ea)).collect();
    fit(&forces, &ratios) * 1e6
}

fn fit(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let (mx, my) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
    let sxy: f64 = x.iter().zip(y).map(|(a, b)| (a - mx) * (b - my)).sum();
    let sxx: f64 = x.iter().map(|a| (a - mx) * (a - mx)).sum();
    sxy / sxx
}

/// Measured LSV/drum slope in ppm/N: regression on force within each control × direction ×
/// speed cell (cell means removed), pooled over all runs. Also returns the per-cell slopes.
fn measured_slope() -> (f64, Vec<f64>) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../tests/fixtures/reference_data/darus3600_runs.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|&h| h == name).unwrap();
    let (control, direction, speed, force, lsv) =
        (col("control"), col("direction"), col("speed"), col("force_test"), col("lsv_ratio"));
    let mut cells: std::collections::BTreeMap<String, (Vec<f64>, Vec<f64>)> = Default::default();
    for line in lines {
        let f: Vec<&str> = line.split(',').collect();
        let cell = cells.entry(format!("{} {} {}", f[control], f[direction], f[speed])).or_default();
        cell.0.push(f[force].parse().unwrap());
        cell.1.push(f[lsv].parse().unwrap());
    }
    let (mut sxy, mut sxx, mut slopes) = (0.0, 0.0, vec![]);
    for (x, y) in cells.values() {
        let n = x.len() as f64;
        let (mx, my) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
        sxy += x.iter().zip(y).map(|(a, b)| (a - mx) * (b - my)).sum::<f64>();
        sxx += x.iter().map(|a| (a - mx) * (a - mx)).sum::<f64>();
        slopes.push(fit(x, y) * 1e6);
    }
    (sxy / sxx * 1e6, slopes)
}

/// hp-sim5 stores spooled rope as rest length, so at constant tension span rope outruns the
/// drum surface by the strain: the slope is +1e6/EA ppm/N.
#[test]
fn spool_pays_out_rest_length() {
    for ea in [1e5, 3e5] {
        let slope = simulated_slope(ea);
        eprintln!("EA {ea:.0e} N: simulated slope {slope:+.3} ppm/N, 1e6/EA {:+.3}", 1e6 / ea);
        assert!((slope - 1e6 / ea).abs() < 0.02 * 1e6 / ea);
    }
}

#[test]
fn rig_data_slope_is_negative_in_every_cell() {
    let (pooled, cells) = measured_slope();
    eprintln!("measured slope {pooled:+.2} ppm/N, cells {cells:.1?}");
    assert_eq!(cells.len(), 12);
    assert!(cells.iter().all(|&s| (-12.0..-5.0).contains(&s)));
}

/// The real rope runs slower than the drum as tension rises (running radius shrinks), the
/// opposite of the simulated strain effect. Any EA gives a positive simulated slope, so this
/// fails until the spool–rope interface models a tension-dependent running radius or strained
/// storage. EA 3e5 N is a placeholder for the 6 mm LIROS D-Pro until it is identified.
#[test]
#[ignore = "known model gap: hp-sim5 has no tension-dependent running radius (DaRUS-3600 Stage 1)"]
fn rig_matches_measured_running_radius() {
    let (measured, _) = measured_slope();
    let simulated = simulated_slope(3e5);
    eprintln!("simulated {simulated:+.2} ppm/N, measured {measured:+.2} ppm/N");
    assert!((simulated - measured).abs() < 2.0, "simulated {simulated:+.2} vs measured {measured:+.2} ppm/N");
}
