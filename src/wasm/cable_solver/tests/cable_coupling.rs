//! Coupled friction / transport scenarios: stick-slip transitions, reversals,
//! changing wrap geometry, capstan chains and wheel inertia, each against its
//! closed-form reference.
mod common;
use common::*;

const EA: f64 = 1e5;

fn pos(w: &PhysicsWorld, id: usize) -> [f64; 3] {
    w.entities[id][0..3].try_into().unwrap()
}
fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
fn place(w: &mut PhysicsWorld, id: usize, p: [f64; 3]) {
    w.entities[id][0..3].copy_from_slice(&p);
}
/// Tension of span `i` between point entities `a` and `b` from its strain.
fn strain_tension(w: &mut PhysicsWorld, i: usize, a: usize, b: usize, ea: f64) -> f64 {
    let d = dist(pos(w, a), pos(w, b));
    ea * (d - rest(w, 0, i)).max(0.0) / d
}
/// How far outside the capstan cone two tensions are (0 inside).
fn cone_violation(a: f64, b: f64, alpha: f64) -> f64 {
    let (hi, lo) = (a.max(b), a.min(b));
    if hi < 1e-6 {
        0.0
    } else {
        (hi / (alpha * lo.max(1e-12)) - 1.0).max(0.0)
    }
}

/// Fixed end - 90 degree pinhole - driven end, loaded then unloaded.
/// Returns (worst cone violation, loading ratio at peak, fixed-side tension at
/// peak, fixed-side tension after a small unload, final unloading ratio).
fn hysteresis(mu: f64, steps: usize) -> (f64, f64, f64, f64, f64) {
    let es = vec![fixed(0, [-1.0, 0.0, 0.0], 0.0), fixed(1, [0.0; 3], mu), fixed(2, [0.0, -1.0, 0.0], 0.0)];
    let segs = [
        Seg { a: 0, b: 1, rest: 1.0, pa: [-1.0, 0.0, 0.0], pb: [0.0; 3] },
        Seg { a: 1, b: 2, rest: 1.0, pa: [0.0; 3], pb: [0.0, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
    let mut w = world(es, vec![path(&links, &segs, EA, 0.0, 4.0)]);
    let alpha = (mu * PI / 2.0).exp();
    let peak = 2e-3;
    let mut worst: f64 = 0.0;
    let (mut loading, mut fixed_peak, mut fixed_latched, mut unloading) = (0.0, 0.0, 0.0, 0.0);
    for k in 1..=2 * steps {
        let u = if k <= steps { k } else { 2 * steps - k } as f64 / steps as f64;
        place(&mut w, 2, [0.0, -1.0 - peak * u, 0.0]);
        w.step(0.001);
        let fix = strain_tension(&mut w, 0, 0, 1, EA);
        let drv = strain_tension(&mut w, 1, 1, 2, EA);
        worst = worst.max(cone_violation(fix, drv, alpha));
        if k == steps {
            loading = drv / fix;
            fixed_peak = fix;
        }
        if k == steps + steps / 20 {
            fixed_latched = fix;
        }
        if k == 2 * steps - steps / 4 {
            unloading = fix / drv;
        }
    }
    (worst, loading, fixed_peak, fixed_latched, unloading)
}

#[test]
fn capstan_hysteresis_follows_both_branches() {
    for mu in [0.0, 0.2, 0.6] {
        let alpha = (mu * PI / 2.0).exp();
        for steps in [2000, 50] {
            let (worst, loading, peak, latched, unloading) = hysteresis(mu, steps);
            eprintln!(
                "hysteresis mu {mu} {steps} steps/branch: loading {loading:.5} unloading {unloading:.5} cone {alpha:.5}, latch {:.2e}, violation {worst:.2e}",
                latched / peak - 1.0
            );
            assert!((loading / alpha - 1.0).abs() < 1e-3);
            assert!((unloading / alpha - 1.0).abs() < 1e-3);
            assert!(worst < 1e-3, "cone violated by {worst}");
            if mu > 0.0 {
                // Just after the reversal the pinhole holds its material.
                assert!((latched / peak - 1.0).abs() < 1e-6);
            }
        }
    }
}

/// Hairpin Atwood at production step and stiffness: the stick/slip decision
/// must not be biased by the unconstrained prediction of a step.
#[test]
fn capstan_threshold_is_unbiased_at_coarse_steps() {
    let mu: f64 = 0.3;
    let alpha = (mu * PI).exp();
    for r in [0.9, 0.97, 1.03, 1.1, 1.5] {
        let (m1, m2) = (r * alpha, 1.0);
        let es = vec![fixed(0, [0.0; 3], mu), mass(1, [-1e-9, -1.0, 0.0], m1), mass(2, [1e-9, -1.0, 0.0], m2)];
        let segs = [
            Seg { a: 1, b: 0, rest: 1.0, pa: [-1e-9, -1.0, 0.0], pb: [0.0; 3] },
            Seg { a: 0, b: 2, rest: 1.0, pa: [0.0; 3], pb: [1e-9, -1.0, 0.0] },
        ];
        let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
        let mut w = world(es, vec![path(&links, &segs, 1e6, 2e3, 10.0)]);
        let dt = 0.002;
        for _ in 0..250 {
            w.step(dt);
        }
        let (v0, y0) = (vy(&w, 1), y(&w, 1));
        for _ in 0..250 {
            w.step(dt);
        }
        let a = -(vy(&w, 1) - v0) / (250.0 * dt);
        let expected = (G * (m1 - m2 * alpha) / (m1 + m2 * alpha)).max(0.0);
        eprintln!("threshold x{r}: a {a:.4} expected {expected:.4}, moved {:.5}", y0 - y(&w, 1));
        assert!((a - expected).abs() < 0.01 * G, "a {a} vs {expected}");
    }
}

/// Two equal masses sliding over a frictional hairpin decelerate at the
/// kinetic capstan rate, stop, and stay stopped. Friction only dissipates.
#[test]
fn sliding_hairpin_stops_and_dissipates() {
    let mu: f64 = 0.3;
    let alpha = (mu * PI).exp();
    let v = 1.0;
    for dt in [0.0005, 0.002] {
        let mut left = mass(1, [-1e-9, -1.0, 0.0], 1.0);
        let mut right = mass(2, [1e-9, -1.0, 0.0], 1.0);
        left[70] = -v;
        right[70] = v;
        let es = vec![fixed(0, [0.0; 3], mu), left, right];
        let segs = [
            Seg { a: 1, b: 0, rest: 1.0, pa: [-1e-9, -1.0, 0.0], pb: [0.0; 3] },
            Seg { a: 0, b: 2, rest: 1.0, pa: [0.0; 3], pb: [1e-9, -1.0, 0.0] },
        ];
        let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
        let ea = 1e6;
        let mut w = world(es, vec![path(&links, &segs, ea, 0.0, 10.0)]);
        // The left mass descends, so the right span is the tight one.
        let decel = G * (alpha - 1.0) / (alpha + 1.0);
        let travel = v * v / (2.0 * decel);
        let energy = |w: &mut PhysicsWorld| {
            let mut e = kinetic_plus_potential(w, &[(1, 1.0), (2, 1.0)]);
            for (i, id) in [(0, 1), (1, 2)] {
                let d = dist(pos(w, id), [0.0; 3]);
                let stretch = (d - rest(w, 0, i)).max(0.0);
                e += 0.5 * ea / d * stretch * stretch;
            }
            e
        };
        let mut previous = energy(&mut w);
        let mut gain: f64 = 0.0;
        for _ in 0..(1.5 / dt) as usize {
            w.step(dt);
            let e = energy(&mut w);
            gain = gain.max(e - previous);
            previous = previous.min(e);
        }
        let moved = -1.0 - y(&w, 1);
        eprintln!(
            "sliding stop dt {dt}: travel {moved:.5} expected {travel:.5}, final v {:.2e}, max energy gain {gain:.3e} J",
            vy(&w, 1)
        );
        assert!((moved - travel).abs() < 0.02 * travel, "travel {moved} vs {travel}");
        assert!(vy(&w, 1).abs() < 1e-3);
        assert!(gain < 1e-4, "friction added {gain} J");
    }
}

fn kinetic_plus_potential(w: &PhysicsWorld, ids: &[(usize, f64)]) -> f64 {
    ids.iter()
        .map(|&(id, m)| {
            let e = &w.entities[id];
            0.5 * m * (e[69] * e[69] + e[70] * e[70] + e[71] * e[71]) + m * G * e[1]
        })
        .sum()
}

/// A loaded pinhole whose wrap angle swings from a hairpin to straight: the
/// narrowing capstan cone must release the held tension difference as it goes.
#[test]
fn narrowing_wrap_releases_tension() {
    let mu = 0.4;
    for swing in [400, 20] {
        let es = vec![fixed(0, [-1.0, 0.0, 0.0], 0.0), fixed(1, [0.0; 3], mu), fixed(2, [-1.0, 1e-6, 0.0], 0.0)];
        let segs = [
            Seg { a: 0, b: 1, rest: 1.0, pa: [-1.0, 0.0, 0.0], pb: [0.0; 3] },
            Seg { a: 1, b: 2, rest: 1.0, pa: [0.0; 3], pb: [-1.0, 1e-6, 0.0] },
        ];
        let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
        let mut w = world(es, vec![path(&links, &segs, EA, 0.0, 4.0)]);
        let radius = 1.002;
        // Load at a full hairpin.
        for k in 1..=200 {
            let r = 1.0 + (radius - 1.0) * k as f64 / 200.0;
            place(&mut w, 2, [-r, 1e-6, 0.0]);
            w.step(0.001);
        }
        let mut worst: f64 = 0.0;
        for k in 1..=swing {
            // Wrap angle from pi down to zero.
            let wrap = PI * (1.0 - k as f64 / swing as f64);
            place(&mut w, 2, [radius * wrap.cos(), radius * wrap.sin(), 0.0]);
            w.step(0.001);
            let fix = strain_tension(&mut w, 0, 0, 1, EA);
            let drv = strain_tension(&mut w, 1, 1, 2, EA);
            worst = worst.max(cone_violation(fix, drv, (mu * wrap).exp()));
        }
        let fix = strain_tension(&mut w, 0, 0, 1, EA);
        let drv = strain_tension(&mut w, 1, 1, 2, EA);
        eprintln!("narrowing wrap over {swing} steps: worst cone violation {worst:.3e}, final T {fix:.3}/{drv:.3}");
        assert!(worst < 1e-3, "cone violated by {worst}");
        assert!((fix / drv - 1.0).abs() < 1e-3);
    }
}

/// Three 90 degree pinholes in series between a fixed and a driven end.
#[test]
fn capstan_chain_loads_and_unloads() {
    let mu = 0.3;
    let alpha = (mu * PI / 2.0).exp();
    let pts = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, -1.0, 0.0], [2.0, -1.0, 0.0], [2.0, -2.0, 0.0]];
    for steps in [2000, 50] {
        let es: Vec<Entity> = (0..5).map(|i| fixed(i, pts[i], if (1..4).contains(&i) { mu } else { 0.0 })).collect();
        let segs: Vec<Seg> = (0..4).map(|i| Seg { a: i, b: i + 1, rest: 1.0, pa: pts[i], pb: pts[i + 1] }).collect();
        let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (PINHOLE, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
        let mut w = world(es, vec![path(&links, &segs, EA, 0.0, 4.0)]);
        let before = material(&mut w, 0);
        let mut worst: f64 = 0.0;
        let (mut loading, mut unloading) = (0.0, 0.0);
        for k in 1..=2 * steps {
            let u = if k <= steps { k } else { 2 * steps - k } as f64 / steps as f64;
            place(&mut w, 4, [2.0, -2.0 - 4e-3 * u, 0.0]);
            w.step(0.001);
            let t: Vec<f64> = (0..4).map(|i| strain_tension(&mut w, i, i, i + 1, EA)).collect();
            for i in 0..3 {
                worst = worst.max(cone_violation(t[i], t[i + 1], alpha));
            }
            if k == steps {
                loading = t[3] / t[0];
            }
            if k == 2 * steps - steps / 10 {
                unloading = t[0] / t[3];
            }
        }
        let drift = material(&mut w, 0) - before;
        eprintln!(
            "capstan chain {steps} steps/branch: loading {loading:.5} unloading {unloading:.5} expected {:.5}, violation {worst:.2e}, drift {drift:.1e}",
            alpha.powi(3)
        );
        assert!((loading / alpha.powi(3) - 1.0).abs() < 1e-3);
        assert!((unloading / alpha.powi(3) - 1.0).abs() < 1e-3);
        assert!(worst < 1e-3, "cone violated by {worst}");
        assert!(drift.abs() < 1e-9);
    }
}

/// A cable slides through a pinhole on a free body. The row's
/// tension-weighted forces are internal: linear momentum is conserved exactly
/// and angular momentum to the first order of the position projection, with
/// friction no worse than without. Friction only removes energy.
fn free_pinhole_body(mu: f64, dt: f64) -> ([f64; 2], f64, f64, f64) {
    let masses = [1.0, 1.5, 2.0];
    let mut es = vec![mass(0, [-1.0, 0.0, 0.0], masses[0]), mass(1, [0.0; 3], masses[1]), mass(2, [0.0, -1.0, 0.0], masses[2])];
    es[1][105] = mu;
    es[0][69] = -1.0;
    es[0][70] = 0.3;
    es[2][70] = -0.5;
    let segs = [
        Seg { a: 0, b: 1, rest: 1.0, pa: [-1.0, 0.0, 0.0], pb: [0.0; 3] },
        Seg { a: 1, b: 2, rest: 1.0, pa: [0.0; 3], pb: [0.0, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
    let ea = 1e5;
    let mut w = world(es, vec![path(&links, &segs, ea, 0.0, 10.0)]);
    w.options[1] = 0.0;
    let momentum = |w: &PhysicsWorld| {
        let mut p = [0.0; 3];
        for (id, m) in masses.iter().enumerate() {
            let e = &w.entities[id];
            p[0] += m * e[69];
            p[1] += m * e[70];
            p[2] += m * (e[0] * e[70] - e[1] * e[69]);
        }
        p
    };
    let energy = |w: &mut PhysicsWorld| {
        let mut e = 0.0;
        for (id, m) in masses.iter().enumerate() {
            let b = &w.entities[id];
            e += 0.5 * m * (b[69] * b[69] + b[70] * b[70]);
        }
        for i in 0..2 {
            let d = dist(pos(w, i), pos(w, i + 1));
            let stretch = (d - rest(w, 0, i)).max(0.0);
            e += 0.5 * ea / d * stretch * stretch;
        }
        e
    };
    let (p0, e0) = (momentum(&w), energy(&mut w));
    let before = rest(&mut w, 0, 0);
    let (mut drift, mut gain, mut previous) = ([0.0f64; 2], 0.0f64, e0);
    for _ in 0..(2.0 / dt) as usize {
        w.step(dt);
        let p = momentum(&w);
        drift[0] = drift[0].max((p[0] - p0[0]).abs()).max((p[1] - p0[1]).abs());
        drift[1] = drift[1].max((p[2] - p0[2]).abs());
        let e = energy(&mut w);
        gain = gain.max(e - previous);
        previous = previous.min(e);
    }
    (drift, gain, e0 - previous, (rest(&mut w, 0, 0) - before).abs())
}

#[test]
fn slipping_row_on_a_free_body_conserves_momentum() {
    let (smooth, ..) = free_pinhole_body(0.0, 0.001);
    for dt in [0.001, 0.00025] {
        let (drift, gain, lost, slid) = free_pinhole_body(0.5, dt);
        eprintln!(
            "free pinhole body dt {dt}: linear drift {:.2e}, angular drift {:.2e} (frictionless at 1 ms {:.2e}), lost {lost:.4} J, largest gain {gain:.2e} J, slid {slid:.4} m",
            drift[0], drift[1], smooth[1]
        );
        assert!(slid > 0.01);
        assert!(drift[0] < 1e-9);
        assert!(drift[1] < 2.0 * smooth[1] + 1e-12);
        assert!(gain < 1e-9 && lost > 0.0);
    }
}

/// Slack stays in the span it is in. It crosses a guide only when the other
/// span is pulled taut, and then only as much as that span is short of.
#[test]
fn slack_stays_until_pulled() {
    for mu in [0.0, 0.5] {
        let es = vec![fixed(0, [-1.0, 0.0, 0.0], 0.0), fixed(1, [0.0; 3], mu), fixed(2, [0.0, -1.0, 0.0], 0.0)];
        let segs = [
            Seg { a: 0, b: 1, rest: 1.3, pa: [-1.0, 0.0, 0.0], pb: [0.0; 3] },
            Seg { a: 1, b: 2, rest: 1.1, pa: [0.0; 3], pb: [0.0, -1.0, 0.0] },
        ];
        let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
        let mut w = world(es, vec![path(&links, &segs, EA, 0.0, 4.0)]);
        // The right end moves away while its span still has slack.
        for k in 1..=100 {
            place(&mut w, 2, [0.0, -1.0 - 0.05 * k as f64 / 100.0, 0.0]);
            w.step(0.001);
        }
        let held = (rest(&mut w, 0, 0), rest(&mut w, 0, 1));
        // Then 0.1 m beyond its own material.
        for k in 1..=100 {
            place(&mut w, 2, [0.0, -1.05 - 0.15 * k as f64 / 100.0, 0.0]);
            w.step(0.001);
        }
        let pulled = (rest(&mut w, 0, 0), rest(&mut w, 0, 1));
        let force = tension(&mut w, 0, 1);
        eprintln!("slack mu {mu}: held {held:?}, pulled {pulled:?}, tension {force:.2e}");
        assert_eq!(held, (1.3, 1.1));
        assert!((pulled.0 - 1.2).abs() < 1e-9 && (pulled.1 - 1.2).abs() < 1e-9);
        assert!(force < 1e-6);
    }
}

/// A row longer than any fixed buffer reports every segment's own tension
/// direction.
#[test]
fn long_rows_report_every_segment() {
    let n = 24;
    let mut es: Vec<Entity> = (0..n).map(|i| fixed(i, [i as f64 * 0.1, if i % 2 == 0 { 0.0 } else { 0.05 }, 0.0], 0.0)).collect();
    es.push(mass(n, [(n - 1) as f64 * 0.1, -0.5, 0.0], 1.0));
    let pts: Vec<[f64; 3]> = es.iter().map(|e| e[0..3].try_into().unwrap()).collect();
    let segs: Vec<Seg> = (0..n).map(|i| Seg { a: i, b: i + 1, rest: dist(pts[i], pts[i + 1]), pa: pts[i], pb: pts[i + 1] }).collect();
    let mut links = vec![(PINHOLE, true, 0.0); n + 1];
    links[0] = (ATTACHMENT, true, 0.0);
    links[n] = (ATTACHMENT, true, 0.0);
    let mut w = world(es, vec![path(&links, &segs, 1e6, 1e3, 4.0)]);
    for _ in 0..2000 {
        w.step(0.001);
    }
    for i in 0..n {
        let force: Vec<f64> = (13..16).map(|slot| seg_field(&mut w, 0, i, slot)).collect();
        let (a, b) = (pos(&w, i), pos(&w, i + 1));
        let d = dist(a, b);
        let along: f64 = (0..3).map(|k| force[k] * (b[k] - a[k]) / d).sum();
        let magnitude = tension(&mut w, 0, i);
        assert!((magnitude - G).abs() < 1e-3 * G, "segment {i} tension {magnitude}");
        assert!((along.abs() - magnitude).abs() < 1e-9 * magnitude, "segment {i} force {force:?} is off its span");
    }
}

/// Atwood machine over a wheel with a worn bearing: friction torque `bearing`,
/// rim-equivalent mass `rim`, line friction `mu` over half a turn.
/// Returns (acceleration of the heavy side, rim travel over line travel).
fn worn_wheel_atwood(mu: f64, rim: f64, bearing: f64) -> (f64, f64) {
    let (m1, m2, r) = (2.0, 1.0, 0.05);
    let es = vec![
        wheel(0, [0.0; 3], r, mu, rim * r * r, Some(bearing)),
        mass(1, [-r, -1.0, 0.0], m1),
        mass(2, [r, -1.0, 0.0], m2),
    ];
    let segs = [
        Seg { a: 1, b: 0, rest: 1.0, pa: [-r, -1.0, 0.0], pb: [-r, 0.0, 0.0] },
        Seg { a: 0, b: 2, rest: 1.0, pa: [r, 0.0, 0.0], pb: [r, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (ROLLING, true, PI * r), (ATTACHMENT, true, 0.0)];
    let mut w = world(es, vec![path(&links, &segs, 1e6, 1e3, 10.0)]);
    let before = material(&mut w, 0);
    let dt = 0.0005;
    for _ in 0..400 {
        w.step(dt);
    }
    let (v0, y0, angle0) = (vy(&w, 1), y(&w, 1), w.entities[0][66]);
    for _ in 0..400 {
        w.step(dt);
    }
    assert!((material(&mut w, 0) - before).abs() < 1e-9);
    let travel = y0 - y(&w, 1);
    let turned = (w.entities[0][66] - angle0).abs() * r;
    (-(vy(&w, 1) - v0) / (400.0 * dt), if travel.abs() > 1e-6 { turned / travel } else { turned })
}

#[test]
fn worn_bearing_resists_the_line() {
    let (m1, m2, r): (f64, f64, f64) = (2.0, 1.0, 0.05);
    // (line friction, rim mass, bearing torque, expected acceleration, rim follows)
    for (mu, rim, bearing, expected, follows) in [
        // Bearing drag as a force at the rim.
        (0.5, 1e-4, 0.1, (G * (m1 - m2) - 0.1 / r) / (m1 + m2), true),
        // Too little load to turn the bearing or to slide over the wheel.
        (0.5, 1e-4, 1.0, 0.0, false),
        // Seized: the line slides over the wheel at the capstan ratio.
        (0.1, 1e-4, 1.0, G * (m1 - m2 * (0.1 * PI).exp()) / (m1 + m2 * (0.1 * PI).exp()), false),
        // Inertia alone.
        (0.5, 0.5, 0.0, G * (m1 - m2) / (m1 + m2 + 0.5), true),
        // No grip: the wheel is left behind.
        (0.0, 0.5, 0.1, G * (m1 - m2) / (m1 + m2), false),
    ] {
        let (a, turn) = worn_wheel_atwood(mu, rim, bearing);
        eprintln!("worn wheel mu {mu} rim {rim} kg bearing {bearing} Nm: a {a:.4} expected {expected:.4}, rim/line travel {turn:.4}");
        assert!((a - expected).abs() < 0.01 * G, "a {a} vs {expected}");
        assert!(if follows { (turn - 1.0).abs() < 0.01 } else { turn < 0.01 }, "rim/line {turn}");
    }
}
