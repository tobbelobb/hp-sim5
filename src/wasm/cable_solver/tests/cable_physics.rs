//! Analytic cable physics scenarios. Each scenario has a closed-form reference
//! (statics, capstan law, Atwood dynamics, energy), so the solver is checked
//! against physics rather than against its own previous output.
use hp_sim5_cable_solver::{
    engine::{PhysicsWorld, HEADER, LINK, SEGMENT},
    Entity,
};
use std::f64::consts::PI;

const G: f64 = 9.81;
const ATTACHMENT: f64 = 0.0;
const ROLLING: f64 = 1.0;
const PINHOLE: f64 = 2.0;

fn base(id: usize, p: [f64; 3]) -> Entity {
    let mut e = [0.0; 160];
    e[0..3].copy_from_slice(&p);
    e[7..10].copy_from_slice(&p);
    for i in [6, 13, 20, 24, 82, 89, 96, 100] {
        e[i] = 1.0;
    }
    e[50] = -1.0;
    e[51] = 1.0;
    e[53] = 1.0;
    e[104] = f64::NAN;
    e[106] = f64::NAN;
    e[115..121].fill(f64::NAN);
    e[127] = f64::NAN;
    e[128] = id as f64;
    e[101..104].copy_from_slice(&[0.0, 0.0, 1.0]);
    e[90..93].copy_from_slice(&p);
    e[93..97].copy_from_slice(&[0.0, 0.0, 0.0, 1.0]);
    e[97..101].copy_from_slice(&[0.0, 0.0, 0.0, 1.0]);
    e
}
/// A fixed point the cable is tied to or threaded through.
fn fixed(id: usize, p: [f64; 3], mu: f64) -> Entity {
    let mut e = base(id, p);
    e[75] = 256.0;
    e[105] = mu;
    e
}
/// A point mass under gravity.
fn mass(id: usize, p: [f64; 3], m: f64) -> Entity {
    let mut e = base(id, p);
    e[75] = (1 | 4 | 128 | 256) as f64;
    e[124] = m;
    e
}
/// A pulley in the xy plane. `inertia` 0 makes it a fixed (non-spinning) roller.
fn pulley(id: usize, p: [f64; 3], r: f64, mu: f64, inertia: f64) -> Entity {
    let mut e = base(id, p);
    e[75] = (256 | 1024) as f64;
    e[104] = r;
    e[105] = mu;
    if inertia > 0.0 {
        e[52] = 1.0;
        e[54] = 1.0;
        e[75] = (2 | 64 | 256 | 512 | 1024) as f64;
        e[44..47].copy_from_slice(&[0.0, 0.0, 1.0]);
        for k in 0..3 {
            e[25 + 4 * k] = inertia;
            e[34 + 4 * k] = 1.0 / inertia;
        }
        e[56] = 1.0;
    }
    e
}

struct Seg {
    a: usize,
    b: usize,
    rest: f64,
    pa: [f64; 3],
    pb: [f64; 3],
}
/// `links` are (kind, cw, stored) for each of the `segs.len() + 1` links.
fn path(links: &[(f64, bool, f64)], segs: &[Seg], stiffness: f64, damping: f64, iterations: f64) -> Vec<f64> {
    let n = segs.len();
    assert_eq!(links.len(), n + 1);
    let mut d = vec![0.0; HEADER + (n + 1) * LINK + n * SEGMENT];
    let total: f64 = segs.iter().map(|s| s.rest).sum::<f64>() + links.iter().map(|l| l.2).sum::<f64>();
    d[..16].copy_from_slice(&[
        n as f64, 1000.0, stiffness, 1.0 / stiffness, damping, iterations, 0.0, total, 0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
    ]);
    for (i, &(kind, cw, stored)) in links.iter().enumerate() {
        let o = HEADER + i * LINK;
        d[o..o + LINK].copy_from_slice(&[kind, f64::from(cw), stored, f64::NAN, 0.0]);
    }
    for (i, s) in segs.iter().enumerate() {
        let o = HEADER + (n + 1) * LINK + i * SEGMENT;
        d[o] = 2000.0 + i as f64;
        d[o + 1] = s.a as f64;
        d[o + 2] = s.b as f64;
        d[o + 3] = s.rest;
        d[o + 4..o + 7].copy_from_slice(&s.pa);
        d[o + 7..o + 10].copy_from_slice(&s.pb);
    }
    d
}
fn world(entities: Vec<Entity>, paths: Vec<Vec<f64>>) -> PhysicsWorld {
    let mut w = PhysicsWorld::new(entities, paths);
    w.options[1] = -G;
    w.options[4] = 1.0; // attachment updates
    w.options[5] = 1.0; // rest-length clamping
    w.options[9] = -1.0;
    w.options[12..14].fill(1.0);
    w
}
fn seg_count(w: &mut PhysicsWorld, p: usize) -> usize {
    w.path_data(p)[0] as usize
}
fn seg_field(w: &mut PhysicsWorld, p: usize, i: usize, slot: usize) -> f64 {
    let n = seg_count(w, p);
    w.path_data(p)[HEADER + (n + 1) * LINK + i * SEGMENT + slot]
}
fn tension(w: &mut PhysicsWorld, p: usize, i: usize) -> f64 {
    seg_field(w, p, i, 11)
}
fn rest(w: &mut PhysicsWorld, p: usize, i: usize) -> f64 {
    seg_field(w, p, i, 3)
}
fn stored(w: &mut PhysicsWorld, p: usize, li: usize) -> f64 {
    w.path_data(p)[HEADER + li * LINK + 2]
}
fn material(w: &mut PhysicsWorld, p: usize) -> f64 {
    let n = seg_count(w, p);
    (0..n).map(|i| rest(w, p, i)).sum::<f64>() + (0..=n).map(|li| stored(w, p, li)).sum::<f64>()
}
fn y(w: &PhysicsWorld, id: usize) -> f64 {
    w.entities[id][1]
}
fn vy(w: &PhysicsWorld, id: usize) -> f64 {
    w.entities[id][70]
}

/// Static line stiffness must follow the firmware's k = EA / L, independent of
/// how many guides the line threads. The mass hangs 1 m below the anchor with
/// zero, one or three straight-through frictionless pinholes in between.
fn hanging_stretch(pinholes: usize) -> f64 {
    let ea = 20000.0;
    let m = 10.0;
    let n = pinholes + 1;
    let mut es = vec![fixed(0, [0.0, 0.0, 0.0], 0.0)];
    for k in 1..=pinholes {
        es.push(fixed(k, [0.0, -(k as f64) / n as f64, 0.0], 0.0));
    }
    let mid = es.len();
    es.push(mass(mid, [0.0, -1.0, 0.0], m));
    let mut links = vec![(ATTACHMENT, true, 0.0)];
    links.extend((0..pinholes).map(|_| (PINHOLE, true, 0.0)));
    links.push((ATTACHMENT, true, 0.0));
    let segs: Vec<Seg> = (0..n)
        .map(|i| Seg {
            a: i,
            b: i + 1,
            rest: 1.0 / n as f64,
            pa: es[i][0..3].try_into().unwrap(),
            pb: es[i + 1][0..3].try_into().unwrap(),
        })
        .collect();
    let mut w = world(es, vec![path(&links, &segs, ea, 50.0, 4.0)]);
    for _ in 0..4000 {
        w.step(0.001);
    }
    -1.0 - y(&w, mid)
}

#[test]
fn line_stiffness_is_independent_of_guide_count() {
    let expected = 10.0 * G * 1.0 / 20000.0;
    let stretches: Vec<f64> = [0, 1, 3].iter().map(|&k| hanging_stretch(k)).collect();
    eprintln!("expected EA/L stretch {expected:.6e}, measured {stretches:?}");
    for s in stretches {
        assert!((s - expected).abs() < 0.02 * expected, "stretch {s} vs {expected}");
    }
}

/// Two masses hang from one fixed pinhole (a hairpin, wrap angle pi).
/// Capstan: static while m1/m2 <= exp(mu*pi); slipping acceleration is
/// a = g (m1 - m2 R) / (m1 + m2 R), R = exp(mu*pi).
fn hairpin(mu: f64, m1: f64, m2: f64, kind: f64) -> (f64, f64, f64) {
    let r = if kind == ROLLING { 0.02 } else { 0.0 };
    let top = if kind == ROLLING {
        pulley(0, [0.0, 0.0, 0.0], r, mu, 0.0)
    } else {
        fixed(0, [0.0, 0.0, 0.0], mu)
    };
    let left = mass(1, [-r - 1e-9, -1.0, 0.0], m1);
    let right = mass(2, [r + 1e-9, -1.0, 0.0], m2);
    let tl = [-r, 0.0, 0.0];
    let tr = [r, 0.0, 0.0];
    let arc = PI * r;
    let segs = [
        Seg { a: 1, b: 0, rest: 1.0, pa: [-r - 1e-9, -1.0, 0.0], pb: tl },
        Seg { a: 0, b: 2, rest: 1.0, pa: tr, pb: [r + 1e-9, -1.0, 0.0] },
    ];
    // Going up the left side and over the top is clockwise about +z.
    let links = [(ATTACHMENT, true, 0.0), (kind, true, arc), (ATTACHMENT, true, 0.0)];
    let mut w = world(vec![top, left, right], vec![path(&links, &segs, 1e6, 0.0, 4.0)]);
    let dt = 0.0005;
    // Let the stretch settle, then measure the acceleration over 0.2 s.
    for _ in 0..200 {
        w.step(dt);
    }
    let v0 = vy(&w, 1);
    let steps = 400;
    for _ in 0..steps {
        w.step(dt);
    }
    let a = -(vy(&w, 1) - v0) / (steps as f64 * dt);
    let t0 = tension(&mut w, 0, 0);
    let t1 = tension(&mut w, 0, 1);
    (a, t0, t1)
}

#[test]
fn pinhole_hairpin_capstan_friction() {
    let mut ok = true;
    for (mu, m1, m2) in [(0.0, 2.0, 1.0), (0.1, 2.0, 1.0), (0.3, 2.0, 1.0), (0.3, 3.0, 1.0)] {
        let ratio = (mu * PI).exp();
        let expected = (G * (m1 - m2 * ratio) / (m1 + m2 * ratio)).max(0.0);
        let (a, t0, t1) = hairpin(mu, m1, m2, PINHOLE);
        eprintln!("pinhole mu {mu} m1 {m1} m2 {m2}: a {a:.4} expected {expected:.4}, T {t0:.3}/{t1:.3}");
        ok &= (a - expected).abs() < 0.02 * G;
    }
    assert!(ok);
}

#[test]
fn fixed_roller_capstan_friction() {
    let mut ok = true;
    for (mu, m1, m2) in [(0.0, 2.0, 1.0), (0.1, 2.0, 1.0), (0.3, 3.0, 1.0)] {
        let ratio = (mu * PI).exp();
        let expected = (G * (m1 - m2 * ratio) / (m1 + m2 * ratio)).max(0.0);
        let (a, t0, t1) = hairpin(mu, m1, m2, ROLLING);
        eprintln!("roller mu {mu} m1 {m1} m2 {m2}: a {a:.4} expected {expected:.4}, T {t0:.3}/{t1:.3}");
        ok &= (a - expected).abs() < 0.02 * G;
    }
    assert!(ok);
}

/// A straight-through pinhole has zero wrap angle, so friction must not act.
#[test]
fn straight_pinhole_has_no_capstan_friction() {
    // Mass on a horizontal frictionless table is not modelled; instead pull a
    // hanging mass through a straight vertical pinhole by a heavier mass via a
    // frictionless hairpin above it. The straight pinhole must not hold load.
    let m1 = 2.0;
    let m2 = 1.0;
    let es = vec![
        fixed(0, [0.0, 0.0, 0.0], 0.0),         // frictionless hairpin
        fixed(1, [-1e-9, -0.5, 0.0], 1.0),      // straight pinhole, high mu
        mass(2, [-1e-9, -1.0, 0.0], m1),
        mass(3, [1e-9, -1.0, 0.0], m2),
    ];
    let segs = [
        Seg { a: 2, b: 1, rest: 0.5, pa: [-1e-9, -1.0, 0.0], pb: [-1e-9, -0.5, 0.0] },
        Seg { a: 1, b: 0, rest: 0.5, pa: [-1e-9, -0.5, 0.0], pb: [0.0, 0.0, 0.0] },
        Seg { a: 0, b: 3, rest: 1.0, pa: [0.0, 0.0, 0.0], pb: [1e-9, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
    let mut w = world(es, vec![path(&links, &segs, 1e6, 1e3, 4.0)]);
    let dt = 0.0005;
    for _ in 0..200 {
        w.step(dt);
    }
    let v0 = vy(&w, 2);
    for _ in 0..400 {
        w.step(dt);
    }
    let a = -(vy(&w, 2) - v0) / (400.0 * dt);
    let expected = G * (m1 - m2) / (m1 + m2);
    eprintln!("straight pinhole: a {a:.4} expected {expected:.4}");
    assert!((a - expected).abs() < 0.02 * G, "a {a} vs {expected}");
}

/// Atwood machine over a free-spinning wheel of negligible inertia (as the
/// scenes' "Wheel" bearings): equal tensions, frictionless acceleration, and
/// the wheel turns with the line so its encoder reads line travel.
#[test]
fn free_wheel_atwood_turns_with_line() {
    let (m1, m2, r) = (2.0, 1.0, 0.05);
    let mut top = pulley(0, [0.0, 0.0, 0.0], r, 0.5, 5e-8);
    top[75] += 8.0; // free spool-type wheel
    top[76..79].copy_from_slice(&[0.0, 0.0, 1.0]);
    top[79..83].copy_from_slice(&[0.0, 0.0, 0.0, 1.0]);
    top[67] = 1.0;
    let left = mass(1, [-r, -1.0, 0.0], m1);
    let right = mass(2, [r, -1.0, 0.0], m2);
    let segs = [
        Seg { a: 1, b: 0, rest: 1.0, pa: [-r, -1.0, 0.0], pb: [-r, 0.0, 0.0] },
        Seg { a: 0, b: 2, rest: 1.0, pa: [r, 0.0, 0.0], pb: [r, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (ROLLING, true, PI * r), (ATTACHMENT, true, 0.0)];
    let mut w = world(vec![top, left, right], vec![path(&links, &segs, 1e6, 1e3, 4.0)]);
    let before = material(&mut w, 0);
    let dt = 0.0005;
    for _ in 0..200 {
        w.step(dt);
    }
    let (v0, y0, angle0) = (vy(&w, 1), y(&w, 1), w.entities[0][66]);
    for _ in 0..400 {
        w.step(dt);
    }
    let a = -(vy(&w, 1) - v0) / (400.0 * dt);
    let travel = y0 - y(&w, 1);
    let turned = (w.entities[0][66] - angle0).abs() * r;
    let expected = G * (m1 - m2) / (m1 + m2);
    let (t0, t1) = (tension(&mut w, 0, 0), tension(&mut w, 0, 1));
    let after = material(&mut w, 0);
    eprintln!("free wheel: a {a:.4} expected {expected:.4}, T {t0:.3}/{t1:.3}, travel {travel:.5} wheel {turned:.5}, material drift {:.3e}", after - before);
    assert!((a - expected).abs() < 0.02 * G, "a {a} vs {expected}");
    assert!((t0 - t1).abs() < 1e-6 * t0);
    assert!((turned - travel).abs() < 0.01 * travel, "wheel {turned} vs line {travel}");
    assert!((after - before).abs() < 1e-9);
}

/// Undamped pendulum on a stiff cable. XPBD's position projection dissipates
/// energy at first order in dt (substeps reduce it); it must never gain energy.
fn pendulum_loss(dt: f64) -> f64 {
    let m = 1.0;
    let es = vec![fixed(0, [0.0, 0.0, 0.0], 0.0), mass(1, [1.0, 0.0, 0.0], m)];
    let segs = [Seg { a: 0, b: 1, rest: 1.0, pa: [0.0; 3], pb: [1.0, 0.0, 0.0] }];
    let links = [(ATTACHMENT, true, 0.0), (ATTACHMENT, true, 0.0)];
    let mut w = world(es, vec![path(&links, &segs, 1e7, 0.0, 2.0)]);
    let energy = |w: &PhysicsWorld| {
        let e = &w.entities[1];
        0.5 * m * (e[69] * e[69] + e[70] * e[70] + e[71] * e[71]) + m * G * e[1]
    };
    let e0 = energy(&w);
    let mut gain: f64 = 0.0;
    for _ in 0..(4.0 / dt) as usize {
        w.step(dt);
        gain = gain.max(energy(&w) - e0);
    }
    assert!(gain < 1e-3 * m * G, "pendulum gained {gain} J");
    e0 - energy(&w)
}

#[test]
fn pendulum_energy() {
    let coarse = pendulum_loss(0.001);
    let fine = pendulum_loss(0.00025);
    eprintln!("pendulum: 4 s energy loss {coarse:.4} J at 1 ms, {fine:.4} J at 0.25 ms");
    assert!(fine < 0.02 * G && (coarse / fine - 4.0).abs() < 0.5);
}

/// Atwood machine through a pinhole gate on a fixed body. The hidden flow has
/// the reflected bearing inertia M and dry friction Fc:
/// a = (g(m1 - m2) - Fc) / (m1 + m2 + M), and static friction holds while
/// g|m1 - m2| <= Fc.
fn gate_atwood(inertia: f64, coulomb: f64) -> (f64, f64) {
    let (m1, m2) = (2.0, 1.0);
    let es = vec![
        fixed(0, [-0.05, 0.0, 0.0], 0.0),
        fixed(1, [0.05, 0.0, 0.0], 0.0),
        mass(2, [-0.05, -1.0, 0.0], m1),
        mass(3, [0.05, -1.0, 0.0], m2),
    ];
    let segs = [
        Seg { a: 2, b: 0, rest: 1.0, pa: [-0.05, -1.0, 0.0], pb: [-0.05, 0.0, 0.0] },
        Seg { a: 1, b: 3, rest: 1.0, pa: [0.05, 0.0, 0.0], pb: [0.05, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (5.0, true, 0.1), (ATTACHMENT, true, 0.0)];
    let mut p = path(&links, &segs, 1e6, 1e3, 4.0);
    p[9] = 1.0;
    p[12] = inertia;
    p[14] = coulomb;
    let mut w = world(es, vec![p]);
    let dt = 0.0005;
    for _ in 0..200 {
        w.step(dt);
    }
    let v0 = vy(&w, 2);
    for _ in 0..400 {
        w.step(dt);
    }
    let a = -(vy(&w, 2) - v0) / (400.0 * dt);
    (a, tension(&mut w, 0, 0) - tension(&mut w, 0, 1))
}

#[test]
fn gate_flow_inertia_and_bearing_friction() {
    let mut ok = true;
    for (inertia, coulomb) in [(0.5, 0.0), (0.0237, 0.0), (0.5, 5.0), (0.5, 25.0)] {
        let expected = ((G - coulomb) / (3.0 + inertia)).max(0.0);
        let (a, difference) = gate_atwood(inertia, coulomb);
        eprintln!("gate M {inertia} Fc {coulomb}: a {a:.4} expected {expected:.4}, entry-exit tension {difference:.4}");
        ok &= (a - expected).abs() < 0.02 * G;
    }
    assert!(ok);
}


fn kinetic_plus_potential(w: &PhysicsWorld, ids: &[(usize, f64)]) -> f64 {
    ids.iter()
        .map(|&(id, m)| {
            let e = &w.entities[id];
            0.5 * m * (e[69] * e[69] + e[70] * e[70] + e[71] * e[71]) + m * G * e[1]
        })
        .sum()
}

/// Stress: extreme mass ratios, stiffness and slack-taut snapping through a
/// frictionless pinhole row. Undamped systems must never gain energy (beyond
/// rounding) and must stay finite.
#[test]
fn stress_energy_never_grows() {
    let mut worst: f64 = 0.0;
    for (m1, m2, ea, slack) in [
        (1000.0, 0.001, 1e6, 0.0),
        (0.001, 1000.0, 1e9, 0.0),
        (1.0, 1.0, 1e9, 0.05),
        (5.0, 0.01, 2e4, 0.2),
        (0.01, 0.01, 1e8, 0.3),
    ] {
        // Pinhole at the origin, masses hanging with `slack` of extra cable,
        // and a sideways offset so they swing as well as fall.
        let es = vec![
            fixed(0, [0.0; 3], 0.0),
            mass(1, [-0.3, -0.8, 0.0], m1),
            mass(2, [0.4, -0.6, 0.0], m2),
        ];
        let d1 = (0.3f64 * 0.3 + 0.8 * 0.8).sqrt();
        let d2 = (0.4f64 * 0.4 + 0.6 * 0.6).sqrt();
        let segs = [
            Seg { a: 1, b: 0, rest: d1 + slack, pa: [-0.3, -0.8, 0.0], pb: [0.0; 3] },
            Seg { a: 0, b: 2, rest: d2, pa: [0.0; 3], pb: [0.4, -0.6, 0.0] },
        ];
        let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
        let mut w = world(es, vec![path(&links, &segs, ea, 0.0, 2.0)]);
        let ids = [(1, m1), (2, m2)];
        let e0 = kinetic_plus_potential(&w, &ids);
        let scale = (m1 + m2) * G;
        let mut gain: f64 = 0.0;
        for _ in 0..4000 {
            w.step(0.001);
            let e = kinetic_plus_potential(&w, &ids);
            assert!(e.is_finite());
            gain = gain.max((e - e0) / scale);
        }
        eprintln!("m {m1}/{m2} EA {ea:e} slack {slack}: max relative energy gain {gain:.3e}");
        worst = worst.max(gain);
    }
    assert!(worst < 1e-6, "energy grew by {worst}");
}

/// Exactly at the capstan threshold the hairpin must hold still without
/// chattering, and just above it must slip at the analytic rate.
#[test]
fn capstan_threshold_is_stable() {
    let mu: f64 = 0.3;
    let ratio = (mu * PI).exp();
    for (m1, expected) in [(ratio * (1.0 - 1e-6), 0.0), (ratio * 1.05, G * (ratio * 1.05 - ratio) / (ratio * 1.05 + ratio))] {
        let (a, t0, t1) = hairpin(mu, m1, 1.0, PINHOLE);
        eprintln!("threshold m1 {m1:.5}: a {a:.5} expected {expected:.5}, T ratio {:.5} cone {ratio:.5}", t0 / t1);
        assert!((a - expected).abs() < 0.01 * G);
        assert!(t0 / t1 <= ratio * (1.0 + 1e-6));
    }
}

/// Slack cable slides freely over a frictional pinhole when one side pulls:
/// with no tension on the slack side the capstan allows no holding force.
#[test]
fn slack_slides_over_frictional_pinhole() {
    // A horizontal pull: the mass hangs from a pinhole whose other side is a
    // long slack tail tied to an anchor. Friction cannot hold the mass.
    let es = vec![fixed(0, [0.0; 3], 0.5), fixed(1, [-0.5, 0.0, 0.0], 0.0), mass(2, [0.0, -0.5, 0.0], 1.0)];
    let segs = [
        Seg { a: 1, b: 0, rest: 0.9, pa: [-0.5, 0.0, 0.0], pb: [0.0; 3] },
        Seg { a: 0, b: 2, rest: 0.5, pa: [0.0; 3], pb: [0.0, -0.5, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
    let mut w = world(es, vec![path(&links, &segs, 1e6, 1e3, 4.0)]);
    let total = 0.9 + 0.5;
    for _ in 0..2000 {
        w.step(0.0005);
    }
    // The tail straightens, then the mass hangs at the end of all the cable.
    let hang = -y(&w, 2);
    let expected = total - 0.5;
    eprintln!("slack slide: mass hangs {hang:.4} m below the pinhole, all slack used {expected:.4}");
    assert!((hang - expected).abs() < 2e-3);
}

/// Spans can shrink to nothing at a pinhole without blowing up.
#[test]
fn vanishing_span_is_stable() {
    // m1 is pulled up into the pinhole by a much heavier m2.
    let es = vec![fixed(0, [0.0; 3], 0.0), mass(1, [-1e-9, -0.05, 0.0], 0.1), mass(2, [1e-9, -1.0, 0.0], 3.0)];
    let segs = [
        Seg { a: 1, b: 0, rest: 0.05, pa: [-1e-9, -0.05, 0.0], pb: [0.0; 3] },
        Seg { a: 0, b: 2, rest: 1.0, pa: [0.0; 3], pb: [1e-9, -1.0, 0.0] },
    ];
    let links = [(ATTACHMENT, true, 0.0), (PINHOLE, true, 0.0), (ATTACHMENT, true, 0.0)];
    let mut w = world(es, vec![path(&links, &segs, 1e6, 1e3, 2.0)]);
    let before = material(&mut w, 0);
    for _ in 0..4000 {
        w.step(0.0005);
        assert!(w.entities.iter().all(|e| e[..3].iter().all(|x| x.is_finite())));
    }
    let after = material(&mut w, 0);
    eprintln!("vanishing span: m1 y {:.6}, rests {:.3e} {:.6}, material drift {:.2e}", y(&w, 1), rest(&mut w, 0, 0), rest(&mut w, 0, 1), after - before);
    assert!(y(&w, 1) <= 1e-3 && (after - before).abs() < 1e-9);
}

/// Replays prepared records dumped from a production world (HP_SIM5_REPLAY
/// directory of regionN.bin little-endian f64 files: entities, options,
/// paths) for profiling, e.g. under callgrind.
#[test]
#[ignore]
fn replay_prepared_world() {
    let Ok(dir) = std::env::var("HP_SIM5_REPLAY") else { return };
    let read = |i: usize| -> Option<Vec<f64>> {
        let bytes = std::fs::read(format!("{dir}/region{i}.bin")).ok()?;
        Some(bytes.as_chunks::<8>().0.iter().map(|b| f64::from_le_bytes(*b)).collect())
    };
    let entities: Vec<Entity> = read(0).unwrap().as_chunks::<160>().0.to_vec();
    let options = read(1).unwrap();
    let paths: Vec<Vec<f64>> = (2..).map_while(read).collect();
    let mut w = PhysicsWorld::new(entities, paths);
    w.options.copy_from_slice(&options);
    let steps: usize = std::env::var("HP_SIM5_STEPS").ok().and_then(|s| s.parse().ok()).unwrap_or(2000);
    let start = std::time::Instant::now();
    for _ in 0..steps {
        w.step(0.002);
    }
    eprintln!("replayed {steps} steps in {:?}", start.elapsed());
}
