//! Scenario builders shared by the analytic cable tests.
#![allow(dead_code)]
pub use hp_sim5_cable_solver::{
    engine::{PhysicsWorld, HEADER, LINK, SEGMENT},
    Entity,
};
pub use std::f64::consts::PI;

pub const G: f64 = 9.81;
pub const ATTACHMENT: f64 = 0.0;
pub const ROLLING: f64 = 1.0;
pub const PINHOLE: f64 = 2.0;

pub fn base(id: usize, p: [f64; 3]) -> Entity {
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
pub fn fixed(id: usize, p: [f64; 3], mu: f64) -> Entity {
    let mut e = base(id, p);
    e[75] = 256.0;
    e[105] = mu;
    e
}
/// A point mass under gravity.
pub fn mass(id: usize, p: [f64; 3], m: f64) -> Entity {
    let mut e = base(id, p);
    e[75] = (1 | 4 | 128 | 256) as f64;
    e[124] = m;
    e
}
/// A pulley in the xy plane. `inertia` 0 makes it a fixed (non-spinning) roller.
pub fn pulley(id: usize, p: [f64; 3], r: f64, mu: f64, inertia: f64) -> Entity {
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

/// A free-spinning wheel in the xy plane with an encoder. A `bearing` friction
/// torque makes its bearing worn.
pub fn wheel(id: usize, p: [f64; 3], r: f64, mu: f64, inertia: f64, bearing: Option<f64>) -> Entity {
    let mut e = pulley(id, p, r, mu, inertia);
    e[75] += 8.0;
    e[76..79].copy_from_slice(&[0.0, 0.0, 1.0]);
    e[79..83].copy_from_slice(&[0.0, 0.0, 0.0, 1.0]);
    e[67] = 1.0;
    if let Some(torque) = bearing {
        e[75] += 2048.0;
        e[137] = torque;
    }
    e
}

pub struct Seg {
    pub a: usize,
    pub b: usize,
    pub rest: f64,
    pub pa: [f64; 3],
    pub pb: [f64; 3],
}
/// `links` are (kind, cw, stored) for each of the `segs.len() + 1` links.
pub fn path(links: &[(f64, bool, f64)], segs: &[Seg], stiffness: f64, damping: f64, iterations: f64) -> Vec<f64> {
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
pub fn world(entities: Vec<Entity>, paths: Vec<Vec<f64>>) -> PhysicsWorld {
    let mut w = PhysicsWorld::new(entities, paths);
    w.options[1] = -G;
    w.options[4] = 1.0; // attachment updates
    w.options[5] = 1.0; // rest-length clamping
    w.options[9] = -1.0;
    w.options[12..14].fill(1.0);
    w
}
pub fn seg_count(w: &mut PhysicsWorld, p: usize) -> usize {
    w.path_data(p)[0] as usize
}
pub fn seg_field(w: &mut PhysicsWorld, p: usize, i: usize, slot: usize) -> f64 {
    let n = seg_count(w, p);
    w.path_data(p)[HEADER + (n + 1) * LINK + i * SEGMENT + slot]
}
pub fn tension(w: &mut PhysicsWorld, p: usize, i: usize) -> f64 {
    seg_field(w, p, i, 11)
}
pub fn rest(w: &mut PhysicsWorld, p: usize, i: usize) -> f64 {
    seg_field(w, p, i, 3)
}
pub fn stored(w: &mut PhysicsWorld, p: usize, li: usize) -> f64 {
    w.path_data(p)[HEADER + li * LINK + 2]
}
pub fn material(w: &mut PhysicsWorld, p: usize) -> f64 {
    let n = seg_count(w, p);
    (0..n).map(|i| rest(w, p, i)).sum::<f64>() + (0..=n).map(|li| stored(w, p, li)).sum::<f64>()
}
pub fn y(w: &PhysicsWorld, id: usize) -> f64 {
    w.entities[id][1]
}
pub fn vy(w: &PhysicsWorld, id: usize) -> f64 {
    w.entities[id][70]
}

