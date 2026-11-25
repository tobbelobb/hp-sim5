//! Persistent physics world. Prepared f64 records are also the observation/edit
//! interface; the core has no browser, ECS, transport or JavaScript dependency.
use super::*;

pub const HEADER: usize = 9;
pub const LINK: usize = 5;
pub const SEGMENT: usize = 16;
pub const OPTIONS: usize = 20;
const VEL: u32 = 1;
const OMEGA: u32 = 2;
const GRAVITY: u32 = 4;
const SPOOL: u32 = 8;
const MOTOR: u32 = 16;
const BODY: u32 = 32;
const INERTIA: u32 = 64;
const MASS: u32 = 128;
const CABLE_LINK: u32 = 256;
const LOCAL_AXIS: u32 = 512;
const RADIUS: u32 = 1024;
const BEARING: u32 = 2048;
const IDENTITY: Q = [0.0, 0.0, 0.0, 1.0];
const Z: V = [0.0, 0.0, 1.0];
const TAU: f64 = 2.0 * PI;

fn has(e: &Entity, flag: u32) -> bool {
    e[75] as u32 & flag != 0
}
fn wrap(mut a: f64) -> f64 {
    if !a.is_finite() {
        return a;
    }
    if a > PI || a < -PI {
        a %= TAU;
        if a > PI {
            a -= TAU;
        } else if a < -PI {
            a += TAU;
        }
    }
    a
}
fn unwrap(reference: f64, a: f64) -> f64 {
    if !reference.is_finite() || !a.is_finite() {
        return a;
    }
    let delta = a - reference;
    if (-PI..=PI).contains(&delta) {
        return a;
    }
    // Reduce in constant time, even when adding TAU cannot change the angle.
    // Separate remainders also avoid overflow for opposite finite extremes.
    let delta = if delta.is_finite() {
        delta
    } else {
        a % TAU - reference % TAU
    };
    reference + wrap(delta)
}
fn twist(a: Q, axis: V, eps: f64) -> f64 {
    let axis = norm(axis);
    let p = dot([a[0], a[1], a[2]], axis);
    let t = [axis[0] * p, axis[1] * p, axis[2] * p, a[3]];
    if t.iter().map(|x| x * x).sum::<f64>() <= eps {
        return 0.0;
    }
    let t = qnorm(t);
    wrap(2.0 * atan2(dot([t[0], t[1], t[2]], axis), t[3]))
}
fn orientation_angle(a: Q, axis: V, epsilon: f64) -> f64 {
    let b = basis(axis);
    let u = rotate(a, b.u);
    let projected = sub(u, scale(b.n, dot(u, b.n)));
    if dot(projected, projected) <= epsilon {
        return 0.0;
    }
    let p = norm(projected);
    atan2(dot(p, b.v), dot(p, b.u))
}
fn plane(es: &[Entity], id: usize) -> V {
    let e = &es[id];
    if has(e, LOCAL_AXIS) {
        let local = v(e, 44);
        if let Some(q) = world_quat(es, id) {
            let n = rotate(q, local);
            if dot(n, n) > EPS {
                return norm(n);
            }
        }
        if dot(local, local) > EPS {
            return norm(local);
        }
    }
    if has(e, CABLE_LINK) {
        v(e, 101)
    } else {
        Z
    }
}
fn angle_for(es: &[Entity], id: usize, quat: Q, local: Q) -> f64 {
    if has(&es[id], LOCAL_AXIS) {
        twist(local, v(&es[id], 44), EPS)
    } else {
        orientation_angle(quat, plane(es, id), EPS)
    }
}
fn local_point(es: &[Entity], id: usize, point: V) -> V {
    let Some(pos) = world_pos(es, id) else {
        return point;
    };
    let rel = sub(point, pos);
    world_quat(es, id).map_or(rel, |q| rotate(qnorm(conj(q)), rel))
}
fn axis_inertia(e: &Entity, orientation: Q, axis: V) -> f64 {
    let local = norm(rotate(qnorm(conj(orientation)), norm(axis)));
    positive_inertia(dot(local, matrix_vec(e, 25, local)))
}
fn spool_axis(e: &Entity, orientation: Q) -> V {
    let local = if dot(v(e, 76), v(e, 76)) > 1e-12 {
        norm(v(e, 76))
    } else {
        Z
    };
    let axis = rotate(orientation, local);
    if dot(axis, axis) > 1e-12 {
        norm(axis)
    } else {
        local
    }
}
fn spool_angle(reference: Q, orientation: Q, axis: V) -> f64 {
    let relative = qnorm(mul(qnorm(conj(qnorm(reference))), qnorm(orientation)));
    twist(relative, axis, 1e-12)
}
fn spool_compose(reference: Q, axis: V, angle: f64) -> Q {
    qnorm(mul(
        qnorm(reference),
        qnorm(mul(IDENTITY, axis_angle(norm(axis), angle))),
    ))
}
fn spool_frame(es: &[Entity], id: usize) -> (Q, Q, Q) {
    let e = &es[id];
    if e[50] >= 0.0 && es[e[50] as usize][52] != 0.0 {
        let body = q(&es[e[50] as usize], 3);
        let reference = qnorm(mul(qnorm(conj(body)), q(e, 79)));
        (qnorm(mul(body, q(e, 17))), reference, q(e, 17))
    } else {
        (q(e, 3), q(e, 79), q(e, 3))
    }
}

/// Maximum coalesced coil entries retained per spool end (bounded memory).
const MAX_SPOOL_COILS: usize = 256;
/// Adjacent wound segments whose rest-per-geometric ratio differs by at most this
/// tolerance are coalesced by summing `(geom, rest)`, conserving both exactly.
const SPOOL_COIL_MERGE_TOL: f64 = 1e-6;
/// Version tag for serialized per-cable spool state.
const SPOOL_STATE_VERSION: f64 = 1.0;

/// Coiled rope history on one spool end (`end = 0` for link `0`, `end = 1` for link `count`).
///
/// Invariants when `initialized` is true:
/// - `geom_stored == base_geom + sum(coils.geom_arc)` is the nominal geometric drum arc
///   occupied by wound rope (used for layering radius/theta, hybrid transitions, and knot checks).
/// - `c.stored(li) == base_rest + sum(coils.rest_len)` is the unstretched material length stored
///   on the drum (so `sum(rest) + sum(stored) == totalRestLength` holds at every step).
/// - `(base_geom, base_rest)` stores pre-wound rope from initialization (explicitly wound at
///   `init_storage_tension`, defaulting to 0 N = unstretched).
/// - `coils` is a LIFO stack of `(geom_arc, rest_len)` for rope wound during simulation,
///   with `geom_arc > 0` and `rest_len > 0`.
#[derive(Clone, Default)]
struct SpoolEnd {
    initialized: bool,
    geom_stored: f64,
    base_geom: f64,
    base_rest: f64,
    coils: Vec<(f64, f64)>,
}

impl SpoolEnd {
    fn reset_from_stored(&mut self, stored_rest: f64, rho_init: f64) {
        let rho = if rho_init.is_finite() && rho_init > EPS {
            rho_init
        } else {
            1.0
        };
        self.initialized = true;
        self.base_rest = stored_rest;
        self.base_geom = stored_rest / rho;
        self.geom_stored = self.base_geom;
        self.coils.clear();
    }
    fn reset_empty(&mut self) {
        self.initialized = true;
        self.geom_stored = 0.0;
        self.base_geom = 0.0;
        self.base_rest = 0.0;
        self.coils.clear();
    }
    fn clear_inactive(&mut self) {
        self.initialized = false;
        self.geom_stored = 0.0;
        self.base_geom = 0.0;
        self.base_rest = 0.0;
        self.coils.clear();
    }
    fn total_rest(&self) -> f64 {
        self.base_rest + self.coils.iter().map(|c| c.1).sum::<f64>()
    }
    fn total_geom(&self) -> f64 {
        self.base_geom + self.coils.iter().map(|c| c.0).sum::<f64>()
    }
    fn sync_with_stored(&mut self, stored_rest: f64, rho_init: f64) {
        if !self.initialized || (stored_rest - self.total_rest()).abs() > 1e-7 {
            self.reset_from_stored(stored_rest, rho_init);
        } else {
            self.geom_stored = self.total_geom();
        }
    }
    /// Pure preview of the rest-length delta produced by nominal geometric arc transfer `d_geom`
    /// (`> 0` winding onto spool, `< 0` unwinding off spool). Does not mutate the stack.
    fn preview_rest_delta(&self, d_geom: f64, rho_now: f64, rho_init: f64) -> f64 {
        if d_geom > EPS {
            let mut rem = d_geom;
            let mut rest = 0.0;
            if self.geom_stored < 0.0 {
                let recover = rem.min(-self.geom_stored);
                rest += recover * rho_init;
                rem -= recover;
            }
            rest + rem * rho_now
        } else if d_geom < -EPS {
            let mut need = -d_geom;
            let mut paid = 0.0;
            for &(g, r) in self.coils.iter().rev() {
                if need <= EPS {
                    break;
                }
                if need >= g - EPS {
                    paid += r;
                    need = (need - g).max(0.0);
                } else {
                    paid += r * (need / g);
                    need = 0.0;
                }
            }
            if need > EPS && self.base_geom > EPS {
                if need >= self.base_geom - EPS {
                    paid += self.base_rest;
                    need = (need - self.base_geom).max(0.0);
                } else {
                    paid += self.base_rest * (need / self.base_geom);
                    need = 0.0;
                }
            }
            if need > EPS {
                paid += need * rho_init;
            }
            -paid
        } else {
            0.0
        }
    }
    /// Exact inverse of `preview_rest_delta`: returns the nominal geometric arc transfer `d_geom`
    /// that yields rest-length delta `target_d_rest`. Does not mutate the stack.
    fn geom_for_rest_delta(&self, target_d_rest: f64, rho_now: f64, rho_init: f64) -> f64 {
        let r_now = rho_now.max(EPS);
        let r_init = rho_init.max(EPS);
        if target_d_rest > EPS {
            let mut rem_r = target_d_rest;
            let mut geom = 0.0;
            if self.geom_stored < 0.0 {
                let neg_r = (-self.geom_stored) * r_init;
                if rem_r >= neg_r {
                    geom += -self.geom_stored;
                    rem_r -= neg_r;
                } else {
                    return rem_r / r_init;
                }
            }
            geom + rem_r / r_now
        } else if target_d_rest < -EPS {
            let mut need_r = -target_d_rest;
            let mut geom = 0.0;
            for &(g, r) in self.coils.iter().rev() {
                if need_r <= EPS {
                    break;
                }
                if need_r >= r - EPS {
                    geom += g;
                    need_r = (need_r - r).max(0.0);
                } else {
                    geom += g * (need_r / r);
                    need_r = 0.0;
                }
            }
            if need_r > EPS && self.base_rest > EPS && self.base_geom > EPS {
                if need_r >= self.base_rest - EPS {
                    geom += self.base_geom;
                    need_r = (need_r - self.base_rest).max(0.0);
                } else {
                    geom += self.base_geom * (need_r / self.base_rest);
                    need_r = 0.0;
                }
            }
            if need_r > EPS {
                geom += need_r / r_init;
            }
            -geom
        } else {
            0.0
        }
    }
    /// Commit nominal geometric arc transfer `d_geom` and return the exact rest-length delta.
    fn apply_geom_delta(&mut self, d_geom: f64, rho_now: f64, rho_init: f64) -> f64 {
        let d_rest = if d_geom > EPS {
            let mut rem = d_geom;
            let mut rest = 0.0;
            if self.base_geom < 0.0 && self.coils.is_empty() {
                let recover = rem.min(-self.base_geom);
                let dr = recover * rho_init;
                self.base_geom += recover;
                self.base_rest += dr;
                if self.base_geom.abs() <= EPS {
                    self.base_geom = 0.0;
                    self.base_rest = 0.0;
                }
                rest += dr;
                rem -= recover;
            }
            if rem > EPS {
                let dr = rem * rho_now;
                if let Some(top) = self.coils.last_mut() {
                    if (top.1 / top.0 - rho_now).abs() <= SPOOL_COIL_MERGE_TOL {
                        // Material-preserving coalescing: exact sum of both geom and rest.
                        top.0 += rem;
                        top.1 += dr;
                    } else {
                        self.coils.push((rem, dr));
                    }
                } else {
                    self.coils.push((rem, dr));
                }
                if self.coils.len() > MAX_SPOOL_COILS {
                    let mut best = 0;
                    let mut best_diff = f64::INFINITY;
                    for k in 0..self.coils.len() - 1 {
                        let diff = (self.coils[k].1 / self.coils[k].0
                            - self.coils[k + 1].1 / self.coils[k + 1].0)
                            .abs();
                        if diff < best_diff {
                            best_diff = diff;
                            best = k;
                        }
                    }
                    let next = self.coils.remove(best + 1);
                    self.coils[best].0 += next.0;
                    self.coils[best].1 += next.1;
                }
                rest += dr;
            }
            rest
        } else if d_geom < -EPS {
            let mut need = -d_geom;
            let mut paid = 0.0;
            while need > EPS {
                let Some(top) = self.coils.last_mut() else {
                    break;
                };
                if need >= top.0 - EPS {
                    let (g, r) = self.coils.pop().unwrap();
                    paid += r;
                    need = (need - g).max(0.0);
                } else {
                    let dr = top.1 * (need / top.0);
                    top.0 -= need;
                    top.1 -= dr;
                    paid += dr;
                    need = 0.0;
                }
            }
            if need > EPS && self.base_geom > EPS {
                if need >= self.base_geom - EPS {
                    paid += self.base_rest;
                    need = (need - self.base_geom).max(0.0);
                    self.base_geom = 0.0;
                    self.base_rest = 0.0;
                } else {
                    let dr = self.base_rest * (need / self.base_geom);
                    self.base_geom -= need;
                    self.base_rest -= dr;
                    paid += dr;
                    need = 0.0;
                }
            }
            if need > EPS {
                let dr = need * rho_init;
                self.base_geom -= need;
                self.base_rest -= dr;
                paid += dr;
            }
            -paid
        } else {
            0.0
        };
        self.geom_stored = self.total_geom();
        d_rest
    }
}

/// Default radial compliance of rope on a drum, k_r·R in m²/N. A rope pressed onto a drum of
/// radius R carries line load F/R, so its centreline sinks by F/(R·E_t) whatever its diameter.
/// An effective E_t ≈ 47 MPa follows from k_r = 0.422 µm/N on the R = 50.18 mm DaRUS-5171 D-Pro rig. Stored
/// negated in `Cable::spool_config` so explicit per-cable k_r (m/N, >= 0) stays distinct.
const DEFAULT_SPOOL_RADIAL_COMPLIANCE: f64 = 0.422e-6 * 0.05018;

/// Running-radius contraction per newton at radius `r` (negative `k_r` is a compliance in m²/N).
fn flattening_at(k_r: f64, r: f64) -> f64 {
    if k_r < 0.0 { -k_r / r } else { k_r }
}

fn spool_rho(radius: f64, k_r: f64, cpl: f64, strained: bool, tension: f64) -> f64 {
    let f = tension.max(0.0);
    let r_scale = if k_r.is_finite() && k_r != 0.0 && radius > EPS {
        ((radius - flattening_at(k_r, radius) * f) / radius).max(0.5)
    } else {
        1.0
    };
    let strain_scale = if strained && cpl.is_finite() && cpl > EPS {
        1.0 / (1.0 + f * cpl)
    } else {
        1.0
    };
    r_scale * strain_scale
}

#[derive(Clone)]
struct Cable {
    // Public records are mutable observations/edits. Scratch is kept in Rust.
    data: Vec<f64>,
    // Per link: capstan tension ratio (infinite where no material can cross)
    // and slip state (STICK, FREE, or the tighter side LEFT/RIGHT), which
    // persists between steps, and the states (as bits) this step's solves
    // have assumed. Per segment: whether it shares a row, and its strain when
    // the last step ended.
    ratio: Vec<f64>,
    slip: Vec<f64>,
    tried: Vec<u8>,
    merged: Vec<bool>,
    strain: Vec<f64>,
    // Per link on a wheel with a worn bearing.
    wheels: Vec<Option<Wheel>>,
    // Accumulated hysteretic elongation strain (active when options[11] > 0).
    hyst_strain: f64,
    // Per-cable spool constitutive parameters (None: elastic cables use the defaults, others none).
    radial_flattening: Option<f64>,
    strained_spool: Option<bool>,
    init_storage_tension: f64,
    // Geometric occupancy and LIFO coil history for the two spool ends (0 and `count`).
    spool_ends: [SpoolEnd; 2],
}
/// A wheel whose bearing resists turning. The line turns it through capstan
/// friction, against the bearing's friction torque and the wheel's inertia.
#[derive(Clone, Copy, Default)]
struct Wheel {
    /// Whether the line's grip overcomes the bearing, so the rim follows the
    /// line. Otherwise the line slides over the wheel.
    grip: bool,
    /// Rim travel the sliding line's drag adds per step, and the bearing
    /// removes.
    drive: f64,
    brake: f64,
    /// Rim travel of the last two steps, as material gained left of the link.
    travel: [f64; 2],
}
impl Wheel {
    /// Rim travel this step beside a line that travels `line`.
    fn turn(&self, line: f64) -> f64 {
        if self.grip {
            return line;
        }
        let dragged = self.travel[0] + (line - self.travel[0]).clamp(-self.drive, self.drive);
        dragged.signum() * (dragged.abs() - self.brake).max(0.0)
    }
}
const STICK: f64 = 0.0;
const RIGHT: f64 = 1.0;
const LEFT: f64 = -1.0;
const FREE: f64 = 2.0;
/// Solves per step while link states still contradict their solution.
const SOLVES: usize = 4;
fn tried(state: f64) -> u8 {
    1 << (state + 1.0) as u8
}
/// Set once a link's sticking and slipping solves have contradicted each
/// other this step: its slipping solve is then kept.
const SETTLED: u8 = 16;
/// Closed-loop servos integrate their error with this time constant (s): slow
/// enough not to hunt in the soft sag modes of a cable robot, fast enough to
/// pull a loaded motor back to its target within seconds.
const SERVO_INTEGRAL_TIME: f64 = 1.0;
impl Cable {
    fn count(&self) -> usize {
        self.data[0] as usize
    }
    fn lo(&self, i: usize) -> usize {
        HEADER + i * LINK
    }
    fn jo(&self, i: usize) -> usize {
        HEADER + (self.count() + 1) * LINK + i * SEGMENT
    }
    fn kind(&self, i: usize) -> u32 {
        self.data[self.lo(i)] as u32
    }
    fn cw(&self, i: usize, first: bool) -> bool {
        (self.data[self.lo(i) + 1] != 0.0) ^ (i == 0 && first)
    }
    fn stored(&self, i: usize) -> f64 {
        self.data[self.lo(i) + 2]
    }
    fn set_stored(&mut self, i: usize, s: f64) {
        let o = self.lo(i);
        self.data[o + 2] = s;
    }
    fn spool_config(&self) -> (f64, bool, f64) {
        let elastic = self.data[3].is_finite() && self.data[3] > EPS;
        let k_r = self.radial_flattening.unwrap_or(if elastic { -DEFAULT_SPOOL_RADIAL_COMPLIANCE } else { 0.0 });
        let strained = self.strained_spool.unwrap_or(elastic);
        let init_f = if self.init_storage_tension.is_finite() && self.init_storage_tension > 0.0 {
            self.init_storage_tension
        } else {
            0.0
        };
        (k_r, strained, init_f)
    }
    fn geom_stored(&self, i: usize) -> f64 {
        if i == 0 && self.spool_ends[0].initialized {
            self.spool_ends[0].geom_stored
        } else if i == self.count() && self.spool_ends[1].initialized {
            self.spool_ends[1].geom_stored
        } else {
            self.stored(i)
        }
    }
    fn entity(&self, i: usize, side: usize) -> usize {
        self.data[self.jo(i) + 1 + side] as usize
    }
    fn point(&self, i: usize, side: usize) -> V {
        v(&self.data, self.jo(i) + 4 + 3 * side)
    }
    fn rest(&self, i: usize) -> f64 {
        self.data[self.jo(i) + 3]
    }
    fn radius(&self, es: &[Entity], i: usize, id: usize, layering: bool) -> (f64, f64) {
        let e = &es[id];
        let base = if has(e, RADIUS) { e[104] } else { f64::NAN };
        let half = if layering { self.data[6] } else { 0.0 };
        if half <= EPS || !base.is_finite() {
            return (base, 0.0);
        }
        let effective = base + half;
        let geom = self.geom_stored(i);
        if (i == 0 || i == self.count()) && hybrid(self.kind(i)) && geom > EPS {
            let (r, t) = stored_radius(geom.max(0.0), base, half);
            (effective.max(r), t)
        } else {
            (effective, 0.0)
        }
    }
}
fn hybrid(kind: u32) -> bool {
    kind == 3 || kind == 4
}
fn rolling(kind: u32) -> bool {
    kind == 1 || kind == 3
}
fn attachment_kind(kind: u32) -> bool {
    kind == 0 || kind == 2 || kind == 4
}

// Identical layer/ramp mechanics to the JS reference, with bounded layers.
fn layer(base: f64, half: f64, n: usize) -> (f64, f64, f64, f64, f64) {
    let r = base + half + 2.0 * half * n as f64;
    let ramp = (base.max(0.0) * (PI / 30.0) / (r + half)).clamp(0.0, TAU);
    let phi = TAU - ramp;
    let lc = r * phi;
    (r, ramp, phi, lc, lc + ramp * (r + half))
}
fn theta_stored(theta: f64, base: f64, half: f64) -> f64 {
    if !theta.is_finite() || theta.abs() <= EPS {
        return 0.0;
    }
    if theta < 0.0 {
        return -theta_stored(-theta, base, half);
    }
    if base + half <= EPS || 2.0 * half <= EPS {
        return base.max(0.0) * theta;
    }
    let mut theta = theta;
    let mut stored = 0.0;
    let mut n = 0;
    while theta > TAU + EPS && n < 2048 {
        stored += layer(base, half, n).4;
        theta -= TAU;
        n += 1;
    }
    if n >= 2048 {
        return stored + (base + half + 2.0 * half * 2048.0) * theta;
    }
    let (r, ramp, phi, lc, _) = layer(base, half, n);
    let theta = theta.clamp(0.0, TAU);
    stored
        + if ramp <= EPS || theta <= phi + EPS {
            r * phi.min(theta)
        } else {
            let x = theta - phi;
            lc + r * x + (2.0 * half / (2.0 * ramp)) * x * x
        }
}
fn stored_radius(stored: f64, base: f64, half: f64) -> (f64, f64) {
    if base + half <= EPS || 2.0 * half <= EPS {
        let r = base.max(0.0);
        return (r, if r > EPS { stored.max(0.0) / r } else { 0.0 });
    }
    let mut s = stored.max(0.0);
    let mut theta = 0.0;
    for n in 0..2048 {
        let (r, ramp, phi, lc, lw) = layer(base, half, n);
        if s > lw + EPS {
            s -= lw;
            theta += TAU;
            continue;
        }
        if s <= lc + EPS || ramp <= EPS {
            return (r, theta + phi.min(s / r));
        }
        let a = 2.0 * half / (2.0 * ramp);
        let x = ((-r + (r * r + 4.0 * a * (s - lc).max(0.0)).max(0.0).sqrt()) / (2.0 * a))
            .clamp(0.0, ramp);
        return (r + 2.0 * half * (x / ramp), theta + phi + x);
    }
    (base + half + 2.0 * half * 2048.0, theta)
}
fn stored_theta(s: f64, base: f64, half: f64) -> f64 {
    if !s.is_finite() || s.abs() <= EPS {
        0.0
    } else if s < 0.0 {
        -stored_radius(-s, base, half).1
    } else {
        stored_radius(s, base, half).1
    }
}
fn stored_delta(theta: f64, delta: f64, cw: bool, base: f64, half: f64, radius: f64) -> f64 {
    let d = if cw { delta } else { -delta };
    if !d.is_finite() || d.abs() <= EPS {
        return 0.0;
    }
    if half > EPS && base.is_finite() {
        theta_stored(theta + d, base, half) - theta_stored(theta, base, half)
    } else {
        d * radius
    }
}
fn angle_shift(
    theta: f64,
    delta: f64,
    cw: bool,
    base: f64,
    half: f64,
    shift: f64,
    radius: f64,
) -> f64 {
    if !shift.is_finite() || shift.abs() <= EPS {
        return 0.0;
    }
    let sign = if cw { 1.0 } else { -1.0 };
    if half > EPS && base.is_finite() {
        let target = theta_stored(theta + sign * delta, base, half) + shift;
        (stored_theta(target, base, half) - theta) / sign - delta
    } else if radius.abs() > EPS {
        shift / (sign * radius)
    } else {
        0.0
    }
}
fn arc(a: V, b: V, center: V, r: f64, cw: bool, n: V, positive: bool) -> f64 {
    let basis = basis(n);
    let a = project(a, center, basis);
    let b = project(b, center, basis);
    let mut angle = atan2(b[1], b[0]) - atan2(a[1], a[0]);
    if angle > PI {
        angle -= TAU;
    }
    if angle < -PI {
        angle += TAU;
    }
    if cw {
        angle *= -1.0;
    }
    if positive && angle < 0.0 {
        angle += TAU;
    }
    r * angle
}

#[derive(Clone, Copy)]
struct Endpoint {
    pos: Option<V>,
    prev: Option<V>,
    normal: V,
    quat: Q,
    prev_quat: Q,
    frame: Q,
    prev_frame: Q,
    delta: f64,
}
fn internal(es: &[Entity], a: usize, b: usize) -> bool {
    es[a][50] >= 0.0 && es[a][50] == es[b][50]
}
fn endpoint(es: &[Entity], id: usize, other: usize) -> Endpoint {
    let e = &es[id];
    let quat = world_quat(es, id).unwrap_or(IDENTITY);
    let prev_quat = q(e, 93);
    let frame = if internal(es, id, other) {
        q(e, 17)
    } else {
        quat
    };
    let prev_frame = if internal(es, id, other) {
        q(e, 97)
    } else {
        prev_quat
    };
    let normal = plane(es, id);
    let delta = if !has(e, CABLE_LINK) || e[52] == 0.0 {
        0.0
    } else if has(e, LOCAL_AXIS) {
        twist(qnorm(mul(qnorm(conj(prev_frame)), frame)), v(e, 44), EPS)
    } else {
        let rel = qnorm(mul(quat, qnorm(conj(prev_quat))));
        let w = rel[3].clamp(-1.0, 1.0);
        let mut a = 2.0 * acos(w);
        if a < EPS {
            0.0
        } else {
            let s = (1.0 - w * w).max(0.0).sqrt();
            let axis = if s < EPS {
                norm(normal)
            } else {
                [rel[0] / s, rel[1] / s, rel[2] / s]
            };
            let d = dot(axis, normal);
            if d == 0.0 {
                a
            } else {
                if a > PI {
                    a -= TAU;
                }
                a * d.signum()
            }
        }
    };
    Endpoint {
        pos: world_pos(es, id),
        prev: if has(e, CABLE_LINK) {
            Some(v(e, 90))
        } else {
            None
        },
        normal,
        quat,
        prev_quat,
        frame,
        prev_frame,
        delta,
    }
}
type AttachmentGeometry = ([Option<V>; 2], [Endpoint; 2], [(f64, f64); 2]);
fn geometry(c: &Cable, i: usize, es: &[Entity], layering: bool) -> AttachmentGeometry {
    let ids = [c.entity(i, 0), c.entity(i, 1)];
    let endpoints = [endpoint(es, ids[0], ids[1]), endpoint(es, ids[1], ids[0])];
    let radii = [
        c.radius(es, i, ids[0], layering),
        c.radius(es, i + 1, ids[1], layering),
    ];
    let mut buf = [0.0; 46];
    for side in 0..2 {
        let e = endpoints[side];
        let o = side * 19;
        let kind = c.kind(i + side);
        if let Some(pos) = e.pos {
            put_v(&mut buf, o, pos);
        }
        if let Some(prev) = e.prev {
            put_v(&mut buf, o + 3, prev);
        }
        put_v(&mut buf, o + 6, e.normal);
        put_v(&mut buf, o + 9, c.point(i, side));
        buf[o + 12] = e.delta;
        buf[o + 13] = (u32::from(e.pos.is_some())
            | (u32::from(e.prev.is_some()) << 1)
            | (u32::from(radii[side].0.is_finite()) << 2)) as f64;
        buf[o + 14] = radii[side].0;
        buf[o + 15] = f64::from(c.cw(i + side, side == 0));
        buf[o + 16] = f64::from(rolling(kind));
        buf[o + 17] = f64::from(hybrid(kind));
        buf[o + 18] = f64::from(kind == 4);
    }
    // The same pure tangent kernel used by the standalone ABI.
    attachment_data(&mut buf);
    (
        [
            if buf[44] != 0.0 {
                Some(v(&buf, 38))
            } else {
                None
            },
            if buf[45] != 0.0 {
                Some(v(&buf, 41))
            } else {
                None
            },
        ],
        endpoints,
        radii,
    )
}
fn relative_angle(es: &[Entity], id: usize, e: Endpoint, point: V) -> f64 {
    let rel = sub(point, e.pos.unwrap_or([0.0; 3]));
    if has(&es[id], LOCAL_AXIS) {
        let rel = rotate(qnorm(conj(e.quat)), rel);
        let b = basis(v(&es[id], 44));
        atan2(dot(rel, b.v), dot(rel, b.u))
    } else {
        let b = basis(e.normal);
        wrap(atan2(dot(rel, b.v), dot(rel, b.u)) - angle_for(es, id, e.quat, e.frame))
    }
}
fn rotate_entity(es: &mut [Entity], id: usize, axis: V, angle: f64) {
    if !angle.is_finite() || angle.abs() <= EPS || dot(axis, axis) <= EPS {
        return;
    }
    let result = qnorm(mul(axis_angle(norm(axis), angle), q(&es[id], 3)));
    put_q(&mut es[id], 3, result);
    update_local(es, id);
}

#[derive(Debug, PartialEq, Eq)]
pub struct InvalidPosition {
    pub row: usize,
}

/// One continuing world, usable directly from native Rust as well as the C ABI.
pub struct PhysicsWorld {
    pub entities: Vec<Entity>,
    pub options: [f64; OPTIONS],
    cables: Vec<Cable>,
    children: Vec<Vec<usize>>,
    joints: Vec<Joint>,
    rows: Vec<Row>,
    shared: Vec<Vec<(usize, usize)>>,
    pub topology_version: u32,
    scratch: Scratch,
    // Entities before the cable solve, restored when it is repeated.
    snapshot: Vec<Entity>,
}
impl PhysicsWorld {
    /// Entity records and path records are prepared once by the scene adapter.
    pub fn new(entities: Vec<Entity>, paths: Vec<Vec<f64>>) -> Self {
        let mut children = vec![Vec::new(); entities.len()];
        for (id, e) in entities.iter().enumerate() {
            // A parent link alone does not opt a member into body sync/inertia.
            // Negative membership order preserves the JS body-inertia fallback.
            if e[50] >= 0.0 && e[131] >= 0.0 {
                children[e[50] as usize].push(id);
            }
        }
        for ids in &mut children {
            ids.sort_by(|a, b| entities[*a][131].total_cmp(&entities[*b][131]));
        }
        let cables: Vec<Cable> = paths
            .into_iter()
            .map(|data| Cable {
                data,
                ratio: Vec::new(),
                slip: Vec::new(),
                tried: Vec::new(),
                merged: Vec::new(),
                strain: Vec::new(),
                wheels: Vec::new(),
                hyst_strain: 0.0,
                radial_flattening: None,
                strained_spool: None,
                init_storage_tension: 0.0,
                spool_ends: [SpoolEnd::default(), SpoolEnd::default()],
            })
            .collect();
        let mut by_id = std::collections::BTreeMap::<u64, Vec<(usize, usize)>>::new();
        for (p, c) in cables.iter().enumerate() {
            for i in 0..c.count() {
                by_id
                    .entry(c.data[c.jo(i)] as u64)
                    .or_default()
                    .push((p, i));
            }
        }
        let shared = by_id
            .into_values()
            .filter(|owners| owners.len() > 1)
            .collect();
        let mut options = [0.0; OPTIONS];
        options[15] = f64::NAN;
        Self {
            entities,
            options,
            cables,
            children,
            joints: Vec::new(),
            rows: Vec::new(),
            shared,
            topology_version: 0,
            scratch: Scratch::default(),
            snapshot: Vec::new(),
        }
    }
    pub fn path_data(&mut self, index: usize) -> &mut [f64] {
        &mut self.cables[index].data
    }
    pub fn path_count(&self) -> usize {
        self.cables.len()
    }
    /// Configure per-cable spool constitutive parameters:
    /// - `radial_flattening`: drum running-radius contraction per unit tension (m/N, >= 0).
    /// - `strained_spool`: store coiled rope strained at its winding tension with LIFO memory.
    /// - `init_storage_tension`: explicit tension (N, >= 0) at which pre-wound stored rope was wound.
    ///
    /// Runtime parameter-change semantics:
    /// - Spool parameters are intended to be configured at initialization or between experiments.
    /// - Changing `radial_flattening`, `strained_spool` (while keeping spool physics enabled), or
    ///   cable stiffness mid-run preserves already-wound `(geom, rest)` coil records and applies
    ///   the updated parameters to subsequent winding and current flattening.
    /// - Disabling spool physics (`radial_flattening == 0 && !strained_spool`) clears the internal
    ///   LIFO history; re-enabling later reconstructs from `c.stored(li)` using `init_storage_tension`.
    /// - Changing `init_storage_tension` affects only uninitialized spool ends or future external-edit
    ///   resyncs; it does not retroactively rewrite already-initialized coil history.
    pub fn set_cable_spool_params(
        &mut self,
        index: usize,
        radial_flattening: f64,
        strained_spool: bool,
        init_storage_tension: f64,
    ) {
        let kr = if radial_flattening.is_finite() {
            radial_flattening.max(0.0)
        } else {
            0.0
        };
        self.set_cable_spool_params_raw(
            index,
            kr,
            if strained_spool { 1.0 } else { 0.0 },
            init_storage_tension,
        );
    }
    /// C-ABI / JS helper for per-cable spool parameters.
    /// Negative or non-finite `radial_flattening` / `strained_mode` inherits the solver default. Zero explicitly disables the corresponding effect.
    pub fn set_cable_spool_params_raw(
        &mut self,
        index: usize,
        radial_flattening: f64,
        strained_mode: f64,
        init_storage_tension: f64,
    ) {
        if let Some(c) = self.cables.get_mut(index) {
            c.radial_flattening = if radial_flattening.is_finite() && radial_flattening >= 0.0 {
                Some(radial_flattening)
            } else {
                None
            };
            c.strained_spool = if !strained_mode.is_finite() || strained_mode < 0.0 {
                None
            } else {
                Some(strained_mode != 0.0)
            };
            c.init_storage_tension = if init_storage_tension.is_finite() {
                init_storage_tension.max(0.0)
            } else {
                0.0
            };
        }
    }
    /// Geometric nominal drum arc occupied on spool end `side` (0 = link 0, 1 = link `count`).
    pub fn spool_geom_stored(&self, index: usize, side: usize) -> f64 {
        let Some(c) = self.cables.get(index) else {
            return f64::NAN;
        };
        let li = if side == 0 { 0 } else { c.count() };
        c.geom_stored(li)
    }
    /// Authoritative pure prediction of stored unstretched material length on spool end `side`
    /// after rotating the endpoint motor by `delta_angle` around its spool axis.
    /// Matches `update_attachments` kinematics (`c.cw(li, side == 0)` and `geom_stored` layering)
    /// and previews material transfer through the active LIFO coil stack without mutating state.
    pub fn predict_spool_stored_after_rotation(
        &self,
        index: usize,
        side: usize,
        delta_angle: f64,
        layering: bool,
    ) -> f64 {
        let Some(c) = self.cables.get(index) else {
            return f64::NAN;
        };
        let n = c.count();
        if n == 0 || side > 1 {
            return f64::NAN;
        }
        let li = if side == 0 { 0 } else { n };
        let seg_i = if side == 0 { 0 } else { n - 1 };
        let id = c.entity(seg_i, side);
        let stored_rest = c.stored(li);
        if !delta_angle.is_finite() || delta_angle.abs() <= EPS {
            return stored_rest;
        }
        let e = &self.entities[id];
        let base = if has(e, RADIUS) && e[104].is_finite() {
            e[104]
        } else {
            0.0
        };
        let half = if layering { c.data[6] } else { 0.0 };
        // Convert motor rotation about `spool.axisLocal` into rotation about the cable plane normal.
        let axis_sign = if has(e, SPOOL) && has(e, LOCAL_AXIS) {
            let d = dot(v(e, 76), v(e, 44));
            if d < -EPS { -1.0 } else { 1.0 }
        } else {
            1.0
        };
        let plane_delta = axis_sign * delta_angle;
        let cw_eff = c.cw(li, side == 0);
        let (k_r, strained, init_f) = c.spool_config();
        let spool_physics = k_r != 0.0 || strained;
        let geom_now = if spool_physics && c.spool_ends[side].initialized {
            c.spool_ends[side].geom_stored
        } else if spool_physics {
            let r_init = c.radius(&self.entities, li, id, layering).0;
            let rho_init = spool_rho(r_init, k_r, c.data[3], strained, init_f);
            if rho_init > EPS { stored_rest / rho_init } else { stored_rest }
        } else {
            stored_rest
        };
        let shift = if c.kind(li) != 3 {
            (if cw_eff { 1.0 } else { -1.0 }) * plane_delta * (base + half)
        } else {
            let theta = stored_theta(geom_now, base, half);
            stored_delta(theta, plane_delta, cw_eff, base, half, base + half)
        };
        // In `update_attachments`, link 0 adds `shift` to stored while link `n` subtracts `shift`.
        let d_geom = if side == 0 { shift } else { -shift };
        if !spool_physics || !hybrid(c.kind(li)) {
            return stored_rest + d_geom;
        }
        let r_now = if half > EPS && base.is_finite() && geom_now > EPS {
            (base + half).max(stored_radius(geom_now, base, half).0)
        } else {
            base + half
        };
        let force = segment_tension(c, seg_i);
        let rho_now = spool_rho(r_now, k_r, c.data[3], strained, force);
        let rho_init = spool_rho(r_now, k_r, c.data[3], strained, init_f);
        let d_rest = if c.spool_ends[side].initialized {
            c.spool_ends[side].preview_rest_delta(d_geom, rho_now, rho_init)
        } else {
            let mut tmp = SpoolEnd::default();
            tmp.reset_from_stored(stored_rest, rho_init);
            tmp.preview_rest_delta(d_geom, rho_now, rho_init)
        };
        stored_rest + d_rest
    }
    /// Number of coalesced coil entries currently in the LIFO stack of spool end `side`.
    pub fn spool_coil_count(&self, index: usize, side: usize) -> usize {
        self.cables[index].spool_ends[side.min(1)].coils.len()
    }
    /// Export per-cable spool parameters and both spool ends' winding histories for checkpointing,
    /// ECS re-prepare, or Native↔WASM backend transfer.
    pub fn export_spool_state(&self, index: usize) -> Vec<f64> {
        let Some(c) = self.cables.get(index) else {
            return Vec::new();
        };
        let kr_tag = c.radial_flattening.unwrap_or(-1.0);
        let strained_tag = match c.strained_spool {
            None => -1.0,
            Some(false) => 0.0,
            Some(true) => 1.0,
        };
        let mut out = vec![
            SPOOL_STATE_VERSION,
            kr_tag,
            strained_tag,
            c.init_storage_tension,
        ];
        for end in &c.spool_ends {
            out.push(f64::from(end.initialized));
            out.push(end.geom_stored);
            out.push(end.base_geom);
            out.push(end.base_rest);
            out.push(end.coils.len() as f64);
            for &(g, r) in &end.coils {
                out.push(g);
                out.push(r);
            }
        }
        out
    }
    /// Import per-cable spool parameters and winding histories produced by `export_spool_state`.
    pub fn import_spool_state(&mut self, index: usize, data: &[f64]) -> bool {
        let Some(c) = self.cables.get_mut(index) else {
            return false;
        };
        if data.len() < 4 || data[0] != SPOOL_STATE_VERSION {
            return false;
        }
        if !data.iter().all(|v| v.is_finite()) {
            return false;
        }
        let radial_flattening = if data[1] < 0.0 { None } else { Some(data[1]) };
        let strained_spool = if data[2] < 0.0 {
            None
        } else {
            Some(data[2] != 0.0)
        };
        let init_storage_tension = data[3].max(0.0);
        let mut cursor = 4;
        let mut ends = [SpoolEnd::default(), SpoolEnd::default()];
        for end in &mut ends {
            if cursor + 5 > data.len() {
                return false;
            }
            end.initialized = data[cursor] != 0.0;
            end.geom_stored = data[cursor + 1];
            end.base_geom = data[cursor + 2];
            end.base_rest = data[cursor + 3];
            let count = data[cursor + 4];
            if count < 0.0 || count.fract() != 0.0 || count as usize > MAX_SPOOL_COILS {
                return false;
            }
            let n = count as usize;
            if end.base_geom < 0.0 && n > 0 {
                return false;
            }
            cursor += 5;
            if cursor + 2 * n > data.len() {
                return false;
            }
            end.coils = Vec::with_capacity(n);
            for k in 0..n {
                let g = data[cursor + 2 * k];
                let r = data[cursor + 2 * k + 1];
                if g <= 0.0 || r <= 0.0 {
                    return false;
                }
                end.coils.push((g, r));
            }
            cursor += 2 * n;
        }
        if cursor != data.len() {
            return false;
        }
        c.radial_flattening = radial_flattening;
        c.strained_spool = strained_spool;
        c.init_storage_tension = init_storage_tension;
        c.spool_ends = ends;
        true
    }
    /// Rejects a non-finite current position before modifying the world.
    pub fn step(&mut self, dt: f64) -> Result<(), InvalidPosition> {
        if let Some(row) = self.entities.iter().position(|e| {
            e[51] != 0.0 && !(e[0].is_finite() && e[1].is_finite() && e[2].is_finite())
        }) {
            return Err(InvalidPosition { row });
        }
        self.sync_shared(None);
        self.previous();
        self.motors(dt, false);
        self.integrate(dt);
        self.sync_bodies(false);
        let layering = self.options[3] != 0.0;
        for stage in 4..=8 {
            if stage == 5 || self.options[stage] == 0.0 {
                continue;
            }
            for p in 0..self.cables.len() {
                let cable = &mut self.cables[p];
                match stage {
                    4 => update_attachments(
                        cable,
                        &mut self.entities,
                        layering,
                        self.options[5] != 0.0,
                    ),
                    6 => merge(cable, &self.entities, layering, &mut self.topology_version),
                    7 => split(
                        cable,
                        &self.entities,
                        layering,
                        &mut self.options[10],
                        &mut self.topology_version,
                    ),
                    8 => hybrid_states(cable, &mut self.entities, layering, self.options[5] != 0.0),
                    _ => unreachable!(),
                }
                self.sync_shared(Some(p));
            }
        }
        if self
            .cables
            .iter()
            .any(|c| (0..=c.count()).any(|i| c.data[c.lo(i) + 4] == 1.0))
        {
            self.topology_version += 1;
        }
        let solver_dt = if self.options[15].is_nan() {
            dt
        } else {
            self.options[15]
        };
        self.cache();
        self.friction(solver_dt);
        self.constraints(solver_dt);
        self.elongation_hysteresis(solver_dt);
        self.overcorrections(solver_dt);
        self.sync_bodies(true);
        self.velocities(dt);
        self.motors(dt, true);
        self.encoders();
        self.diagnostics();
        Ok(())
    }
    fn sync_shared(&mut self, source: Option<usize>) {
        for owners in &self.shared {
            let selected =
                source.and_then(|p| owners.iter().rev().find(|(owner, _)| *owner == p).copied());
            let (p, i) = match selected {
                Some(owner) => owner,
                None if source.is_none() => *owners.last().unwrap(),
                _ => continue,
            };
            let o = self.cables[p].jo(i);
            let state: [f64; SEGMENT] = self.cables[p].data[o..o + SEGMENT].try_into().unwrap();
            for &(p, i) in owners {
                let o = self.cables[p].jo(i);
                self.cables[p].data[o..o + SEGMENT].copy_from_slice(&state);
            }
        }
    }
    fn previous(&mut self) {
        for e in &mut self.entities {
            if e[51] != 0.0 && e[53] != 0.0 {
                let a = v(e, 0);
                put_v(e, 7, a);
            }
            if e[52] != 0.0 && e[54] != 0.0 {
                let a = q(e, 3);
                put_q(e, 10, a);
            }
        }
    }
    fn integrate(&mut self, dt: f64) {
        let gravity = v(&self.options, 0);
        let frames: Vec<Q> = self
            .entities
            .iter()
            .map(|e| if e[54] != 0.0 { q(e, 10) } else { q(e, 3) })
            .collect();
        for e in &mut self.entities {
            if e[50] < 0.0 {
                if has(e, VEL | GRAVITY) && has(e, VEL) && has(e, GRAVITY) {
                    let vel = add(v(e, 69), scale(gravity, dt));
                    put_v(e, 69, vel);
                }
                if e[51] != 0.0 && has(e, VEL) {
                    let pos = add(v(e, 0), scale(v(e, 69), dt));
                    put_v(e, 0, pos);
                }
            }
            if e[52] == 0.0 || !has(e, OMEGA) {
                continue;
            }
            if e[50] >= 0.0 && has(e, SPOOL) {
                if !has(e, MOTOR) {
                    let omega = rotate(qnorm(conj(frames[e[50] as usize])), v(e, 72));
                    let speed = length(omega);
                    if speed > 1e-12 {
                        let old = q(e, 17);
                        let local =
                            qnorm(mul(axis_angle(scale(omega, 1.0 / speed), speed * dt), old));
                        let raw = qnorm(mul(mul(q(e, 3), qnorm(conj(old))), local));
                        put_q(e, 17, local);
                        put_q(e, 3, raw);
                    }
                }
                continue;
            }
            let omega = v(e, 72);
            let speed = length(omega);
            if speed > 1e-12 {
                let a = qnorm(mul(
                    axis_angle(scale(omega, 1.0 / speed), speed * dt),
                    q(e, 3),
                ));
                put_q(e, 3, a);
            }
        }
    }
    fn sync_bodies(&mut self, mounted_only: bool) {
        for e in &mut self.entities {
            if !mounted_only && has(e, SPOOL) && e[50] < 0.0 && e[52] != 0.0 {
                project_spool(e);
            }
        }
        for body in 0..self.entities.len() {
            if !has(&self.entities[body], BODY)
                || self.entities[body][51] == 0.0
                || self.entities[body][52] == 0.0
                || self.children[body].is_empty()
                || (mounted_only
                    && !self.children[body]
                        .iter()
                        .any(|&id| has(&self.entities[id], SPOOL)))
            {
                continue;
            }
            let current = qnorm(q(&self.entities[body], 3));
            let previous = qnorm(q(&self.entities[body], 86));
            let delta = qnorm(mul(current, qnorm(conj(previous))));
            for &id in &self.children[body] {
                if self.entities[id][52] != 0.0 {
                    let e = &mut self.entities[id];
                    let orient = qnorm(mul(delta, q(e, 3)));
                    put_q(e, 3, orient);
                    if has(e, SPOOL) {
                        let reference = qnorm(mul(delta, q(e, 79)));
                        put_q(e, 79, reference);
                    }
                    update_local(&mut self.entities, id);
                    let orient = qnorm(mul(current, q(&self.entities[id], 17)));
                    put_q(&mut self.entities[id], 3, orient);
                    if has(&self.entities[id], SPOOL) {
                        project_spool(&mut self.entities[id]);
                        update_local(&mut self.entities, id);
                    }
                }
                let (e, b) = (&self.entities[id], &self.entities[body]);
                let offset = rotate(current, v(e, 14));
                let pos = (e[51] != 0.0).then(|| add(v(b, 0), offset));
                let vel = has(e, VEL).then(|| {
                    let vel = if has(b, VEL) { v(b, 69) } else { [0.0; 3] };
                    if has(b, OMEGA) {
                        add(vel, cross(v(b, 72), offset))
                    } else {
                        vel
                    }
                });
                if let Some(pos) = pos {
                    put_v(&mut self.entities[id], 0, pos);
                }
                if let Some(vel) = vel {
                    put_v(&mut self.entities[id], 69, vel);
                }
            }
            let pos = v(&self.entities[body], 0);
            put_v(&mut self.entities[body], 83, pos);
            put_q(&mut self.entities[body], 86, current);
        }
    }
    fn cache(&mut self) {
        for id in 0..self.entities.len() {
            if !has(&self.entities[id], CABLE_LINK) || self.entities[id][51] == 0.0 {
                continue;
            }
            if let Some(p) = world_pos(&self.entities, id) {
                put_v(&mut self.entities[id], 90, p);
            }
            if let Some(a) = world_quat(&self.entities, id) {
                put_q(&mut self.entities[id], 93, a);
                let local = if self.entities[id][50] >= 0.0 {
                    q(&self.entities[id], 17)
                } else {
                    a
                };
                put_q(&mut self.entities[id], 97, local);
            }
        }
    }
    fn velocities(&mut self, dt: f64) {
        if dt <= EPS {
            return;
        }
        for id in 0..self.entities.len() {
            let mut e = self.entities[id];
            let parent = e[50];
            if parent >= 0.0 && !has(&e, SPOOL) {
                continue;
            }
            if parent < 0.0
                && e[51] != 0.0
                && e[53] != 0.0
                && has(&e, VEL)
                && has(&e, MASS)
                && e[124] > 0.0
            {
                let vel = scale(sub(v(&e, 0), v(&e, 7)), 1.0 / dt);
                put_v(&mut e, 69, vel);
            }
            if e[52] != 0.0
                && e[54] != 0.0
                && has(&e, OMEGA)
                && has(&e, INERTIA)
                && e[34..43].iter().any(|x| x.abs() > 1e-12)
            {
                let (current, previous) = if parent >= 0.0 {
                    let b = &self.entities[parent as usize];
                    (q(&e, 17), qnorm(mul(qnorm(conj(q(b, 10))), q(&e, 10))))
                } else {
                    (q(&e, 3), q(&e, 10))
                };
                let mut delta = qnorm(mul(current, qnorm(conj(previous))));
                if delta[3] < 0.0 {
                    delta = delta.map(|x| -x);
                }
                let sine = length([delta[0], delta[1], delta[2]]);
                let angle = 2.0 * atan2(sine, delta[3].clamp(-1.0, 1.0));
                let mut omega = if sine <= 1e-12 || angle <= EPS {
                    [0.0; 3]
                } else {
                    scale([delta[0], delta[1], delta[2]], angle / (dt * sine))
                };
                if parent >= 0.0 {
                    omega = rotate(q(&self.entities[parent as usize], 3), omega);
                }
                put_v(&mut e, 72, omega);
            }
            self.entities[id] = e;
        }
    }
}
fn project_spool(e: &mut Entity) {
    let angle = spool_angle(q(e, 79), q(e, 3), v(e, 76));
    let orient = spool_compose(q(e, 79), v(e, 76), angle);
    put_q(e, 3, orient);
    if has(e, OMEGA) {
        let axis = spool_axis(e, orient);
        let omega = scale(axis, dot(v(e, 72), axis));
        put_v(e, 72, omega);
    }
}

fn segment_tension(c: &Cable, i: usize) -> f64 {
    let cpl = c.data[3];
    if !(cpl.is_finite() && cpl > EPS) {
        return 0.0;
    }
    if i < c.strain.len() || c.data[c.jo(i) + 11] > 0.0 {
        c.data[c.jo(i) + 11].max(0.0)
    } else {
        let d = length(sub(c.point(i, 0), c.point(i, 1)));
        let rest = c.rest(i);
        if rest > EPS {
            (d - rest).max(0.0) / (rest * cpl)
        } else {
            0.0
        }
    }
}

fn sync_cable_spools(c: &mut Cable, es: &[Entity], layering: bool) {
    let (k_r, strained, init_f) = c.spool_config();
    let active = k_r != 0.0 || strained;
    if !active || c.count() == 0 {
        for end in &mut c.spool_ends {
            end.clear_inactive();
        }
        return;
    }
    let cpl = c.data[3];
    let n = c.count();
    for (side, li) in [(0usize, 0usize), (1usize, n)] {
        if !hybrid(c.kind(li)) {
            c.spool_ends[side].clear_inactive();
            continue;
        }
        let seg_i = if side == 0 { 0 } else { n - 1 };
        let id = c.entity(seg_i, side);
        let base_r = c.radius(es, li, id, layering).0;
        let rho_init = spool_rho(base_r, k_r, cpl, strained, init_f);
        let stored_rest = c.stored(li);
        c.spool_ends[side].sync_with_stored(stored_rest, rho_init);
    }
}

fn update_attachments(c: &mut Cable, es: &mut [Entity], layering: bool, clamp: bool) {
    let half = if layering { c.data[6] } else { 0.0 };
    sync_cable_spools(c, es, layering);
    let (k_r, strained, init_f) = c.spool_config();
    let spool_physics = k_r != 0.0 || strained;
    let cpl = c.data[3];
    let n_segs = c.count();
    for i in 0..n_segs {
        let ids = [c.entity(i, 0), c.entity(i, 1)];
        let (points, mut ends, radii) = geometry(c, i, es, layering);
        for side in 0..2 {
            let li = i + side;
            let lo = c.lo(li);
            let id = ids[side];
            if c.kind(li) != 3
                || (side == 0 && i != 0)
                || (side == 1 && i + 1 != n_segs)
                || es[id][133] == 0.0
            {
                continue;
            }
            if c.data[lo + 4] == 2.0 {
                if es[id][135] == 0.0 {
                    c.data[lo + 4] = 1.0;
                    es[id][135] += 1.0;
                } else {
                    c.data[lo + 3] = f64::NAN;
                    c.data[lo + 4] = 0.0;
                }
            }
            if c.data[lo + 3].is_finite() {
                continue;
            }
            let e = ends[side];
            if e.prev.is_none() {
                continue;
            }
            let previous = Endpoint {
                pos: e.prev,
                quat: e.prev_quat,
                frame: e.prev_frame,
                ..e
            };
            let theta = stored_theta(c.geom_stored(li), es[id][104], half).abs();
            let sign = if c.cw(li, side == 0) == (side == 0) {
                -1.0
            } else {
                1.0
            };
            c.data[lo + 3] =
                wrap(relative_angle(es, id, previous, c.point(i, side)) - sign * theta);
            c.data[lo + 4] = 1.0;
            es[id][135] += 1.0;
        }
        let mut geom_shifts = [0.0; 2];
        for side in 0..2 {
            let e = ends[side];
            let (radius, theta) = radii[side];
            let kind = c.kind(i + side);
            if !rolling(kind) || !radius.is_finite() {
                continue;
            }
            let (Some(prev), Some(pos), Some(point)) = (e.prev, e.pos, points[side]) else {
                continue;
            };
            let old = c.point(i, side);
            if length(sub(old, prev)) < 1e-4 {
                continue;
            }
            let mut old_rel = sub(old, prev);
            let mut new_rel = sub(point, pos);
            let mut normal = e.normal;
            if internal(es, ids[0], ids[1]) {
                let old_body = qnorm(mul(e.prev_quat, qnorm(conj(e.prev_frame))));
                let new_body = qnorm(mul(e.quat, qnorm(conj(e.frame))));
                old_rel = rotate(qnorm(conj(old_body)), old_rel);
                new_rel = rotate(qnorm(conj(new_body)), new_rel);
                normal = rotate(qnorm(conj(new_body)), normal);
                if dot(normal, normal) > EPS {
                    normal = norm(normal);
                }
            }
            let cw = c.cw(i + side, side == 0);
            geom_shifts[side] = arc(old_rel, new_rel, [0.0; 3], radius, cw, normal, false);
            // A wheel no motor turns follows the line; its rotation carries
            // no material (the row moves it, see `transport`).
            geom_shifts[side] += if hybrid(kind) {
                stored_delta(theta, e.delta, cw, es[ids[side]][104], half, radius)
            } else if passive_wheel(c, i + side, es) {
                0.0
            } else if cw {
                e.delta * radius
            } else {
                -e.delta * radius
            };
        }

        // Compute per-side spool constitutive conversion factors and preview rest transfers
        // BEFORE clamping so clamping enforces the minimum segment rest length in true material units
        // and never mutates the spool coil stack on rolled-back moves.
        let force = if spool_physics { segment_tension(c, i) } else { 0.0 };
        let is_end_spool = [
            spool_physics && i == 0 && hybrid(c.kind(i)),
            spool_physics && i + 1 == n_segs && hybrid(c.kind(i + 1)),
        ];
        let rho_now = [
            if is_end_spool[0] { spool_rho(radii[0].0, k_r, cpl, strained, force) } else { 1.0 },
            if is_end_spool[1] { spool_rho(radii[1].0, k_r, cpl, strained, force) } else { 1.0 },
        ];
        let rho_init = [
            if is_end_spool[0] { spool_rho(radii[0].0, k_r, cpl, strained, init_f) } else { 1.0 },
            if is_end_spool[1] { spool_rho(radii[1].0, k_r, cpl, strained, init_f) } else { 1.0 },
        ];
        let mut rest_shifts = [
            if is_end_spool[0] {
                c.spool_ends[0].preview_rest_delta(geom_shifts[0], rho_now[0], rho_init[0])
            } else {
                geom_shifts[0]
            },
            if is_end_spool[1] {
                -c.spool_ends[1].preview_rest_delta(-geom_shifts[1], rho_now[1], rho_init[1])
            } else {
                geom_shifts[1]
            },
        ];

        let rest = c.rest(i);
        if clamp && rest.is_finite() {
            let unclamped = rest - rest_shifts[0] + rest_shifts[1];
            if unclamped < 1e-6 {
                let lift = 1e-6 - unclamped;
                let a = rest_shifts[0].max(0.0);
                let b = (-rest_shifts[1]).max(0.0);
                let da_rest = if a + b > EPS {
                    lift * (a / (a + b))
                } else {
                    0.0
                };
                let db_rest = lift - da_rest;
                let target_rest = [rest_shifts[0] - da_rest, rest_shifts[1] + db_rest];
                let target_geom = [
                    if is_end_spool[0] {
                        c.spool_ends[0].geom_for_rest_delta(target_rest[0], rho_now[0], rho_init[0])
                    } else {
                        target_rest[0]
                    },
                    if is_end_spool[1] {
                        -c.spool_ends[1].geom_for_rest_delta(-target_rest[1], rho_now[1], rho_init[1])
                    } else {
                        target_rest[1]
                    },
                ];
                let corr_geom = [target_geom[0] - geom_shifts[0], target_geom[1] - geom_shifts[1]];
                geom_shifts = target_geom;
                for side in [1, 0] {
                    let shift = corr_geom[side];
                    let kind = c.kind(i + side);
                    let e = ends[side];
                    if !hybrid(kind) && !rolling(kind) {
                        continue;
                    }
                    let cw = c.cw(i + side, side == 0);
                    let (r, t) = radii[side];
                    let angle = if hybrid(kind) {
                        angle_shift(t, e.delta, cw, es[ids[side]][104], half, shift, r)
                    } else if r.abs() > EPS {
                        shift / ((if cw { 1.0 } else { -1.0 }) * r)
                    } else {
                        0.0
                    };
                    rotate_entity(es, ids[side], e.normal, angle);
                    if angle.abs() > EPS {
                        ends[side].quat = qnorm(mul(axis_angle(norm(e.normal), angle), e.quat));
                    }
                }
            }
        }

        // Commit accepted geometric shifts into the spool history and apply exact material transfers.
        rest_shifts[0] = if is_end_spool[0] {
            c.spool_ends[0].apply_geom_delta(geom_shifts[0], rho_now[0], rho_init[0])
        } else {
            geom_shifts[0]
        };
        rest_shifts[1] = if is_end_spool[1] {
            -c.spool_ends[1].apply_geom_delta(-geom_shifts[1], rho_now[1], rho_init[1])
        } else {
            geom_shifts[1]
        };
        c.set_stored(i, c.stored(i) + rest_shifts[0]);
        let jo = c.jo(i);
        c.data[jo + 3] -= rest_shifts[0];
        c.set_stored(i + 1, c.stored(i + 1) - rest_shifts[1]);
        c.data[jo + 3] += rest_shifts[1];

        // Hybrid-knot geometric synchronization: compare knot-implied geometric drum occupancy
        // against `c.geom_stored(li)` (not unstretched rest length) and transfer any geometric
        // correction consistently through the spool model so geometry and material stay coupled.
        for side in 0..2 {
            let li = i + side;
            let lo = c.lo(li);
            let id = ids[side];
            if !hybrid(c.kind(li))
                || (side == 0 && i != 0)
                || (side == 1 && i + 1 != n_segs)
            {
                continue;
            }
            let knot = c.data[lo + 3];
            let base = es[id][104];
            let (Some(point), Some(_)) = (points[side], ends[side].pos) else {
                continue;
            };
            if !knot.is_finite() || !base.is_finite() {
                continue;
            }
            let cur_geom = c.geom_stored(li);
            let theta = stored_theta(cur_geom, base, half).abs();
            let sign = if c.cw(li, side == 0) == (side == 0) {
                -1.0
            } else {
                1.0
            };
            let target = sign
                * unwrap(
                    sign * theta,
                    wrap(relative_angle(es, id, ends[side], point) - knot),
                );
            let target_geom = theta_stored(target, base, half);
            let delta_geom = target_geom - cur_geom;
            if target_geom.is_finite() && delta_geom.abs() > 1e-3 {
                if is_end_spool[side] {
                    let mut d_geom = delta_geom;
                    if clamp {
                        let d_rest_preview =
                            c.spool_ends[side].preview_rest_delta(d_geom, rho_now[side], rho_init[side]);
                        if c.data[jo + 3] - d_rest_preview < 1e-6 {
                            let max_rest = c.data[jo + 3] - 1e-6;
                            d_geom = c.spool_ends[side].geom_for_rest_delta(
                                max_rest,
                                rho_now[side],
                                rho_init[side],
                            );
                        }
                    }
                    let d_rest =
                        c.spool_ends[side].apply_geom_delta(d_geom, rho_now[side], rho_init[side]);
                    c.set_stored(li, c.stored(li) + d_rest);
                    c.data[jo + 3] -= d_rest;
                } else {
                    c.set_stored(li, target_geom);
                    c.data[jo + 3] -= delta_geom;
                }
            }
        }
        for (side, point) in points.iter().enumerate() {
            if let Some(p) = point {
                put_v(&mut c.data, jo + 4 + side * 3, *p);
            }
        }
    }
}

fn point_tangent(point: V, center: V, r: f64, n: V, cw: bool, first: bool) -> V {
    let b = basis(n);
    lift(
        circle_point(project(point, center, b), [0.0; 2], r, cw, first),
        center,
        b,
    )
}
fn sphere_tangents(a: V, ra: f64, cwa: bool, b: V, rb: f64, cwb: bool, n: V) -> (V, V) {
    let basis = basis(n);
    let offset = dot(sub(b, a), basis.n);
    let (ta, tb) = circle_circle([0.0; 2], ra, cwa, project(b, a, basis), rb, cwb);
    (
        lift(ta, a, basis),
        add(lift(tb, a, basis), scale(basis.n, offset)),
    )
}
fn transition_threshold(c: &Cable) -> f64 {
    if c.data[6] > EPS {
        (0.25 * c.data[6]).max(1e-6)
    } else {
        1e-6
    }
}
fn hybrid_states(c: &mut Cable, es: &mut [Entity], layering: bool, clamp: bool) {
    if c.count() == 0 {
        return;
    }
    let (k_r, strained, init_f) = c.spool_config();
    let spool_physics = k_r != 0.0 || strained;
    let cpl = c.data[3];
    for li in [0, c.count()] {
        let side = usize::from(li != 0);
        let i = if side == 0 { 0 } else { c.count() - 1 };
        let id = c.entity(i, side);
        let Some(center) = world_pos(es, id) else {
            continue;
        };
        let lo = c.lo(li);
        let jo = c.jo(i);
        let kind = c.kind(li);
        let geom = c.geom_stored(li);
        if kind == 3 && geom < -transition_threshold(c) {
            let stored_rest = c.stored(li);
            c.data[lo] = 4.0;
            let r = c.radius(es, li, id, layering).0;
            let n = plane(es, id);
            c.data[jo + 3] += stored_rest;
            c.set_stored(li, 0.0);
            if spool_physics {
                c.spool_ends[side].reset_empty();
            } else {
                c.spool_ends[side].clear_inactive();
            }
            if r.is_finite() && r > EPS {
                let a = -geom / r * (if c.data[lo + 1] != 0.0 { 1.0 } else { -1.0 });
                let point = add(rotate_axis(sub(c.point(i, side), center), n, a), center);
                put_v(&mut c.data, jo + 4 + 3 * side, point);
            }
        } else if kind == 4 {
            let r = c.radius(es, li, id, layering).0;
            if !r.is_finite() || r <= EPS {
                continue;
            }
            let point = c.point(i, side);
            let other = c.point(i, 1 - side);
            if length(sub(point, other)) <= (2.0 * c.data[6] + 1e-6).max(1e-6) {
                continue;
            }
            let n = plane(es, id);
            let tcw = point_tangent(other, center, r, n, true, false);
            let tccw = point_tangent(other, center, r, n, false, false);
            let acw = arc(point, tcw, center, r, true, n, false);
            let accw = arc(point, tccw, center, r, false, n, false);
            let dcw = dot(sub(point, tcw), sub(point, tcw));
            let dccw = dot(sub(point, tccw), sub(point, tccw));
            let candidate = if accw > 0.0 && dccw < dcw {
                Some((true, tccw, accw))
            } else if acw > 0.0 && dcw < dccw {
                Some((false, tcw, acw))
            } else {
                None
            };
            if let Some((cw, p, geom_arc)) = candidate {
                if geom_arc <= transition_threshold(c) {
                    continue;
                }
                if spool_physics {
                    let force = segment_tension(c, i);
                    let rho_now = spool_rho(r, k_r, cpl, strained, force);
                    let rho_init = spool_rho(r, k_r, cpl, strained, init_f);
                    c.spool_ends[side].sync_with_stored(c.stored(li), rho_init);
                    let d_geom = geom_arc - c.geom_stored(li);
                    let d_rest_preview =
                        c.spool_ends[side].preview_rest_delta(d_geom, rho_now, rho_init);
                    // Reject the rewrap cleanly if absorbing the tangent arc would violate the
                    // minimum free-span rest-length invariant, leaving link state and coil stack untouched.
                    if clamp && c.data[jo + 3].is_finite() && c.data[jo + 3] - d_rest_preview < 1e-6
                    {
                        continue;
                    }
                    c.data[lo] = 3.0;
                    c.data[lo + 1] = f64::from(cw);
                    let d_rest = c.spool_ends[side].apply_geom_delta(d_geom, rho_now, rho_init);
                    c.data[jo + 3] -= d_rest;
                    c.set_stored(li, c.stored(li) + d_rest);
                } else {
                    let d_rest = geom_arc - c.stored(li);
                    if clamp && c.data[jo + 3].is_finite() && c.data[jo + 3] - d_rest < 1e-6 {
                        continue;
                    }
                    c.data[lo] = 3.0;
                    c.data[lo + 1] = f64::from(cw);
                    c.data[jo + 3] -= d_rest;
                    c.set_stored(li, geom_arc);
                }
                put_v(&mut c.data, jo + 4 + side * 3, p);
                if c.data[lo + 4] == 2.0 {
                    if es[id][135] == 0.0 {
                        c.data[lo + 4] = 1.0;
                        es[id][135] += 1.0;
                    } else {
                        c.data[lo + 3] = f64::NAN;
                        c.data[lo + 4] = 0.0;
                    }
                }
                if !c.data[lo + 3].is_finite() {
                    let e = endpoint(es, id, c.entity(i, 1 - side));
                    let half = if layering { c.data[6] } else { 0.0 };
                    let theta = stored_theta(c.geom_stored(li), es[id][104], half).abs();
                    let sign = if c.cw(li, side == 0) == (side == 0) {
                        -1.0
                    } else {
                        1.0
                    };
                    c.data[lo + 3] = wrap(relative_angle(es, id, e, p) - sign * theta);
                    c.data[lo + 4] = 1.0;
                    es[id][135] += 1.0;
                    es[id][133] = 1.0;
                }
            }
        }
    }
}
fn path_radius(
    c: &Cable,
    es: &[Entity],
    id: usize,
    preferred: Option<usize>,
    layering: bool,
) -> f64 {
    let mut r = es[id][104];
    if !has(&es[id], RADIUS) || r <= EPS {
        return r;
    }
    if let Some(li) = preferred {
        r = r.max(c.radius(es, li, id, layering).0);
    }
    for li in 0..=c.count() {
        let match_id = if li == 0 {
            c.entity(0, 0) == id
        } else if li == c.count() {
            c.entity(li - 1, 1) == id
        } else {
            c.entity(li - 1, 1) == id && c.entity(li, 0) == id
        };
        if match_id {
            r = r.max(c.radius(es, li, id, layering).0);
        }
    }
    r
}
fn rebuild(c: &mut Cable, links: Vec<[f64; LINK]>, segments: Vec<[f64; SEGMENT]>) {
    c.data.truncate(HEADER);
    c.data[0] = segments.len() as f64;
    for link in links {
        c.data.extend_from_slice(&link);
    }
    for segment in segments {
        c.data.extend_from_slice(&segment);
    }
}
fn records(c: &Cable) -> (Vec<[f64; LINK]>, Vec<[f64; SEGMENT]>) {
    (
        (0..=c.count())
            .map(|i| c.data[c.lo(i)..c.lo(i) + LINK].try_into().unwrap())
            .collect(),
        (0..c.count())
            .map(|i| c.data[c.jo(i)..c.jo(i) + SEGMENT].try_into().unwrap())
            .collect(),
    )
}
fn merge(c: &mut Cable, es: &[Entity], layering: bool, version: &mut u32) {
    let mut rerun = true;
    while rerun {
        rerun = false;
        let mut i = 0;
        while i + 1 < c.count() {
            if c.kind(i + 1) != 1
                || c.stored(i + 1) >= 0.0
                || c.entity(i, 1) != c.entity(i + 1, 0)
                || (!layering && c.entity(i, 0) == c.entity(i + 1, 1))
            {
                i += 1;
                continue;
            }
            let a = c.entity(i, 0);
            let b = c.entity(i + 1, 1);
            let (Some(pa), Some(pb)) = (world_pos(es, a), world_pos(es, b)) else {
                i += 1;
                continue;
            };
            let ra = path_radius(c, es, a, Some(i), layering);
            let rb = path_radius(c, es, b, Some(i + 2), layering);
            let olda = c.point(i, 0);
            let oldb = c.point(i + 1, 1);
            let ka = c.kind(i);
            let kb = c.kind(i + 2);
            let cwa = c.cw(i, true);
            let cwb = c.data[c.lo(i + 2) + 1] != 0.0;
            let (mut ta, mut tb) = (olda, oldb);
            if rolling(ka) && rolling(kb) {
                (ta, tb) = sphere_tangents(pa, ra, cwa, pb, rb, cwb, plane(es, a));
            } else if rolling(ka) && attachment_kind(kb) {
                ta = point_tangent(oldb, pa, ra, plane(es, a), cwa, false);
            } else if attachment_kind(ka) && rolling(kb) {
                tb = point_tangent(olda, pb, rb, plane(es, b), cwb, true);
            }
            let sa = if rolling(ka) {
                arc(olda, ta, pa, ra, cwa, plane(es, a), false)
            } else {
                0.0
            };
            let sb = if rolling(kb) {
                arc(oldb, tb, pb, rb, cwb, plane(es, b), false)
            } else {
                0.0
            };
            let (mut links, mut joints) = records(c);
            joints[i][3] += joints[i + 1][3] + links[i + 1][2];
            joints[i][2] = b as f64;
            links[i][2] += sa;
            joints[i][3] -= sa;
            links[i + 2][2] -= sb;
            joints[i][3] += sb;
            rerun = links[i][2] < 0.0 || links[i + 2][2] < 0.0;
            put_v(&mut joints[i], 4, ta);
            put_v(&mut joints[i], 7, tb);
            joints.remove(i + 1);
            links.remove(i + 1);
            rebuild(c, links, joints);
            *version += 1;
            i += 1;
        }
    }
}
fn intersects(a: V, b: V, p: V, r: f64) -> bool {
    if length(sub(a, p)) <= r || length(sub(b, p)) <= r {
        return false;
    }
    let d = sub(b, a);
    let dl = dot(d, d);
    let mut t = dot(sub(p, a), d);
    if dl > EPS {
        t /= dl;
    }
    (0.0..=1.0).contains(&t) && length(sub(add(a, scale(d, t)), p)) <= r
}
fn split(c: &mut Cable, es: &[Entity], layering: bool, next_id: &mut f64, version: &mut u32) {
    let mut i = 0;
    while i < c.count() {
        let pa = c.point(i, 0);
        let pb = c.point(i, 1);
        for (id, e) in es.iter().enumerate() {
            if !has(e, CABLE_LINK)
                || !has(e, RADIUS)
                || e[51] == 0.0
                || e[129] != c.data[8]
                || id == c.entity(i, 0)
                || id == c.entity(i, 1)
            {
                continue;
            }
            let Some(center) = world_pos(es, id) else {
                continue;
            };
            let r = path_radius(c, es, id, None, layering);
            if !intersects(pa, pb, center, r) {
                continue;
            }
            let a = c.entity(i, 0);
            let b = c.entity(i, 1);
            let ka = c.kind(i);
            let kb = c.kind(i + 1);
            let (Some(ca), Some(cb)) = (world_pos(es, a), world_pos(es, b)) else {
                continue;
            };
            let ra = path_radius(c, es, a, Some(i), layering);
            let rb = path_radius(c, es, b, Some(i + 1), layering);
            let normal = plane(es, id);
            let cw = dot(cross(sub(pb, pa), sub(v(e, 90), pa)), normal) < 0.0;
            let cwa = c.cw(i, true);
            let cwb = c.data[c.lo(i + 1) + 1] != 0.0;
            let (ta, ts) = if rolling(ka) {
                sphere_tangents(ca, ra, cwa, center, r, cw, plane(es, a))
            } else if attachment_kind(ka) {
                (pa, point_tangent(pa, center, r, normal, cw, true))
            } else {
                continue;
            };
            let (st, tb) = if rolling(kb) {
                sphere_tangents(center, r, cw, cb, rb, cwb, plane(es, b))
            } else if attachment_kind(kb) {
                (point_tangent(pb, center, r, normal, cw, false), pb)
            } else {
                continue;
            };
            let sa = if rolling(ka) {
                arc(pa, ta, ca, ra, cwa, plane(es, a), false)
            } else {
                0.0
            };
            let sb = if rolling(kb) {
                arc(pb, tb, cb, rb, cwb, plane(es, b), false)
            } else {
                0.0
            };
            let s = arc(ts, st, center, r, cw, normal, false);
            if s <= 0.0 || s + EPS >= TAU * r {
                continue;
            }
            let da = length(sub(ta, ts));
            let db = length(sub(st, tb));
            let avail = c.rest(i) + sb - sa - s;
            if da + db > EPS && avail < EPS {
                continue;
            }
            let (mut links, mut joints) = records(c);
            let mut new = [0.0; SEGMENT];
            new[0] = *next_id;
            *next_id += 1.0;
            new[1] = id as f64;
            new[2] = b as f64;
            new[3] = if da + db > EPS {
                avail * db / (da + db)
            } else {
                0.0
            };
            put_v(&mut new, 4, st);
            put_v(&mut new, 7, tb);
            joints[i][2] = id as f64;
            joints[i][3] = if da + db > EPS {
                avail * da / (da + db)
            } else {
                0.0
            };
            put_v(&mut joints[i], 4, ta);
            put_v(&mut joints[i], 7, ts);
            links[i][2] += sa;
            links[i + 1][2] -= sb;
            links.insert(i + 1, [1.0, f64::from(cw), s, f64::NAN, 0.0]);
            joints.insert(i + 1, new);
            rebuild(c, links, joints);
            *version += 1;
        }
        i += 1;
    }
}

pub(super) fn inverse_tensor(m: [f64; 9]) -> [f64; 9] {
    let max = m.iter().map(|x| x.abs()).fold(0.0, f64::max);
    if max == 0.0 {
        return [0.0; 9];
    }
    let mut a = m.map(|x| x / max);
    let mut vectors = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    for _ in 0..32 {
        let (mut p, mut q) = (0, 1);
        for (i, j) in [(0, 2), (1, 2)] {
            if a[i * 3 + j].abs() > a[p * 3 + q].abs() {
                (p, q) = (i, j);
            }
        }
        if a[p * 3 + q].abs() <= f64::EPSILON {
            break;
        }
        let tau = (a[q * 3 + q] - a[p * 3 + p]) / (2.0 * a[p * 3 + q]);
        let t = (if tau >= 0.0 { 1.0 } else { -1.0 }) / (tau.abs() + length2([1.0, tau]));
        let c = 1.0 / length2([1.0, t]);
        let s = t * c;
        let off = a[p * 3 + q];
        a[p * 3 + p] -= t * off;
        a[q * 3 + q] += t * off;
        a[p * 3 + q] = 0.0;
        a[q * 3 + p] = 0.0;
        for k in 0..3 {
            if k != p && k != q {
                let kp = a[k * 3 + p];
                let kq = a[k * 3 + q];
                a[k * 3 + p] = c * kp - s * kq;
                a[p * 3 + k] = a[k * 3 + p];
                a[k * 3 + q] = s * kp + c * kq;
                a[q * 3 + k] = a[k * 3 + q];
            }
            let vp = vectors[k * 3 + p];
            let vq = vectors[k * 3 + q];
            vectors[k * 3 + p] = c * vp - s * vq;
            vectors[k * 3 + q] = s * vp + c * vq;
        }
    }
    let tolerance = f64::EPSILON * 3.0 * a[0].abs().max(a[4].abs()).max(a[8].abs());
    let mut out = [0.0; 9];
    for k in 0..3 {
        let inv = if a[k * 3 + k] > tolerance {
            1.0 / (a[k * 3 + k] * max)
        } else {
            0.0
        };
        for r in 0..3 {
            for c in 0..3 {
                out[r * 3 + c] += vectors[r * 3 + k] * inv * vectors[c * 3 + k];
            }
        }
    }
    out
}
impl PhysicsWorld {
    fn aggregate_inertia(&self, body: usize) -> Entity {
        let mut out = self.entities[body];
        let mut tensor = [0.0; 9];
        let mut contributed = false;
        for &id in &self.children[body] {
            let e = &self.entities[id];
            if has(e, INERTIA) {
                let [x, y, z, w] = qnorm(q(e, 17));
                let (x2, y2, z2) = (x + x, y + y, z + z);
                let (xx, xy, xz, yy, yz, zz, wx, wy, wz) = (
                    x * x2,
                    x * y2,
                    x * z2,
                    y * y2,
                    y * z2,
                    z * z2,
                    w * x2,
                    w * y2,
                    w * z2,
                );
                // Columns of the reference rotation matrix, without three
                // quaternion/vector transforms and their extra rounding.
                let axes = [
                    [1.0 - yy - zz, xy + wz, xz - wy],
                    [xy - wz, 1.0 - xx - zz, yz + wx],
                    [xz + wy, yz - wx, 1.0 - xx - yy],
                ];
                // R I R^T, retaining the JS matrix multiplication order.
                let mut ri = [0.0; 9];
                for r in 0..3 {
                    for c in 0..3 {
                        ri[r * 3 + c] = axes[0][r] * e[25 + c]
                            + axes[1][r] * e[28 + c]
                            + axes[2][r] * e[31 + c];
                    }
                }
                for r in 0..3 {
                    for c in 0..3 {
                        tensor[r * 3 + c] += ri[r * 3] * axes[0][c]
                            + ri[r * 3 + 1] * axes[1][c]
                            + ri[r * 3 + 2] * axes[2][c];
                    }
                }
                contributed = true;
            }
            let mass = if e[106].is_finite() { e[106] } else { e[124] };
            if mass > 0.0 {
                let offset = v(e, 14);
                let d = dot(offset, offset);
                for r in 0..3 {
                    for c in 0..3 {
                        tensor[r * 3 + c] += if r == c {
                            mass * (d - offset[r] * offset[c])
                        } else {
                            -mass * offset[r] * offset[c]
                        };
                    }
                }
                contributed = true;
            }
        }
        if contributed {
            out[25..34].copy_from_slice(&tensor);
            out[34..43].copy_from_slice(&inverse_tensor(tensor));
        }
        out
    }
    fn reaction(&mut self, id: usize, axis: V, value: f64, dt: f64, rotation: bool, rotor: f64) {
        let parent = self.entities[id][50];
        if parent < 0.0 || value.abs() <= 1e-12 {
            return;
        }
        let body = parent as usize;
        let moi = self.aggregate_inertia(body);
        if rotation {
            let inertia = axis_inertia(&moi, q(&moi, 3), axis);
            if self.entities[body][52] != 0.0 && inertia > 1e-12 && rotor > 1e-12 {
                let orient = qnorm(mul(
                    axis_angle(axis, -value * (rotor / inertia)),
                    q(&self.entities[body], 3),
                ));
                put_q(&mut self.entities[body], 3, orient);
            }
        } else if has(&self.entities[body], OMEGA) {
            let omega = add(
                v(&self.entities[body], 72),
                world_inverse(&moi, scale(axis, -value * dt)),
            );
            put_v(&mut self.entities[body], 72, omega);
        }
    }
    fn motors(&mut self, dt: f64, torque_mode: bool) {
        for id in 0..self.entities.len() {
            let e = &self.entities[id];
            if !has(e, MOTOR)
                || !has(e, SPOOL)
                || !has(e, OMEGA)
                || !has(e, INERTIA)
                || e[52] == 0.0
                || (e[57] != 0.0) != torque_mode
            {
                continue;
            }
            let e = *e;
            let (orientation, reference, local) = spool_frame(&self.entities, id);
            let angle = spool_angle(reference, local, v(&e, 76));
            let axis = spool_axis(&e, orientation);
            let omega = dot(v(&e, 72), axis);
            let inertia = axis_inertia(&e, orientation, axis);
            let closed = e[113] != 0.0 || self.options[14] != 0.0;
            if inertia <= if torque_mode { 0.0 } else { 1e-12 } {
                continue;
            }
            let valid_dt = dt.is_finite() && dt > 0.0;
            let target = e[107] - e[108];
            let housing = if e[50] >= 0.0 {
                let moi = self.aggregate_inertia(e[50] as usize);
                dot(axis, world_inverse(&moi, axis)).max(0.0)
            } else {
                0.0
            };
            // Angle per torque over one step for the rotor against its housing.
            let compliance = dt * dt * (1.0 / inertia + housing);
            // Position modes give a torque and its stiffness ∂τ/∂θ, which the
            // spool's step and the cable solve both treat implicitly.
            let (drive, stiffness) = if torque_mode {
                self.entities[id][136] = 0.0;
                (0.0, 0.0)
            } else if closed {
                self.servo(id, target, angle, inertia, compliance, dt)
            } else {
                self.entities[id][136] = 0.0;
                // Open loop: the rotor sits in a detent of the stator field and
                // skips to the next one when pulled past its holding torque.
                // The detent torque is averaged over the electrical phase the
                // rotor sweeps this step: sampled once per step, a slipping
                // rotor would alias the field into a net drive and spin away.
                let from = e[110] * wrap(angle - target);
                let sweep = if valid_dt { e[110] * omega * dt } else { 0.0 };
                let (mean, slope) = if sweep.abs() > 1e-6 {
                    let to = from + sweep;
                    ((cos(from) - cos(to)) / sweep, (sin(to) - sin(from)) / sweep)
                } else {
                    (sin(from), cos(from))
                };
                (
                    -e[109] * mean - e[111] * omega,
                    (e[109] * e[110] * slope).max(0.0),
                )
            };
            let drive = if !torque_mode {
                drive
            } else {
                let speed = e[112].max(1e-6);
                let droop = (1.0 - omega.abs() / speed).clamp(0.0, 1.0);
                let electric = e[114] * droop;
                let damping = -(e[109] / speed) * omega;
                let windage = -(if e[115].is_nan() {
                    e[111] * 1e-3
                } else {
                    e[115]
                }) * omega
                    * omega.abs();
                let smooth = omega / (omega.abs() + 1e-3);
                let coulomb = if e[116].is_nan() {
                    0.002 * e[109]
                } else {
                    e[116]
                };
                let stiction = if e[117].is_nan() {
                    0.003 * e[109]
                } else {
                    e[117]
                };
                let speed = if e[118].is_nan() { 1.0 } else { e[118] };
                let friction = -(coulomb + stiction * (-omega.abs() / speed).exp()) * smooth;
                let amp = if e[119].is_nan() {
                    0.01 * e[109]
                } else {
                    e[119]
                };
                let freq = if e[120].is_nan() { e[110] } else { e[120] };
                let cogging = -amp * sin(freq * angle);
                electric + damping + windage + friction + cogging
            };
            let delta = if torque_mode && e[60] > 0.0 && dt.is_finite() && dt > 0.0 {
                let spring = e[59] + e[58] * omega;
                (omega + (dt / inertia) * (drive + spring))
                    / (1.0 + dt * e[58] / inertia + dt * dt * e[60] / inertia)
                    - omega
            } else if torque_mode {
                (drive + e[59]) / inertia * dt
            } else {
                // Linearly implicit Euler on the rotor's angle relative to its
                // housing: the cable solve sees the same stiffness (see
                // `constraints`), so steady torque balance is exact, and a stiff
                // motor stays stable even when its housing is a light body.
                self.entities[id][47] = if valid_dt { stiffness } else { 0.0 };
                drive / (1.0 + stiffness * compliance) / inertia * dt
            };
            let next = add(v(&self.entities[id], 72), scale(axis, delta));
            put_v(&mut self.entities[id], 72, next);
            // The housing on a moving body takes the torque the rotor received.
            let applied = if torque_mode || !valid_dt { drive } else { delta * inertia / dt };
            self.reaction(id, axis, applied, dt, false, inertia);
            if e[50] >= 0.0 {
                let new = spool_compose(reference, v(&e, 76), angle + dot(next, axis) * dt);
                put_q(&mut self.entities[id], 17, new);
                let orient = qnorm(mul(q(&self.entities[e[50] as usize], 3), new));
                put_q(&mut self.entities[id], 3, orient);
            }
        }
    }
    /// A closed-loop stepper: a current-limited PID servo on the multi-turn
    /// encoder angle, tuned from the motor itself. Full current (the holding
    /// torque) is reached about one electrical radian off target, the rotor
    /// alone is critically damped, and the integrator pulls a loaded motor back
    /// to target. Overpowered, it yields and later returns: it never loses
    /// steps. Returns the torque and its stiffness ∂τ/∂θ.
    ///
    /// The current limit applies to the torque the servo ends the implicit
    /// step with, assuming the cables keep last step's torque on the spool,
    /// not to the explicit command. Then the rotor's velocity change is
    /// continuous where the limit engages. Testing the explicit command
    /// instead lets a strong servo (stiffness · compliance ≳ 4, e.g. 1 Nm at
    /// 2 ms) jump across its linear band in one step and limit-cycle between
    /// the two current limits.
    fn servo(
        &mut self,
        id: usize,
        target: f64,
        angle: f64,
        inertia: f64,
        compliance: f64,
        dt: f64,
    ) -> (f64, f64) {
        let encoded = self.entities[id][67] != 0.0 && self.entities[id][66].is_finite();
        if self.entities[id][136] == 0.0 {
            // Engaging calibrates the encoder to the step position the rotor
            // holds, to the nearest electrical cycle, then tracks it multi-turn.
            let e = &mut self.entities[id];
            let cycle = TAU / (e[110] + 0.5).floor().abs().max(1.0);
            e[127] = if encoded { ((e[66] - target) / cycle + 0.5).floor() * cycle } else { 0.0 };
            e[130] = 0.0;
            e[132] = 0.0;
            e[136] = 1.0;
        }
        let e = &self.entities[id];
        let measured = if encoded { e[66] - e[127] } else { unwrap(target, angle) };
        let error = target - measured;
        let limit = e[109].max(0.0);
        let kp = limit * e[110];
        let kd = 2.0 * (kp * inertia).sqrt();
        let ki = kp / SERVO_INTEGRAL_TIME;
        let valid_dt = dt.is_finite() && dt > 0.0;
        let rate = if valid_dt { (error - e[132]) / dt } else { 0.0 };
        let command = kp * error + e[130] + kd * rate;
        let stiffness = if valid_dt { kp + kd / dt } else { 0.0 };
        let spring = if valid_dt { stiffness * compliance } else { 0.0 };
        let end = (command - spring * e[59]) / (1.0 + spring);
        let saturated = end.abs() > limit;
        let e = &mut self.entities[id];
        // Integrate only while the current is not limited (anti-windup).
        if !saturated && valid_dt {
            e[130] = (e[130] + ki * error * dt).clamp(-limit, limit);
        }
        e[132] = error;
        if saturated {
            (limit.copysign(end), 0.0)
        } else {
            (command, stiffness)
        }
    }
    fn encoders(&mut self) {
        for id in 0..self.entities.len() {
            let e = &self.entities[id];
            if e[52] == 0.0 || e[67] == 0.0 {
                continue;
            }
            let e = *e;
            let (axis, angle) = if has(&e, SPOOL) {
                let (orientation, reference, local) = spool_frame(&self.entities, id);
                (
                    spool_axis(&e, orientation),
                    spool_angle(reference, local, v(&e, 76)),
                )
            } else {
                let mut axis = v(&e, 121);
                if dot(axis, axis) <= 1e-12 {
                    axis = v(&self.options, 16);
                }
                axis = norm(axis);
                (
                    axis,
                    orientation_angle(
                        world_quat(&self.entities, id).unwrap_or(q(&e, 3)),
                        axis,
                        1e-12,
                    ),
                )
            };
            // Like a fast-sampled encoder: follow the shaft pose turn by turn.
            if angle.is_finite() {
                self.entities[id][66] = unwrap(e[66], angle);
                put_v(&mut self.entities[id], 121, axis);
            }
        }
    }
    fn diagnostics(&mut self) {
        for id in 0..self.entities.len() {
            let e = &self.entities[id];
            if !has(e, MOTOR) || !has(e, SPOOL) {
                continue;
            }
            let e = *e;
            // 125 counts slips (electrical cycles the rotor fell into another
            // detent, in either direction); 126 is the signed net offset.
            let old = if e[125].is_finite() {
                (e[125] + 0.5).floor().max(0.0)
            } else {
                0.0
            };
            // Torque mode and closed-loop servos do not lose steps.
            if e[57] != 0.0 || e[113] != 0.0 || self.options[14] != 0.0 {
                self.entities[id][126] = 0.0;
                self.entities[id][125] = old;
                continue;
            }
            let target = e[107] - e[108];
            let measured = if e[67] != 0.0 && e[66].is_finite() {
                if !e[127].is_finite() {
                    self.entities[id][127] = ((e[66] - target) / TAU + 0.5).floor() * TAU;
                }
                e[66] - self.entities[id][127]
            } else {
                let (_, reference, local) = spool_frame(&self.entities, id);
                unwrap(target, spool_angle(reference, local, v(&e, 76)))
            };
            let pairs = (e[110] + 0.5).floor().abs().max(1.0);
            let offset = ((measured - target) / (TAU / pairs) + 0.5).floor();
            let previous = if e[126].is_finite() { e[126] } else { 0.0 };
            self.entities[id][126] = offset;
            self.entities[id][125] = old + (offset - previous).abs();
        }
    }
    /// Capstan ratio of every interior link and its state for this step's
    /// first solve. Frictionless links are FREE and join their segments into
    /// one row, as do links slipping at the capstan limit; sticking links and
    /// attachments separate rows. A frictional link starts from the state it
    /// ended the last step with; `transport` then checks that state against
    /// the solution.
    ///
    /// A wheel with a worn bearing holds a tension difference of its friction
    /// torque and inertial reaction over its radius, taken against the lower
    /// of its spans' tensions when the last step ended, up to what the line's
    /// capstan friction on the wheel can carry.
    fn friction(&mut self, dt: f64) {
        let layering = self.options[3] != 0.0;
        for c in &mut self.cables {
            let n = c.count();
            if c.slip.len() != n + 1 {
                c.slip = vec![STICK; n + 1];
                c.wheels = vec![None; n + 1];
            }
            if c.strain.len() != n {
                c.strain = (0..n)
                    .map(|i| {
                        let d = length(sub(c.point(i, 0), c.point(i, 1)));
                        if d > EPS { (d - c.rest(i)).max(0.0) / d } else { 0.0 }
                    })
                    .collect();
            }
            c.ratio = vec![f64::INFINITY; n + 1];
            for li in 1..n {
                let Some(ratio) = capstan(c, li, &self.entities, layering) else {
                    c.slip[li] = STICK;
                    continue;
                };
                c.ratio[li] = ratio;
                let id = c.entity(li - 1, 1);
                let e = &self.entities[id];
                if passive_wheel(c, li, &self.entities) && has(e, BEARING) {
                    let radius = c.radius(&self.entities, li, id, layering).0;
                    let axis = norm(v(e, 44));
                    let inertia = positive_inertia(dot(axis, matrix_vec(e, 25, axis)));
                    let tension = c.strain[li - 1].min(c.strain[li]) / c.data[3];
                    let mut wheel = c.wheels[li].unwrap_or_default();
                    let toward = match c.slip[li] {
                        LEFT => 1.0,
                        RIGHT => -1.0,
                        _ if wheel.travel[0] != 0.0 => wheel.travel[0].signum(),
                        _ => 0.0,
                    };
                    let step = if dt.is_finite() && dt > EPS { dt * dt } else { f64::INFINITY };
                    let force = (e[137].max(0.0) / radius
                        + toward * inertia * (wheel.travel[0] - wheel.travel[1]) / (radius * radius * step))
                        .max(0.0);
                    let bearing = if tension > EPS { 1.0 + force / tension } else { f64::INFINITY };
                    wheel.grip = bearing <= ratio;
                    wheel.drive = radius * radius * (ratio - 1.0) * tension * dt * dt / inertia;
                    wheel.brake = e[137].max(0.0) * radius * dt * dt / inertia;
                    c.ratio[li] = ratio.min(bearing);
                    c.wheels[li] = Some(wheel);
                } else {
                    c.wheels[li] = None;
                }
                if c.ratio[li] <= 1.0 + 1e-12 {
                    c.slip[li] = FREE;
                } else if c.slip[li] == FREE {
                    c.slip[li] = STICK;
                }
            }
            c.tried = c.slip.iter().map(|&state| tried(state)).collect();
        }
    }
    fn constraints(&mut self, dt: f64) {
        let valid_dt = dt.is_finite() && dt > EPS;
        for id in 0..self.entities.len() {
            let es = &self.entities;
            let e = &es[id];
            let parent = e[50];
            let prev = if parent >= 0.0 {
                qnorm(mul(qnorm(conj(q(&es[parent as usize], 10))), q(e, 10)))
            } else {
                q(e, 10)
            };
            let spin = if parent >= 0.0 { e[54] * es[parent as usize][54] } else { e[54] };
            let inverse_mass = if e[124] > 0.0 { 1.0 / e[124] } else { 0.0 };
            let motor = has(e, MOTOR) && e[57] == 0.0;
            let closed = e[113] != 0.0 || self.options[14] != 0.0;
            // The motor's linearization from motors(), which scales its torque
            // by the same effective inertia.
            let stiffness = if motor { e[47] } else { 0.0 };
            // Beyond its torque a motor yields: an open-loop rotor skips
            // detents, a closed-loop one is overpowered until the load drops.
            let limit = if !motor {
                f64::INFINITY
            } else if closed {
                e[109].max(0.0)
            } else {
                e[109].max(0.0) * self.options[13].max(0.0)
            };
            let e = &mut self.entities[id];
            for slot in [58, 59, 60, 63, 64, 65, 68] {
                e[slot] = 0.0;
            }
            e[43] = inverse_mass;
            put_q(e, 21, prev);
            e[62] = spin;
            e[47] = if dt > EPS { stiffness } else { 0.0 };
            e[48] = stiffness * dt * dt;
            e[61] = limit;
        }
        // A frictional link's state is an assumption of the solve. When the
        // solution contradicts it, the solve is repeated from the same
        // predicted state with the corrected assumption.
        let frictional = self
            .cables
            .iter()
            .any(|c| (1..c.count()).any(|li| c.slip[li] != FREE && c.ratio[li].is_finite()));
        if frictional {
            self.snapshot.clone_from(&self.entities);
        }
        let mut firsts = Vec::with_capacity(self.cables.len());
        for attempt in 0.. {
            self.joints.clear();
            self.rows.clear();
            firsts.clear();
            for c in self.cables.iter_mut() {
                let first = self.joints.len();
                firsts.push(first);
                let count = c.count();
                // Row boundaries and each segment's share of its row tension.
                let mut weights = vec![1.0; count];
                for i in 1..count {
                    weights[i] = match c.slip[i] {
                        FREE => weights[i - 1],
                        RIGHT => weights[i - 1] * c.ratio[i],
                        LEFT => weights[i - 1] / c.ratio[i],
                        _ => 1.0,
                    };
                }
                c.merged = vec![false; count];
                let mut start = 0;
                for end in 1..=count {
                    if end < count && (c.slip[end] == FREE || c.slip[end] == LEFT || c.slip[end] == RIGHT) {
                        continue;
                    }
                    // A span's spring constant is EA over its length, as in the
                    // firmware's springKPerUnitLength / distance.
                    let cpl = c.data[3];
                    let zero = cpl == f64::INFINITY;
                    let span = |i: usize| length(sub(c.point(i, 0), c.point(i, 1)));
                    let weighted: f64 = (start..end).map(|i| weights[i] * span(i)).sum();
                    let spans: f64 = (start..end).map(span).sum();
                    let compliance = if zero { 0.0 } else { cpl * weighted };
                    let b = if c.data[4].is_finite() { c.data[4].max(0.0) } else { 0.0 };
                    let damping = if spans > EPS { b / spans } else { 0.0 };
                    for i in start..end {
                        c.merged[i] = end - start > 1;
                    }
                    self.rows.push(Row {
                        start: first + start,
                        end: first + end,
                        iterations: c.data[5] as usize,
                        zero,
                        alpha: if !zero && valid_dt { compliance / (dt * dt) } else { 0.0 },
                        gamma: if !zero && valid_dt { (cpl * b / dt).max(0.0) } else { 0.0 },
                        damping_step: if valid_dt { damping * dt } else { 0.0 },
                        inv_dt2: if valid_dt { 1.0 / (dt * dt) } else { 0.0 },
                        spring: if compliance > EPS && compliance.is_finite() { 1.0 / compliance } else { 0.0 },
                        damping,
                    });
                    start = end;
                }
                for (i, &weight) in weights.iter().enumerate() {
                    let mut j = [0.0; J];
                    let a = c.entity(i, 0);
                    let b = c.entity(i, 1);
                    let mapped = [
                        mapped_id(&self.entities, a, b),
                        mapped_id(&self.entities, b, a),
                    ];
                    j[..4].copy_from_slice(&[a as f64, b as f64, mapped[0] as f64, mapped[1] as f64]);
                    put_v(&mut j, 4, local_point(&self.entities, a, c.point(i, 0)));
                    put_v(&mut j, 7, local_point(&self.entities, b, c.point(i, 1)));
                    j[10] = c.rest(i);
                    j[38] = weight;
                    for slot in [13, 18, 29, 30] {
                        j[slot] = -1.0;
                    }
                    // A spool or free wheel at a row boundary turns with the
                    // material its segment winds on or off. Inside a row a free
                    // wheel passes material on instead (see `transport`).
                    for side in 0..2 {
                        let id = if side == 0 { a } else { b };
                        let li = i + side;
                        if !has(&self.entities[id], SPOOL)
                            || !candidate(&self.entities[id])
                            || c.slip[li] != STICK
                            || passive_wheel(c, li, &self.entities)
                        {
                            continue;
                        }
                        let so = if side == 0 { 13 } else { 18 };
                        j[so] = id as f64;
                        let mut stored = 0.0;
                        if hybrid(c.kind(li)) {
                            let r = c.radius(&self.entities, li, id, self.options[3] != 0.0).0;
                            if r > EPS {
                                let mut r_eff = r;
                                if (i == 0 && side == 0) || (i + 1 == count && side == 1) {
                                    let (k_r, _, _) = c.spool_config();
                                    if k_r != 0.0 {
                                        // Instantaneous liftoff running radius r_eff(F_current).
                                        // Small-strain approximation to dC/dtheta during historical
                                        // coil unwinding: error is O(F/EA + k_r*|F_L - F_S|/R) (<0.3%
                                        // for braided Dyneema/Vectran in the linear-elastic regime).
                                        let force = segment_tension(c, i);
                                        r_eff = (r - flattening_at(k_r, r) * force).max(0.5 * r);
                                    }
                                }
                                stored = (if c.cw(li, side == 0) { 1.0 } else { -1.0 }) * r_eff;
                                if side == 1 {
                                    stored = -stored;
                                }
                            }
                        }
                        j[23 + side] = stored;
                        if stored.abs() > EPS {
                            j[25 + side] = 7.0;
                        }
                    }
                    if c.kind(i) == 2 && i > 0 {
                        j[29] = (first + i - 1) as f64;
                    }
                    if c.kind(i + 1) == 2 && i + 1 < count {
                        j[30] = (first + i + 1) as f64;
                    }
                    self.joints.push(j);
                }
            }
            let mut kinematic = vec![false; self.entities.len()];
            for c in &self.cables {
                for li in 1..c.count() {
                    let id = c.entity(li - 1, 1);
                    if passive_wheel(c, li, &self.entities) {
                        kinematic[id] = true;
                    }
                }
            }
            let members = rotor_members(&self.entities, &kinematic);
            let max = self.rows.iter().map(|r| r.iterations).max().unwrap_or(1);
            let mut multipliers = vec![0.0; self.rows.len()];
            for iter in 0..max {
                let forward = iter % 2 == 0;
                for k in 0..self.rows.len() {
                    let r = if forward { k } else { self.rows.len() - 1 - k };
                    let row = self.rows[r];
                    if iter >= row.iterations {
                        continue;
                    }
                    solve_row(
                        &mut self.entities,
                        &mut self.joints,
                        &members,
                        &row,
                        iter,
                        &mut multipliers[r],
                        &mut self.scratch,
                    );
                }
            }
            finalize_force_transfers(&mut self.joints);
            for (p, c) in self.cables.iter_mut().enumerate() {
                let first = firsts[p];
                for i in 0..c.count() {
                    let jo = c.jo(i);
                    let j = &self.joints[first + i];
                    c.data[jo + 10] = j[31];
                    c.data[jo + 11] = j[35];
                    c.data[jo + 12] = j[36];
                    put_v(&mut c.data, jo + 13, v(j, 32));
                }
            }
            if self.transport(&firsts, !frictional || attempt + 1 == SOLVES) {
                break;
            }
            self.entities.clone_from(&self.snapshot);
        }
        for owners in &self.shared {
            let mut result = [0.0; 6];
            let mut transfer: f64 = 0.0;
            for &(p, i) in owners {
                let j = &self.joints[firsts[p] + i];
                transfer = transfer.max(j[36]);
                if j[37] != 0.0 {
                    result = [j[31], j[35], j[36], j[32], j[33], j[34]];
                }
            }
            result[2] = transfer;
            for &(p, i) in owners {
                let o = self.cables[p].jo(i);
                self.cables[p].data[o + 10..o + 16].copy_from_slice(&result);
            }
        }
    }
    /// Viscoelastic / retained elongation hysteresis when `options[11] > 0`.
    /// Active spans develop additional hysteretic strain toward `h * eps_el` on loading
    /// (`tau_load = 0.25 s`) and either retain it permanently (`tau_rec <= 0`) or recover
    /// slowly with time constant `tau_rec = options[12]` on unloading, adjusting active rest
    /// lengths and `totalRestLength` without altering stored spool length.
    fn elongation_hysteresis(&mut self, dt: f64) {
        let h = self.options[11];
        if !(dt.is_finite() && dt > 0.0 && h.is_finite() && h > 0.0) {
            return;
        }
        let tau_rec = self.options[12];
        const TAU_LOAD: f64 = 0.25;
        let mut joint_offset = 0usize;
        for c in &mut self.cables {
            let count = c.count();
            let cpl = c.data[3];
            if cpl.is_finite() && cpl > 0.0 && joint_offset + count <= self.joints.len() {
                let mut stretch_sum = 0.0;
                let mut rest_sum = 0.0;
                for i in 0..count {
                    let rest = c.rest(i);
                    if rest <= EPS {
                        continue;
                    }
                    let j = &self.joints[joint_offset + i];
                    let a = attachment(&self.entities, j[0] as usize, v(j, 4));
                    let b = attachment(&self.entities, j[1] as usize, v(j, 7));
                    let span = length(sub(b, a));
                    stretch_sum += (span - rest).max(0.0);
                    rest_sum += rest;
                }
                if rest_sum > EPS {
                    let eps_el = stretch_sum / rest_sum;
                    let target = h * eps_el;
                    let diff = target - c.hyst_strain;
                    let delta_eps = if diff > 0.0 {
                        diff * (1.0 - (-dt / TAU_LOAD).exp())
                    } else if diff < 0.0 {
                        if tau_rec.is_finite() && tau_rec > 0.0 {
                            diff * (1.0 - (-dt / tau_rec).exp())
                        } else {
                            0.0
                        }
                    } else {
                        0.0
                    };
                    if delta_eps.abs() > 1e-15 {
                        let scale = delta_eps / (1.0 + c.hyst_strain).max(EPS);
                        let mut delta_total = 0.0;
                        for i in 0..count {
                            let jo = c.jo(i);
                            let rest = c.data[jo + 3];
                            if rest > EPS {
                                let dr = rest * scale;
                                c.data[jo + 3] = (rest + dr).max(EPS);
                                delta_total += dr;
                            }
                        }
                        c.data[7] += delta_total;
                        c.hyst_strain = (c.hyst_strain + delta_eps).max(0.0);
                    }
                }
            }
            joint_offset += count;
        }
    }
    /// After the solve, material inside each slipping row settles where every
    /// segment carries its share ρᵢT of the row's tension, conserving the
    /// row's material (see `settle`).
    ///
    /// The outcome is checked against the link states it was solved with.
    /// Material only moves toward a slipping link's tighter side: otherwise
    /// the link holds it and sticks. A sticking link's tensions must lie
    /// inside its capstan cone: otherwise it slips. Returns false, with the
    /// states corrected and nothing else changed, for the solve to be
    /// repeated with them. A link whose sticking solve calls for the slip
    /// that its slipping solve refused keeps the slipping solve, holding its
    /// material: few solver iterations leave sticking rows the less accurate.
    fn transport(&mut self, firsts: &[usize], last: bool) -> bool {
        let layering = self.options[3] != 0.0;
        // Per joint: span length (where needed) and rest length.
        let mut spans = vec![f64::NAN; self.joints.len()];
        let mut rests: Vec<f64> = self.joints.iter().map(|j| j[10]).collect();
        let mut states = Vec::new();
        let mut wheels = Vec::new();
        let frictional: Vec<bool> = self
            .cables
            .iter()
            .map(|c| (1..c.count()).any(|li| c.slip[li] != FREE && c.ratio[li].is_finite()))
            .collect();
        for row in &self.rows {
            let p = firsts.iter().rposition(|&f| f <= row.start).unwrap();
            let count = row.end - row.start;
            if count < 2 && !frictional[p] {
                continue;
            }
            let c = &self.cables[p];
            let s = row.start - firsts[p];
            let lengths = &mut spans[row.start..row.end];
            let settled = &mut rests[row.start..row.end];
            let mut weights = vec![0.0; count];
            for k in 0..count {
                let j = &self.joints[row.start + k];
                let a = attachment(&self.entities, j[0] as usize, v(j, 4));
                let b = attachment(&self.entities, j[1] as usize, v(j, 7));
                lengths[k] = length(sub(b, a));
                weights[k] = lengths[k] * j[38];
            }
            if count < 2 {
                continue;
            }
            // The row settles between the links found to hold their material.
            let mut held = vec![false; count + 1];
            held[count] = true;
            let mut state = vec![FREE; count];
            loop {
                for (k, rest) in settled.iter_mut().enumerate() {
                    *rest = c.rest(s + k);
                }
                let mut start = 0;
                let mut moved = 0.0;
                let mut again = false;
                for end in 1..=count {
                    if !held[end] {
                        continue;
                    }
                    let taut = settle(
                        &lengths[start..end],
                        &weights[start..end],
                        &c.strain[s + start..s + end],
                        row.gamma,
                        &mut settled[start..end],
                    );
                    for k in start..end - 1 {
                        // `moved` is the material gained left of link `li`.
                        moved += settled[k] - c.rest(s + k);
                        let li = s + k + 1;
                        state[k + 1] = c.slip[li];
                        if c.slip[li] == FREE {
                            continue;
                        }
                        let toward = if moved > 1e-9 {
                            LEFT
                        } else if moved < -1e-9 {
                            RIGHT
                        } else {
                            c.slip[li]
                        };
                        if toward == c.slip[li] {
                            continue;
                        }
                        // Without tension the direction carries no force.
                        if taut {
                            held[k + 1] = true;
                            again = true;
                        } else {
                            state[k + 1] = toward;
                        }
                    }
                    moved = 0.0;
                    start = end;
                }
                if !again {
                    break;
                }
            }
            let mut moved = 0.0;
            for k in 0..count - 1 {
                moved += settled[k] - c.rest(s + k);
                let li = s + k + 1;
                if held[k + 1] {
                    states.push((p, li, STICK, c.tried[li] & (tried(STICK) | SETTLED) == 0));
                } else if state[k + 1] != c.slip[li] {
                    states.push((p, li, state[k + 1], false));
                }
                if passive_wheel(c, li, &self.entities) {
                    wheels.push((p, li, c.wheels[li].map_or(moved, |wheel| wheel.turn(moved))));
                }
            }
        }
        // A worn wheel at a sticking link meets a line that does not travel.
        for (p, c) in self.cables.iter().enumerate() {
            for li in 1..c.count() {
                if let (true, Some(wheel)) = (c.slip[li] == STICK, c.wheels[li]) {
                    wheels.push((p, li, wheel.turn(0.0)));
                }
            }
        }
        let strain = |joint: usize| {
            if spans[joint] > EPS { (spans[joint] - rests[joint]).max(0.0) / spans[joint] } else { 0.0 }
        };
        for row in &self.rows {
            let p = firsts.iter().rposition(|&f| f <= row.start).unwrap();
            let c = &self.cables[p];
            // The link a row starts at sticks. Tensions are over EA, by the
            // law the rows are solved with.
            let li = row.start - firsts[p];
            if li == 0 || !c.ratio[li].is_finite() {
                continue;
            }
            let tension = |joint: usize| {
                if row.zero {
                    self.joints[joint][35]
                } else {
                    let old = c.strain[joint - firsts[p]];
                    (strain(joint) + row.gamma * (strain(joint) - old)).max(0.0)
                }
            };
            let (left, right) = (tension(row.start - 1), tension(row.start));
            let limit = c.ratio[li] * (1.0 + 1e-9);
            let state = if left > limit * right + 1e-12 {
                LEFT
            } else if right > limit * left + 1e-12 {
                RIGHT
            } else {
                continue;
            };
            states.push((p, li, state, c.tried[li] & SETTLED == 0));
        }
        let repeat = !last && states.iter().any(|state| state.3);
        for (p, li, state, solve) in states {
            let c = &mut self.cables[p];
            if repeat && solve {
                if c.tried[li] & tried(state) != 0 {
                    c.tried[li] |= SETTLED;
                }
                c.tried[li] |= tried(state);
                c.slip[li] = state;
            } else if !repeat {
                c.slip[li] = state;
            }
        }
        if repeat {
            return false;
        }
        for (p, c) in self.cables.iter_mut().enumerate() {
            for i in 0..c.count() {
                let joint = firsts[p] + i;
                if spans[joint].is_nan() {
                    continue;
                }
                let jo = c.jo(i);
                c.data[jo + 3] = rests[joint];
                c.strain[i] = strain(joint);
            }
        }
        for (p, li, travel) in wheels {
            let c = &mut self.cables[p];
            if let Some(wheel) = &mut c.wheels[li] {
                wheel.travel = [travel, wheel.travel[0]];
            }
            let id = c.entity(li - 1, 1);
            let wheel = &self.entities[id];
            let radius = c.radius(&self.entities, li, id, layering).0;
            if candidate(wheel) && radius > EPS {
                let angle = travel / if c.cw(li, false) { radius } else { -radius };
                self.turn_wheel(id, angle);
            }
        }
        true
    }
    /// Set a free wheel's rotation this step to `angle` about its own axis,
    /// relative to its carrier, replacing any predicted spin.
    fn turn_wheel(&mut self, id: usize, angle: f64) {
        let e = self.entities[id];
        if e[54] == 0.0 {
            return;
        }
        let spin = axis_angle(norm(v(&e, 44)), angle);
        if e[50] >= 0.0 {
            let parent = &self.entities[e[50] as usize];
            let start = qnorm(mul(qnorm(conj(q(parent, 10))), q(&e, 10)));
            let local = qnorm(mul(start, spin));
            let world = qnorm(mul(q(parent, 3), local));
            put_q(&mut self.entities[id], 17, local);
            put_q(&mut self.entities[id], 3, world);
        } else {
            let world = qnorm(mul(q(&e, 10), spin));
            put_q(&mut self.entities[id], 3, world);
        }
    }
}
/// Rest lengths of consecutive segments that share their material. The line
/// is a Kelvin–Voigt solid, as in the row solve: with strain εᵢ, strain εᵢⁿ
/// when the last step ended and relaxation time γ·dt, a segment's tension is
/// EA(εᵢ + γ(εᵢ − εᵢⁿ)), and the segments settle where these are in proportion
/// to their weights over length. Slack segments keep their material until
/// another segment pulls it taut. Returns whether the segments are taut.
fn settle(lengths: &[f64], weights: &[f64], strains: &[f64], gamma: f64, rests: &mut [f64]) -> bool {
    let count = rests.len();
    let stretch = lengths.iter().sum::<f64>() - rests.iter().sum::<f64>();
    let weight: f64 = weights.iter().sum();
    if stretch <= 0.0 || weight <= EPS {
        // Taut spans draw their shortage from the slack further along, then
        // from the slack behind them.
        for k in 0..count - 1 {
            let shortage = (lengths[k] - rests[k]).max(0.0);
            rests[k] += shortage;
            rests[k + 1] -= shortage;
        }
        for k in (1..count).rev() {
            let shortage = (lengths[k] - rests[k]).max(0.0);
            rests[k] += shortage;
            rests[k - 1] -= shortage;
        }
        return false;
    }
    let retained = gamma / (1.0 + gamma);
    let held: f64 = (0..count).map(|k| strains[k] * lengths[k]).sum();
    let shared = stretch - retained * held;
    for k in 0..count {
        let old = strains[k] * lengths[k];
        // Damping that cancels the whole tension leaves no share.
        rests[k] = lengths[k]
            - if shared >= 0.0 {
                retained * old + shared * weights[k] / weight
            } else {
                stretch * old / held
            };
    }
    true
}
/// Capstan tension ratio exp(μθ) across interior link `li`, or None when
/// material cannot cross it (attachments bound rows).
fn capstan(c: &Cable, li: usize, es: &[Entity], layering: bool) -> Option<f64> {
    let kind = c.kind(li);
    if kind == 0 {
        return None;
    }
    let e = &es[c.entity(li - 1, 1)];
    if kind == 1 && has(e, SPOOL) {
        // A motor holds its material. A wheel on an ideal bearing, of
        // negligible inertia, passes it on at equal tension and turns with
        // it. On a worn bearing the line grips the wheel as it would a roller.
        if has(e, MOTOR) {
            return None;
        }
        if !has(e, BEARING) {
            return Some(1.0);
        }
    }
    if !(kind == 1 || kind == 2) || e[105] <= EPS {
        return Some(1.0);
    }
    let angle = if kind == 1 {
        let r = e[104] + if layering { c.data[6] } else { 0.0 };
        if r > EPS {
            (c.stored(li) / r).abs()
        } else {
            0.0
        }
    } else {
        // The wrap angle is the turn from the incoming to the outgoing span.
        let incoming = sub(c.point(li - 1, 1), c.point(li - 1, 0));
        let outgoing = sub(c.point(li, 1), c.point(li, 0));
        if dot(incoming, incoming) <= EPS || dot(outgoing, outgoing) <= EPS {
            0.0
        } else {
            acos(dot(norm(incoming), norm(outgoing)).clamp(-1.0, 1.0))
        }
    };
    Some((e[105] * angle).exp())
}
/// Whether interior link `li` runs over a wheel no motor turns: the line
/// moves over it, and its rotation follows from the line.
fn passive_wheel(c: &Cable, li: usize, es: &[Entity]) -> bool {
    if li == 0 || li >= c.count() || c.kind(li) != 1 {
        return false;
    }
    let e = &es[c.entity(li - 1, 1)];
    has(e, SPOOL) && !has(e, MOTOR)
}
fn candidate(e: &Entity) -> bool {
    has(e, LOCAL_AXIS) && e[52] != 0.0 && e[56] != 0.0
}
fn mapped_id(es: &[Entity], id: usize, other: usize) -> usize {
    if es[id][50] >= 0.0 && !internal(es, id, other) {
        es[id][50] as usize
    } else {
        id
    }
}
fn member_angular(
    c: &Cable,
    i: usize,
    side: usize,
    es: &[Entity],
    point: V,
    grad: V,
) -> Option<(usize, V, V)> {
    let id = c.entity(i, side);
    let other = c.entity(i, 1 - side);
    if internal(es, id, other) || mapped_id(es, id, other) == id {
        return None;
    }
    if candidate(&es[id]) {
        return Some((id, point, grad));
    }
    let li = i + side;
    if c.kind(li) != 2 {
        return None;
    }
    let ni = if side == 0 {
        i as isize - 1
    } else {
        i as isize + 1
    };
    let nl = if side == 0 {
        li as isize - 1
    } else {
        li as isize + 1
    };
    if ni < 0
        || ni >= c.count() as isize
        || nl < 0
        || nl > c.count() as isize
        || !hybrid(c.kind(nl as usize))
    {
        return None;
    }
    let id = c.entity(ni as usize, side);
    let p = c.point(ni as usize, side);
    let g = sub(point, p);
    if !candidate(&es[id]) || dot(g, g) <= EPS {
        None
    } else {
        Some((id, p, norm(g)))
    }
}
impl PhysicsWorld {
    fn overcorrections(&mut self, dt: f64) {
        let mut over = Vec::new();
        let mut candidates = Vec::new();
        let mut unique = std::collections::BTreeMap::new();
        for (p, c) in self.cables.iter().enumerate() {
            for i in 0..c.count() {
                // A shared row already balances its segments' material.
                if c.merged.get(i) == Some(&true) {
                    continue;
                }
                let id = c.data[c.jo(i)] as u64;
                if let Some(&index) = unique.get(&id) {
                    candidates[index] = (p, i);
                } else {
                    unique.insert(id, candidates.len());
                    candidates.push((p, i));
                }
            }
        }
        for (p, i) in candidates {
            let c = &self.cables[p];
            // Mounted assemblies use XPBD's unilateral multiplier release;
            // the legacy averaging heuristic lacks their coupled mass matrix.
            if [c.entity(i, 0), c.entity(i, 1)].iter().any(|&id| {
                let body = if self.entities[id][50] >= 0.0 {
                    self.entities[id][50] as usize
                } else {
                    id
                };
                self.children[body]
                    .iter()
                    .any(|&child| has(&self.entities[child], SPOOL))
            }) {
                continue;
            }
            if length(sub(c.point(i, 0), c.point(i, 1))) < c.rest(i) {
                continue;
            }
            let (points, _, _) = geometry(c, i, &self.entities, self.options[3] != 0.0);
            let (Some(a), Some(b)) = (points[0], points[1]) else {
                continue;
            };
            let post = length(sub(a, b));
            if post < c.rest(i) {
                over.push((p, i, a, b, post));
            }
        }
        if over.len() < 2 {
            return;
        }
        let mut pos = vec![([0.0; 3], 0usize); self.entities.len()];
        let mut ang = pos.clone();
        for (p, i, a, b, post) in over {
            let c = &self.cables[p];
            let error = post - c.rest(i);
            if error >= -EPS {
                continue;
            }
            let diff = sub(b, a);
            let len = length(diff);
            if len <= EPS {
                continue;
            }
            let dir = scale(diff, 1.0 / len);
            let mut denom = 0.0;
            let mut corrections = Vec::new();
            for side in 0..2 {
                let id = c.entity(i, side);
                let other = c.entity(i, 1 - side);
                let mapped = mapped_id(&self.entities, id, other);
                let e = &self.entities[mapped];
                if e[51] == 0.0 {
                    continue;
                }
                let point = if side == 0 { a } else { b };
                let grad = if side == 0 { scale(dir, -1.0) } else { dir };
                let solver_point = attachment(
                    &self.entities,
                    mapped,
                    local_point(&self.entities, mapped, point),
                );
                let angular = cross(sub(solver_point, v(e, 0)), scale(grad, -1.0));
                let angular_denom = if e[55] != 0.0 {
                    positive_inertia(dot(angular, world_inverse(e, angular)))
                } else {
                    0.0
                };
                let reaction = self.entities[id][50];
                let reaction_denom = if reaction >= 0.0 {
                    let body = &self.entities[reaction as usize];
                    if has(body, INERTIA) && body[52] != 0.0 {
                        positive_inertia(dot(angular, world_inverse(body, angular)))
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };
                let member = member_angular(c, i, side, &self.entities, point, grad);
                let member_grad = member.map(|(id, p, g)| {
                    cross(sub(p, world_pos(&self.entities, id).unwrap_or([0.0; 3])), g)
                });
                let member_denom = if let (Some((id, _, _)), Some(g)) = (member, member_grad) {
                    positive_inertia(dot(g, world_inverse(&self.entities[id], g)))
                } else {
                    0.0
                };
                denom += e[43] * dot(grad, grad);
                denom += angular_denom;
                denom += reaction_denom;
                denom += member_denom;
                if e[43] > 0.0 {
                    corrections.push((false, mapped, scale(grad, e[43])));
                }
                if angular_denom > 0.0 {
                    corrections.push((true, mapped, scale(world_inverse(e, angular), -1.0)));
                }
                if member_denom > 0.0 {
                    let id = member.unwrap().0;
                    corrections.push((
                        true,
                        id,
                        scale(
                            world_inverse(&self.entities[id], member_grad.unwrap()),
                            -1.0,
                        ),
                    ));
                }
                if reaction_denom > 0.0 {
                    corrections.push((
                        true,
                        reaction as usize,
                        world_inverse(&self.entities[reaction as usize], angular),
                    ));
                }
            }
            if dt > 0.0 {
                denom += c.data[3] / (dt * dt);
            }
            if denom <= EPS {
                continue;
            }
            let lambda = -error / denom;
            for (is_ang, id, delta) in corrections {
                let list = if is_ang { &mut ang } else { &mut pos };
                list[id].0 = add(list[id].0, scale(delta, lambda));
                list[id].1 += 1;
            }
        }
        for (id, (sum, n)) in pos.into_iter().enumerate() {
            if n >= 2 && self.entities[id][51] != 0.0 {
                let p = add(v(&self.entities[id], 0), scale(sum, 1.0 / n as f64));
                put_v(&mut self.entities[id], 0, p);
            }
        }
        for (id, (sum, n)) in ang.into_iter().enumerate() {
            if n < 2 || self.entities[id][52] == 0.0 {
                continue;
            }
            let delta = scale(sum, 1.0 / n as f64);
            let angle = length(delta);
            if angle > EPS {
                let orient = qnorm(mul(
                    axis_angle(scale(delta, 1.0 / angle), angle),
                    q(&self.entities[id], 3),
                ));
                put_q(&mut self.entities[id], 3, orient);
            }
            update_local(&mut self.entities, id);
        }
    }
}
