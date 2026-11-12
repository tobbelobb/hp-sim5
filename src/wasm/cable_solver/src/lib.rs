//! f64 XPBD cable rows, spool spins and mounted-rotor coupling used by
//! `engine::PhysicsWorld`. No ECS calls inside the loop.
use std::f64::consts::PI;

const EPS: f64 = 1e-9;
const INERTIA_EPS: f64 = 1e-12;
pub const E: usize = 160;
const J: usize = 39;
type V = [f64; 3];
type Q = [f64; 4];
pub type Entity = [f64; E];
type Joint = [f64; J];

// Use Rust's math on both targets: no calls across the JS/WASM boundary.
fn sin(x: f64) -> f64 {
    x.sin()
}
fn cos(x: f64) -> f64 {
    x.cos()
}
fn acos(x: f64) -> f64 {
    x.acos()
}
fn asin(x: f64) -> f64 {
    x.asin()
}
fn atan2(y: f64, x: f64) -> f64 {
    y.atan2(x)
}
mod abi;
pub mod engine;
fn length(v: V) -> f64 {
    // Scaled, compensated three-term sum, matching V8's three-argument hypot.
    // See https://github.com/v8/v8/blob/13.6.233/src/builtins/math.tq#L406-L426.
    let [a, b, c] = v.map(f64::abs);
    if a.is_infinite() || b.is_infinite() || c.is_infinite() {
        return f64::INFINITY;
    }
    if a.is_nan() || b.is_nan() || c.is_nan() {
        return f64::NAN;
    }
    let max = a.max(b).max(c);
    if max == 0.0 {
        return 0.0;
    }
    let x = a / max;
    let y = b / max;
    let z = c / max;
    let aa = x * x;
    let bb = y * y;
    let compensation = (aa + bb) - aa - bb;
    (aa + bb + (z * z - compensation)).sqrt() * max
}
fn length2(a: [f64; 2]) -> f64 {
    let x = a[0].abs();
    let y = a[1].abs();
    let max = x.max(y);
    if max.is_infinite() {
        return max;
    }
    if x.is_nan() || y.is_nan() {
        return f64::NAN;
    }
    if max == 0.0 {
        return 0.0;
    }
    let x = x / max;
    let y = y / max;
    (x * x + y * y).sqrt() * max
}
fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(v: V) -> V {
    let l = length(v);
    if l > 0.0 {
        scale(v, 1.0 / l)
    } else {
        v
    }
}
fn v(a: &[f64], i: usize) -> V {
    [a[i], a[i + 1], a[i + 2]]
}
fn q(a: &[f64], i: usize) -> Q {
    [a[i], a[i + 1], a[i + 2], a[i + 3]]
}
fn put_v(a: &mut [f64], i: usize, b: V) {
    a[i..i + 3].copy_from_slice(&b);
}
fn put_q(a: &mut [f64], i: usize, b: Q) {
    a[i..i + 4].copy_from_slice(&b);
}
fn qnorm(a: Q) -> Q {
    let l = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]).sqrt();
    if l == 0.0 {
        [0.0, 0.0, 0.0, 1.0]
    } else {
        let s = 1.0 / l;
        [a[0] * s, a[1] * s, a[2] * s, a[3] * s]
    }
}
fn conj(a: Q) -> Q {
    [-a[0], -a[1], -a[2], a[3]]
}
fn mul(a: Q, b: Q) -> Q {
    let [x, y, z, w] = a;
    let [u, v, t, s] = b;
    [
        x * s + w * u + y * t - z * v,
        y * s + w * v + z * u - x * t,
        z * s + w * t + x * v - y * u,
        w * s - x * u - y * v - z * t,
    ]
}
fn rotate(a: Q, b: V) -> V {
    let [qx, qy, qz, qw] = a;
    let [x, y, z] = b;
    let ix = qw * x + qy * z - qz * y;
    let iy = qw * y + qz * x - qx * z;
    let iz = qw * z + qx * y - qy * x;
    let iw = -qx * x - qy * y - qz * z;
    [
        ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy),
        iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz),
        iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx),
    ]
}
fn axis_angle(axis: V, angle: f64) -> Q {
    let half = angle / 2.0;
    let s = sin(half);
    [axis[0] * s, axis[1] * s, axis[2] * s, cos(half)]
}
fn matrix_vec(a: &[f64], i: usize, b: V) -> V {
    [
        a[i] * b[0] + a[i + 1] * b[1] + a[i + 2] * b[2],
        a[i + 3] * b[0] + a[i + 4] * b[1] + a[i + 5] * b[2],
        a[i + 6] * b[0] + a[i + 7] * b[1] + a[i + 8] * b[2],
    ]
}
fn world_inverse(e: &Entity, grad: V) -> V {
    let local = rotate(qnorm(conj(q(e, 3))), grad);
    rotate(qnorm(q(e, 3)), matrix_vec(e, 34, local))
}
fn positive_inertia(x: f64) -> f64 {
    if x.is_finite() && x > INERTIA_EPS {
        x
    } else {
        0.0
    }
}
fn rotation_vector(e: &Entity) -> V {
    if e[54] == 0.0 || e[52] == 0.0 {
        return [0.0; 3];
    }
    let mut delta = qnorm(mul(q(e, 3), qnorm(conj(q(e, 10)))));
    if delta[3] < 0.0 {
        for x in &mut delta {
            *x *= -1.0;
        }
    }
    let w = delta[3].clamp(-1.0, 1.0);
    let s = length([delta[0], delta[1], delta[2]]);
    let angle = 2.0 * atan2(s, w);
    if angle <= EPS {
        return [0.0; 3];
    }
    if s <= EPS {
        return [0.0; 3];
    }
    scale([delta[0] / s, delta[1] / s, delta[2] / s], angle)
}
fn spin_angle(e: &Entity) -> f64 {
    if e[62] == 0.0 {
        return 0.0;
    }
    let curr = if e[50] >= 0.0 && e[49] as u32 & 2 != 0 {
        q(e, 17)
    } else {
        q(e, 3)
    };
    let rel = qnorm(mul(qnorm(conj(q(e, 21))), curr));
    let axis = norm(v(e, 44));
    let projection = rel[0] * axis[0] + rel[1] * axis[1] + rel[2] * axis[2];
    let twist = [
        axis[0] * projection,
        axis[1] * projection,
        axis[2] * projection,
        rel[3],
    ];
    if twist[0] * twist[0] + twist[1] * twist[1] + twist[2] * twist[2] + twist[3] * twist[3] <= EPS
    {
        return 0.0;
    }
    let twist = qnorm(twist);
    let s = twist[0] * axis[0] + twist[1] * axis[1] + twist[2] * axis[2];
    let mut angle = 2.0 * atan2(s, twist[3]);
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}
fn world_pos(es: &[Entity], id: usize) -> Option<V> {
    let e = &es[id];
    if e[50] >= 0.0 {
        let body = &es[e[50] as usize];
        if body[51] != 0.0 && body[52] != 0.0 && e[49] as u32 & 1 != 0 {
            return Some(add(v(body, 0), rotate(q(body, 3), v(e, 14))));
        }
    }
    if e[51] != 0.0 {
        Some(v(e, 0))
    } else {
        None
    }
}
fn world_quat(es: &[Entity], id: usize) -> Option<Q> {
    let e = &es[id];
    if e[50] >= 0.0 {
        let body = &es[e[50] as usize];
        if body[52] != 0.0 && e[49] as u32 & 2 != 0 {
            return Some(qnorm(mul(q(body, 3), q(e, 17))));
        }
    }
    if e[52] != 0.0 {
        Some(qnorm(q(e, 3)))
    } else {
        None
    }
}
fn attachment(es: &[Entity], id: usize, local: V) -> V {
    let Some(pos) = world_pos(es, id) else {
        return local;
    };
    let rotated = world_quat(es, id).map_or(local, |q| rotate(q, local));
    add(pos, rotated)
}
fn update_local(es: &mut [Entity], id: usize) {
    let (parent, oriented) = (es[id][50], es[id][52]);
    if parent < 0.0 || oriented == 0.0 {
        return;
    }
    let body = &es[parent as usize];
    if body[52] == 0.0 {
        return;
    }
    let local = qnorm(mul(qnorm(conj(q(body, 3))), q(&es[id], 3)));
    put_q(&mut es[id], 17, local);
    es[id][65] = 1.0;
}

// Attachment geometry uses caller-prepared live endpoint frames. Each result
// is independent; winding/topology/history remain owned by the JS update.
#[derive(Clone, Copy)]
struct Basis {
    n: V,
    u: V,
    v: V,
}
fn basis(n: V) -> Basis {
    if dot(n, n) <= EPS {
        return Basis {
            n: [0.0, 0.0, 1.0],
            u: [1.0, 0.0, 0.0],
            v: [0.0, 1.0, 0.0],
        };
    }
    let n = norm(n);
    let reference = if n[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let mut u = sub(reference, scale(n, dot(n, reference)));
    if dot(u, u) <= EPS {
        let r = [0.0, 0.0, 1.0];
        u = sub(r, scale(n, dot(n, r)));
    }
    let u = norm(u);
    Basis {
        n,
        u,
        v: cross(n, u),
    }
}
fn project(point: V, origin: V, b: Basis) -> [f64; 2] {
    let rel = sub(point, origin);
    [dot(rel, b.u), dot(rel, b.v)]
}
fn lift(point: [f64; 2], origin: V, b: Basis) -> V {
    add(add(origin, scale(b.u, point[0])), scale(b.v, point[1]))
}
fn circle_point(point: [f64; 2], center: [f64; 2], r: f64, cw: bool, first: bool) -> [f64; 2] {
    let d = [center[0] - point[0], center[1] - point[1]];
    let d_sq = d[0] * d[0] + d[1] * d[1];
    if d_sq <= r * r + EPS {
        let dir = if d_sq > EPS {
            let s = 1.0 / length2(d);
            [d[0] * s, d[1] * s]
        } else {
            [1.0, 0.0]
        };
        return [center[0] - dir[0] * r, center[1] - dir[1] * r];
    }
    let alpha = atan2(d[1], d[0]);
    let phi = asin(r / d_sq.sqrt());
    let angle = if cw == first {
        alpha + phi + PI / 2.0
    } else {
        alpha - phi - PI / 2.0
    };
    [center[0] + r * cos(angle), center[1] + r * sin(angle)]
}
fn circle_circle(
    a: [f64; 2],
    ra: f64,
    cwa: bool,
    b: [f64; 2],
    rb: f64,
    cwb: bool,
) -> ([f64; 2], [f64; 2]) {
    let delta = [b[0] - a[0], b[1] - a[1]];
    let d = length2(delta);
    if d < EPS {
        return ([a[0] + ra, a[1]], [b[0] + rb, b[1]]);
    }
    let r = if cwa == cwb { rb - ra } else { ra + rb };
    let alpha = atan2(delta[1], delta[0]);
    let phi = asin((r / d).clamp(-1.0, 1.0));
    let (aa, ab) = if cwa != cwb {
        if !cwa {
            (alpha - PI / 2.0 + phi, alpha + PI / 2.0 + phi)
        } else {
            (alpha + PI / 2.0 - phi, alpha - PI / 2.0 - phi)
        }
    } else if !cwa {
        (alpha - PI / 2.0 - phi, alpha - PI / 2.0 - phi)
    } else {
        (alpha + PI / 2.0 + phi, alpha + PI / 2.0 + phi)
    };
    (
        [a[0] + ra * cos(aa), a[1] + ra * sin(aa)],
        [b[0] + rb * cos(ab), b[1] + rb * sin(ab)],
    )
}
fn rolling_attachment(
    center: Option<V>,
    other: Option<V>,
    r: f64,
    has_radius: bool,
    n: V,
    cw: bool,
    first: bool,
) -> Option<V> {
    let center = center?;
    let Some(mut other) = other else {
        return Some(center);
    };
    if !has_radius {
        return Some(center);
    }
    if dot(n, n) > EPS {
        let n = norm(n);
        other = sub(other, scale(n, dot(sub(other, center), n)));
    }
    let b = basis(n);
    // A rolling endpoint uses sphere-to-point (first=false); B uses the reverse.
    let tangent = circle_point(
        project(other, center, b),
        project(center, center, b),
        r,
        cw,
        !first,
    );
    Some(lift(tangent, center, b))
}
fn rotate_axis(point: V, axis: V, angle: f64) -> V {
    let n = norm(axis);
    let c = cos(angle);
    let s = sin(angle);
    add(
        add(scale(point, c), scale(cross(n, point), s)),
        scale(n, dot(point, n) * (1.0 - c)),
    )
}
fn attachment_data(data: &mut [f64]) {
    let a = &data[..19];
    let b = &data[19..38];
    let flags_a = a[13] as u32;
    let flags_b = b[13] as u32;
    let pa = if flags_a & 1 != 0 {
        Some(v(a, 0))
    } else {
        None
    };
    let pb = if flags_b & 1 != 0 {
        Some(v(b, 0))
    } else {
        None
    };
    let na = v(a, 6);
    let nb = v(b, 6);
    let rotated_a = a[18] != 0.0 && a[12].abs() > EPS;
    let rotated_b = b[18] != 0.0 && b[12].abs() > EPS;
    let parallel = dot(na, na) > EPS
        && dot(nb, nb) > EPS
        && (dot(norm(na), norm(nb)).abs() - 1.0).abs() <= 1e-6;
    let common = a[16] != 0.0 && b[16] != 0.0 && parallel && !rotated_a && !rotated_b;
    let mut current_a = pa;
    let mut current_b = pb;
    if common {
        if let (Some(pa), Some(pb)) = (pa, pb) {
            if flags_a & 4 != 0 && flags_b & 4 != 0 {
                let basis = basis(na);
                let offset = dot(sub(pb, pa), basis.n);
                let (ta, tb) = circle_circle(
                    project(pa, pa, basis),
                    a[14],
                    a[15] != 0.0,
                    project(pb, pa, basis),
                    b[14],
                    b[15] != 0.0,
                );
                current_a = Some(lift(ta, pa, basis));
                current_b = Some(add(lift(tb, pa, basis), scale(basis.n, offset)));
            }
        }
    } else {
        for side in 0..2 {
            let e = if side == 0 { a } else { b };
            let flags = e[13] as u32;
            let rotated = if side == 0 { rotated_a } else { rotated_b };
            if (rotated || e[16] == 0.0) && e[17] != 0.0 && flags & 3 == 3 {
                let old = v(e, 9);
                let prev = v(e, 3);
                let rotation_delta =
                    sub(add(rotate_axis(sub(old, prev), v(e, 6), e[12]), prev), old);
                let result = Some(add(add(old, sub(v(e, 0), prev)), rotation_delta));
                if side == 0 {
                    current_a = result;
                } else {
                    current_b = result;
                }
            }
        }
        if a[16] != 0.0 && !rotated_a {
            current_a = rolling_attachment(
                pa,
                current_b,
                a[14],
                flags_a & 4 != 0,
                na,
                a[15] != 0.0,
                true,
            );
        }
        if b[16] != 0.0 && !rotated_b {
            current_b = rolling_attachment(
                pb,
                current_a,
                b[14],
                flags_b & 4 != 0,
                nb,
                b[15] != 0.0,
                false,
            );
        }
    }
    if let Some(a) = current_a {
        put_v(data, 38, a);
    }
    if let Some(b) = current_b {
        put_v(data, 41, b);
    }
    data[44] = if current_a.is_some() { 1.0 } else { 0.0 };
    data[45] = if current_b.is_some() { 1.0 } else { 0.0 };
}


#[derive(Clone, Copy, Default)]
struct Spin {
    id: usize,
    active: bool,
    axis: V,
    free: f64,
    inv: f64,
    // Geometric length gradient about the spool axis, and the same gradient
    // scaled by the segment's share of the row tension (force direction).
    grad: f64,
    grad_f: f64,
    load_grad: f64,
    disp: f64,
    flags: u32,
    limit: f64,
}
fn spin(es: &[Entity], joint: &Joint, side: usize, point: V, grad: V) -> Spin {
    let offset = if side == 0 { 13 } else { 18 };
    if joint[offset] < 0.0 {
        return Spin::default();
    }
    let id = joint[offset] as usize;
    let e = &es[id];
    if e[56] == 0.0 || e[52] == 0.0 {
        return Spin::default();
    }
    let Some(center) = world_pos(es, id) else {
        return Spin::default();
    };
    let orient = world_quat(es, id).unwrap_or(q(e, 3));
    let axis = rotate(orient, v(e, 44));
    if dot(axis, axis) <= EPS {
        return Spin::default();
    }
    let axis = norm(axis);
    // Match both normalizations in effectiveInertiaAboutWorldAxis/localAxis.
    let local = norm(rotate(qnorm(conj(orient)), norm(axis)));
    let inertia = positive_inertia(dot(local, matrix_vec(e, 25, local)));
    let free = if inertia > INERTIA_EPS {
        1.0 / inertia
    } else {
        0.0
    };
    if free <= EPS {
        return Spin::default();
    }
    let inv = if e[57] != 0.0 {
        0.0
    } else if e[47] > EPS {
        // dt² is packed into stiffness by the adapter, except for its threshold.
        let effective = inertia + e[48];
        if effective > EPS {
            1.0 / effective
        } else {
            free
        }
    } else {
        free
    };
    let geom = dot(cross(sub(point, center), grad), axis);
    let flags = joint[25 + side] as u32;
    let stored = joint[23 + side];
    let solve_grad = if flags & 1 != 0 && e[57] != 0.0 {
        stored
    } else {
        geom
    };
    let weight = joint[38];
    Spin {
        id,
        active: true,
        axis,
        free,
        inv,
        grad: solve_grad,
        grad_f: weight * solve_grad,
        load_grad: weight * if flags & 2 != 0 { stored } else { solve_grad },
        disp: solve_grad * spin_angle(e),
        flags,
        limit: e[61],
    }
}
/// Rotor members (spools/rollers mounted on a body) of each body. They
/// counter-spin when a cable turns their carrier, so they enter its mass
/// matrix. Wheels turned by the material passing them this step are excluded.
pub(crate) fn rotor_members(es: &[Entity], kinematic: &[bool]) -> Vec<Vec<usize>> {
    let mut members = vec![Vec::new(); es.len()];
    for id in 0..es.len() {
        if !kinematic[id] && rotor(es, id).is_some() {
            members[es[id][50] as usize].push(id);
        }
    }
    members
}
fn rotor(es: &[Entity], id: usize) -> Option<(V, V, f64)> {
    let e = &es[id];
    if e[50] < 0.0 || e[56] == 0.0 || e[55] != 0.0 {
        return None;
    }
    let orient = world_quat(es, id)?;
    let axis = norm(rotate(orient, v(e, 44)));
    let local = norm(rotate(qnorm(conj(orient)), axis));
    let inertia = positive_inertia(dot(local, matrix_vec(e, 25, local)));
    if inertia <= INERTIA_EPS {
        return None;
    }
    let coupling = rotate(orient, matrix_vec(e, 25, local));
    let inverse = if e[57] != 0.0 {
        0.0
    } else {
        1.0 / (inertia + e[48])
    };
    Some((axis, coupling, inverse))
}
/// One body touched by a row: summed geometric (c) and tension-weighted (f)
/// gradients. `arm` keeps the unconditional rotational arm for damping.
#[derive(Clone, Copy, Default)]
struct Body {
    id: usize,
    gc: V,
    gf: V,
    ac: V,
    af: V,
    arm: V,
    coupled: bool,
}
// Coupled carrier / relative-rotor mass matrix. Include unloaded rotors:
// they counter-spin when a cable turns their carrier.
#[derive(Default)]
struct AngularResponse {
    denominator: f64,
    bodies: Vec<(usize, V)>,
    rotors: Vec<(usize, V, f64, Q)>,
}
fn angular_response(
    es: &[Entity],
    members: &[Vec<usize>],
    bodies: &mut [Body],
    spins: &[Spin],
) -> AngularResponse {
    let mut result = AngularResponse::default();
    // (carrier, geometric gradient, force gradient)
    let mut groups: Vec<(usize, V, V)> = Vec::new();
    for b in bodies.iter_mut() {
        let parent = es[b.id][50];
        let carrier = if parent >= 0.0 { parent as usize } else { b.id };
        if members[carrier].is_empty() {
            continue;
        }
        b.coupled = true;
        let index = groups.iter().position(|g| g.0 == carrier).unwrap_or_else(|| {
            groups.push((carrier, [0.0; 3], [0.0; 3]));
            groups.len() - 1
        });
        if parent < 0.0 {
            groups[index].1 = add(groups[index].1, b.ac);
            groups[index].2 = add(groups[index].2, b.af);
        }
    }
    for (carrier, gc, gf) in groups {
        let orient = qnorm(q(&es[carrier], 3));
        let axes = [
            rotate(orient, [1.0, 0.0, 0.0]),
            rotate(orient, [0.0, 1.0, 0.0]),
            rotate(orient, [0.0, 0.0, 1.0]),
        ];
        let mut tensor = [0.0; 9];
        for r in 0..3 {
            for c in 0..3 {
                for k in 0..3 {
                    for j in 0..3 {
                        tensor[r * 3 + c] += axes[k][r] * es[carrier][25 + k * 3 + j] * axes[j][c];
                    }
                }
            }
        }
        let mut rhs = gf;
        let mut rotors = Vec::new();
        for &id in &members[carrier] {
            let Some((axis, coupling, mut inverse)) = rotor(es, id) else {
                continue;
            };
            let mut grad = 0.0;
            let mut grad_f = 0.0;
            for spin in spins {
                if spin.active && spin.id == id {
                    inverse = spin.inv;
                    grad += spin.grad;
                    grad_f += spin.grad_f;
                }
            }
            for r in 0..3 {
                for c in 0..3 {
                    tensor[r * 3 + c] -= inverse * coupling[r] * coupling[c];
                }
            }
            rhs = add(rhs, scale(coupling, -inverse * grad_f));
            rotors.push((id, axis, coupling, inverse, grad, grad_f));
        }
        let angular = matrix_vec(&engine::inverse_tensor(tensor), 0, rhs);
        result.denominator += dot(gc, angular);
        result.bodies.push((carrier, angular));
        for (id, axis, coupling, inverse, grad, grad_f) in rotors {
            let angle = inverse * (grad_f - dot(coupling, angular));
            result.denominator += grad * angle;
            result.rotors.push((id, axis, angle, orient));
        }
    }
    result
}
impl AngularResponse {
    fn owns(&self, id: usize) -> bool {
        self.rotors.iter().any(|r| r.0 == id)
    }
    fn apply(&self, es: &mut [Entity], l: f64) {
        for &(body, angular) in &self.bodies {
            let delta = scale(angular, -l);
            let angle = length(delta);
            if angle > 1e-12 {
                let orient = qnorm(mul(
                    axis_angle(scale(delta, 1.0 / angle), angle),
                    q(&es[body], 3),
                ));
                put_q(&mut es[body], 3, orient);
                es[body][64] = 1.0;
            }
        }
        for &(id, axis, coefficient, body_q) in &self.rotors {
            let angle = -l * coefficient;
            if angle.abs() <= 1e-12 {
                continue;
            }
            let old_local = q(&es[id], 17);
            let local_axis = rotate(qnorm(conj(body_q)), axis);
            let local = qnorm(mul(axis_angle(local_axis, angle), old_local));
            let raw = qnorm(mul(mul(q(&es[id], 3), qnorm(conj(old_local))), local));
            put_q(&mut es[id], 17, local);
            put_q(&mut es[id], 3, raw);
            es[id][64] = 1.0;
            es[id][65] = 1.0;
        }
    }
}
/// Generalized inverse mass of a row: J_c M⁻¹ J_fᵀ.
fn mechanical(es: &[Entity], bodies: &[Body], spins: &[Spin], response: &AngularResponse) -> f64 {
    let mut d = 0.0;
    for b in bodies {
        let e = &es[b.id];
        d += e[43] * dot(b.gc, b.gf);
        if !b.coupled && e[55] != 0.0 {
            d += positive_inertia(dot(b.ac, world_inverse(e, b.af)));
        }
    }
    for s in spins {
        if s.active && !response.owns(s.id) && s.inv > 0.0 && s.grad.abs() > EPS {
            d += s.inv * s.grad * s.grad_f;
        }
    }
    d + response.denominator
}
/// One XPBD row. A single cable segment, or several segments that share one
/// tension because the guides between them slip (frictionless, or at the
/// capstan limit). Material moves freely inside a row, so its constraint is
/// the row's total length C = Σdᵢ − Σrᵢ (plus a gate's hidden flow), while
/// segment i pulls with ρᵢ·T: forces use the weighted gradient Σρᵢ∇dᵢ.
#[derive(Clone, Copy, Default)]
pub(crate) struct Row {
    pub start: usize,
    pub end: usize,
    pub iterations: usize,
    /// Damping only: the cable has no stiffness.
    pub zero: bool,
    /// Σρᵢcᵢ / dt², with cᵢ = rᵢ / EA.
    pub alpha: f64,
    /// Relaxation time B/EA over dt (the same for every segment).
    pub gamma: f64,
    pub damping_step: f64,
    pub inv_dt2: f64,
    /// Row stiffness and damping seen by motors.
    pub spring: f64,
    pub damping: f64,
    /// A gate's hidden flow coordinate shared with the row on its other side
    /// (`usize::MAX` for none). It adds `flow_sign * q` of available span.
    pub flow: usize,
    pub flow_sign: f64,
}
/// A gate's hidden material flow q through its reeving, with the reflected
/// bearing inertia as its mass and dry bearing friction as a friction row.
#[derive(Clone, Copy, Default)]
pub(crate) struct Flow {
    pub q: f64,
    pub start: f64,
    pub w: f64,
    /// Friction multiplier bound (Coulomb force · dt²) and its accumulation.
    pub limit: f64,
    pub friction: f64,
}
impl Flow {
    /// Dry friction resists this step's flow, holding it while the tension
    /// difference stays below breakaway and opposing it with the Coulomb
    /// force while it slips.
    pub(crate) fn solve_friction(&mut self) {
        if self.w <= 0.0 || self.limit <= 0.0 {
            return;
        }
        let total = (self.friction - (self.q - self.start) / self.w).clamp(-self.limit, self.limit);
        self.q += self.w * (total - self.friction);
        self.friction = total;
    }
}
fn lambda(row: &Row, error: f64, disp: f64, denom: f64, previous: f64) -> Option<f64> {
    // Cable damping may cancel tension while shortening, but cannot push.
    if row.zero {
        let delta = (row.damping_step * disp - previous) / (1.0 + row.damping_step * denom);
        return Some((previous + delta).min(0.0) - previous);
    }
    let d = (1.0 + row.gamma) * denom + row.alpha;
    if d <= EPS {
        None
    } else {
        let delta = (-error - row.alpha * previous + row.gamma * disp) / d;
        Some((previous + delta).min(0.0) - previous)
    }
}
/// Cable torque on a spool. Position motors keep it for the next step's
/// servo current limit; only torque mode also records its observation flags
/// and stiffness/damping coefficients.
fn load(es: &mut [Entity], s: Spin, row: &Row, lambda: f64, record_coefficients: bool) {
    if !s.active || row.inv_dt2 <= 0.0 || s.load_grad.abs() <= EPS {
        return;
    }
    let torque = lambda * row.inv_dt2 * s.load_grad;
    if !torque.is_finite() || torque == 0.0 {
        return;
    }
    let e = &mut es[s.id];
    e[59] += torque;
    if e[57] == 0.0 {
        return;
    }
    e[68] = (e[68] as u32 | 1) as f64;
    if s.flags & 4 != 0 && record_coefficients {
        let g = s.load_grad * s.load_grad;
        if row.spring > 0.0 {
            e[60] += row.spring * g;
            e[68] = (e[68] as u32 | 2) as f64;
        }
        if row.damping > 0.0 {
            e[58] += row.damping * g;
            e[68] = (e[68] as u32 | 4) as f64;
        }
    }
}
fn transfer(js: &mut [Joint], id: f64, force: f64) {
    if id >= 0.0 && force.is_finite() && force > 0.0 {
        let j = &mut js[id as usize];
        j[36] = j[36].max(force);
    }
}
// Transfer only final force estimates, never intermediate iteration peaks.
pub(crate) fn finalize_force_transfers(js: &mut [Joint]) {
    for idx in 0..js.len() {
        let j = js[idx];
        let flags = j[37] as u32;
        if flags == 0 {
            continue;
        }
        transfer(js, j[29], j[35]);
        transfer(js, j[30], j[35]);
    }
}
fn correct(es: &mut [Entity], b: &Body, l: f64) {
    let id = b.id;
    if es[id][43] > 0.0 {
        let delta = scale(b.gf, -es[id][43] * l);
        let pos = add(v(&es[id], 0), delta);
        put_v(&mut es[id], 0, pos);
        es[id][63] = 1.0;
    }
    if !b.coupled && es[id][55] != 0.0 && es[id][52] != 0.0 {
        let delta = scale(world_inverse(&es[id], b.af), -l);
        let angle = length(delta);
        if angle > EPS {
            let dq = axis_angle(scale(delta, 1.0 / angle), angle);
            let result = qnorm(mul(dq, q(&es[id], 3)));
            put_q(&mut es[id], 3, result);
            es[id][64] = 1.0;
            update_local(es, id);
        }
    }
}
fn correct_spin(es: &mut [Entity], s: &Spin, l: f64) {
    if !s.active || s.inv <= 0.0 || s.grad.abs() <= EPS {
        return;
    }
    let angle = -s.inv * l * s.grad_f;
    if es[s.id][52] != 0.0 && angle.abs() > EPS {
        let result = qnorm(mul(axis_angle(s.axis, angle), q(&es[s.id], 3)));
        put_q(&mut es[s.id], 3, result);
        es[s.id][64] = 1.0;
        update_local(es, s.id);
    }
}
/// A row's current geometry: its bodies and spins, total error, and each
/// segment's direction and spin flags.
struct Gathered {
    bodies: Vec<Body>,
    spins: Vec<Spin>,
    error: f64,
    directions: [V; 16],
    joint_flags: [u32; 16],
}
fn gather(es: &[Entity], js: &[Joint], row: &Row, flow: &Flow) -> Gathered {
    let mut g = Gathered {
        bodies: Vec::with_capacity(4),
        spins: Vec::with_capacity(2),
        error: row.flow_sign * flow.q,
        directions: [[0.0; 3]; 16],
        joint_flags: [1; 16],
    };
    for (n, &j) in js[row.start..row.end].iter().enumerate() {
        let k = n.min(15);
        let weight = j[38];
        let points = [
            attachment(es, j[0] as usize, v(&j, 4)),
            attachment(es, j[1] as usize, v(&j, 7)),
        ];
        let distance = length(sub(points[1], points[0]));
        g.error += distance - j[10];
        if distance <= EPS {
            continue;
        }
        let direction = scale(sub(points[1], points[0]), 1.0 / distance);
        g.directions[k] = direction;
        for side in 0..2 {
            // A member carried by its body acts on that body: the forces of a
            // span between two members of one body cancel on it.
            let mut id = j[2 + side] as usize;
            if es[id][50] >= 0.0 && es[id][49] as u32 & 1 != 0 {
                id = es[id][50] as usize;
            }
            if es[id][51] == 0.0 {
                continue;
            }
            let grad = if side == 0 { direction } else { scale(direction, -1.0) };
            let s = spin(es, &j, side, points[side], grad);
            if s.active {
                g.joint_flags[k] |= 2 << side;
                match g.spins.iter_mut().find(|x| x.id == s.id) {
                    Some(x) => {
                        x.grad += s.grad;
                        x.grad_f += s.grad_f;
                        x.load_grad += s.load_grad;
                        x.disp += s.disp;
                    }
                    None => g.spins.push(s),
                }
            }
            let arm = cross(sub(points[side], v(&es[id], 0)), grad);
            let slot = match g.bodies.iter().position(|b| b.id == id) {
                Some(slot) => slot,
                None => {
                    g.bodies.push(Body { id, ..Body::default() });
                    g.bodies.len() - 1
                }
            };
            let b = &mut g.bodies[slot];
            b.gc = add(b.gc, grad);
            b.gf = add(b.gf, scale(grad, weight));
            b.arm = add(b.arm, arm);
            if es[id][55] != 0.0 {
                b.ac = add(b.ac, arm);
                b.af = add(b.af, scale(arm, weight));
            }
        }
    }
    g
}
#[allow(clippy::too_many_arguments)]
pub(crate) fn solve_row(
    es: &mut [Entity],
    js: &mut [Joint],
    members: &[Vec<usize>],
    flows: &mut [Flow],
    row: &Row,
    iter: usize,
    accumulated: &mut f64,
) {
    let flow = flows.get(row.flow).copied().unwrap_or_default();
    let Gathered { mut bodies, mut spins, error, directions, joint_flags } = gather(es, js, row, &flow);
    if error <= EPS && *accumulated == 0.0 {
        return;
    }
    // XPBD damping acts on the row's lengthening this step at the current
    // iterate: ∇C·(x − xⁿ) over every body, spool and gate flow it touches.
    let mut disp = spins.iter().map(|s| s.disp).sum::<f64>() - row.flow_sign * (flow.q - flow.start);
    for b in &bodies {
        let e = &es[b.id];
        if e[53] != 0.0 {
            disp += dot(b.gc, sub(v(e, 0), v(e, 7)));
        }
        disp += dot(b.arm, rotation_vector(e));
    }

    let mut response = angular_response(es, members, &mut bodies, &spins);
    let Some(mut l) = lambda(row, error, disp, mechanical(es, &bodies, &spins, &response) + flow.w, *accumulated) else {
        return;
    };
    let mut released = false;
    for s in &mut spins {
        if s.active
            && s.limit.is_finite()
            && s.limit >= 0.0
            && row.inv_dt2 > 0.0
            && s.grad_f.abs() > EPS
            && (l + *accumulated).abs() * row.inv_dt2 * s.grad_f.abs() > s.limit + EPS
        {
            s.inv = s.free;
            released = true;
        }
    }
    if released {
        response = angular_response(es, members, &mut bodies, &spins);
        let Some(new_l) =
            lambda(row, error, disp, mechanical(es, &bodies, &spins, &response) + flow.w, *accumulated)
        else {
            return;
        };
        l = new_l;
    }
    *accumulated += l;
    // Sum signed corrections into torque, counting K/D only once.
    for s in &spins {
        load(es, *s, row, l, iter == 0);
    }
    // Force uses the total multiplier, including immobile compliant spans.
    for (n, j) in js[row.start..row.end].iter_mut().enumerate() {
        let k = n.min(15);
        let share = j[38] * *accumulated;
        j[37] = joint_flags[k] as f64;
        j[31] = share;
        put_v(j, 32, scale(directions[k], share * row.inv_dt2));
        j[35] = share.abs() * row.inv_dt2;
    }
    if let Some(f) = flows.get_mut(row.flow) {
        f.q += f.w * l * row.flow_sign;
    }
    if mechanical(es, &bodies, &spins, &response) <= EPS {
        return;
    }
    response.apply(es, l);
    for b in &bodies {
        correct(es, b, l);
    }
    for s in &spins {
        if !response.owns(s.id) {
            correct_spin(es, s, l);
        }
    }
}
#[no_mangle]
pub extern "C" fn abi_version() -> u32 {
    3
}
#[no_mangle]
pub extern "C" fn alloc_buffer(len: usize) -> *mut f64 {
    Box::into_raw(vec![0.0; len].into_boxed_slice()) as *mut f64
}
/// The JS owner supplies the exact allocation length; buffers never alias.
///
/// # Safety
/// `ptr` and `len` must be an outstanding allocation from `alloc_buffer`.
#[no_mangle]
pub unsafe extern "C" fn free_buffer(ptr: *mut f64, len: usize) {
    drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)));
}
