//! Batched f64 XPBD cable iterations. The packed ABI is mirrored in
//! src/js/cable_joints_3d/wasm_cable_solver.js. No ECS calls inside the loop.
use std::f64::consts::PI;

const EPS: f64 = 1e-9;
const INERTIA_EPS: f64 = 1e-12;
const E: usize = 69;
const J: usize = 38;
const P: usize = 10;
type V = [f64; 3];
type Q = [f64; 4];
type Entity = [f64; E];
type Joint = [f64; J];
type Path = [f64; P];

// Preserve JS transcendental rounding. Elementary arithmetic and sqrt stay in
// WASM; Rust/LLVM's strict floating point retains the JS operation order.
#[link(wasm_import_module = "math")]
extern "C" {
    fn sin(x: f64) -> f64;
    fn cos(x: f64) -> f64;
    fn acos(x: f64) -> f64;
    fn asin(x: f64) -> f64;
    fn atan2(y: f64, x: f64) -> f64;
}
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
    let s = unsafe { sin(half) };
    [axis[0] * s, axis[1] * s, axis[2] * s, unsafe { cos(half) }]
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
    let angle = 2.0 * unsafe { acos(w) };
    if angle <= EPS {
        return [0.0; 3];
    }
    let s = (1.0 - w * w).max(0.0).sqrt();
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
    let mut angle = 2.0 * unsafe { atan2(s, twist[3]) };
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
    let e = es[id];
    if e[50] < 0.0 || e[52] == 0.0 {
        return;
    }
    let body = &es[e[50] as usize];
    if body[52] == 0.0 {
        return;
    }
    let local = qnorm(mul(qnorm(conj(q(body, 3))), q(&e, 3)));
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
    let alpha = unsafe { atan2(d[1], d[0]) };
    let phi = unsafe { asin(r / d_sq.sqrt()) };
    let angle = if cw == first {
        alpha + phi + PI / 2.0
    } else {
        alpha - phi - PI / 2.0
    };
    [
        center[0] + r * unsafe { cos(angle) },
        center[1] + r * unsafe { sin(angle) },
    ]
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
    let alpha = unsafe { atan2(delta[1], delta[0]) };
    let phi = unsafe { asin((r / d).clamp(-1.0, 1.0)) };
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
        [
            a[0] + ra * unsafe { cos(aa) },
            a[1] + ra * unsafe { sin(aa) },
        ],
        [
            b[0] + rb * unsafe { cos(ab) },
            b[1] + rb * unsafe { sin(ab) },
        ],
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
    let c = unsafe { cos(angle) };
    let s = unsafe { sin(angle) };
    add(
        add(scale(point, c), scale(cross(n, point), s)),
        scale(n, dot(point, n) * (1.0 - c)),
    )
}
/// # Safety
/// `ptr` must address 46 writable, aligned f64 slots allocated by this instance.
#[no_mangle]
pub unsafe extern "C" fn attachment_points(ptr: *mut f64) {
    let data = std::slice::from_raw_parts_mut(ptr, 46);
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
    grad: f64,
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
    let axis = rotate(q(e, 3), v(e, 44));
    if dot(axis, axis) <= EPS {
        return Spin::default();
    }
    let axis = norm(axis);
    // Match both normalizations in effectiveInertiaAboutWorldAxis/localAxis.
    let local = norm(rotate(qnorm(conj(q(e, 3))), norm(axis)));
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
    let coupled = joint[offset + 4] != 0.0;
    let spin_point = if coupled {
        attachment(es, id, v(joint, offset + 1))
    } else {
        point
    };
    let spin_grad = if coupled {
        let g = sub(point, spin_point);
        if dot(g, g) <= EPS {
            return Spin::default();
        }
        norm(g)
    } else {
        grad
    };
    let geom = dot(cross(sub(spin_point, center), spin_grad), axis);
    let flags = joint[25 + side] as u32;
    let stored = joint[23 + side];
    let solve_grad = if flags & 1 != 0 && e[57] != 0.0 {
        stored
    } else {
        geom
    };
    let load_grad = if flags & 2 != 0 { stored } else { solve_grad };
    Spin {
        id,
        active: true,
        axis,
        free,
        inv,
        grad: solve_grad,
        load_grad,
        disp: solve_grad * spin_angle(e),
        flags,
        limit: e[61],
    }
}
// Explicit endpoint terms retain the JavaScript equation's summation order.
#[allow(clippy::too_many_arguments)]
fn mechanical(a: &Entity, b: &Entity, ga: V, gb: V, aa: f64, ab: f64, sa: Spin, sb: Spin) -> f64 {
    let mut d = 0.0;
    d += a[43] * dot(ga, ga);
    d += aa;
    if sa.inv > 0.0 && sa.grad.abs() > EPS {
        d += sa.inv * sa.grad * sa.grad;
    }
    d += b[43] * dot(gb, gb);
    d += ab;
    if sb.inv > 0.0 && sb.grad.abs() > EPS {
        d += sb.inv * sb.grad * sb.grad;
    }
    d
}
fn lambda(path: &Path, error: f64, disp: f64, denom: f64) -> Option<f64> {
    if path[3] != 0.0 {
        return Some(path[6] * disp / (1.0 + path[6] * denom));
    }
    let d = (1.0 + path[5]) * denom + path[4];
    if d <= EPS {
        None
    } else {
        Some((-error + path[5] * disp) / d)
    }
}
fn load(es: &mut [Entity], s: Spin, path: &Path, lambda: f64) {
    if !s.active
        || es[s.id][57] == 0.0
        || s.flags & 8 != 0
        || path[7] <= 0.0
        || s.load_grad.abs() <= EPS
    {
        return;
    }
    let torque = lambda * path[7] * s.load_grad;
    if !torque.is_finite() || torque.abs() <= EPS {
        return;
    }
    let e = &mut es[s.id];
    e[59] += torque;
    e[68] = (e[68] as u32 | 1) as f64;
    if s.flags & 4 != 0 {
        let g = s.load_grad * s.load_grad;
        if path[8] > 0.0 {
            e[60] += path[8] * g;
            e[68] = (e[68] as u32 | 2) as f64;
        }
        if path[9] > 0.0 {
            e[58] += path[9] * g;
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
fn correct(es: &mut [Entity], id: usize, grad: V, angular: V, angular_denom: f64, s: Spin, l: f64) {
    if es[id][43] > 0.0 {
        let delta = scale(grad, -es[id][43] * l);
        let pos = add(v(&es[id], 0), delta);
        put_v(&mut es[id], 0, pos);
        es[id][63] = 1.0;
    }
    if angular_denom > 0.0 && es[id][52] != 0.0 {
        let delta = scale(world_inverse(&es[id], angular), -l);
        let angle = length(delta);
        if angle > EPS {
            let dq = axis_angle(scale(delta, 1.0 / angle), angle);
            let result = qnorm(mul(dq, q(&es[id], 3)));
            put_q(&mut es[id], 3, result);
            es[id][64] = 1.0;
            update_local(es, id);
        }
    }
    if s.active && s.inv > 0.0 && s.grad.abs() > EPS {
        let angle = -s.inv * l * s.grad;
        if es[s.id][52] != 0.0 && angle.abs() > EPS {
            let result = qnorm(mul(axis_angle(s.axis, angle), q(&es[s.id], 3)));
            put_q(&mut es[s.id], 3, result);
            es[s.id][64] = 1.0;
            update_local(es, s.id);
            if angle.is_finite() && es[s.id][67] != 0.0 {
                es[s.id][66] += angle;
            }
        }
    }
}
fn solve_joint(es: &mut [Entity], js: &mut [Joint], path: &Path, idx: usize, iter: usize) {
    let j = js[idx];
    let a_id = j[2] as usize;
    let b_id = j[3] as usize;
    let pa = attachment(es, j[0] as usize, v(&j, 4));
    let pb = attachment(es, j[1] as usize, v(&j, 7));
    let distance = length(sub(pa, pb));
    if distance <= EPS {
        return;
    }
    let ga = scale(sub(pb, pa), 1.0 / distance);
    let gb = scale(ga, -1.0);
    if dot(ga, ga) <= EPS {
        return;
    }
    let error = distance - j[10];
    if error <= EPS {
        return;
    }
    let a = es[a_id];
    let b = es[b_id];
    if a[51] == 0.0 || b[51] == 0.0 {
        return;
    }
    let mut sa = spin(es, &j, 0, pa, ga);
    let mut sb = spin(es, &j, 1, pb, gb);
    let aa = cross(sub(pa, v(&a, 0)), ga);
    let ab = cross(sub(pb, v(&b, 0)), gb);
    let da = if a[55] != 0.0 {
        positive_inertia(dot(aa, world_inverse(&a, aa)))
    } else {
        0.0
    };
    let db = if b[55] != 0.0 {
        positive_inertia(dot(ab, world_inverse(&b, ab)))
    } else {
        0.0
    };
    let ta = if a[53] != 0.0 {
        dot(ga, sub(v(&a, 0), v(&a, 7)))
    } else {
        0.0
    };
    let tb = if b[53] != 0.0 {
        dot(gb, sub(v(&b, 0), v(&b, 7)))
    } else {
        0.0
    };
    let ra = dot(aa, rotation_vector(&a));
    let rb = dot(ab, rotation_vector(&b));
    let disp = ta + ra + sa.disp + tb + rb + sb.disp;
    let Some(mut l) = lambda(
        path,
        error,
        disp,
        mechanical(&a, &b, ga, gb, da, db, sa, sb),
    ) else {
        return;
    };
    let release = |s: Spin| {
        s.active
            && s.limit.is_finite()
            && s.limit >= 0.0
            && path[7] > 0.0
            && s.grad.abs() > EPS
            && l.abs() * path[7] * s.grad.abs() > s.limit + EPS
    };
    let release_a = release(sa);
    let release_b = release(sb);
    if release_a || release_b {
        if release_a {
            sa.inv = sa.free;
        }
        if release_b {
            sb.inv = sb.free;
        }
        let Some(new_l) = lambda(
            path,
            error,
            disp,
            mechanical(&a, &b, ga, gb, da, db, sa, sb),
        ) else {
            return;
        };
        l = new_l;
    }
    load(es, sa, path, l);
    load(es, sb, path, l);
    if mechanical(&a, &b, ga, gb, da, db, sa, sb) <= EPS {
        return;
    }
    js[idx][37] = 1.0;
    if iter == 0 {
        js[idx][31] = l;
        put_v(&mut js[idx], 32, scale(ga, l * path[7]));
        let force = l.abs() * path[7];
        js[idx][35] = force;
        transfer(js, j[29], force);
        transfer(js, j[30], force);
        if sa.active {
            transfer(js, j[27], force);
        }
        if sb.active && (!sa.active || j[28] != j[27]) {
            transfer(js, j[28], force);
        }
    }
    correct(es, a_id, ga, aa, da, sa, l);
    correct(es, b_id, gb, ab, db, sb, l);
}

#[no_mangle]
pub extern "C" fn abi_version() -> u32 {
    1
}
#[no_mangle]
pub extern "C" fn hypot3(x: f64, y: f64, z: f64) -> f64 {
    length([x, y, z])
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
#[no_mangle]
/// # Safety
/// `ptr` must contain entities*E + joints*J + paths*P aligned, initialized f64
/// slots. Record indices must reference valid entities/joints within this buffer.
pub unsafe extern "C" fn solve(ptr: *mut f64, entities: usize, joints: usize, paths: usize) {
    let es = std::slice::from_raw_parts_mut(ptr as *mut Entity, entities);
    let js = std::slice::from_raw_parts_mut(ptr.add(entities * E) as *mut Joint, joints);
    let ps = std::slice::from_raw_parts(ptr.add(entities * E + joints * J) as *const Path, paths);
    let max_iterations = ps.iter().map(|p| p[2] as usize).max().unwrap_or(1);
    for iter in 0..max_iterations {
        let forward = iter % 2 == 0;
        for k in 0..paths {
            let path = &ps[if forward { k } else { paths - 1 - k }];
            if iter >= path[2] as usize {
                continue;
            }
            let first = path[0] as usize;
            let count = path[1] as usize;
            for k in 0..count {
                let index = first + if forward { k } else { count - 1 - k };
                solve_joint(es, js, path, index, iter);
            }
        }
    }
}
