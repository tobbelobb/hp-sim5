// Shared scene builders and analytic references for the numerical-integration probes.
// Everything runs on the checked-in production Wasm build; no native addon, no build.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { WasmPhysicsEngine } from '../../../../src/js/cable_joints_3d/wasm_physics_engine.js';
import {
  AngularVelocityComponent, MomentOfInertiaComponent, OrientationComponent, PositionComponent,
  SceneEntityInfoComponent, VelocityComponent,
} from '../../../../src/js/cable_joints_3d/ecs.js';

export const G = 9.81;
export const wasmBytes = fs.readFileSync(new URL('../../../../public/wasm/cable_solver.wasm', import.meta.url));
export const wasmModule = new WebAssembly.Module(wasmBytes);

export const header = (tps = 500, gravity = [0, -1, 0], magnitude = G) => `#usda 1.0
(
    defaultPrim = "Scene"
    timeCodesPerSecond = ${tps}
)
def Xform "World" {
    def PhysicsScene "PhysicsScene" {
        vector3d physics:gravityDirection = (${gravity.join(', ')})
        double physics:gravityMagnitude = ${magnitude}
    }
    def Xform "Scene" {
`;
export const footer = `    }
}`;
export const sceneText = (parts, tps = 500, gravity, magnitude) =>
  header(tps, gravity, magnitude) + parts.filter(Boolean).join('\n') + '\n' + footer;

export function wheelPrim(name, p, { mass = 1, radius = 0.01, inertia = 1e-3, tags = ['Wheel'] } = {}) {
  return `        def Circle "${name}" {
            token[] ecs:tags = ${JSON.stringify(tags)}
            double3 xformOp:translate = (${p.join(', ')})
            double radius = ${radius}
            double physics:mass = ${mass}
            double3 physics:velocity = (0, 0, 0)
            double3 physics:angularVelocity = (0, 0, 0)
            matrix3d physics:inertiaTensor = ((${inertia}, 0, 0), (0, ${inertia}, 0), (0, 0, ${inertia}))
            bool cable:linkable = 1
        }`;
}
export function pointPrim(name, p, mass) {
  return `        def Circle "${name}" {
            token[] ecs:tags = ["Pinhole"]
            double3 xformOp:translate = (${p.join(', ')})
            double radius = 0.002
            double physics:mass = ${mass}
            double3 physics:velocity = (0, 0, 0)
            bool cable:linkable = 1
        }`;
}
export function anchorPrim(name, p) {
  return `        def Xform "${name}" {
            token[] ecs:tags = ["Anchor"]
            double3 xformOp:translate = (${p.join(', ')})
            bool cable:linkable = 1
        }`;
}
export function jointPrim(name, a, b, { rest, localA = [0, 0, 0], localB = [0, 0, 0] }) {
  return `        def CableJoint "${name}" {
            rel physics:body0 = </World/Scene/${a}>
            rel physics:body1 = </World/Scene/${b}>
            double restLength = ${rest}
            point3d localPos0 = (${localA.join(', ')})
            point3d localPos1 = (${localB.join(', ')})
        }`;
}
export function pathPrim(name, joints, { links, cw = null, stored = null, ea = 1e7, damping = 0, iterations = 1, halfWidth = 0 }) {
  const n = links.length;
  const cwArr = cw ?? new Array(n).fill(1);
  const stArr = stored ?? new Array(n).fill(0);
  return `        def Xform "${name}" (apiSchemas = ["CablePathAPI"]) {
            rel cablePath:joints = [${joints.map(j => `</World/Scene/${j}>`).join(', ')}]
            token[] cablePath:linkTypes = [${links.map(x => `"${x}"`).join(', ')}]
            bool[] cablePath:clockwise = [${cwArr.map(Number).join(', ')}]
            double[] cablePath:stored = [${stArr.join(', ')}]
            token[] cablePath:storedMode = [${new Array(n).fill('"manual"').join(', ')}]
            double cablePath:stiffnessPerUnitLength = ${ea}
            double cablePath:dampingPerUnitLength = ${damping}
            double cablePath:halfWidth = ${halfWidth}
            int cablePath:solverIterations = ${iterations}
        }`;
}
export const dummyRigidBody = () => `        def RigidBody "Dummy" {
            rel rigidBody:members = [</World/Scene/DummyA>, </World/Scene/DummyB>]
        }
${wheelPrim('DummyA', [5, 5, 0], { mass: 0.1 })}
${wheelPrim('DummyB', [5.1, 5, 0], { mass: 0.1 })}`;

export function makeWorld(text, opts = {}) {
  const backend = new WasmPhysicsEngine(wasmModule);
  const made = createHeadlessWorld(text, { cableSolverBackend: backend, lineLayeringEnabled: true, ...opts });
  const byName = new Map();
  for (const id of made.world.query([SceneEntityInfoComponent])) {
    const info = made.world.getComponent(id, SceneEntityInfoComponent);
    if (!byName.has(info.name)) byName.set(info.name, id);
  }
  const pos = name => {
    const p = made.world.getComponent(byName.get(name), PositionComponent).pos;
    return [p.x, p.y, p.z];
  };
  const vel = name => {
    const v = made.world.getComponent(byName.get(name), VelocityComponent)?.vel;
    return v ? [v.x, v.y, v.z] : null;
  };
  const quat = name => {
    const q = made.world.getComponent(byName.get(name), OrientationComponent)?.quaternion;
    return q ? [q.x, q.y, q.z, q.w] : null;
  };
  const omega = name => {
    const v = made.world.getComponent(byName.get(name), AngularVelocityComponent)?.omega;
    return v ? [v.x, v.y, v.z] : null;
  };
  return { ...made, backend, byName, pos, vel, quat, omega,
    setOmega: (name, v) => {
      const o = made.world.getComponent(byName.get(name), AngularVelocityComponent).omega;
      o.x = v[0]; o.y = v[1]; o.z = v[2];
    },
    inertia: name => made.world.getComponent(byName.get(name), MomentOfInertiaComponent).inertiaTensor,
  };
}

// ---------------------------------------------------------------- oscillator
// A mass hangs from one compliant span. The engine's span compliance is
// L_current/EA, so the equilibrium is L/(1 - mg/EA) and the tangent stiffness
// at equilibrium is EA*L/x^2. The damping coefficient is B/L (B per unit length).
export function oscillatorScene({ m = 1, L = 1, EA = 1000, B = 6.324555320336759, A = 1e-3,
  dt = 0.002, iterations = 4, dummy = false, xeq }) {
  const eq = xeq ?? L / (1 - m * G / EA);
  return sceneText([
    anchorPrim('Anchor', [0, 0, 0]),
    wheelPrim('Bob', [0, -(eq + A), 0], { mass: m }),
    jointPrim('J0', 'Anchor', 'Bob', { rest: L }),
    pathPrim('Path', ['J0'], { links: ['attachment', 'attachment'], ea: EA, damping: B, iterations }),
    dummy ? dummyRigidBody() : '',
  ], 1 / dt);
}
export function settleEquilibrium({ m = 1, L = 1, EA = 1000, B = 6.324555320336759, dt = 0.002 }) {
  const w = makeWorld(oscillatorScene({ m, L, EA, B, A: 0, dt }), { requireRrfTimestep: false });
  const samples = [];
  for (let i = 0; i < 2500; i++) { w.world.update(dt); if (i > 2000) samples.push(-w.pos('Bob')[1]); }
  w.backend.dispose();
  return samples.reduce((s, x) => s + x, 0) / samples.length;
}
export function zeroCrossings(rows, key = 'u') {
  const out = [];
  for (let i = 1; i < rows.length; i++) {
    const a = rows[i - 1], b = rows[i];
    if ((a[key] > 0) !== (b[key] > 0) && a[key] !== b[key]) {
      out.push(a.t + (b.t - a.t) * a[key] / (a[key] - b[key]));
    }
  }
  return out;
}

// ---------------------------------------------------------------- pendulum
export function pendulumScene({ theta0 = 0.05, L = 1, EA = 1e7, dt = 0.002, iterations = 2, dummy = false }) {
  const x = L * Math.sin(theta0), y = -L * Math.cos(theta0);
  return sceneText([
    anchorPrim('Anchor', [0, 0, 0]),
    pointPrim('Bob', [x, y], 1),
    jointPrim('J0', 'Anchor', 'Bob', { rest: L }),
    pathPrim('Path', ['J0'], { links: ['attachment', 'attachment'], ea: EA, iterations }),
    dummy ? dummyRigidBody() : '',
  ], 1 / dt);
}

// ---------------------------------------------------------------- Atwood
export function atwoodScene({ r = 0.05, I = 0.0025, m1 = 2, m2 = 1, dt = 0.002, iterations = 8 }) {
  const arc = Math.PI * r;
  return sceneText([
    wheelPrim('Pulley', [0, 0, 0], { mass: -1, radius: r, inertia: I }),
    pointPrim('M1', [-r, -1], m1),
    pointPrim('M2', [r, -1], m2),
    jointPrim('J0', 'M1', 'Pulley', { rest: 1, localA: [0, 0, 0], localB: [-r, 0, 0] }),
    jointPrim('J1', 'Pulley', 'M2', { rest: 1, localA: [r, 0, 0], localB: [0, 0, 0] }),
    pathPrim('Path', ['J0', 'J1'], {
      links: ['attachment', 'rolling', 'attachment'], cw: [1, 1, 1], stored: [0, arc, 0],
      ea: 1e7, iterations,
    }),
  ], 1 / dt);
}

// Fit y = y0 + v0 t + (a/2) t^2 by least squares over a time window.
export function quadraticFit(rows, t0 = 0.05) {
  const win = rows.filter(r => r.t >= t0);
  const n = win.length;
  let st = 0, stt = 0, sttt = 0, stttt = 0, sy = 0, sty = 0, stty = 0;
  for (const { t, y } of win) { st += t; stt += t * t; sttt += t ** 3; stttt += t ** 4; sy += y; sty += t * y; stty += t * t * y; }
  const A = [[n, st, stt], [st, stt, sttt], [stt, sttt, stttt]];
  const b = [sy, sty, stty];
  for (let i = 0; i < 3; i++) {
    const p = A[i][i];
    for (let j = i; j < 3; j++) A[i][j] /= p;
    b[i] /= p;
    for (let k = 0; k < 3; k++) if (k !== i) {
      const f = A[k][i];
      for (let j = i; j < 3; j++) A[k][j] -= f * A[i][j];
      b[k] -= f * b[i];
    }
  }
  return { y0: b[0], v0: b[1], a: 2 * b[2] };
}

// ---------------------------------------------------------------- free rotation
export const rotScene = ({ A = 1, d = 0.5, memberI = 1e-3, dt = 0.002 } = {}) => sceneText([
  wheelPrim('A', [-d, 0, 0], { mass: A, inertia: memberI }),
  wheelPrim('B', [d, 0, 0], { mass: A, inertia: memberI }),
  `        def RigidBody "Body" {
            rel rigidBody:members = [</World/Scene/A>, </World/Scene/B>]
        }`,
], 1 / dt, [0, 0, 0], 0);

export const qmul = (a, b) => [
  a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]];
export const qnorm = q => { const n = Math.hypot(...q); return q.map(x => x / n); };
export const qconj = q => [-q[0], -q[1], -q[2], q[3]];
export const axisAngle = (axis, ang) => { const s = Math.sin(ang / 2), n = Math.hypot(...axis) || 1;
  return [axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, Math.cos(ang / 2)]; };
export const quatMul = qmul;
export const rotMatrix = q => { const [x, y, z, w] = q; return [
  [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]; };
export const mv = (M, v) => M.map(row => row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
export const mm = (A, B) => A.map(row => [0, 1, 2].map(c => row[0] * B[0][c] + row[1] * B[1][c] + row[2] * B[2][c]));
export const transpose = A => A[0].map((_, c) => [0, 1, 2].map(r => A[r][c]));
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const inv3 = A => { const [a, b, c] = A;
  const det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
  return [[(b[1] * c[2] - b[2] * c[1]) / det, (a[2] * c[1] - a[1] * c[2]) / det, (a[1] * b[2] - a[2] * b[1]) / det],
    [(b[2] * c[0] - b[0] * c[2]) / det, (a[0] * c[2] - a[2] * c[0]) / det, (a[2] * b[0] - a[0] * b[2]) / det],
    [(b[0] * c[1] - b[1] * c[0]) / det, (a[1] * c[0] - a[0] * c[1]) / det, (a[0] * b[1] - a[1] * b[0]) / det]]; };

// Exact torque-free motion in the world frame: L = R I0 R^T w is constant.
export function exactFree(I0, w0, q0, T, h) {
  let q = q0.slice(), w = w0.slice();
  const fq = (q, w) => qmul([0.5 * w[0], 0.5 * w[1], 0.5 * w[2], 0], q);
  const fw = (q, w) => { const R = rotMatrix(q), Iw = mm(mm(R, I0), transpose(R));
    return mv(inv3(Iw), cross(w, mv(Iw, w)).map(x => -x)); };
  const n = Math.round(T / h);
  for (let i = 0; i < n; i++) {
    const k1q = fq(q, w), k1w = fw(q, w);
    const q2 = qnorm(q.map((v, j) => v + 0.5 * h * k1q[j])), w2 = w.map((v, j) => v + 0.5 * h * k1w[j]);
    const k2q = fq(q2, w2), k2w = fw(q2, w2);
    const q3 = qnorm(q.map((v, j) => v + 0.5 * h * k2q[j])), w3 = w.map((v, j) => v + 0.5 * h * k2w[j]);
    const k3q = fq(q3, w3), k3w = fw(q3, w3);
    const q4 = qnorm(q.map((v, j) => v + h * k3q[j])), w4 = w.map((v, j) => v + h * k3w[j]);
    const k4q = fq(q4, w4), k4w = fw(q4, w4);
    q = qnorm(q.map((v, j) => v + h / 6 * (k1q[j] + 2 * k2q[j] + 2 * k3q[j] + k4q[j])));
    w = w.map((v, j) => v + h / 6 * (k1w[j] + 2 * k2w[j] + 2 * k3w[j] + k4w[j]));
  }
  return { q, w };
}
export const dot4 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
export const angBetween = (a, b) => 2 * Math.acos(Math.min(1, Math.abs(dot4(a, b))));
