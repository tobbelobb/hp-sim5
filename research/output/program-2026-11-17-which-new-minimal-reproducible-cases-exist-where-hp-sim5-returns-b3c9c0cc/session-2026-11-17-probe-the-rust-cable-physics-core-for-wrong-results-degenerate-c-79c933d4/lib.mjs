// Minimal native-Rust physics world builder for edge-case probes.
// Entity/path record layouts mirror src/wasm/cable_solver/tests/common/mod.rs
// and the JS adapter src/js/cable_joints_3d/rust_physics_engine.js.
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '../../../..');
const addonPath = path.join(repo, 'src/wasm/cable_solver/target/native/physics.node');
const addon = createRequire(import.meta.url)(addonPath);

export const E = 160, H = 9, L = 5, J = 16;
export const ATTACHMENT = 0, ROLLING = 1, PINHOLE = 2, HYBRID = 3, HYBRID_ATTACH = 4;
export const G = 9.81;

// Same defaults as the crate's own scenario builders (tests/common/mod.rs).
export function base(id, p) {
  const e = new Float64Array(E);
  e.set(p, 0); e.set(p, 7);
  for (const i of [6, 13, 20, 24, 82, 89, 96, 100]) e[i] = 1.0;
  e[50] = -1;
  e[51] = 1; e[53] = 1;
  e[104] = NaN; e[106] = NaN;
  e.fill(NaN, 115, 121); e[127] = NaN;
  e[128] = id;
  e.set([0, 0, 1], 101);
  e.set(p, 90);
  e.set([0, 0, 0, 1], 93); e.set([0, 0, 0, 1], 97);
  return e;
}

export function fixed(id, p, mu = 0) {
  const e = base(id, p);
  e[75] = 256; // CABLE_LINK
  e[105] = mu;
  return e;
}

export function mass(id, p, m) {
  const e = base(id, p);
  e[75] = 1 | 4 | 128 | 256; // VEL | GRAVITY | MASS | CABLE_LINK
  e[124] = m;
  return e;
}

// A pulley/spool in the xy plane, axis +z. `spool` adds the SPOOL flag used by
// the JS adapter for spools/wheels; `motor` adds the stepper motor record.
export function spool(id, p, r, { mu = 0, inertia = 0.01, spool = true, motor = false,
  axis = [0, 0, 1], radius = r } = {}) {
  const e = base(id, p);
  e[75] = (spool ? 8 : 0) | 2 | 64 | 256 | 512 | 1024; // OMEGA|INERTIA|LINK|LOCAL_AXIS|RADIUS
  e[52] = 1; e[54] = 1;
  e.set(axis, 44); // cable plane normal == spool axis in the crate tests
  for (let k = 0; k < 3; k++) { e[25 + 4 * k] = inertia; e[34 + 4 * k] = 1 / inertia; }
  e[56] = 1;
  e[104] = radius;
  e[105] = mu;
  e.set(axis, 76);
  e.set([0, 0, 0, 1], 79);
  if (motor) {
    e[75] += 16; // MOTOR
    e[57] = 0; // position motor
    e.set([0, 0, 0, 50, 0, 600, 0, 0], 107);
    e[109] = 1; // holding torque
    e[66] = 0; e[67] = 1;
  }
  return e;
}

export function wheel(id, p, r, mu, inertia) {
  const e = spool(id, p, r, { mu, inertia });
  e[67] = 1; // encoder
  return e;
}

// links: [{ kind, cw, stored }] length n+1; segs: [{ a, b, rest, pa, pb }]
export function pathRecord(links, segs, { stiffness = 1e5, damping = 0, iterations = 4,
  halfWidth = 0, totalRestLength = undefined } = {}) {
  const n = segs.length;
  if (links.length !== n + 1) throw new Error('links must be n+1');
  const d = new Float64Array(H + (n + 1) * L + n * J);
  const total = totalRestLength ?? segs.reduce((s, x) => s + x.rest, 0)
    + links.reduce((s, x) => s + x.stored, 0);
  d.set([n, 1000, stiffness, 1 / stiffness, damping, iterations, halfWidth, total, 0], 0);
  links.forEach((l, i) => {
    d.set([l.kind, +l.cw, l.stored, NaN, 0], H + i * L);
  });
  segs.forEach((s, i) => {
    const o = H + (n + 1) * L + i * J;
    d.set([2000 + i, s.a, s.b, s.rest], o);
    d.set(s.pa, o + 4);
    d.set(s.pb, o + 7);
  });
  return d;
}

export function stack(paths) {
  let len = 0;
  for (const p of paths) len += 1 + p.length;
  const out = new Float64Array(len);
  let o = 0;
  for (const p of paths) { out[o++] = p.length; out.set(p, o); o += p.length; }
  return out;
}

// The crate test-world defaults: gravity -9.81 in y, attachments and rest
// clamping on, layering off.
export function world(entities, paths, opts = {}) {
  const bodies = new Float64Array(entities.length * E);
  entities.forEach((e, i) => bodies.set(e, i * E));
  const h = addon.create(bodies, stack(paths), paths.length);
  const o = h.options();
  o[1] = -G;
  o[4] = 1; o[5] = 1; o[9] = -1; o[12] = 1; o[13] = 1;
  if (opts.layering) {
    o[3] = 1;
    o[6] = opts.merge === undefined ? 1 : (opts.merge ? 1 : 0);
    o[7] = opts.split === undefined ? 1 : (opts.split ? 1 : 0);
    o[8] = opts.hybrid === undefined ? 1 : (opts.hybrid ? 1 : 0);
  }
  const step = (dt = 0.001) => {
    if (h.step(dt) !== undefined) {}
    return h;
  };
  return {
    host: h,
    entities: () => h.entities(),
    options: () => h.options(),
    path: i => h.path(i),
    step,
    dispose: () => h.dispose(),
    tension(p, i) {
      const d = h.path(p); const n = d[0];
      return d[H + (n + 1) * L + i * J + 11];
    },
    rest(p, i) {
      const d = h.path(p); const n = d[0];
      return d[H + (n + 1) * L + i * J + 3];
    },
    lambda(p, i) {
      const d = h.path(p); const n = d[0];
      return d[H + (n + 1) * L + i * J + 10];
    },
    stored(p, li) { return h.path(p)[H + li * L + 2]; },
    linkKind(p, li) { return h.path(p)[H + li * L]; },
    n(p) { return h.path(p)[0]; },
  };
}

export const pos = (w, id) => Array.from(w.entities().slice(id * E, id * E + 3));
export const vel = (w, id) => Array.from(w.entities().slice(id * E + 69, id * E + 72));
export function allFinite(w) {
  const e = w.entities();
  for (let i = 0; i < e.length; i++) if (!Number.isFinite(e[i])) return false;
  for (let p = 0; p < 1e3; p++) {
    let d; try { d = w.path(p); } catch { break; }
    for (let i = 0; i < d.length; i++) if (!Number.isFinite(d[i])) return false;
  }
  return true;
}
export function nonFiniteEntities(w) {
  const e = w.entities(); const out = [];
  for (let id = 0; id * E < e.length; id++) {
    for (let i = 0; i < E; i++) if (!Number.isFinite(e[id * E + i])) { out.push([id, i, e[id * E + i]]); break; }
  }
  return out;
}
export function nonFinitePath(w, p) {
  const d = w.path(p); const out = [];
  for (let i = 0; i < d.length; i++) if (!Number.isFinite(d[i])) out.push([i, d[i]]);
  return out;
}
export { addon };
