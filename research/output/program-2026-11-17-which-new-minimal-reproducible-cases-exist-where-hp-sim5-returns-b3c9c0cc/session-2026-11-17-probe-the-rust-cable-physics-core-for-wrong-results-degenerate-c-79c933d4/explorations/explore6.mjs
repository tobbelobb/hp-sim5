// Exploratory (corrected): mirror symmetry and spool payout kinematics.
import { mass, spool, pathRecord, world, pos, E, ATTACHMENT, ROLLING } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

// Valid Atwood hairpins over the TOP of a fixed roller.
// side=-1: heavy mass on the left, path left->top->right, cw=true.
// side=+1: heavy mass on the right, path right->top->left, cw=false.
function atwood({ r = 0.05, m1 = 2, m2 = 1, side = -1, mu = 0, inertia = 0, rest = 1 } = {}) {
  const cw = side < 0; // left start wraps clockwise over the top
  const es = [spool(0, P(0, 0), r, { mu, inertia, spool: false }),
    mass(1, P(side * (r + 1e-9), -1), m1), mass(2, P(-side * (r + 1e-9), -1), m2)];
  const arc = Math.PI * r;
  return world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw, stored: 0 }, { kind: ROLLING, cw, stored: arc },
      { kind: ATTACHMENT, cw, stored: 0 }],
    [{ a: 1, b: 0, rest, pa: P(side * (r + 1e-9), -1), pb: P(side * r, 0) },
      { a: 0, b: 2, rest, pa: P(-side * r, 0), pb: P(-side * (r + 1e-9), -1) }],
    { stiffness: 1e6, iterations: 8 })]);
}
const accel = (w, id) => {
  for (let i = 0; i < 200; i++) w.step(0.0005);
  const v0 = w.entities()[id * E + 70];
  for (let i = 0; i < 400; i++) w.step(0.0005);
  const v1 = w.entities()[id * E + 70];
  return -(v1 - v0) / (400 * 0.0005);
};
for (const side of [-1, 1]) {
  for (const [m1, m2] of [[2, 1], [1, 1]]) {
    const w = atwood({ side, m1, m2 });
    // measure from the heavier side's mass (entity 1)
    const a1 = accel(w, 1);
    console.log('atwood side', side, 'm', m1, m2, 'a1', a1.toFixed(6),
      'y1', pos(w, 1)[1].toFixed(6), 'y2', pos(w, 2)[1].toFixed(6),
      'T', w.tension(0, 0).toFixed(4), w.tension(0, 1).toFixed(4),
      'stored', w.stored(0, 1).toFixed(6));
  }
}
// Spool payout: mass hangs from a tangent point; spool turned about +z by dtheta.
function payout(r, dtheta, { stiffness = 1e7, mu = 0, inertia = 0.01, n = 20, sign = 1 } = {}) {
  const es = [spool(0, P(0, 0), r, { mu, inertia }), mass(1, P(r, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < 100; i++) w.step(0.001);
  const y0 = pos(w, 1)[1], s0 = w.stored(0, 0), r0 = w.rest(0, 0);
  for (let k = 0; k < n; k++) {
    const a = sign * dtheta * (k + 1) / n;
    ent[3] = 0; ent[4] = 0; ent[5] = Math.sin(a / 2); ent[6] = Math.cos(a / 2);
    w.step(0.001);
  }
  return { dy: pos(w, 1)[1] - y0, stored: w.stored(0, 0) - s0, rest: w.rest(0, 0) - r0,
    tension: w.tension(0, 0) };
}
for (const r of [0.05, 0.02, 0.005]) {
  for (const dtheta of [0.1, 0.5]) {
    for (const sign of [1, -1]) {
      const q = payout(r, dtheta, { sign });
      console.log('payout r', r, 'dtheta', sign * dtheta, 'dy', q.dy.toFixed(9),
        'expected', (-sign * r * dtheta).toFixed(9), 'err', (q.dy + sign * r * dtheta).toExponential(3),
        'dstored', q.stored.toFixed(9), 'drest', q.rest.toFixed(9), 'T', q.tension.toFixed(3));
    }
  }
}
