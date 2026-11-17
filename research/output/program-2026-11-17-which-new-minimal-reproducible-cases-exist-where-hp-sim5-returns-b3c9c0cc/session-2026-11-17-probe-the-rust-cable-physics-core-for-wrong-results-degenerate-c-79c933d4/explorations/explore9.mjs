// Exploratory: correct spool winding convention, payout equilibrium, zero-length tether.
import { fixed, mass, spool, pathRecord, world, pos, E, H, L, J, ATTACHMENT, ROLLING } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

// Pick the cw that keeps the tangent point at (+r, 0) for a mass below it.
for (const cw of [true, false]) {
  const r = 0.05;
  const es = [spool(0, P(0, 0), r, { inertia: 1e6 }), mass(1, P(r, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness: 2e4, damping: 50, iterations: 8 })]);
  w.step(0.001);
  const d = w.path(0), n = d[0];
  const pa = Array.from(d.slice(H + (n + 1) * L + 4, H + (n + 1) * L + 7));
  console.log('cw', cw, 'tangent A after 1 step', pa.map(x => x.toFixed(6)), 'stored', w.stored(0, 0).toFixed(6));
}

// Payout/wind equilibrium with the correct cw: rotate, then settle.
function payoutEq(r, dtheta, { cw = false, inertia = 1e6, stiffness = 2e4, damping = 50,
  n = 10, settle = 6000, dt = 0.001 } = {}) {
  const es = [spool(0, P(0, 0), r, { inertia }), mass(1, P(r, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness, damping, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < settle; i++) w.step(dt);
  const y0 = pos(w, 1)[1], s0 = w.stored(0, 0), r0 = w.rest(0, 0);
  for (let k = 0; k < n; k++) {
    const a = dtheta * (k + 1) / n;
    ent[3] = 0; ent[4] = 0; ent[5] = Math.sin(a / 2); ent[6] = Math.cos(a / 2);
    w.step(dt);
  }
  for (let i = 0; i < settle; i++) w.step(dt);
  return { dy: pos(w, 1)[1] - y0, expected: -r * dtheta, dstored: w.stored(0, 0) - s0,
    drest: w.rest(0, 0) - r0, T: w.tension(0, 0) };
}
for (const r of [0.05, 0.02]) {
  for (const dtheta of [1e-2, -1e-2]) {
    const q = payoutEq(r, dtheta);
    console.log('EQ payout r', r, 'dth', dtheta, 'dy', q.dy.toExponential(6), 'exp',
      q.expected.toExponential(6), 'err', (q.dy - q.expected).toExponential(3),
      'dstored', q.dstored.toExponential(6), 'drest', q.drest.toExponential(6), 'T', q.T.toFixed(3));
  }
}

// Zero-length tether: mass tied to an anchor with rest length 0.
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, 0), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0, pa: P(0, 0), pb: P(0, 0) }],
    { stiffness: 2e4, iterations: 8 })]);
  let maxd = 0, maxT = 0;
  for (let i = 0; i < 1000; i++) {
    w.step(0.001);
    const p = pos(w, 1);
    maxd = Math.max(maxd, Math.hypot(...p));
    maxT = Math.max(maxT, w.tension(0, 0));
  }
  console.log('zero-length tether: final pos', pos(w, 1).map(x => x.toExponential(3)),
    'max |pos|', maxd.toExponential(3), 'max T', maxT.toExponential(3),
    'rest', w.rest(0, 0).toExponential(3));
}
// Duplicate path points: coincident anchors + zero-length segment in a merged row with a hanging mass.
{
  const es = [fixed(0, P(0, 0)), fixed(1, P(0, 0)), mass(2, P(0, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: 0 },
      { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0, pa: P(0, 0), pb: P(0, 0) },
      { a: 1, b: 2, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, iterations: 8, damping: 0 })]);
  for (let i = 0; i < 4000; i++) w.step(0.001);
  const d = w.path(0), n = d[0], o = H + (n + 1) * L;
  console.log('duplicate points: T', w.tension(0, 0), w.tension(0, 1),
    'force0', Array.from(d.slice(o + 13, o + 16)).map(x => x.toFixed(6)),
    'rests', w.rest(0, 0), w.rest(0, 1), 'y', pos(w, 2)[1].toFixed(6));
}
