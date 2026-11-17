// Exploratory: closed-form statics/dynamics + mirror symmetry + spool payout.
import { fixed, mass, spool, pathRecord, world, pos, E, ATTACHMENT, ROLLING, PINHOLE } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

// (1) settled hanging mass: stretch = mgL/EA, reported tension = mg.
{
  const ea = 20000, m = 10, L = 1;
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -L), m)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: L, pa: P(0, 0), pb: P(0, -L) }],
    { stiffness: ea, damping: 50, iterations: 4 })]);
  for (let i = 0; i < 4000; i++) w.step(0.001);
  const stretch = -pos(w, 1)[1] - L;
  console.log('hanging: stretch', stretch.toExponential(6), 'exact', (m * 9.81 / ea).toExponential(6),
    'T', w.tension(0, 0).toFixed(6), 'mg', (m * 9.81).toFixed(6),
    'lambda', w.lambda(0, 0).toFixed(6));
}
// Atwood hairpin builder with explicit side placement.
function atwood({ r = 0.05, m1 = 2, m2 = 1, mirror = false, mu = 0, inertia = 0, rest = 1 } = {}) {
  const s = mirror ? -1 : 1;
  const es = [spool(0, P(0, 0), r, { mu, inertia, spool: false }),
    mass(1, P(s * (r + 1e-9), -1), m1), mass(2, P(-s * (r + 1e-9), -1), m2)];
  const arc = Math.PI * r;
  return world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: !mirror, stored: 0 }, { kind: ROLLING, cw: !mirror, stored: arc },
      { kind: ATTACHMENT, cw: !mirror, stored: 0 }],
    [{ a: 1, b: 0, rest, pa: P(s * (r + 1e-9), -1), pb: P(s * r, 0) },
      { a: 0, b: 2, rest, pa: P(-s * r, 0), pb: P(-s * (r + 1e-9), -1) }],
    { stiffness: 1e6, iterations: 8 })]);
}
const accel = (w, id) => {
  for (let i = 0; i < 200; i++) w.step(0.0005);
  const v0 = w.entities()[id * E + 70];
  for (let i = 0; i < 400; i++) w.step(0.0005);
  const v1 = w.entities()[id * E + 70];
  return -(v1 - v0) / (400 * 0.0005);
};
for (const mirror of [false, true]) {
  const w = atwood({ mirror });
  const a1 = accel(w, 1);
  console.log('atwood mirror', mirror, 'a1', a1.toFixed(6), 'y1', pos(w, 1)[1].toFixed(6),
    'T0', w.tension(0, 0).toFixed(4), 'T1', w.tension(0, 1).toFixed(4),
    'stored', w.stored(0, 1).toFixed(6));
}
// Equal masses: static in both mirror orientations.
for (const mirror of [false, true]) {
  const w = atwood({ mirror, m1: 1, m2: 1 });
  for (let i = 0; i < 1000; i++) w.step(0.0005);
  console.log('equal mirror', mirror, 'y1', pos(w, 1)[1].toFixed(9), 'y2', pos(w, 2)[1].toFixed(9),
    'T', w.tension(0, 0).toFixed(4), w.tension(0, 1).toFixed(4));
}
// (3) Spool payout: mass hangs from a spool of radius r turned by a known angle.
function payout(r, dtheta, { stiffness = 1e7, mu = 0, inertia = 0.01, n = 20 } = {}) {
  const es = [spool(0, P(0, 0), r, { mu, inertia }), mass(1, P(r, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < 100; i++) w.step(0.001); // settle under gravity through the cable
  const y0 = pos(w, 1)[1], s0 = w.stored(0, 0), r0 = w.rest(0, 0);
  // Turn the spool by dtheta/n per step, directly editing its orientation.
  const step = dtheta / n;
  for (let k = 0; k < n; k++) {
    const a = step * (k + 1);
    ent[3] = Math.sin(a / 2); ent[4] = 0; ent[5] = 0; ent[6] = Math.cos(a / 2);
    w.step(0.001);
  }
  const y1 = pos(w, 1)[1];
  return { dy: y1 - y0, expected: -r * dtheta, stored: w.stored(0, 0) - s0,
    rest: w.rest(0, 0) - r0, tension: w.tension(0, 0) };
}
for (const r of [0.05, 0.02, 0.005]) {
  for (const dtheta of [0.1, 0.5]) {
    const q = payout(r, dtheta);
    console.log('payout r', r, 'dtheta', dtheta, 'dy', q.dy.toFixed(9), 'expected', q.expected.toFixed(9),
      'err', (q.dy - q.expected).toExponential(3), 'dstored', q.stored.toFixed(9), 'drest', q.rest.toFixed(9),
      'T', q.tension.toFixed(3));
  }
}
