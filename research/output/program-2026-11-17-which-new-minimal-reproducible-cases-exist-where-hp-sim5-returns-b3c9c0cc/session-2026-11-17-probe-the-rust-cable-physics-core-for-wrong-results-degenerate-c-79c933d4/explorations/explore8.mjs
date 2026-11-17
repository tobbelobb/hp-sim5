// Exploratory: payout with a nearly immobile (huge-inertia) spool.
import { mass, spool, pathRecord, world, pos, E, ATTACHMENT, ROLLING } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

function payout(r, dtheta, { inertia = 1e6, stiffness = 2e4, damping = 50, n = 10,
  settle = 4000, dt = 0.001, m = 1, stored0 = 0 } = {}) {
  const es = [spool(0, P(0, 0), r, { mu: 0, inertia }), mass(1, P(r, -1), m)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw: true, stored: stored0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness, damping, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < settle; i++) w.step(dt);
  const y0 = pos(w, 1)[1], s0 = w.stored(0, 0), r0 = w.rest(0, 0), q0 = w.tension(0, 0);
  const spool0 = ent[3 * 1 + 5]; // not used
  for (let k = 0; k < n; k++) {
    const a = dtheta * (k + 1) / n;
    ent[3] = 0; ent[4] = 0; ent[5] = Math.sin(a / 2); ent[6] = Math.cos(a / 2);
    w.step(dt);
  }
  return { dy: pos(w, 1)[1] - y0, expected: -r * dtheta, dstored: w.stored(0, 0) - s0,
    drest: w.rest(0, 0) - r0, T0: q0, T: w.tension(0, 0) };
}
for (const r of [0.05, 0.02, 0.005]) {
  for (const dtheta of [1e-2, 1e-1]) {
    const q = payout(r, dtheta);
    console.log('payout r', r, 'dtheta', dtheta, 'dy', q.dy.toExponential(9), 'exp', q.expected.toExponential(9),
      'err', (q.dy - q.expected).toExponential(3), 'dstored', q.dstored.toExponential(6),
      'drest', q.drest.toExponential(6), 'T', q.T.toFixed(4));
  }
}
// Wind-up instead of payout (negative dtheta).
for (const r of [0.05]) {
  for (const dtheta of [-1e-2, -1e-1]) {
    const q = payout(r, dtheta);
    console.log('wind r', r, 'dtheta', dtheta, 'dy', q.dy.toExponential(9), 'exp', q.expected.toExponential(9),
      'err', (q.dy - q.expected).toExponential(3), 'dstored', q.dstored.toExponential(6),
      'drest', q.drest.toExponential(6), 'T', q.T.toFixed(4));
  }
}
// Starting with wound line (yo-yo absorbed) and paying out.
for (const stored0 of [0, 1e-6, 1e-3, 1e-2]) {
  const q = payout(0.05, 1e-2, { stored0 });
  console.log('stored0', stored0, 'dy', q.dy.toExponential(9), 'exp', q.expected.toExponential(9),
    'err', (q.dy - q.expected).toExponential(3), 'dstored', q.dstored.toExponential(6),
    'drest', q.drest.toExponential(6), 'T', q.T.toFixed(4));
}
