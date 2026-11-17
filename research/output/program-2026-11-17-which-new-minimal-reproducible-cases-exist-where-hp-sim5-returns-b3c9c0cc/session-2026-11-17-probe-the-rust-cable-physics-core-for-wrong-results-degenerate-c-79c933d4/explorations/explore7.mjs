// Exploratory: spool payout kinematics, slow payout so the cable stays taut.
import { mass, spool, pathRecord, world, pos, E, ATTACHMENT, ROLLING } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

function payout(r, dtheta, { stiffness = 2e4, damping = 50, inertia = 0.01, n = 10,
  settle = 4000, dt = 0.001, sign = 1, mass: m = 1 } = {}) {
  const es = [spool(0, P(0, 0), r, { mu: 0, inertia }), mass(1, P(r, -1), m)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness, damping, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < settle; i++) w.step(dt);
  const y0 = pos(w, 1)[1], s0 = w.stored(0, 0), r0 = w.rest(0, 0);
  for (let k = 0; k < n; k++) {
    const a = sign * dtheta * (k + 1) / n;
    ent[3] = 0; ent[4] = 0; ent[5] = Math.sin(a / 2); ent[6] = Math.cos(a / 2);
    w.step(dt);
  }
  return { dy: pos(w, 1)[1] - y0, stored: w.stored(0, 0) - s0, rest: w.rest(0, 0) - r0,
    tension: w.tension(0, 0) };
}
// Settle check without any payout.
const q0 = payout(0.05, 0);
console.log('settled (no payout): dy', q0.dy.toExponential(3), 'T', q0.tension.toFixed(4));
for (const r of [0.05, 0.02, 0.005]) {
  for (const dtheta of [1e-3, 1e-2]) {
    const q = payout(r, dtheta);
    console.log('payout r', r, 'dtheta', dtheta, 'dy', q.dy.toExponential(9), 'expected',
      (-r * dtheta).toExponential(9), 'err', (q.dy + r * dtheta).toExponential(3),
      'dstored', q.stored.toExponential(6), 'drest', q.rest.toExponential(6), 'T', q.tension.toFixed(4));
  }
}
// Print one trajectory.
{
  const r = 0.05, n = 10, dt = 0.001;
  const es = [spool(0, P(0, 0), r, { mu: 0, inertia: 0.01 }), mass(1, P(r, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness: 2e4, damping: 50, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < 4000; i++) w.step(dt);
  for (let k = 0; k < n; k++) {
    const a = 1e-2 * (k + 1) / n;
    ent[3] = 0; ent[4] = 0; ent[5] = Math.sin(a / 2); ent[6] = Math.cos(a / 2);
    w.step(dt);
    console.log('  k', k, 'y', pos(w, 1)[1].toFixed(9), 'rest', w.rest(0, 0).toFixed(9),
      'stored', w.stored(0, 0).toFixed(9), 'T', w.tension(0, 0).toFixed(4),
      'spool omega', w.entities()[E + 74].toFixed(3));
  }
}
