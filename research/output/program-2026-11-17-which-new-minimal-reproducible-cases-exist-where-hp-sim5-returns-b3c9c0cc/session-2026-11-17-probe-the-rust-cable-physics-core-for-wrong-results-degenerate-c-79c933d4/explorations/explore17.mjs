// Exploratory: force sign convention, rigid constraint residual, negative rest.
import { fixed, mass, pathRecord, world, pos, H, L, J, ATTACHMENT } from '../lib.mjs';
const P = (x, y) => [x, y, 0];
const G = 9.81;

// Force sign: fixed anchor at 0, mass at (0,-1). Joint a=0 (anchor), b=1 (mass).
{
  const w = world([fixed(0, P(0, 0)), mass(1, P(0, -1), 10)], [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, damping: 50, iterations: 4 })]);
  for (let i = 0; i < 4000; i++) w.step(0.001);
  const d = w.path(0), n = d[0], o = H + (n + 1) * L;
  console.log('hanging: lambda', d[o + 10].toFixed(6), 'force vec', Array.from(d.slice(o + 13, o + 16)).map(x => x.toFixed(6)),
    '|F|', d[o + 11].toFixed(6), 'mg', (10 * G).toFixed(6));
}
// Rigid cable residual: stretch should be ~0 (alpha = 0).
for (const iterations of [1, 2, 4, 8]) {
  const w = world([fixed(0, P(0, 0)), mass(1, P(0, -1), 10)], [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: Infinity, damping: 50, iterations })]);
  for (let i = 0; i < 4000; i++) w.step(0.001);
  const d = w.path(0), n = d[0], o = H + (n + 1) * L;
  console.log('rigid 1 seg it', iterations, 'residual', (1 + pos(w, 1)[1]).toExponential(3),
    'T', d[o + 11].toFixed(4), 'lambda', d[o + 10].toExponential(3));
}
// Rigid 3-segment chain (two masses on two segments + a fixed anchor).
for (const iterations of [1, 4, 8]) {
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -1), 1), mass(2, P(0, -2), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 },
      { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) },
      { a: 1, b: 2, rest: 1, pa: P(0, -1), pb: P(0, -2) }],
    { stiffness: Infinity, damping: 50, iterations })]);
  for (let i = 0; i < 4000; i++) w.step(0.001);
  const d = w.path(0), n = d[0], o = H + (n + 1) * L;
  const r0 = (1 + pos(w, 1)[1]), r1 = pos(w, 2)[1] + pos(w, 1)[1] + 1;
  console.log('rigid 2 seg it', iterations, 'residuals', r0.toExponential(3), r1.toExponential(3),
    'T', d[o + 11].toFixed(4), d[o + J + 11].toFixed(4));
}
// Negative rest length: silently clamped to 1e-6 by the rest-length clamp.
{
  const w = world([fixed(0, P(0, 0)), mass(1, P(0, -1), 1)], [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: -1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, iterations: 8 })]);
  w.step(0.001);
  const d = w.path(0), n = d[0], o = H + (n + 1) * L;
  console.log('negative rest -1: stored rest', d[o + 3].toExponential(3), 'T', d[o + 11].toFixed(3),
    'y', pos(w, 1)[1].toFixed(6), 'totalRestLength field', d[7], 'sum rest+stored', d[o + 3]);
}
// Zero spool radius with a motor: no moment arm, so no drive (documented?).
{
  const w = world([fixed(0, P(0, 0)), mass(1, P(0, -1), 1)], [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, iterations: 8 })]);
  for (let i = 0; i < 100; i++) w.step(0.001);
  console.log('dt sweep base ok', pos(w, 1)[1].toFixed(6));
}
for (const dt of [0, 1e-6, 1e-3, 1]) {
  const w = world([fixed(0, P(0, 0)), mass(1, P(0, -1), 1)], [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, iterations: 8 })]);
  let ok = true;
  for (let i = 0; i < 50; i++) {
    w.step(dt);
    const d = w.path(0), n = d[0], o = H + (n + 1) * L;
    if (!Number.isFinite(d[o + 3]) || !Number.isFinite(d[o + 11]) || !Number.isFinite(pos(w, 1)[1])) ok = false;
  }
  console.log('dt', dt, 'finite', ok, 'y', pos(w, 1)[1]);
}
