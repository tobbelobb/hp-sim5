// Exploratory: valid split (both links attachment) around a crossing roller.
import { fixed, mass, spool, pathRecord, world, pos, E, H, L, J, ATTACHMENT } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

function run(cy, r, { split = true, layering = true } = {}) {
  const es = [spool(0, P(0.3, cy), r, { inertia: 0.01 }), fixed(1, P(0, 0)), mass(2, P(1, 0), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 1, b: 2, rest: 1, pa: P(0, 0), pb: P(1, 0) }],
    { stiffness: 2e4, iterations: 4, halfWidth: 0.001 })],
  { layering, split });
  let n = 1, ok = true;
  for (let k = 0; k < 20; k++) {
    w.step(0.001);
    const d = w.path(0); n = d[0];
    const bad = !Number.isFinite(d[H + 0 * L + 2]) || !Number.isFinite(d[H + L + 2])
      || !Number.isFinite(d[H + 2 * L + 2]);
    const nanPose = [0, 1, 2].some(i => [0, 1, 2, 3, 4, 5, 6].some(s => !Number.isFinite(w.entities()[i * E + s])));
    if (bad || nanPose) { ok = false; console.log('  NaN at step', k, 'n', n, 'store', d[H], d[H + L], d[H + 2 * L]); break; }
  }
  const d = w.path(0); n = d[0];
  return { n, ok, stored: Array.from({ length: n + 1 }, (_, li) => +d[H + li * L + 2].toFixed(6)),
    rests: Array.from({ length: n }, (_, i) => +d[H + (n + 1) * L + i * J + 3].toFixed(6)),
    kinds: Array.from({ length: n + 1 }, (_, li) => d[H + li * L]), y: +pos(w, 2)[1].toFixed(6) };
}
for (const cy of [0, 0.01, 0.04, 0.049, 0.05, 0.06, -0.02]) {
  console.log('center y', cy, 'r 0.05 split');
  console.log('  ', run(cy, 0.05, { split: true }));
  console.log('   no split', run(cy, 0.05, { split: false }));
  console.log('   no layering', run(cy, 0.05, { split: false, layering: false }));
}
