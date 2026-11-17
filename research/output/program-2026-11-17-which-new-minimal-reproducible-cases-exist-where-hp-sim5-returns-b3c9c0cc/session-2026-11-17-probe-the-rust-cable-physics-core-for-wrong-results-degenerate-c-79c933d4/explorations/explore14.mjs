// Exploratory: minimal split-with-NaN-radius repro.
import { fixed, mass, spool, pathRecord, world, pos, E, H, L, J, ATTACHMENT, ROLLING } from '../lib.mjs';
const P = (x, y) => [x, y, 0];

function build({ split = true, rolling0 = ROLLING, rolling1 = ROLLING, r0 = NaN } = {}) {
  const es = [
    spool(0, P(0.3, 0), 0.05, { inertia: 0.01 }), // bystander roller the span crosses
    fixed(1, P(0, 0)),                            // no radius
    mass(2, P(1, 0), 1),                          // no radius
  ];
  if (Number.isFinite(r0)) { es[0] = 0; } // unused
  return world(es, [pathRecord(
    [{ kind: rolling0, cw: true, stored: 0 }, { kind: rolling1, cw: true, stored: 0 }],
    [{ a: 1, b: 2, rest: 1, pa: P(0, 0), pb: P(1, 0) }],
    { stiffness: 2e4, iterations: 4 })],
  { layering: true, split });
}
for (const [split, k0, k1] of [[false, ROLLING, ROLLING], [true, ROLLING, ROLLING],
  [true, ATTACHMENT, ROLLING], [true, ROLLING, ATTACHMENT]]) {
  const w = build({ split, rolling0: k0, rolling1: k1 });
  w.step(0.001);
  const d = w.path(0), n = d[0];
  const linkStored = Array.from({ length: n + 1 }, (_, li) => d[H + li * L + 2]);
  const rests = Array.from({ length: n }, (_, i) => d[H + (n + 1) * L + i * J + 3]);
  const e = w.entities();
  const nanPose = [0, 1, 2].some(i => [0, 1, 2, 3, 4, 5, 6].some(s => !Number.isFinite(e[i * E + s])));
  console.log('split', split, 'kinds', k0, k1, 'n', n, 'stored', linkStored, 'rests', rests,
    'nanPose', nanPose, 'pos2', pos(w, 2));
}
