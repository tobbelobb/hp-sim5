// Exploratory: non-finite inputs, isolation, reset, zero-length, symmetry.
import { fixed, mass, spool, pathRecord, world, pos, ATTACHMENT, ROLLING,
  nonFiniteEntities, nonFinitePath, E } from '../lib.mjs';
const P = (x, y) => [x, y, 0];
const hairpin = (r, m1, m2, { mu = 0, inertia = 0, rest = 1, cw = true, kind = ROLLING } = {}) => {
  const es = [spool(0, P(0, 0), r, { mu, inertia, spool: false }),
    mass(1, P(-r - 1e-9, -1), m1), mass(2, P(r + 1e-9, -1), m2)];
  const arc = Math.PI * r;
  return world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw, stored: 0 }, { kind, cw, stored: arc }, { kind: ATTACHMENT, cw, stored: 0 }],
    [{ a: 1, b: 0, rest, pa: P(-r - 1e-9, -1), pb: P(-r, 0) },
      { a: 0, b: 2, rest, pa: P(r, 0), pb: P(r + 1e-9, -1) }],
    { stiffness: 1e6, iterations: 8 })]);
};

// (1) Symmetric hairpin, both windings.
for (const cw of [true, false]) {
  const w = hairpin(0.05, 1, 1, { cw });
  for (let i = 0; i < 300; i++) w.step(0.0005);
  console.log('sym cw', cw, 'y1', pos(w, 1)[1].toFixed(9), 'y2', pos(w, 2)[1].toFixed(9),
    'T', w.tension(0, 0).toFixed(6), w.tension(0, 1).toFixed(6),
    'stored', w.stored(0, 1).toFixed(9), 'rest', w.rest(0, 0).toFixed(9), w.rest(0, 1).toFixed(9));
}
// (2) radius sweep, Atwood m1=2 m2=1 -> a = g/3 downward for the heavier.
function accel(r, kind = ROLLING) {
  const w = hairpin(r, 2, 1, { kind });
  for (let i = 0; i < 200; i++) w.step(0.0005);
  const v0 = w.entities()[1 * E + 70];
  for (let i = 0; i < 400; i++) w.step(0.0005);
  const v1 = w.entities()[1 * E + 70];
  return -(v1 - v0) / (400 * 0.0005);
}
for (const r of [0.05, 0.02, 1e-3, 1e-6, 1e-9, 0]) {
  console.log('r', r, 'a', accel(r).toFixed(6), 'expected', (9.81 / 3).toFixed(6));
}
// (3) NaN in one cable endpoint.
{
  const w = hairpin(0.05, 1, 1);
  for (let i = 0; i < 10; i++) w.step(0.0005);
  w.entities()[1 * E + 0] = NaN; // mass 1 x = NaN
  w.step(0.0005);
  console.log('NaN endpoint: non-finite entities', nonFiniteEntities(w).map(x => x[0]), 'path', nonFinitePath(w, 0).slice(0, 4));
  w.step(0.0005);
  console.log('  after 2nd step: entities', nonFiniteEntities(w).map(x => x[0]), 'path nf', nonFinitePath(w, 0).length);
}
// (4) Inf in one cable endpoint.
{
  const w = hairpin(0.05, 1, 1);
  for (let i = 0; i < 10; i++) w.step(0.0005);
  w.entities()[1 * E + 0] = Infinity;
  w.step(0.0005);
  console.log('Inf endpoint: entities', nonFiniteEntities(w).map(x => x[0]), 'path nf', nonFinitePath(w, 0).slice(0, 4));
}
// (5) NaN in an unrelated entity (no cable).
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -1), 1), mass(2, P(5, 0), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, iterations: 4 })]);
  w.step(0.001);
  w.entities()[2 * E + 0] = NaN;
  w.step(0.001);
  console.log('NaN unrelated: entities nf', nonFiniteEntities(w).map(x => x[0]), 'tension', w.tension(0, 0));
}
// (6) Two identical worlds: independent trajectories.
{
  const a = hairpin(0.05, 1, 1), b = hairpin(0.05, 1, 1);
  for (let i = 0; i < 50; i++) { a.step(0.0005); }
  for (let i = 0; i < 50; i++) { b.step(0.0005); }
  console.log('isolation: a y1', pos(a, 1)[1], 'b y1', pos(b, 1)[1], 'same', pos(a, 1)[1] === pos(b, 1)[1]);
  // stepping a only must not move b
  const yb = pos(b, 1)[1];
  for (let i = 0; i < 50; i++) a.step(0.0005);
  console.log('  b unchanged', pos(b, 1)[1] === yb);
}
// (7) dispose then rebuild: first step identical to a fresh world.
{
  const mk = () => hairpin(0.05, 1, 1);
  const fresh = mk(); fresh.step(0.0005);
  const f = []; fresh.path(0).forEach(x => f.push(x));
  const reused = mk(); for (let i = 0; i < 20; i++) reused.step(0.0005); reused.dispose();
  const again = mk(); again.step(0.0005);
  const g = []; again.path(0).forEach(x => g.push(x));
  console.log('reset: path identical', JSON.stringify(f) === JSON.stringify(g));
}
