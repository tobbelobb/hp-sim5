// Repro 2: the opt-in layering "split" pass writes NaN into the persistent path
// records when a rolling link sits on an entity without a finite radius.
//
// Geometry: a straight span from a fixed point (0,0) to a mass (1,0). A
// bystander roller of radius 0.05 at (0.3,0) is crossed by that span, so the
// split pass inserts a rolling link for it. The span's own two links are
// declared 'rolling' but their entities carry no RadiusComponent, which the
// attachment kernel explicitly tolerates (radius flags bit 4 = "finite", so a
// missing radius degrades to a point attachment). The split pass does not: it
// calls sphere_tangents/arc with radius = NaN, then stores NaN tangents, NaN
// wrap angle and NaN rest lengths.
//
// Flags: enableLayering = layeringSplitJoints = true (merge/hybrid off). The
// browser/headless adapter leaves layeringSplitJoints false, so this path is
// opt-in; with the flag off the same world is finite and correct.
//
// Expected: split skips a link whose radius is not finite (like geometry does),
//           leaving a finite world.
// Actual:   path rest lengths, stored arc and both endpoints' state become NaN,
//           and every following step keeps the world NaN.
import { fixed, mass, spool, pathRecord, world, pos, E, H, L, J, ATTACHMENT, ROLLING } from './lib.mjs';
const P = (x, y) => [x, y, 0];

function build({ split }) {
  const es = [
    spool(0, P(0.3, 0), 0.05, { inertia: 0.01 }), // bystander roller, crossed by the span
    fixed(1, P(0, 0)),                            // no RadiusComponent (radius = NaN)
    mass(2, P(1, 0), 1),                          // no RadiusComponent
  ];
  return world(es, [pathRecord(
    [{ kind: ROLLING, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: 0 }],
    [{ a: 1, b: 2, rest: 1, pa: P(0, 0), pb: P(1, 0) }],
    { stiffness: 2e4, iterations: 4 })], { layering: true, split, merge: false });
}
function state(w) {
  const d = w.path(0), n = d[0];
  const e = w.entities();
  return {
    n,
    stored: Array.from({ length: n + 1 }, (_, li) => d[H + li * L + 2]),
    rests: Array.from({ length: n }, (_, i) => d[H + (n + 1) * L + i * J + 3]),
    posesFinite: [0, 1, 2].map(i => [0, 1, 2, 3, 4, 5, 6, 69, 70, 71].every(s => Number.isFinite(e[i * E + s]))),
    y2: pos(w, 2)[1],
  };
}

const control = build({ split: false });
for (let i = 0; i < 10; i++) control.step(0.001);
const c = state(control);
console.log('split off (control): n =', c.n, 'stored', c.stored, 'rests', c.rests,
  'poses finite', c.posesFinite, 'y2', c.y2);
if (!c.posesFinite.every(Boolean) || c.rests.some(x => !Number.isFinite(x))) {
  console.error('control world is not finite; repro setup is wrong');
  process.exit(2);
}

const w = build({ split: true });
for (let i = 0; i < 10; i++) w.step(0.001);
const s = state(w);
console.log('split on (repro)   : n =', s.n, 'stored', s.stored, 'rests', s.rests,
  'poses finite', s.posesFinite, 'y2', s.y2);

const bad = s.rests.some(x => !Number.isFinite(x)) || s.stored.some(x => !Number.isFinite(x))
  || !s.posesFinite.every(Boolean);
if (bad) {
  console.error('\nBUG: layeringSplitJoints wrote non-finite state into the path records'
    + ` (n = ${s.n}, rests ${s.rests}, stored ${s.stored}) and`
    + ` ${s.posesFinite.filter(x => !x).length}/3 entities are non-finite.`
    + ' A rolling link without a finite radius is tolerated by geometry() but not by split(),'
    + ' and the NaN then reaches the always-on constraint solve.');
  process.exit(1);
}
console.log('\nsplit stayed finite');
