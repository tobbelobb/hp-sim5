// Repro 1: one non-finite input value is propagated by the Rust core into the
// whole cable system's persistent state, and the world never recovers.
//
// World: a fixed roller (r = 0.05) with two 1 kg masses in a hairpin (a valid,
// settled configuration). After settling, one mass's position.x is set to NaN,
// exactly as a bad scene/system write would leave it.
//
// Expected: the affected row is non-finite, or the solver rejects/skips the
//           non-finite body, or at least the other bodies and the path's rest
//           lengths survive.
// Actual:   every entity's pose and every joint's rest/lambda/force become
//           non-finite within one step, and restoring the input does not heal.
import { mass, spool, pathRecord, world, pos, E, H, L, J, ATTACHMENT, ROLLING } from './lib.mjs';
const P = (x, y) => [x, y, 0];

const POSE_SLOTS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 69, 70, 71, 72, 73, 74];
function nonFinitePoses(w, count) {
  const e = w.entities();
  const out = [];
  for (let i = 0; i < count; i++) {
    const bad = POSE_SLOTS.filter(s => !Number.isFinite(e[i * E + s]));
    if (bad.length) out.push({ entity: i, slots: bad, first: bad.slice(0, 3).map(s => [s, e[i * E + s]]) });
  }
  return out;
}
function pathState(w) {
  const d = w.path(0), n = d[0];
  const seg = (i, o) => d[H + (n + 1) * L + i * J + o];
  return {
    n,
    rests: Array.from({ length: n }, (_, i) => seg(i, 3)),
    stored: Array.from({ length: n + 1 }, (_, li) => d[H + li * L + 2]),
    lambda: Array.from({ length: n }, (_, i) => seg(i, 10)),
    tensions: Array.from({ length: n }, (_, i) => seg(i, 11)),
  };
}

function hairpin() {
  const r = 0.05;
  const es = [spool(0, P(0, 0), r, { mu: 0, inertia: 0 }), // fixed roller, no spin
    mass(1, P(-r - 1e-9, -1), 1), mass(2, P(r + 1e-9, -1), 1)];
  const arc = Math.PI * r;
  return world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: arc },
      { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 1, b: 0, rest: 1, pa: P(-r - 1e-9, -1), pb: P(-r, 0) },
      { a: 0, b: 2, rest: 1, pa: P(r, 0), pb: P(r + 1e-9, -1) }],
    { stiffness: 1e6, iterations: 8 })]);
}

const w = hairpin();
for (let i = 0; i < 200; i++) w.step(0.0005);
const before = { poses: nonFinitePoses(w, 3), path: pathState(w), y1: pos(w, 1)[1] };
console.log('settled y1 =', before.y1.toFixed(9), 'tension =', before.path.tensions[0].toFixed(6));
if (before.poses.length) { console.log('FAIL: world was already non-finite before injection'); process.exit(2); }

// Inject a single non-finite coordinate in one cable endpoint.
const value = process.argv[2] === 'inf' ? Infinity : NaN;
w.entities()[1 * E + 0] = value;
w.step(0.0005);
const after1 = nonFinitePoses(w, 3);
const path1 = pathState(w);
console.log('\nafter injecting', value, 'in entity 1 position.x and one step:');
console.log('  entities with non-finite pose:', JSON.stringify(after1));
console.log('  path rests  :', path1.rests);
console.log('  path stored :', path1.stored);
console.log('  path lambda :', path1.lambda);
console.log('  path tension:', path1.tensions);

// Does it heal if the bad input is repaired?
w.entities()[1 * E + 0] = -0.050000001;
w.step(0.0005);
const after2 = nonFinitePoses(w, 3);
const path2 = pathState(w);
console.log('\nafter repairing the input and stepping again:');
console.log('  entities with non-finite pose:', after2.map(x => x.entity));
console.log('  path rests  :', path2.rests);
console.log('  path stored :', path2.stored);

const leaked = after1.filter(x => x.entity !== 1);
const bug = leaked.length > 0 || path1.rests.some(x => !Number.isFinite(x))
  || path1.stored.some(x => !Number.isFinite(x));
if (bug) {
  console.error('\nBUG: a non-finite value in one input coordinate poisoned state outside that entity:'
    + ` entities ${JSON.stringify(leaked.map(x => x.entity))}`
    + ` and ${path1.rests.concat(path1.stored).filter(x => !Number.isFinite(x)).length}`
    + ' path fields are non-finite; repaired input still leaves '
    + `${after2.length} non-finite entities.`);
  process.exit(1);
}
console.log('\nno leakage');
