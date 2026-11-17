// Exploratory: randomized minimal worlds, invariant fuzz.
import { fixed, mass, spool, pathRecord, world, E, H, L, J, ATTACHMENT, ROLLING, PINHOLE } from '../lib.mjs';

let seed = 12345;
const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
const range = (a, b) => a + (b - a) * rnd();
const int = (a, b) => Math.floor(range(a, b + 1));
const P = (x, y) => [x, y, 0];

function build() {
  const n = int(2, 5);
  const es = [];
  const types = [];
  for (let i = 0; i < n; i++) {
    const r = rnd();
    if (r < 0.4) { es.push(fixed(i, P(range(-1, 1), range(-1.5, 0.5)), rnd() < 0.5 ? range(0, 0.6) : 0)); types.push('fixed'); }
    else if (r < 0.75) { es.push(mass(i, P(range(-1, 1), range(-1.5, 0.5)), range(0.1, 5))); types.push('mass'); }
    else {
      const rr = range(0.001, 0.1);
      es.push(spool(i, P(range(-0.5, 0.5), range(-0.5, 0.5)), rr,
        { mu: rnd() < 0.5 ? range(0, 0.5) : 0, inertia: range(1e-4, 1) }));
      types.push('spool');
    }
  }
  const segs = [];
  const links = [];
  for (let i = 0; i < n - 1; i++) {
    const a = i, b = i + 1;
    const pa = [es[a][0], es[a][1], 0], pb = [es[b][0], es[b][1], 0];
    const d = Math.hypot(pa[0] - pb[0], pa[1] - pb[1]);
    segs.push({ a, b, rest: range(0, 1.2) * Math.max(d, 0.05), pa, pb });
  }
  for (let li = 0; li <= n - 1; li++) {
    const kind = li === 0 || li === n - 1
      ? (types[li === 0 ? 0 : n - 1] === 'spool' && rnd() < 0.5 ? ROLLING : ATTACHMENT)
      : [ATTACHMENT, ROLLING, PINHOLE][int(0, 2)];
    const id = li === 0 ? 0 : li === n - 1 ? n - 1 : li;
    const r = es[id] && Number.isFinite(es[id][104]) ? es[id][104] : 0;
    links.push({ kind, cw: rnd() < 0.5, stored: kind === ROLLING ? range(0, 2 * Math.PI) * r : 0 });
  }
  const stiffness = Math.pow(10, range(3, 8));
  const damping = rnd() < 0.5 ? 0 : range(0, 100);
  return { es, path: pathRecord(links, segs, { stiffness, damping, iterations: int(1, 8) }),
    rest0: segs.map(s => s.rest) };
}

const isFinitePose = (e, i) => {
  for (const k of [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 69, 70, 71, 72, 73, 74]) {
    if (!Number.isFinite(e[i * E + k])) return false;
  }
  return true;
};
let bad = 0;
for (let trial = 0; trial < 3000; trial++) {
  const { es, path } = build();
  const w = world(es, [path]);
  const n = path[0];
  const rest0 = []; for (let i = 0; i < n; i++) rest0.push(path[H + (n + 1) * L + i * J + 3]);
  const stored0 = []; for (let li = 0; li <= n; li++) stored0.push(path[H + li * L + 2]);
  const mat0 = rest0.reduce((a, b) => a + b, 0) + stored0.reduce((a, b) => a + b, 0);
  let fail = null;
  for (let k = 0; k < 20 && !fail; k++) {
    w.step(0.001);
    const e = w.entities();
    for (let i = 0; i < es.length; i++) if (!isFinitePose(e, i)) { fail = `non-finite pose entity ${i} step ${k}`; break; }
    if (fail) break;
    for (let i = 0; i < n; i++) {
      const d = w.path(0), o = H + (n + 1) * L + i * J;
      if (!Number.isFinite(d[o + 3]) || !Number.isFinite(d[o + 10]) || !Number.isFinite(d[o + 11])
        || !Number.isFinite(d[o + 13]) || !Number.isFinite(d[o + 14]) || !Number.isFinite(d[o + 15])) {
        fail = `non-finite path seg ${i} step ${k}`; break;
      }
      if (d[o + 11] < 0) { fail = `negative tension ${d[o + 11]} seg ${i} step ${k}`; break; }
    }
    if (fail) break;
    for (let li = 0; li <= n; li++) if (!Number.isFinite(w.path(0)[H + li * L + 2])) { fail = `non-finite stored li ${li} step ${k}`; break; }
    if (fail) break;
    const d = w.path(0);
    let mat = 0;
    for (let i = 0; i < n; i++) mat += d[H + (n + 1) * L + i * J + 3];
    for (let li = 0; li <= n; li++) mat += d[H + li * L + 2];
    if (Math.abs(mat - mat0) > 1e-6 * (1 + Math.abs(mat0))) { fail = `material ${mat} vs ${mat0} step ${k}`; break; }
  }
  if (fail) {
    bad++;
    if (bad <= 12) {
      console.log('FAIL trial', trial, fail);
      console.log('  types', es.map((e, i) => [e[75], e[104], e[124]]));
      console.log('  links', Array.from({ length: n + 1 }, (_, li) => [path[H + li * L], path[H + li * L + 1], path[H + li * L + 2]]));
      console.log('  rests', rest0, 'segs', Array.from({ length: n }, (_, i) => Array.from(path.slice(H + (n + 1) * L + i * J, H + (n + 1) * L + i * J + 10))));
    }
    w.dispose();
  }
  if (bad > 40) break;
}
console.log('failures', bad);
