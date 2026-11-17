// Exploratory: randomized worlds with layering + hybrid links + merge/split.
import { fixed, mass, spool, pathRecord, world, E, H, L, J, ATTACHMENT, ROLLING, PINHOLE, HYBRID, HYBRID_ATTACH } from '../lib.mjs';
let seed = 987654321;
const rnd = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
const range = (a, b) => a + (b - a) * rnd();
const int = (a, b) => Math.floor(range(a, b + 1));
const P = (x, y) => [x, y, 0];
const MALFORMED = process.env.MALFORMED === '1'; // rolling links on radius-less entities
function build() {
  const n = int(2, 5);
  const es = []; const types = [];
  for (let i = 0; i < n; i++) {
    const r = rnd();
    if (r < 0.35) { es.push(fixed(i, P(range(-1, 1), range(-1.5, 0.5)), rnd() < 0.5 ? range(0, 0.6) : 0)); types.push('fixed'); }
    else if (r < 0.7) { es.push(mass(i, P(range(-1, 1), range(-1.5, 0.5)), range(0.1, 5))); types.push('mass'); }
    else {
      es.push(spool(i, P(range(-0.5, 0.5), range(-0.5, 0.5)), range(0.001, 0.1),
        { mu: rnd() < 0.5 ? range(0, 0.5) : 0, inertia: range(1e-4, 1) }));
      types.push('spool');
    }
  }
  const segs = [], links = [];
  for (let i = 0; i < n - 1; i++) {
    const pa = [es[i][0], es[i][1], 0], pb = [es[i + 1][0], es[i + 1][1], 0];
    const d = Math.hypot(pa[0] - pb[0], pa[1] - pb[1]);
    segs.push({ a: i, b: i + 1, rest: range(0, 1.2) * Math.max(d, 0.05), pa, pb });
  }
  for (let li = 0; li <= n - 1; li++) {
    const endpoint = li === 0 || li === n - 1;
    const id = li === 0 ? 0 : li === n - 1 ? n - 1 : li;
    const rollable = types[id] === 'spool' || MALFORMED;
    let kind = endpoint ? (rollable && rnd() < 0.6 ? ROLLING : ATTACHMENT)
      : [ATTACHMENT, PINHOLE, ...(rollable ? [ROLLING] : [])][int(0, rollable ? 2 : 1)];
    if (endpoint && types[id] === 'spool' && rnd() < 0.4) kind = rnd() < 0.5 ? HYBRID : HYBRID_ATTACH;
    const r = Number.isFinite(es[id][104]) ? es[id][104] : 0;
    links.push({ kind, cw: rnd() < 0.5, stored: kind === ROLLING || kind === HYBRID ? range(0, 2 * Math.PI) * r : 0 });
  }
  return { es, path: pathRecord(links, segs, {
    stiffness: Math.pow(10, range(3, 8)), damping: rnd() < 0.5 ? 0 : range(0, 100),
    iterations: int(1, 8), halfWidth: range(0.0005, 0.01) }), layering: true };
}
const pose = (e, i) => [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 69, 70, 71, 72, 73, 74]
  .every(k => Number.isFinite(e[i * E + k]));
let bad = 0, trials = 3000; const kinds = {};
for (let trial = 0; trial < trials; trial++) {
  const { es, path } = build();
  const w = world(es, [path], { layering: true });
  let n = path[0];
  const mat0 = () => {
    const d = w.path(0); const nn = d[0]; let m = 0;
    for (let i = 0; i < nn; i++) m += d[H + (nn + 1) * L + i * J + 3];
    for (let li = 0; li <= nn; li++) m += d[H + li * L + 2];
    return m;
  };
  const m0 = mat0();
  let fail = null;
  try {
    for (let k = 0; k < 20 && !fail; k++) {
      w.step(0.001);
      const e = w.entities();
      for (let i = 0; i < es.length; i++) if (!pose(e, i)) { fail = `pose entity ${i} step ${k}`; break; }
      if (fail) break;
      const d = w.path(0); n = d[0];
      for (let i = 0; i < n; i++) {
        const o = H + (n + 1) * L + i * J;
        if (![d[o + 3], d[o + 10], d[o + 11], d[o + 13], d[o + 14], d[o + 15]].every(Number.isFinite)) { fail = `path seg ${i} step ${k}`; break; }
        if (d[o + 11] < 0) { fail = `negative tension ${d[o + 11]} seg ${i} step ${k}`; break; }
      }
      if (fail) break;
      const m = mat0();
      if (Math.abs(m - m0) > 1e-6 * (1 + Math.abs(m0))) fail = `material ${m} vs ${m0} step ${k}`;
    }
  } catch (err) { fail = `throw ${err.message}`; }
  if (fail) {
    bad++;
    const kind = fail.replace(/[0-9.e+-]+/g, 'N').split(' ').slice(0, 3).join(' ');
    kinds[kind] = (kinds[kind] ?? 0) + 1;
    if (bad <= 2) {
      console.log('FAIL', trial, fail);
      console.log('  paths', Array.from({ length: n + 1 }, (_, li) => [path[H + li * L], path[H + li * L + 1], +path[H + li * L + 2].toFixed(4)]));
      console.log('  segs', Array.from({ length: n }, (_, i) => Array.from(path.slice(H + (n + 1) * L + i * J, H + (n + 1) * L + i * J + 10)).map(x => +(+x).toFixed(4))));
      console.log('  ents', es.map(e => ({ fl: e[75], r: e[104], m: e[124], p: Array.from(e.slice(0, 3)) })));
    }
  }
}
console.log('hybrid/layering failures', bad, '/', trials, kinds);
