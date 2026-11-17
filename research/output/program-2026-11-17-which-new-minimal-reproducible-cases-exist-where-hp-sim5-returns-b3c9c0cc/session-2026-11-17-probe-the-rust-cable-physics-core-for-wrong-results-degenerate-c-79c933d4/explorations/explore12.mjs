// Exploratory: first hybrid/layering NaN, fully dumped.
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
  return { es, links, segs, path: pathRecord(links, segs, {
    stiffness: Math.pow(10, range(3, 8)), damping: rnd() < 0.5 ? 0 : range(0, 100),
    iterations: int(1, 8), halfWidth: range(0.0005, 0.01) }) };
}
const SLOTS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 19, 69, 70, 71, 72, 73, 74];
for (let trial = 0; trial < 3000; trial++) {
  const { es, links, segs, path } = build();
  const w = world(es, [path], { layering: true });
  for (let k = 0; k < 20; k++) {
    w.step(0.001);
    const e = w.entities();
    let hit = null;
    for (let i = 0; i < es.length && !hit; i++) for (const s of SLOTS) if (!Number.isFinite(e[i * E + s])) { hit = [i, s, e[i * E + s]]; break; }
    if (hit) {
      console.log('trial', trial, 'step', k, 'entity', hit[0], 'slot', hit[1], 'value', hit[2]);
      console.log('types', es.map(en => en[75]), 'radii', es.map(en => en[104]), 'masses', es.map(en => en[124]));
      console.log('links', links, 'segs', segs.map(s => ({ a: s.a, b: s.b, rest: +s.rest.toFixed(4) })));
      const d = w.path(0), n = d[0];
      console.log('topology now n =', n, 'links now', Array.from({ length: n + 1 }, (_, li) => [d[H + li * L], d[H + li * L + 1], +d[H + li * L + 2].toFixed(4), d[H + li * L + 3], d[H + li * L + 4]]));
      console.log('segment rests now', Array.from({ length: n }, (_, i) => +d[H + (n + 1) * L + i * J + 3].toFixed(6)));
      console.log('entity', hit[0], 'record', Array.from(e.slice(hit[0] * E, hit[0] * E + 20)));
      console.log('entity vel/omega', Array.from(e.slice(hit[0] * E + 69, hit[0] * E + 75)));
      console.log('quat3', Array.from(e.slice(hit[0] * E + 3, hit[0] * E + 7)));
      process.exit(0);
    }
  }
}
console.log('no NaN found');
