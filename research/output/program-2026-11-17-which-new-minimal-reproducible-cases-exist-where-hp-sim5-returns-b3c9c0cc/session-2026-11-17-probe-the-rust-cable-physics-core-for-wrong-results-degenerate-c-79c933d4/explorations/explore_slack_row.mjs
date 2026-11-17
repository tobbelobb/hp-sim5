// Exploratory: a slack segment inside a taut merged row.
import { fixed, mass, pathRecord, world, pos, H, L, J, ATTACHMENT, PINHOLE } from '../lib.mjs';
const P = (x, y) => [x, y, 0];
// Frictionless pinhole at the origin; left mass taut-ish, right mass slack.
// total C = (1.5-0.8) + (1.4142-2.0) = +0.114 > 0, so the merged row is taut.
const w = world([
  fixed(0, P(0, 0), 0),
  mass(1, P(0, -1.5), 1),
  mass(2, P(1, -1), 1),
], [pathRecord(
  [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: PINHOLE, cw: true, stored: 0 },
    { kind: ATTACHMENT, cw: true, stored: 0 }],
  [{ a: 1, b: 0, rest: 0.8, pa: P(0, -1.5), pb: P(0, 0) },
    { a: 0, b: 2, rest: 2.0, pa: P(0, 0), pb: P(1, -1) }],
  { stiffness: 2e4, iterations: 8 })]);
console.log('before: y1', pos(w, 1)[1], 'pos2', pos(w, 2), 'd1', 1.5, 'd2', Math.hypot(1, 1));
w.step(0.001);
const d = w.path(0), n = d[0], o = H + (n + 1) * L;
console.log('after 1 step:');
console.log('  v1', w.entities()[1 * 100 + 0 + 69 + 1], 'v2_y', w.entities()[2 * 100 + 0] !== undefined ? w.entities()[2 * 16 * 10 + 70] : 0);
console.log('  tensions', d[o + 11], d[o + J + 11]);
console.log('  rests now', d[o + 3], d[o + J + 3]);
console.log('  pos2', pos(w, 2), 'pos1', pos(w, 1));
for (let i = 0; i < 500; i++) w.step(0.001);
console.log('after 500 steps: pos1', pos(w, 1), 'pos2', pos(w, 2),
  'tensions', w.tension(0, 0), w.tension(0, 1), 'rests', w.rest(0, 0), w.rest(0, 1));
