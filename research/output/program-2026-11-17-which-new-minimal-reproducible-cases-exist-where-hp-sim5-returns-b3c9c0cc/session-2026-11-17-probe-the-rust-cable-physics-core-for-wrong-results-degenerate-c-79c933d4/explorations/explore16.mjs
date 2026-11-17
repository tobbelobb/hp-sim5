// Diagnostic: which slots flip to NaN for the bystander roller after injection.
import { mass, spool, pathRecord, world, E, ATTACHMENT, ROLLING } from '../lib.mjs';
const P = (x, y) => [x, y, 0];
const r = 0.05;
const es = [spool(0, P(0, 0), r, { mu: 0, inertia: 0 }), mass(1, P(-r - 1e-9, -1), 1), mass(2, P(r + 1e-9, -1), 1)];
const w = world(es, [pathRecord(
  [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: Math.PI * r },
    { kind: ATTACHMENT, cw: true, stored: 0 }],
  [{ a: 1, b: 0, rest: 1, pa: P(-r - 1e-9, -1), pb: P(-r, 0) },
    { a: 0, b: 2, rest: 1, pa: P(r, 0), pb: P(r + 1e-9, -1) }],
  { stiffness: 1e6, iterations: 8 })]);
for (let i = 0; i < 200; i++) w.step(0.0005);
const e = w.entities();
console.log('roller before: q', Array.from(e.slice(3, 7)), 'flags 43/52/55/56', [e[43], e[52], e[55], e[56]]);
w.entities()[1 * E + 0] = NaN;
w.step(0.0005);
console.log('roller after : q', Array.from(e.slice(3, 7)), 'omega', Array.from(e.slice(72, 74)));
console.log('mass1 after: pos', Array.from(e.slice(E, E + 3)), 'q', Array.from(e.slice(E + 3, E + 7)));
console.log('mass2 after: pos', Array.from(e.slice(2 * E, 2 * E + 3)));
const d = w.path(0), n = d[0];
console.log('path n', n, 'links', Array.from({ length: n + 1 }, (_, li) => Array.from(d.slice(9 + li * 5, 9 + li * 5 + 3))));
console.log('segments', Array.from({ length: n }, (_, i) => Array.from(d.slice(9 + (n + 1) * 5 + i * 16, 9 + (n + 1) * 5 + i * 16 + 12))));
