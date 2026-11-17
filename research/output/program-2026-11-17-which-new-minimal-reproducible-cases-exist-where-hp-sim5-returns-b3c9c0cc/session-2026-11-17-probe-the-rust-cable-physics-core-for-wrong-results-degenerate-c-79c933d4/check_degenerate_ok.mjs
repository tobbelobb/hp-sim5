// Negative-result checks for the degenerate / closed-form cases named in the
// question. Exits nonzero only if an invariant is VIOLATED (i.e. if a bug is
// present). A zero exit means the Rust core behaved correctly on these inputs.
//
// Each check prints the expected closed form, the measured value, the error and
// the tolerance it is compared against.
import { fixed, mass, spool, pathRecord, world, pos, E, H, L, J, ATTACHMENT, ROLLING } from './lib.mjs';
const P = (x, y) => [x, y, 0];
const G = 9.81;
let failures = 0;
const check = (name, ok, detail) => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}: ${detail}`);
  if (!ok) failures++;
};
const finitePath = w => {
  const d = w.path(0), n = d[0];
  const out = { stored: [], rests: [], tensions: [] };
  for (let li = 0; li <= n; li++) out.stored.push(d[H + li * L + 2]);
  for (let i = 0; i < n; i++) {
    const o = H + (n + 1) * L + i * J;
    out.rests.push(d[o + 3]); out.tensions.push(d[o + 11]);
  }
  return out;
};

// ---- 1. Atwood hairpin over a roller: a = g(m1-m2)/(m1+m2), T = m1(g-a) ----
function atwood({ r, m1 = 2, m2 = 1, side = -1 }) {
  const cw = side < 0;
  const es = [spool(0, P(0, 0), r, { mu: 0, inertia: 0, spool: false }),
    mass(1, P(side * (r + 1e-9), -1), m1), mass(2, P(-side * (r + 1e-9), -1), m2)];
  return world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw, stored: 0 }, { kind: ROLLING, cw, stored: Math.PI * r },
      { kind: ATTACHMENT, cw, stored: 0 }],
    [{ a: 1, b: 0, rest: 1, pa: P(side * (r + 1e-9), -1), pb: P(side * r, 0) },
      { a: 0, b: 2, rest: 1, pa: P(-side * r, 0), pb: P(-side * (r + 1e-9), -1) }],
    { stiffness: 1e6, iterations: 8 })]);
}
for (const r of [0.05, 0.02, 1e-3, 1e-9, 0]) {
  const w = atwood({ r });
  for (let i = 0; i < 200; i++) w.step(0.0005);
  const v0 = w.entities()[1 * E + 70];
  for (let i = 0; i < 400; i++) w.step(0.0005);
  const a = -(w.entities()[1 * E + 70] - v0) / (400 * 0.0005);
  const T = w.path(0)[H + L + 2] !== undefined ? w.tension(0, 0) : NaN;
  const aExact = G * 1 / 3, TExact = 2 * (G - aExact);
  check(`hairpin r=${r} acceleration`, Math.abs(a - aExact) < 1e-6,
    `a=${a.toFixed(9)} exact=${aExact.toFixed(9)} err=${(a - aExact).toExponential(2)} tol=1e-6`);
  check(`hairpin r=${r} tension`, Math.abs(T - TExact) < 1e-3 * TExact,
    `T=${T.toFixed(6)} exact=${TExact.toFixed(6)} err=${(T - TExact).toExponential(2)} tol=${(1e-3 * TExact).toExponential(2)}`);
}
// Mirror symmetry: heavy side left vs right must give identical trajectories.
{
  const a = atwood({ r: 0.05, side: -1 }), b = atwood({ r: 0.05, side: 1 });
  for (let i = 0; i < 400; i++) { a.step(0.0005); b.step(0.0005); }
  const ya = pos(a, 1)[1], yb = pos(b, 1)[1];
  check('hairpin mirror symmetry', Math.abs(ya - yb) < 1e-12,
    `y_left=${ya.toFixed(12)} y_right=${yb.toFixed(12)} err=${Math.abs(ya - yb).toExponential(2)} tol=1e-12`);
}

// ---- 2. Statics: hanging mass, stretch = mgL/EA, reported T = mg ----
{
  const ea = 20000, m = 10, L = 1;
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -L), m)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: L, pa: P(0, 0), pb: P(0, -L) }],
    { stiffness: ea, damping: 50, iterations: 4 })]);
  for (let i = 0; i < 4000; i++) w.step(0.001);
  const stretch = -pos(w, 1)[1] - L, exact = m * G / ea;
  const T = w.tension(0, 0);
  check('hanging stretch', Math.abs(stretch - exact) < 0.02 * exact,
    `stretch=${stretch.toExponential(6)} exact=${exact.toExponential(6)} err=${(100 * (stretch - exact) / exact).toFixed(2)}% tol=2%`);
  check('hanging tension', Math.abs(T - m * G) < 1e-3 * m * G,
    `T=${T.toFixed(6)} mg=${(m * G).toFixed(6)} err=${(T - m * G).toExponential(2)} tol=${(1e-3 * m * G).toExponential(2)}`);
}
// Slack: rest longer than the span -> exactly zero tension.
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, -1), 10)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1.05, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 20000, iterations: 8 })]);
  let maxT = 0;
  for (let i = 0; i < 20; i++) { w.step(0.001); maxT = Math.max(maxT, w.tension(0, 0)); }
  check('slack tension is exactly zero', maxT === 0, `max T over 20 steps = ${maxT}`);
}

// ---- 3. Spool payout: a rotation dtheta moves the mass by exactly r*dtheta ----
function payout(r, dtheta, { inertia = 1e6, stiffness = 2e4, damping = 50, n = 10,
  settle = 6000, dt = 0.001 } = {}) {
  const es = [spool(0, P(0, 0), r, { inertia }), mass(1, P(r, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ROLLING, cw: false, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 1, pa: P(r, 0), pb: P(r, -1) }],
    { stiffness, damping, iterations: 8 })]);
  const ent = w.entities();
  for (let i = 0; i < settle; i++) w.step(dt);
  const y0 = pos(w, 1)[1], s0 = w.stored(0, 0), r0 = w.rest(0, 0);
  for (let k = 0; k < n; k++) {
    const a = dtheta * (k + 1) / n;
    ent[3] = 0; ent[4] = 0; ent[5] = Math.sin(a / 2); ent[6] = Math.cos(a / 2);
    w.step(dt);
  }
  for (let i = 0; i < settle; i++) w.step(dt);
  return { dy: pos(w, 1)[1] - y0, dstored: w.stored(0, 0) - s0, drest: w.rest(0, 0) - r0 };
}
for (const r of [0.05, 0.02]) {
  for (const dtheta of [1e-2, -1e-2]) {
    const q = payout(r, dtheta);
    // +dtheta winds line onto the spool and lifts the mass by r*dtheta.
    check(`spool payout r=${r} dtheta=${dtheta}`, Math.abs(q.dy - r * dtheta) < 1e-2 * Math.abs(r * dtheta),
      `dy=${q.dy.toExponential(6)} exact=${(r * dtheta).toExponential(6)} err=${(100 * (q.dy - r * dtheta) / (r * dtheta)).toFixed(2)}% tol=1%`);
    check(`spool stored r=${r} dtheta=${dtheta}`, Math.abs(q.dstored - r * dtheta) < 1e-9,
      `dstored=${q.dstored.toExponential(6)} exact=${(r * dtheta).toExponential(6)}`);
  }
}

// ---- 4. Coincident anchors ----
{
  // rest > 0, coincident: slack, zero tension, rest untouched.
  const es = [fixed(0, P(0, 0)), fixed(1, P(0, 0))];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0.5, pa: P(0, 0), pb: P(0, 0) }],
    { stiffness: 1e5, iterations: 8 })]);
  for (let i = 0; i < 20; i++) w.step(0.001);
  const p = finitePath(w);
  check('coincident anchors, rest 0.5', p.tensions[0] === 0 && Math.abs(p.rests[0] - 0.5) < 1e-12
    && p.stored.every(Number.isFinite), `T=${p.tensions[0]} rest=${p.rests[0]} stored=${p.stored}`);
  // rest = 0, coincident: the clamp lifts the rest to >= 1e-6, still no force.
  const w2 = world([fixed(0, P(0, 0)), fixed(1, P(0, 0))], [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0, pa: P(0, 0), pb: P(0, 0) }],
    { stiffness: 1e5, iterations: 8 })]);
  for (let i = 0; i < 20; i++) w2.step(0.001);
  const p2 = finitePath(w2);
  check('coincident anchors, rest 0', p2.tensions[0] === 0 && p2.rests[0] >= 0 && p2.rests[0] < 1e-5
    && p2.stored.every(Number.isFinite), `T=${p2.tensions[0]} rest=${p2.rests[0].toExponential(3)}`);
}

// ---- 5. Zero rest length tether holds the mass at the anchor ----
{
  const es = [fixed(0, P(0, 0)), mass(1, P(0, 0), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0, pa: P(0, 0), pb: P(0, 0) }],
    { stiffness: 2e4, iterations: 8 })]);
  let maxd = 0, maxT = 0;
  for (let i = 0; i < 1000; i++) {
    w.step(0.001);
    maxd = Math.max(maxd, Math.hypot(...pos(w, 1)));
    maxT = Math.max(maxT, w.tension(0, 0));
  }
  check('zero-length tether', maxd < 5e-6 && maxT < 20 && Number.isFinite(maxT),
    `max |mass anchor offset|=${maxd.toExponential(3)} m (tol 5e-6), max T=${maxT.toFixed(3)} N (tol 20)`);
}

// ---- 6. Duplicate path points (zero-length segment in a merged row) ----
{
  const es = [fixed(0, P(0, 0)), fixed(1, P(0, 0)), mass(2, P(0, -1), 1)];
  const w = world(es, [pathRecord(
    [{ kind: ATTACHMENT, cw: true, stored: 0 }, { kind: ROLLING, cw: true, stored: 0 },
      { kind: ATTACHMENT, cw: true, stored: 0 }],
    [{ a: 0, b: 1, rest: 0, pa: P(0, 0), pb: P(0, 0) },
      { a: 1, b: 2, rest: 1, pa: P(0, 0), pb: P(0, -1) }],
    { stiffness: 2e4, iterations: 8 })]);
  let ok = true;
  for (let i = 0; i < 4000; i++) {
    w.step(0.001);
    const p = finitePath(w);
    if (!p.tensions.every(x => Number.isFinite(x) && x >= 0)) ok = false;
  }
  const p = finitePath(w);
  check('duplicate path points', ok && Math.abs(p.tensions[1] - G) < 1e-3 * G,
    `T=${p.tensions.map(x => x.toFixed(6))} exact seg1=${G.toFixed(6)}`);
  // Reporting note: the zero-length segment reports the row tension magnitude
  // while its force vector is necessarily zero (it has no direction).
  const d = w.path(0), n = d[0], o = H + (n + 1) * L;
  const f0 = Array.from(d.slice(o + 13, o + 16));
  console.log(`NOTE  duplicate-path force reporting: zero-length seg T=${p.tensions[0].toFixed(3)} N`
    + ` but force vector=[${f0.map(x => x.toFixed(3))}]`);
}

// ---- 7. World isolation ----
{
  const mk = () => atwood({ r: 0.05 });
  const a = mk(), b = mk();
  for (let i = 0; i < 100; i++) { a.step(0.0005); b.step(0.0005); }
  const yb = pos(b, 1)[1], qb = Array.from(b.entities().slice(0, 40));
  for (let i = 0; i < 100; i++) a.step(0.0005);
  const untouched = Math.abs(pos(b, 1)[1] - yb) < 1e-15
    && Array.from(b.entities().slice(0, 40)).every((x, i) => x === qb[i]);
  check('two worlds do not share state', untouched, `other world bit-identical while the first ran 100 more steps: ${untouched}`);
  const c = mk(), d = mk();
  for (let i = 0; i < 200; i++) { c.step(0.0005); d.step(0.0005); }
  const ce = c.entities(), de = d.entities(), cp = c.path(0), dp = d.path(0);
  const identical = Array.from(ce).every((x, i) => Object.is(x, de[i]))
    && Array.from(cp).every((x, i) => Object.is(x, dp[i]));
  check('two worlds are deterministic', identical, `identical records: ${identical}`);
}

// ---- 8. Reset semantics: dispose + rebuild reproduces a fresh first step ----
{
  const mk = () => atwood({ r: 0.05 });
  const fresh = mk(); fresh.step(0.0005);
  const expect = Array.from(fresh.path(0));
  const reused = mk();
  for (let i = 0; i < 50; i++) reused.step(0.0005);
  reused.dispose();
  const again = mk(); again.step(0.0005);
  const got = Array.from(again.path(0));
  check('dispose+rebuild first step is fresh', JSON.stringify(expect) === JSON.stringify(got),
    `path records identical after ${expect.length} fields`);
}

console.log(failures ? `\n${failures} check(s) failed` : '\nall degenerate/closed-form checks pass');
process.exit(failures ? 1 : 0);
