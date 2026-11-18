// Numerical-integration accuracy on analytic systems at documented defaults.
// Run from anywhere: node <this file>
// Exit 0 means every expected first-order/second-order/negative property reproduced.
import fs from 'node:fs';
import {
  G, sceneText, wheelPrim, pointPrim, anchorPrim, jointPrim, pathPrim, dummyRigidBody,
  makeWorld, oscillatorScene, zeroCrossings, pendulumScene, atwoodScene, quadraticFit,
  rotScene, angBetween, axisAngle, qmul, qnorm, rotMatrix, mm, transpose, mv,
  exactFree,
} from './common.mjs';
import { CablePathComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';

const evidence = { generated: new Date().toISOString(), node: process.version,
  wasm: 'public/wasm/cable_solver.wasm (checked-in)', defaults: {}, cases: {}, summary: {}, checks: [] };
const check = (name, ok, detail) => { evidence.checks.push({ name, ok, detail }); return ok; };

// ---------------------------------------------------------------- A. documented defaults
{
  const rows = [];
  for (const scene of ['hp4', 'hp3', 'cubecorners', 'slideprinter']) {
    const text = fs.readFileSync(new URL(`../../../../public/usd_scenes/${scene}.usda`, import.meta.url), 'utf8');
    const { world, dt, backend } = makeWorld(text, { lineLayeringEnabled: false });
    rows.push({ scene, dt, substeps: world.getResource('physicsSubsteps'),
      solverIterations: world.getResource('solverIterations'),
      pathIterations: [...new Set(world.query([CablePathComponent]).map(id => world.getComponent(id, CablePathComponent).solverIterations))] });
    backend.dispose();
  }
  evidence.defaults.production = rows;
}

// ---------------------------------------------------------------- B. free mass
{
  const runs = [];
  for (const dt of [0.008, 0.004, 0.002, 0.001, 0.0005]) {
    const w = makeWorld(sceneText([wheelPrim('Bob', [0, 0, 0], { mass: 1 })], 1 / dt), { requireRrfTimestep: false });
    const steps = Math.round(0.12 / dt);
    for (let i = 0; i < steps; i++) w.world.update(dt);
    const y = w.pos('Bob')[1], v = w.vel('Bob')[1], t = steps * dt;
    runs.push({ dt, steps, t, y, v, exactY: -0.5 * G * t * t, exactV: -G * t,
      posErr: y + 0.5 * G * t * t, velErr: v + G * t });
    w.backend.dispose();
  }
  const subScene = () => sceneText([
    wheelPrim('Bob', [0, 0, 0], { mass: 1 }), anchorPrim('Anchor', [0, 5, 0]),
    jointPrim('J0', 'Anchor', 'Bob', { rest: 10 }),
    pathPrim('Path', ['J0'], { links: ['attachment', 'attachment'], ea: 1e7, iterations: 1 }),
    dummyRigidBody(),
  ], 1 / 0.002);
  const subs = [];
  for (const substeps of [1, 2, 4, 8, 16]) {
    const w = makeWorld(subScene(), { requireRrfTimestep: false, physicsSubsteps: substeps });
    for (let i = 0; i < 50; i++) w.world.update(0.002);
    const y = w.pos('Bob')[1], v = w.vel('Bob')[1];
    subs.push({ substeps, y, posErr: y + 0.5 * G * 0.1 * 0.1, velErr: v + G * 0.1 });
    w.backend.dispose();
  }
  const noGate = [];
  for (const substeps of [1, 16]) {
    // Same point-mass scene *with* a cable path but *without* a rigid body:
    // the substep gate still returns 1, so the setting is silently ignored.
    const w = makeWorld(sceneText([
      wheelPrim('Bob', [0, 0, 0], { mass: 1 }), anchorPrim('Anchor', [0, 5, 0]),
      jointPrim('J0', 'Anchor', 'Bob', { rest: 10 }),
      pathPrim('Path', ['J0'], { links: ['attachment', 'attachment'], ea: 1e7, iterations: 1 }),
    ], 1 / 0.002), { requireRrfTimestep: false, physicsSubsteps: substeps });
    for (let i = 0; i < 50; i++) w.world.update(0.002);
    noGate.push({ substeps, y: w.pos('Bob')[1] });
    w.backend.dispose();
  }
  const slope = Math.log(Math.abs(runs[0].posErr) / Math.abs(runs.at(-1).posErr))
    / Math.log(runs[0].dt / runs.at(-1).dt);
  evidence.cases.freeMass = { runs, substeps: subs, substepGateWithoutRigidBody: noGate, order: slope };
  check('free mass position error = 0.5 g t dt at 2 ms',
    Math.abs(runs[2].posErr - (-0.5 * G * 0.12 * 0.002)) < 1e-9, { measured: runs[2].posErr });
  check('free mass velocity exact', runs.every(r => Math.abs(r.velErr) < 1e-9), null);
  check('free mass position order ~1', Math.abs(slope - 1) < 0.01, { slope });
  check('substeps reduce free-mass position error as 1/n',
    subs.every((r, i) => i === 0 || Math.abs(Math.abs(r.posErr / subs[0].posErr) - 1 / r.substeps) < 0.01), null);
  check('substeps are ignored without a RigidBodyComponent (reported API gate)',
    noGate[0].y === noGate[1].y, noGate);
}

// ---------------------------------------------------------------- C. damped oscillator
const xeqCache = new Map();
const equilibrium = (m, L, EA) => {
  const key = `${m}/${L}/${EA}`;
  if (!xeqCache.has(key)) xeqCache.set(key, L / (1 - m * G / EA));
  return xeqCache.get(key);
};
const oscCase = ({ m, L, EA, B, A, dt, iterations = 4, substeps = 1, dummy = false, T = 1.0 }) => {
  const xeq = equilibrium(m, L, EA);
  const k = EA * L / (xeq * xeq), c = B / xeq;
  const w0 = Math.sqrt(k / m), zeta = c / (2 * Math.sqrt(k * m)), wd = w0 * Math.sqrt(1 - zeta * zeta);
  const w = makeWorld(oscillatorScene({ m, L, EA, B, A, dt, iterations, dummy, xeq }),
    { requireRrfTimestep: false, physicsSubsteps: substeps });
  const steps = Math.round(T / dt), rows = [], exact = [];
  for (let i = 0; i <= steps; i++) {
    const t = i * dt, u = -w.pos('Bob')[1] - xeq;
    const ex = A * Math.exp(-zeta * w0 * t) * (Math.cos(wd * t) + (zeta * w0 / wd) * Math.sin(wd * t));
    rows.push({ t, u }); exact.push({ t, u: ex });
    if (i < steps) w.world.update(dt);
  }
  const finite = rows.every(r => Number.isFinite(r.u));
  w.backend.dispose();
  const ct = zeroCrossings(rows);
  const per = ct.slice(2).map((t, i) => t - ct[i]);
  const exactT = 2 * Math.PI / wd;
  return { m, L, EA, B, A, dt, iterations, substeps, dummy, T, xeq, w0, zeta, exactPeriod: exactT,
    periods: per.slice(0, 4), periodErrPct: per.length ? 100 * (per.reduce((s, x) => s + x, 0) / per.length / exactT - 1) : null,
    maxAbsErr: Math.max(...rows.map((r, i) => Math.abs(r.u - exact[i].u))),
    minSlack: Math.min(...rows.map(r => r.u + xeq - L)), finite };
};
{
  const softDt = [], softIter = [], prod = [], prodSub = [], prodIter = [];
  for (const dt of [0.008, 0.002, 0.0005]) softDt.push(oscCase({ m: 1, L: 1, EA: 1000, B: 6.324555320336759, A: 1e-3, dt }));
  for (const iterations of [1, 4, 64]) softIter.push(oscCase({ m: 1, L: 1, EA: 1000, B: 6.324555320336759, A: 1e-3, dt: 0.002, iterations }));
  prod.push(oscCase({ m: 0.081, L: 2, EA: 20000, B: 5, A: 1e-5, dt: 0.002, iterations: 10, T: 0.5 }));
  for (const substeps of [1, 4, 16]) prodSub.push(oscCase({ m: 0.081, L: 2, EA: 20000, B: 5, A: 1e-5, dt: 0.002, iterations: 10, substeps, dummy: true, T: 0.5 }));
  for (const iterations of [1, 10]) prodIter.push(oscCase({ m: 0.081, L: 2, EA: 20000, B: 5, A: 1e-5, dt: 0.002, iterations, T: 0.5 }));
  evidence.cases.dampedOscillator = { softDt, softIterations: softIter, production: prod,
    productionSubsteps: prodSub, productionIterations: prodIter };
  check('oscillator finite and never slack', [...softDt, ...softIter, ...prod, ...prodSub, ...prodIter]
    .every(r => r.finite && r.minSlack > 0), null);
  check('oscillator period error < 2% at 2 ms', Math.abs(softDt[1].periodErrPct) < 2, { pct: softDt[1].periodErrPct });
  check('oscillator period error decreases with dt and substeps',
    Math.abs(softDt[2].periodErrPct) < Math.abs(softDt[1].periodErrPct)
    && Math.abs(prodSub[1].periodErrPct) < Math.abs(prodSub[0].periodErrPct)
    && Math.abs(prodSub[2].periodErrPct) < Math.abs(prodSub[1].periodErrPct), null);
  check('solver iterations do not change a single-row oscillator',
    softIter.every(r => Math.abs(r.maxAbsErr - softIter[0].maxAbsErr) < 1e-12)
    && prodIter.every(r => Math.abs(r.maxAbsErr - prodIter[0].maxAbsErr) < 1e-12), null);
}

// ---------------------------------------------------------------- D. pendulum
const pendCase = ({ dt = 0.002, iterations = 2, substeps = 1, dummy = false, T = 2.1 } = {}) => {
  const w = makeWorld(pendulumScene({ dt, iterations, dummy }), { requireRrfTimestep: false, physicsSubsteps: substeps });
  const steps = Math.round(T / dt), rows = [];
  for (let i = 0; i <= steps; i++) {
    const [x, y] = w.pos('Bob'), [vx, vy] = w.vel('Bob');
    rows.push({ t: i * dt, theta: Math.atan2(x, -y), energy: 0.5 * (vx * vx + vy * vy) + G * y, r: Math.hypot(x, y) });
    if (i < steps) w.world.update(dt);
  }
  w.backend.dispose();
  const ct = zeroCrossings(rows, 'theta');
  const per = ct.slice(2).map((t, i) => t - ct[i]);
  const exact = 2 * Math.PI * Math.sqrt(1 / G) * (1 + 0.05 ** 2 / 16 + 11 * 0.05 ** 4 / 3072);
  return { dt, iterations, substeps, dummy, T, periods: per.slice(0, 3),
    periodErrPct: per.length ? 100 * (per.reduce((s, x) => s + x, 0) / per.length / exact - 1) : null,
    energyLoss: rows[0].energy - rows.at(-1).energy, maxRadialStretch: Math.max(...rows.map(r => r.r)) - 1 };
};
{
  const dt = [], iter = [], sub = [];
  for (const d of [0.008, 0.002, 0.0005]) dt.push(pendCase({ dt: d, T: 4.1 }));
  for (const iterations of [1, 64]) iter.push(pendCase({ iterations, T: 4.1 }));
  for (const substeps of [1, 16]) sub.push(pendCase({ substeps, dummy: true, T: 1.05 }));
  evidence.cases.pendulum = { dt, iterations: iter, substeps: sub };
  check('pendulum period within 0.01% at 2 ms', Math.abs(dt[1].periodErrPct) < 0.01, { pct: dt[1].periodErrPct });
  check('pendulum period converges with dt',
    Math.abs(dt[0].periodErrPct) > Math.abs(dt[1].periodErrPct) && Math.abs(dt[1].periodErrPct) > Math.abs(dt[2].periodErrPct), null);
  check('pendulum energy loss is first order and substeps reduce it',
    Math.abs(dt[0].energyLoss / dt[2].energyLoss - 16) < 3 && sub[1].energyLoss < 0.55 * sub[0].energyLoss,
    { losses: dt.map(r => r.energyLoss), substepLosses: sub.map(r => r.energyLoss) });
}

// ---------------------------------------------------------------- E. Atwood
const atwoodRun = ({ r = 0.05, I = 0.0025, m1 = 2, m2 = 1, T = 0.3 } = {}) => {
  const w = makeWorld(atwoodScene({ r, I, m1, m2 }), { requireRrfTimestep: false });
  const steps = Math.round(T / 0.002), rows = [];
  for (let i = 0; i <= steps; i++) { rows.push({ t: i * 0.002, y: w.pos('M1')[1] }); if (i < steps) w.world.update(0.002); }
  w.backend.dispose();
  const fit = quadraticFit(rows);
  return { r, I, m1, m2, aMeasured: -fit.a,
    aNoInertia: (m1 - m2) * G / (m1 + m2),
    aWithInertia: (m1 - m2) * G / (m1 + m2 + I / (r * r)) };
};
{
  const rows = [
    atwoodRun({ r: 0.05, I: 0, m1: 2, m2: 1 }),
    atwoodRun({ r: 0.05, I: 0.0025, m1: 2, m2: 1 }),
    atwoodRun({ r: 0.05, I: 0.01, m1: 2, m2: 1 }),
    atwoodRun({ r: 0.0065, I: 5e-7, m1: 0.081, m2: 0.0405 }),
    atwoodRun({ r: 0.0065, I: 4 * 5e-7, m1: 0.081, m2: 0.0405 }),
  ];
  evidence.cases.atwood = rows;
  check('Atwood matches the massless-pulley formula', rows.slice(0, 3).every(r => Math.abs(r.aMeasured - r.aNoInertia) < 1e-3), rows);
  check('Atwood ignores authored pulley inertia (reported model limitation)',
    rows.slice(1).every(r => Math.abs(r.aMeasured - r.aWithInertia) > 0.02 * r.aWithInertia), rows);
}

// ---------------------------------------------------------------- F. freely rotating body
const rotRun = ({ omega0 = [1, 2, 0.5], T = 0.5, dt = 0.002 } = {}) => {
  const w = makeWorld(rotScene({}), { requireRrfTimestep: false });
  const I0 = w.inertia('Body');
  w.setOmega('Body', omega0);
  const steps = Math.round(T / dt), qs = [];
  for (let i = 0; i <= steps; i++) { qs.push(w.quat('Body')); if (i < steps) w.world.update(dt); }
  w.backend.dispose();
  const q0 = qs[0];
  const exact = exactFree(I0, omega0, q0, T, 2e-5);
  const constQ = qnorm(qmul(axisAngle(omega0, Math.hypot(...omega0) * T), q0));
  const Iw = q => mm(mm(rotMatrix(q), I0), transpose(rotMatrix(q)));
  const L0 = mv(Iw(q0), omega0), Lend = mv(Iw(qs.at(-1)), omega0);
  return { T, dt, omega0, I0, errVsConstantOmega: angBetween(qs.at(-1), constQ),
    errVsExactFreeBody: angBetween(qs.at(-1), exact.q),
    angularMomentumDirectionErrRad: Math.acos(Math.min(1,
      (L0[0] * Lend[0] + L0[1] * Lend[1] + L0[2] * Lend[2]) / (Math.hypot(...L0) * Math.hypot(...Lend)))) };
};
{
  const control = rotRun({ omega0: [2, 0, 0] });
  const generic = rotRun({ omega0: [1, 2, 0.5] });
  const generic2 = rotRun({ omega0: [3, 1, -2] });
  evidence.cases.freeRotation = { control, generic, generic2 };
  check('axis-aligned free rotation is exact', control.errVsExactFreeBody < 1e-12, control);
  check('generic free rotation holds omega constant (engine model)', generic.errVsConstantOmega < 1e-12, generic);
  check('generic free rotation violates exact torque-free motion (reported limitation)',
    generic.errVsExactFreeBody > 0.2 && generic.angularMomentumDirectionErrRad > 0.4, generic);
}

// ---------------------------------------------------------------- G. guide count
{
  const guideRun = (guides, iterations = 4) => {
    const parts = [anchorPrim('Anchor', [0, 0, 0])];
    const chain = ['Anchor'], ys = [0];
    for (let k = 1; k <= guides; k++) {
      const y = -k / (guides + 1);
      parts.push(pointPrim(`G${k}`, [0, y], -1)); chain.push(`G${k}`); ys.push(y);
    }
    parts.push(wheelPrim('Bob', [0, -1.01981, 0], { mass: 1 })); chain.push('Bob'); ys.push(-1.01981);
    const links = ['attachment'];
    for (let k = 0; k < chain.length - 1; k++) {
      parts.push(jointPrim(`J${k}`, chain[k], chain[k + 1], { rest: Math.abs(ys[k + 1] - ys[k]) }));
      if (k > 0) links.push('pinhole');
    }
    links.push('attachment');
    parts.push(pathPrim('Path', Array.from({ length: chain.length - 1 }, (_, i) => `J${i}`),
      { links, ea: 1000, damping: 6.324555320336759, iterations }));
    const w = makeWorld(sceneText(parts, 1 / 0.002), { requireRrfTimestep: false });
    const rows = [];
    for (let i = 0; i <= 500; i++) { rows.push(w.pos('Bob')[1]); if (i < 500) w.world.update(0.002); }
    w.backend.dispose();
    return rows;
  };
  const ref = guideRun(0);
  const rows = [0, 1, 3].map(g => {
    const r = guideRun(g);
    return { guides: g, maxDiffVsZeroGuides: Math.max(...r.map((x, i) => Math.abs(x - ref[i]))) };
  });
  evidence.cases.guideCount = rows;
  check('guide count does not change the dynamics', rows.every(r => r.maxDiffVsZeroGuides < 1e-9), rows);
}

// ---------------------------------------------------------------- H. solver iterations, two-cable V
{
  const vScene = iterations => {
    const d = 0.5, h = 1, L0 = Math.hypot(d, h);
    return sceneText([anchorPrim('A0', [-d, 0, 0]), anchorPrim('A1', [d, 0, 0]), pointPrim('Bob', [0, -h], 1),
      jointPrim('J0', 'A0', 'Bob', { rest: L0 }), jointPrim('J1', 'A1', 'Bob', { rest: L0 }),
      pathPrim('P0', ['J0'], { links: ['attachment', 'attachment'], ea: 20000, damping: 5, iterations }),
      pathPrim('P1', ['J1'], { links: ['attachment', 'attachment'], ea: 20000, damping: 5, iterations }),
    ], 1 / 0.002);
  };
  const rows = [];
  for (const iterations of [1, 2, 4, 8, 64]) {
    const w = makeWorld(vScene(iterations), { requireRrfTimestep: false });
    for (let i = 0; i < 600; i++) w.world.update(0.002);
    rows.push({ iterations, y: w.pos('Bob')[1] });
    w.backend.dispose();
  }
  evidence.cases.solverIterations = rows;
  const converged = rows.at(-1).y;
  check('solver iterations reduce a two-cable static residual',
    Math.abs(rows[0].y - converged) > 10 * Math.abs(rows.at(-1).y - converged) && Math.abs(rows[0].y - converged) < 2e-5, rows);
}

const failures = evidence.checks.filter(c => !c.ok);
evidence.summary = { checks: evidence.checks.length, failed: failures.length, failedNames: failures.map(c => c.name) };
fs.writeFileSync(new URL('./evidence.json', import.meta.url), JSON.stringify(evidence, null, 1) + '\n');
for (const c of evidence.checks) console.log(`${c.ok ? 'PASS' : 'FAIL'}  ${c.name}`);
console.log(`\nwrote evidence.json (${evidence.checks.length} checks, ${failures.length} failed)`);
process.exit(failures.length ? 1 : 0);
