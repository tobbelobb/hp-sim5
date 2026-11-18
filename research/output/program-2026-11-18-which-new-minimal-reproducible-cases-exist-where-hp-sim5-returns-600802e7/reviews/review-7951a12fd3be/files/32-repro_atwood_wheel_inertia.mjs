// Repro: a passive wheel's authored inertia is ignored, so an Atwood machine
// with a finite-inertia pulley accelerates as if the pulley were massless.
// Run: node repro_atwood_wheel_inertia.mjs   (exit 1 = defect reproduced)
import { G, makeWorld, atwoodScene, quadraticFit } from './common.mjs';

function measure({ r, I, m1, m2, T = 0.3 }) {
  const w = makeWorld(atwoodScene({ r, I, m1, m2 }), { requireRrfTimestep: false });
  const steps = Math.round(T / 0.002), rows = [];
  for (let i = 0; i <= steps; i++) {
    rows.push({ t: i * 0.002, y: w.pos('M1')[1] });
    if (i < steps) w.world.update(0.002);
  }
  w.backend.dispose();
  const fit = quadraticFit(rows);
  return { measured: -fit.a, noInertia: (m1 - m2) * G / (m1 + m2),
    withInertia: (m1 - m2) * G / (m1 + m2 + I / (r * r)) };
}
const cases = [
  { label: 'hoop-like pulley, I/r^2 = 1 kg', r: 0.05, I: 0.0025, m1: 2, m2: 1 },
  { label: 'one shipped HP4 wheel (5e-7 kg m^2, r=6.5 mm), 81 g / 40.5 g loads',
    r: 0.0065, I: 5e-7, m1: 0.081, m2: 0.0405 },
  { label: 'four shipped HP4 wheels (D-line idler count), 81 g / 40.5 g loads',
    r: 0.0065, I: 4 * 5e-7, m1: 0.081, m2: 0.0405 },
];
let failed = false;
for (const c of cases) {
  const m = measure(c);
  const err = 100 * (m.measured / m.withInertia - 1);
  console.log(`${c.label}\n  expected with pulley inertia: ${m.withInertia.toFixed(6)} m/s^2` +
    `\n  expected with massless pulley: ${m.noInertia.toFixed(6)} m/s^2` +
    `\n  measured: ${m.measured.toFixed(6)} m/s^2  (${err >= 0 ? '+' : ''}${err.toFixed(2)}% vs with-inertia)`);
  if (Math.abs(m.measured - m.withInertia) > 0.02 * m.withInertia) failed = true;
}
if (failed) {
  console.error('\nFAIL: the authored physics:inertiaTensor of a passive wheel does not affect the dynamics.');
  console.error('The engine treats every non-motor wheel on a rolling link as massless (engine.rs turn_wheel).');
  process.exit(1);
}
console.log('\nPASS: pulley inertia is honored');
