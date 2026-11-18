// Repro: a torque-free asymmetric rigid body does not obey Euler's equations.
// The engine integrates the orientation with a constant world-frame omega and
// never applies the gyroscopic term, so angular momentum changes direction.
// Run: node repro_gyroscopic_free_body.mjs   (exit 1 = defect reproduced)
import { makeWorld, rotScene, exactFree, angBetween, rotMatrix, transpose, mm, mv, dot } from './common.mjs';

const w = makeWorld(rotScene({}), { requireRrfTimestep: false });
const I0 = w.inertia('Body');
const omega0 = [1, 2, 0.5];
w.setOmega('Body', omega0);
const T = 0.5, dt = 0.002, steps = Math.round(T / dt), qs = [];
for (let i = 0; i <= steps; i++) {
  qs.push(w.quat('Body'));
  if (i < steps) w.world.update(dt);
}
w.backend.dispose();
const q0 = qs[0];
const exact = exactFree(I0, omega0, q0, T, 2e-5);
const err = angBetween(qs.at(-1), exact.q);
const Iw = q => mm(mm(rotMatrix(q), I0), transpose(rotMatrix(q)));
const L0 = mv(Iw(q0), omega0), Lend = mv(Iw(qs.at(-1)), omega0);
const Ldir = Math.acos(Math.min(1, dot(L0, Lend) / (Math.hypot(...L0) * Math.hypot(...Lend))));
console.log(`body inertia diag: ${I0.map((row, i) => row[i].toFixed(6)).join(', ')} kg m^2`);
console.log(`initial omega: ${omega0.join(', ')} rad/s, no torque, ${T} s at ${dt * 1000} ms`);
console.log(`orientation error vs exact torque-free solution: ${err.toFixed(6)} rad (${(err * 180 / Math.PI).toFixed(2)} deg)`);
console.log(`angular-momentum direction change (should be 0): ${Ldir.toFixed(6)} rad (${(Ldir * 180 / Math.PI).toFixed(2)} deg)`);
if (err > 0.01) {
  console.error('\nFAIL: the engine holds omega constant in the world frame instead of integrating Euler\'s equations.');
  console.error('Control: omega along the symmetry axis (2,0,0) is exact, so this is the missing gyroscopic term.');
  process.exit(1);
}
console.log('\nPASS: free asymmetric rotation matches the exact solution');
