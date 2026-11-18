// Negative control: changing physics substeps mid-run does not desynchronise the
// worker->world command contract that the whole playback pipeline relies on.
//
// The contract: the playback worker emits one `Move`/`Add to reference` per
// firmware tick (dt), and RemoteSpoolSystem consumes exactly one command per
// World.update(dt). If `physicsSubsteps` changed how much simulated time one
// World.update(dt) advances (or how many times the non-physics systems run), the
// worker's stream would run ahead of / behind the world after the change.
//
// ecs.js World.update() divides dt by physicsSubsteps for the physics group and
// restores the original dt afterwards, so:
//   * the sum of the physics systems' dt across one update is still dt,
//   * systems outside the physics group still see the full dt exactly once,
//   * the world's `dt` resource is unchanged after the update (so a `set_dt`
//     posted by workerController stays valid across a substep change).
// This script measures that for substeps 1, 2, 4, 8, 16 and for a change made
// mid-run, and reports PASS (exit 0) when the contract holds.
//
// Run:  node probe_substeps_cadence.mjs
import { World } from '../../../../src/js/cable_joints_3d/ecs.js';

const DT = 0.002;  // every reachable scene uses timeCodesPerSecond = 500

class ProbePhysics {
  constructor() { this.physicsSubstep = true; this.calls = 0; this.receivedDt = 0; }
  physicsSubsteps(state) {
    const count = state.getResource('physicsSubsteps');
    return count > 1 && state.query([]).length >= 0 ? count : 1;
  }
  update(world, dt) { this.calls += 1; this.receivedDt += dt; }
}

class ProbeOutside {
  constructor() { this.calls = 0; this.receivedDt = 0; }
  update(world, dt) { this.calls += 1; this.receivedDt += dt; }
}

function measure(substepsBefore, substepsAfter) {
  const world = new World();
  world.setResource('dt', DT);
  world.setResource('physicsSubsteps', substepsBefore);
  const physics = new ProbePhysics();
  const outside = new ProbeOutside();
  world.registerSystem(outside);
  world.registerSystem(physics);

  world.update(world.getResource('dt'));
  const first = { substeps: substepsBefore, physicsCalls: physics.calls,
    physicsTime: physics.receivedDt, outsideCalls: outside.calls,
    outsideDt: outside.receivedDt / outside.calls, dtAfter: world.getResource('dt') };

  world.setResource('physicsSubsteps', substepsAfter);   // settings change mid-run
  physics.calls = 0; physics.receivedDt = 0; outside.calls = 0; outside.receivedDt = 0;
  world.update(world.getResource('dt'));
  const second = { substeps: substepsAfter, physicsCalls: physics.calls,
    physicsTime: physics.receivedDt, outsideCalls: outside.calls,
    outsideDt: outside.receivedDt / outside.calls, dtAfter: world.getResource('dt') };

  return { first, second };
}

const cases = [[1, 1], [2, 2], [4, 4], [8, 8], [16, 16], [1, 8], [8, 1]].map(([a, b]) => ({
  change: `${a}->${b}`, ...measure(a, b),
}));
console.log(JSON.stringify({ dt: DT, cases }, null, 2));

const failures = [];
for (const entry of cases) {
  for (const phase of [entry.first, entry.second]) {
    const label = `${entry.change} substeps=${phase.substeps}`;
    if (Math.abs(phase.physicsTime - DT) > 1e-12) failures.push(`${label}: physics advanced ${phase.physicsTime}s, expected ${DT}s`);
    if (phase.physicsCalls !== phase.substeps) failures.push(`${label}: ${phase.physicsCalls} physics calls, expected ${phase.substeps}`);
    if (phase.outsideCalls !== 1) failures.push(`${label}: outside system ran ${phase.outsideCalls} times, expected 1`);
    if (Math.abs(phase.outsideDt - DT) > 1e-12) failures.push(`${label}: outside system saw dt=${phase.outsideDt}, expected ${DT}`);
    if (phase.dtAfter !== DT) failures.push(`${label}: world dt changed to ${phase.dtAfter}`);
  }
}
if (failures.length) {
  console.error('FAIL (substep change desynchronised the command cadence):\n - ' + failures.join('\n - '));
  process.exit(1);
}
console.error('PASS: one World.update(dt) always advances dt and runs the command consumer once');
