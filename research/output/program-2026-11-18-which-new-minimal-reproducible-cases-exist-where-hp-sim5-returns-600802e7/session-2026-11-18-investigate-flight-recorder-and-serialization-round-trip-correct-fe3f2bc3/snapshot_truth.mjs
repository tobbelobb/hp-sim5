// Ground truth for the recorded line-length error, from the production JS snapshot.
// Usage: node snapshot_truth.mjs <scene.usda> <steps> <radians>
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { captureFlightRecorderSnapshot } from '../../../../hp-sim-3d/app/flightRecorderSnapshot.js';

const [scene, stepsArg, moveArg] = process.argv.slice(2);
const steps = Number(stepsArg);
const moveValue = Number(moveArg);
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(scene, 'utf8'), {
  requireRrfTimestep: false,
});
const axes = Object.keys(remote.axisToEntity).slice(0, 4);
let peak = 0;
const samples = [];
for (let step = 1; step <= steps; step++) {
  if (step === 1) {
    const move = { type: 'Move' };
    for (const axis of axes) move[axis] = moveValue;
    remote.addCommand(move);
  }
  world.update(dt);
  const snapshot = captureFlightRecorderSnapshot(world);
  const row = [];
  for (const cable of snapshot.cables) {
    const error = cable.lengths.error;
    if (error !== null) peak = Math.max(peak, Math.abs(error));
    row.push({ name: `${cable.machine}/${cable.name}`, commanded: cable.lengths.commanded,
      actual: cable.lengths.actual, error });
  }
  samples.push({ step, sim_time_s: step * dt, cables: row });
}
console.log(JSON.stringify({ scene, steps, move_rad: moveValue, dt_s: dt, axes,
  peak_abs_error_m: peak, samples }));
