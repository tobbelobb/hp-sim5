// Ground truth for extrusion deposits, read directly from the ExtruderComponent.
// Usage: node extrusion_truth.mjs <scene.usda> <steps> <radians>
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { ExtruderComponent } from '../../../../hp-sim-3d/app/hangprinter_extruder.js';

const [scene, stepsArg, moveArg] = process.argv.slice(2);
const steps = Number(stepsArg);
const move = Number(moveArg);
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(scene, 'utf8'), {
  requireRrfTimestep: false,
});
const axes = Object.keys(remote.axisToEntity).slice(0, 4);
const command = { type: 'Move', E: move };
for (const axis of axes) command[axis] = move;
remote.addCommand(command);
for (let step = 0; step < steps; step++) world.update(dt);
const extruder = world.getComponent(world.query([ExtruderComponent])[0], ExtruderComponent);
console.log(JSON.stringify({ steps, command, extrusions: extruder.extrusions,
  deposited_length: extruder.extrusions.reduce((sum, row) => sum + row.length, 0) }));
