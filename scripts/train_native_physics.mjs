// Bounded, deterministic PGO training. No recording, firmware or browser.
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { createHeadlessWorld } from '../hp-sim-3d/app/headlessWorld.js';
import { NativePhysicsEngine } from '../src/js/cable_joints_3d/native_physics_engine.js';
import { PositionComponent } from '../src/js/cable_joints_3d/ecs.js';

const root = new URL('../', import.meta.url);
const worlds = [];
for (const machine of ['hp3', 'hp4']) for (const layering of [true, false]) {
  const source = fs.readFileSync(new URL(`public/usd_scenes/${machine}_rigid_body.usda`, root), 'utf8');
  const { world, remote, dt, bakedScene } = createHeadlessWorld(source, {
    lineLayeringEnabled: layering, cableSolverBackend: new NativePhysicsEngine({ addonPath: process.argv[2] }),
  });
  try {
    for (let step = 0; step < 10000; step++) {
      if (step === 2500 || step === 4000) world.setResource('closedLoopMotorsEnabled', step === 2500);
      remote.addCommand(step < 5000 ? { type: 'Move', A: -.002 * step }
        : step === 5000 ? { type: 'SetTorqueMode', axis: 'B', torqueNm: -.003 } : {});
      world.update(dt);
    }
    for (const id of world.query([PositionComponent])) {
      const p = world.getComponent(id, PositionComponent).pos;
      if (![p.x, p.y, p.z].every(Number.isFinite)) throw new Error('Non-finite PGO training position');
    }
    if (world.getResource('errorState')?.hasError || world.getResource('pauseState')?.paused) {
      throw new Error('PGO training stopped before completing its fixed steps');
    }
    worlds.push({ machine, layering, dt, steps: 10000,
      bakedSceneSha256: createHash('sha256').update(bakedScene).digest('hex') });
  } finally { world.getResource('cableSolverBackend').dispose(); }
}
fs.writeFileSync(process.argv[3], JSON.stringify({ worlds, steps: 40000,
  trainer: fileURLToPath(import.meta.url) }, null, 2) + '\n');
