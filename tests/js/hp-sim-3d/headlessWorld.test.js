import fs from 'node:fs';
import path from 'node:path';
import { createHeadlessWorld } from '../../../hp-sim-3d/app/headlessWorld.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { layeringEnabled, PositionComponent, EncoderComponent } from '../../../src/js/cable_joints_3d/ecs.js';

test.each(['hp3', 'hp4'])('%s headless construction uses the browser no-layering geometry', machine => {
  const source = fs.readFileSync(path.resolve(__dirname, `../../../public/usd_scenes/${machine}_rigid_body.usda`), 'utf8');
  const unlayered = createHeadlessWorld(source, { lineLayeringEnabled: false, closedLoopMotorsEnabled: true });
  const layered = createHeadlessWorld(source, { lineLayeringEnabled: true });
  expect(unlayered.bakedScene).toBe(bakeCableSceneUsdaSource(source, { cablePathHalfWidthOverride: 0, deriveAll: true }).source);
  expect(unlayered.bakedScene).not.toBe(layered.bakedScene);
  for (const key of ['enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps']) {
    expect(unlayered.world.getResource(key)).toBe(false);
    expect(layered.world.getResource(key)).toBe(true);
  }
  expect(unlayered.world.getResource('closedLoopMotorsEnabled')).toBe(true);
  expect(layered.world.getResource('closedLoopMotorsEnabled')).toBe(false);
  expect(layeringEnabled(unlayered.world)).toBe(false);
  expect(layeringEnabled(layered.world)).toBe(true);
  const lengths = ({ world }) => world.query([CablePathComponent]).map(entity => world.getComponent(entity, CablePathComponent).totalRestLength);
  // The changed bake must reach the physical cable constraints, not just metadata.
  expect(lengths(unlayered)).not.toEqual(lengths(layered));
  expect(unlayered.dt).toBe(.002);
});

test.each(['hp3', 'hp4'])('%s profiling covers every production system without changing physics', machine => {
  const source = fs.readFileSync(path.resolve(__dirname, `../../../public/usd_scenes/${machine}_rigid_body.usda`), 'utf8');
  const plain = createHeadlessWorld(source);
  const profiled = createHeadlessWorld(source, { profiling: { enabled: true, logEverySteps: 0, slowStepMs: Infinity } });
  expect(plain.world.getResource('performanceMonitor').enabled).toBe(false);
  for (const instance of [plain, profiled]) {
    instance.remote.addCommand({ type: 'Move', A: .01 });
    for (let step = 0; step < 20; step++) instance.world.update(instance.dt);
  }
  const stats = profiled.world.getResource('performanceMonitor').snapshot();
  expect(stats.steps).toBe(20);
  expect(stats.systems.map(row => row.system).sort()).toEqual(profiled.world.systems.map(system => system.constructor.name).sort());
  expect(stats.systems.every(row => row.calls === 20 && row.totalMs >= 0)).toBe(true);
  for (const Type of [PositionComponent, EncoderComponent, CablePathComponent]) {
    const read = ({ world }) => world.query([Type]).map(id => world.getComponent(id, Type));
    expect(read(profiled)).toEqual(read(plain));
  }
});
