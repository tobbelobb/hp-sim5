import { World } from '../../../src/js/cable_joints_3d/ecs.js';

function setup(fail = false) {
  const world = new World();
  const calls = [];
  world.setResource('dt', .002);
  world.setResource('physicsSubsteps', 4);
  for (const [name, physicsSubstep] of [['command', false], ['integrate', true], ['solve', true], ['record', false]]) {
    world.registerSystem({ physicsSubstep, update: (state, dt) => {
      calls.push([name, dt, state.getResource('dt')]);
      if (fail && name === 'solve') throw new Error('solver failed');
    } });
  }
  return { world, calls };
}

test('substeps the complete physics pipeline and dispatches commands/recording once', () => {
  const { world, calls } = setup();
  world.update(.002);
  expect(calls).toEqual([
    ['command', .002, .002],
    ...Array.from({ length: 4 }, () => [['integrate', .0005, .0005], ['solve', .0005, .0005]]).flat(),
    ['record', .002, .002],
  ]);
  expect(world.getResource('dt')).toBe(.002);
});

test('restores the external timestep when physics fails', () => {
  const { world } = setup(true);
  expect(() => world.update(.002)).toThrow('solver failed');
  expect(world.getResource('dt')).toBe(.002);
});
