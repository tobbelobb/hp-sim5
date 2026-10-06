import { createResearchControls, attachResearchControls } from '../../../hp-sim-3d/app/researchControls.js';
import { captureFlightRecorderSnapshot } from '../../../hp-sim-3d/app/flightRecorderSnapshot.js';

jest.mock('../../../hp-sim-3d/app/flightRecorderSnapshot.js', () => ({ captureFlightRecorderSnapshot: jest.fn() }));

function setup() {
  const clock = { step: 20, time: .04 };
  const resources = new Map([['researchClock', clock], ['sceneGeneration', 3], ['pauseState', { paused: true }]]);
  const world = { getResource: (name) => resources.get(name) };
  const remote = { _ensureAxisMapping: jest.fn(), axisToEntity: { A: [1] }, getQueueLength: () => 0, addCommand: jest.fn() };
  const machines = { getActiveSourceKeys: () => ['hp4'], addCatalogMachine: jest.fn(), getMachines: () => [] };
  const commands = { getRemoteSystem: () => remote, handleUserReset: jest.fn() };
  const recorder = { socket: null };
  captureFlightRecorderSnapshot.mockImplementation(() => ({ frames: [{ position: [clock.time, 0, 0] }], cables: [] }));
  const api = createResearchControls({ world, runtime: { getGameControls: () => ({}) }, machines, commands,
    recorder, inspection: { getState: () => ({}) }, pageId: 'selected-page', scenes: ['hp4'] });
  return { api, clock, remote, machines, resources };
}

test('submitted visual context retains its selected page, entity and numerical time after later motion', async () => {
  const { api, clock } = setup();
  const capture = await api.execute('capture_context', { message: 'Investigate this', selected_entity: 'effector' });
  clock.step = 30; clock.time = .06;
  const history = await api.execute('interventions');
  expect(history[0].context).toMatchObject({ backend: 'browser-js', page_id: 'selected-page', sim_step: 20,
    sim_time_s: .04, scene_generation: 3, selected_entity: 'effector', message: 'Investigate this' });
  expect(history[0].context.observation.frames[0].position[0]).toBe(.04);
  expect(capture.capture_id).toBe(history[0].context.capture_id);
  expect((await api.execute('observe')).sim_step).toBe(30);
});

test('invalid command batches and unknown scenes leave the current controller state intact', async () => {
  const { api, remote, machines } = setup();
  await expect(api.execute('commands', { commands: [{ type: 'Move', A: .01 }, { type: 'Move', Q: 1 }] })).rejects.toThrow('Unknown axis');
  expect(remote.addCommand).not.toHaveBeenCalled();
  await expect(api.execute('load_scene', { scene: 'missing' })).rejects.toThrow('Select a scene');
  expect(machines.addCatalogMachine).not.toHaveBeenCalled();
});

test('optional WebMCP registration exposes the same application API and works without a supervisor URL', async () => {
  const { api } = setup();
  const registered = [];
  const document = { modelContext: { registerTool: (tool) => registered.push(tool) }, getElementById: () => null };
  await attachResearchControls({ api, document, window: {}, url: null });
  expect(registered.map((tool) => tool.name)).toContain('hp_sim5_observe');
  const observe = registered.find((tool) => tool.name === 'hp_sim5_observe');
  expect(observe.annotations.readOnlyHint).toBe(true);
  expect((await observe.execute({})).page_id).toBe('selected-page');
  await attachResearchControls({ api, document: { getElementById: () => null }, window: {}, url: null });
});
