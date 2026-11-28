import { resolvePresetStreams } from '../../../hp-sim-3d/app/commandPresetResolver.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';
import { createWorkerController } from '../../../hp-sim-3d/app/workerController.js';
import { createCommandJobController } from '../../../hp-sim-3d/app/commandJobController.js';
import { SpoolStateComponent, SpoolTagComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { MachineTagComponent, World } from '../../../src/js/cable_joints_3d/ecs.js';

jest.mock('../../../integrations/klipper/klipperSimulatorBridge.js', () => ({ createKlipperRawBridge: jest.fn() }));
jest.mock('../../../integrations/klipper/klipperMcuCommandPlayer.js?worker&url', () => '', { virtual: true });
jest.mock('../../../integrations/rrf/rrfCanPlayer.js?worker&url', () => '', { virtual: true });
jest.mock('../../../hp-sim-3d/app/workers/moveCommander.js?worker&url', () => '', { virtual: true });
jest.mock('../../../hp-sim-3d/app/workers/simulationClock.js?worker&url', () => '', { virtual: true });
jest.mock('../../../hp-sim-3d/app/machineCatalog.js', () => ({ buildRepositoryFileUrl: (path) => path }));

test('machines share a preset stream only when they play the same file and config', () => {
  const streams = resolvePresetStreams('straightMoves', false, [
    { id: 'hp4', sourceKey: 'hp4.usda' },
    { id: 'hp3', sourceKey: 'hp3.usda' },
    { id: 'worn', sourceKey: 'hp4_worn_bearings.usda' },
    { id: 'slide', sourceKey: 'slideprinter.usda' },
    { id: 'uploaded', sourceKey: null },
  ]);
  expect(streams.map(({ path, machineIds }) => [path.split('/').at(-1), machineIds])).toEqual([
    ['draw_squares_bigger_hp4_no_buildup.can', ['hp4', 'worn']],
    ['draw_squares_bigger_hp3_no_buildup.can', ['hp3']],
    ['draw_squares_bigger_slideprinter_no_buildup.can', ['slide', 'uploaded']],
  ]);
  expect(streams[1].rrfConfig).toBe('RRF/run/vsd/sys/config_hp3.g');
});

function setupTwoStreams() {
  const world = new World();
  const steppers = {};
  for (const machineId of ['m1', 'm2']) {
    const spool = world.createEntity();
    world.addComponent(spool, new SpoolTagComponent());
    world.addComponent(spool, new SpoolStateComponent('A'));
    world.addComponent(spool, new MachineTagComponent(machineId));
    steppers[machineId] = new StepperMotorComponent();
    world.addComponent(spool, steppers[machineId]);
  }
  const remote = new RemoteSpoolSystem();
  world.registerSystem(remote);
  const onWorkerDrained = jest.fn();
  jest.spyOn(console, 'log').mockImplementation(() => {});
  let commands;
  const workers = createWorkerController({
    state: { currentTimeScale: 1 },
    getRemoteSystem: () => remote,
    onWorkerDrained,
    onWorkerError: () => commands.handleWorkerError(),
    WorkerCtor: class {
      postMessage() {}
      terminate() { this.terminated = true; }
      receive(data) { this.onmessage({ data }); }
    },
  });
  const state = { jobSequenceCounter: 0, printActive: true };
  commands = createCommandJobController({ world, state, dom: {}, machines: {}, workers, runtime: { resume() {} } });
  const [w1, w2] = workers.ensureRrfWorkers(2);
  workers.resetRemoteQueue([{ worker: w1, machineIds: ['m1'] }, { worker: w2, machineIds: ['m2'] }]);
  const step = () => { remote.update(world); remote.afterPhysicsStep(); };
  const angles = () => [steppers.m1.commandedAngle, steppers.m2.commandedAngle];
  return { remote, commands, state, onWorkerDrained, w1, w2, step, angles };
}

test('each stream drives only its own machines and the job ends when all have drained', () => {
  const { onWorkerDrained, w1, w2, step, angles } = setupTwoStreams();
  w1.receive({ action: 'gcode', command: { type: 'Move', A: 1 } });
  w2.receive({ action: 'gcode', command: { type: 'Move', A: 2 } });
  step();
  expect(angles()).toEqual([1, 2]);

  w1.receive({ type: 'done' });
  step();
  expect(onWorkerDrained).not.toHaveBeenCalled();
  w2.receive({ type: 'done' });
  step();
  expect(onWorkerDrained).toHaveBeenCalledTimes(1);
});

test('streams start on the same step however late their first command arrives', () => {
  const { w1, w2, step, angles } = setupTwoStreams();
  w1.receive({ action: 'gcode', command: { type: 'Move', A: 1 } });
  w1.receive({ action: 'gcode', command: { type: 'Move', A: 3 } });
  step();
  step();
  expect(angles()).toEqual([0, 0]);
  w2.receive({ action: 'gcode', command: { type: 'Move', A: 2 } });
  step();
  expect(angles()).toEqual([1, 2]);
});

test('a failed worker fails the whole job and restores one stream for all machines', () => {
  jest.spyOn(console, 'error').mockImplementation(() => {});
  const { remote, state, onWorkerDrained, w1, w2, step } = setupTwoStreams();
  w1.receive({ action: 'gcode', command: { type: 'Move', A: 1 } });
  w2.receive({ action: 'gcode', command: { type: 'Move', A: 2 } });
  w1.receive({ type: 'error', message: 'bad file' });
  expect(state.printActive).toBe(false);
  expect(w2.terminated).toBe(true);
  expect(remote.streams).toHaveLength(1);
  expect(remote.machineIds).toBeNull();
  expect(remote.getQueueLength()).toBe(0);
  w2.receive({ type: 'done' });
  step();
  expect(onWorkerDrained).not.toHaveBeenCalled();
});

test('external commands are rejected while machines play separate streams', () => {
  jest.spyOn(console, 'warn').mockImplementation(() => {});
  const { remote, commands } = setupTwoStreams();
  expect(commands.pushExternalCommands([{ type: 'Move', A: 1 }])).toBe(false);
  expect(commands.ensureExternalKlipperApiBridge()).toBeNull();
  remote.setStreams();
  expect(commands.pushExternalCommands([{ type: 'Move', A: 1 }])).toBe(true);
});
