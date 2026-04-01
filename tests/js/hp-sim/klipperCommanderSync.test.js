let KlipperCommander;
let RemoteSpoolSystem;
let SpoolTagComponent;
let SpoolStateComponent;
let StepperMotorComponent;
let MachineTagComponent;

const SIMULATION_PLAYBACK_RESOURCE = 'simulationPlayback';
const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);

class FakeLineStream {
  constructor(lines = []) {
    this._chunks = lines.map((line) => `${line}\n`);
  }

  pipeThrough() {
    const chunks = this._chunks.slice();
    return {
      getReader() {
        let index = 0;
        return {
          read() {
            if (index >= chunks.length) {
              return Promise.resolve({ value: undefined, done: true });
            }
            const value = chunks[index];
            index += 1;
            return Promise.resolve({ value, done: false });
          },
        };
      },
    };
  }
}

class FakeWorld {
  constructor() {
    this.entities = new Map();
    this.componentStores = new Map();
    this.resources = new Map();
    this.systems = [];
  }

  addEntity(id, components = []) {
    const componentMap = new Map();
    this.entities.set(id, componentMap);
    for (const [ComponentClass, instance] of components) {
      componentMap.set(ComponentClass, instance);
      let store = this.componentStores.get(ComponentClass);
      if (!store) {
        store = new Map();
        this.componentStores.set(ComponentClass, store);
      }
      store.set(id, instance);
    }
  }

  query(componentClasses = []) {
    const result = [];
    for (const [entityId, componentMap] of this.entities.entries()) {
      let matches = true;
      for (const ComponentClass of componentClasses) {
        if (!componentMap.has(ComponentClass)) {
          matches = false;
          break;
        }
      }
      if (matches) {
        result.push(entityId);
      }
    }
    return result;
  }

  getComponent(entityId, ComponentClass) {
    const componentMap = this.entities.get(entityId);
    if (!componentMap) {
      return null;
    }
    return componentMap.get(ComponentClass) || null;
  }

  getComponentStore(ComponentClass) {
    return this.componentStores.get(ComponentClass) || null;
  }

  getResource(name) {
    return this.resources.get(name);
  }

  setResource(name, value) {
    this.resources.set(name, value);
  }
}

async function importModules() {
  if (KlipperCommander) {
    return;
  }

  if (!globalThis.self) {
    globalThis.self = {};
  }
  if (typeof globalThis.self.addEventListener !== 'function') {
    globalThis.self.addEventListener = () => {};
  }
  if (typeof globalThis.postMessage !== 'function') {
    globalThis.postMessage = () => {};
  }
  if (typeof globalThis.TextDecoderStream === 'undefined') {
    globalThis.TextDecoderStream = class {};
  }

  ({ KlipperCommander } = await import('../../../integrations/klipper/klipperMcuCommandPlayer.js'));
  ({
    RemoteSpoolSystem,
    SpoolTagComponent,
    SpoolStateComponent,
    StepperMotorComponent,
  } = await import('../../../examples/js/slideprinter/slideprinter_common.js'));
  ({ MachineTagComponent } = await import('../../../src/js/cable_joints/ecs.js'));
}

async function withImmediateTimeout(fn) {
  const originalSetTimeout = globalThis.setTimeout;
  globalThis.setTimeout = (callback) => {
    callback();
    return 0;
  };
  try {
    return await fn();
  } finally {
    globalThis.setTimeout = originalSetTimeout;
  }
}

async function collectMoveSequence({ speedScale = 1, asapMode = false, lines = null } = {}) {
  await importModules();

  const commander = new KlipperCommander();
  const emittedMoves = [];
  const originalPostMessage = globalThis.postMessage;
  globalThis.postMessage = () => {};

  let now = 0;
  const nowSpy = jest.spyOn(performance, 'now').mockImplementation(() => {
    now += 0.5;
    return now;
  });

  try {
    const runPlayback = async () => {
      commander.sendCommand = async (command) => {
        if (command?.type === 'Move') {
          emittedMoves.push({ ...command });
        }
      };

      commander.setDt(1 / 500);
      commander.setSpeedScale(speedScale);
      commander.setAsapMode(asapMode);

      const commandLines = lines || [
        'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
        'set_next_step_dir oid=0 dir=1',
        'queue_step oid=0 interval=60000 count=2 add=0',
        'queue_step oid=0 interval=50000 count=2 add=0',
        'queue_step oid=0 interval=70000 count=2 add=0',
      ];
      const stream = new FakeLineStream(commandLines);
      await commander.run(stream);
    };

    await withImmediateTimeout(runPlayback);
  } finally {
    nowSpy.mockRestore();
    globalThis.postMessage = originalPostMessage;
  }

  return emittedMoves;
}

describe('KlipperCommander and RemoteSpoolSystem synchronisation', () => {
  beforeAll(async () => {
    await importModules();
  });

  test('KlipperCommander emits identical Move sequences across time scaling and ASAP modes', async () => {
    const baseline = await collectMoveSequence({ speedScale: 1, asapMode: false });
    const doubleSpeed = await collectMoveSequence({ speedScale: 2, asapMode: false });
    const halfSpeed = await collectMoveSequence({ speedScale: 0.5, asapMode: false });
    const asapMode = await collectMoveSequence({ speedScale: 1, asapMode: true });

    expect(doubleSpeed).toEqual(baseline);
    expect(halfSpeed).toEqual(baseline);
    expect(asapMode).toEqual(baseline);
  });

  test('KlipperCommander spreads queue_step timing across the 500 Hz bucket window', async () => {
    const moves = await collectMoveSequence({
      speedScale: 1,
      asapMode: true,
      lines: [
        'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
        'set_next_step_dir oid=0 dir=1',
        'queue_step oid=0 interval=60000 count=2 add=0',
      ],
    });

    expect(moves).toHaveLength(2);
    expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(5 / 3, 6);
    expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('KlipperCommander keeps extrusion anchored to the first real E step', async () => {
    const moves = await collectMoveSequence({
      speedScale: 1,
      asapMode: true,
      lines: [
        'config_stepper oid=0 step_pin=unknown0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
        'config_stepper oid=1 step_pin=unknown1 dir_pin=gpiochip1/gpio2 invert_step=0 step_pulse_ticks=100',
        'config_stepper oid=2 step_pin=unknown2 dir_pin=gpiochip1/gpio3 invert_step=0 step_pulse_ticks=100',
        'config_stepper oid=3 step_pin=unknown3 dir_pin=gpiochip1/gpio4 invert_step=0 step_pulse_ticks=100',
        'set_next_step_dir oid=0 dir=1',
        'set_next_step_dir oid=3 dir=1',
        'queue_step oid=0 interval=60000 count=2 add=0',
        'queue_step oid=3 interval=250000 count=1 add=0',
      ],
    });

    expect(moves).toHaveLength(3);
    expect(moves[0].E).toBeUndefined();
    expect(moves[1].E).toBeUndefined();
    expect(moves[2].E).toBeGreaterThan(0);
  });

  test('RemoteSpoolSystem consumes Move commands in order and waits when the queue is empty', async () => {
    const expectedMoves = await collectMoveSequence({ speedScale: 1, asapMode: false });

    const remoteSystem = new RemoteSpoolSystem();
    remoteSystem.worker = { postMessage: jest.fn() };

    const world = new FakeWorld();
    world.setResource('timeScale', 1);
    world.setResource(SIMULATION_PLAYBACK_RESOURCE, { mode: 'linear' });

    const spoolEntityId = 1;
    const spoolTag = new SpoolTagComponent();
    const spoolState = new SpoolStateComponent('A');
    const machineTag = new MachineTagComponent('machine-1');
    const stepper = new StepperMotorComponent();

    world.addEntity(spoolEntityId, [
      [SpoolTagComponent, spoolTag],
      [SpoolStateComponent, spoolState],
      [MachineTagComponent, machineTag],
      [StepperMotorComponent, stepper],
    ]);

    const executedMoves = [];
    remoteSystem.setCommandExecutedListener((command) => {
      if (command?.type === 'Move') {
        executedMoves.push({ ...command });
      }
    });

    const dt = 1 / 500;
    for (let i = 0; i < expectedMoves.length; i += 1) {
      const move = expectedMoves[i];

      remoteSystem.addCommand({ ...move });
      remoteSystem.update(world, dt);

      expect(stepper.commandedAngle).toBeCloseTo(move.A ?? 0, 12);
      expect(executedMoves.length).toBe(i + 1);
      expect(executedMoves[i]).toEqual(move);

      world.setResource('timeScale', i % 2 === 0 ? 2 : 0.5);
      remoteSystem.update(world, dt);
      expect(executedMoves.length).toBe(i + 1);
      expect(stepper.commandedAngle).toBeCloseTo(move.A ?? 0, 12);
      world.setResource('timeScale', 1);
    }

    remoteSystem.update(world, dt);
    expect(executedMoves).toEqual(expectedMoves);
  });
});
