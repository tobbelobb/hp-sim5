import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import {
  EXTRUDER_MM_PER_STEP_KLIPPER,
  STEP_ANGLE_RAD,
} from '../../../integrations/klipper/klipperFirmwareModel.js';

const encoder = new TextEncoder();
const stepper = (oid, pin) => `config_stepper oid=${oid} step_pin=gpiochip1/gpio${pin}`;

async function startUpload(name, stream) {
  let onMessage;
  const commands = [];
  globalThis.self = { addEventListener: (_, listener) => { onMessage = listener; } };
  const done = new Promise((resolveDone, reject) => {
    globalThis.postMessage = (message) => {
      if (message.action === 'gcode') commands.push(message.command);
      if (message.type === 'done') resolveDone();
      if (message.type === 'error') reject(new Error(message.message));
    };
  });
  jest.resetModules();
  await import('../../../integrations/klipper/klipperMcuCommandPlayer.js');
  const send = (data) => onMessage({ data });
  await send({ type: 'set_asap_mode', enable: true });
  await send({ type: 'filename_upload', filename: { name, stream: () => stream } });
  return { commands, done, send };
}

const originalSelf = globalThis.self;
const originalPostMessage = globalThis.postMessage;

afterEach(() => {
  globalThis.self = originalSelf;
  globalThis.postMessage = originalPostMessage;
});

test('flushes both first step batches together while the upload is still open', async () => {
  let source;
  const stream = new ReadableStream({ start(controller) { source = controller; } });
  const { commands, done, send } = await startUpload('motion.txt', stream);
  const write = (...lines) => source.enqueue(encoder.encode(lines.join('\n') + '\n'));

  write(stepper(0, 0), stepper(1, 3), 'queue_step oid=0 interval=1000000 count=1 add=0');
  await new Promise(setImmediate);
  expect(commands).toHaveLength(0);

  await send({ type: 'pause' });
  write('queue_step oid=1 interval=1000000 count=1 add=0');
  await new Promise(setImmediate);
  expect(commands).toHaveLength(0);
  await send({ type: 'resume' });
  await new Promise(setImmediate);
  expect(commands).toHaveLength(10);
  for (const [i, move] of commands.entries()) {
    expect(move.A / STEP_ANGLE_RAD).toBeCloseTo((i + 1) / 10, 8);
    expect(move.B / STEP_ANGLE_RAD).toBeCloseTo((i + 1) / 10, 8);
  }

  source.close();
  await done;
  expect(commands).toHaveLength(10);
});

test.each([false, true])('preserves quiet-axis and extrusion timing (stationary stepper: %s)', async (stationary) => {
  const lines = [
    stepper(0, 0), stepper(1, 3), stepper(3, 24),
    ...(stationary ? [stepper(2, 6)] : []),
    'queue_step oid=0 interval=100000 count=8 add=0',
    'queue_step oid=1 interval=300000 count=1 add=0',
    'set_next_step_dir oid=1 dir=0',
    'queue_step oid=1 interval=600000 count=1 add=0',
    'queue_step oid=3 interval=450000 count=2 add=50000',
    'set_next_step_dir oid=3 dir=0',
    'queue_step oid=3 interval=160000 count=1 add=0',
  ];
  const stream = new Blob([lines.join('\n')]).stream();
  const { commands, done } = await startUpload('motion.txt', stream);
  await done;

  if (stationary) expect(commands.every((move) => move.C === 0)).toBe(true);
  expect(commands[2].B / STEP_ANGLE_RAD).toBeCloseTo(1, 8);
  expect(commands[3].B / STEP_ANGLE_RAD).toBeCloseTo(5 / 6, 8);
  expect(commands.at(-1).B / STEP_ANGLE_RAD).toBeCloseTo(0, 8);
  expect(commands.at(-1).A / STEP_ANGLE_RAD).toBeCloseTo(8, 8);
  // A moves in each of the first eight buckets, then B ends in bucket 8.
  // E fires in buckets 4, 9 and 11. The empty bucket 10 emits no Move.
  expect(commands).toHaveLength(11);
  expect(commands.map((move) => (move.E || 0) / EXTRUDER_MM_PER_STEP_KLIPPER))
    .toEqual([0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -1]);
});

test.each(['txt', 'serial'])('accounts for all signed steps in draw_squares.%s', async (extension) => {
  const name = `draw_squares.${extension}`;
  const bytes = readFileSync(resolve(__dirname, '../../../public/mcu_commands/slideprinter', name));
  const { commands, done } = await startUpload(name, new Blob([bytes]).stream());
  await done;

  const final = commands.at(-1);
  for (const axis of ['A', 'B', 'C']) {
    expect(final[axis] / STEP_ANGLE_RAD).toBeCloseTo(-5, 8);
  }
  const extrusion = commands.reduce((sum, move) => sum + (move.E || 0), 0);
  expect(extrusion / EXTRUDER_MM_PER_STEP_KLIPPER).toBeCloseTo(38209, 6);
});
