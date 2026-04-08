const fs = require('node:fs');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');

const FRAME_DELIMITER = '\x03';

function writeFrame(socket, payload) {
  socket.write(`${JSON.stringify(payload)}${FRAME_DELIMITER}`);
}

function createServerHarness(socketPath, onMessage) {
  const sockets = new Set();
  const server = net.createServer((socket) => {
    sockets.add(socket);
    let buffer = '';
    socket.on('data', (chunk) => {
      buffer += chunk.toString();
      while (true) {
        const frameIndex = buffer.indexOf(FRAME_DELIMITER);
        if (frameIndex === -1) {
          return;
        }
        const raw = buffer.slice(0, frameIndex);
        buffer = buffer.slice(frameIndex + 1);
        if (!raw) {
          continue;
        }
        onMessage(JSON.parse(raw), socket);
      }
    });
    socket.on('close', () => {
      sockets.delete(socket);
    });
  });

  return new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(socketPath, () => {
      server.removeListener('error', reject);
      resolve({
        server,
        sockets,
        close: async () => {
          for (const socket of sockets) {
            socket.destroy();
          }
          await new Promise((closeResolve) => server.close(closeResolve));
        },
      });
    });
  });
}

async function waitFor(check, timeoutMs = 2_000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (check()) {
      return;
    }
    // eslint-disable-next-line no-await-in-loop
    await new Promise((resolve) => setTimeout(resolve, 20));
  }
  throw new Error('Timed out waiting for condition');
}

describe('Klippy runtime state', () => {
  let KlippyApiClient;
  let KlippyRuntimeState;
  let buildRuntimeObjectSubscription;
  let tmpDir;
  let socketPath;

  beforeAll(async () => {
    ({ KlippyApiClient } = await import('../../../integrations/klipper/klippyApiClient.js'));
    ({
      KlippyRuntimeState,
      buildRuntimeObjectSubscription,
    } = await import('../../../integrations/klipper/klippyRuntimeState.js'));
  });

  beforeEach(() => {
    tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), 'klippy-runtime-state-'));
    socketPath = path.join(tmpDir, 'klippy.sock');
  });

  afterEach(() => {
    if (tmpDir) {
      fs.rmSync(tmpDir, { recursive: true, force: true });
    }
  });

  test('buildRuntimeObjectSubscription adds optional objects only when available', () => {
    expect(buildRuntimeObjectSubscription([
      'webhooks',
      'toolhead',
      'gcode_move',
      'motion_report',
    ])).toEqual({
      webhooks: null,
      toolhead: null,
      gcode_move: null,
      motion_report: null,
    });

    expect(buildRuntimeObjectSubscription([
      'webhooks',
      'toolhead',
      'gcode_move',
      'motion_report',
      'print_stats',
      'virtual_sdcard',
    ])).toEqual({
      webhooks: null,
      toolhead: null,
      gcode_move: null,
      motion_report: null,
      print_stats: null,
      virtual_sdcard: null,
    });
  });

  test('KlippyRuntimeState caches subscriptions and rediscovers motion sources on reconnect', async () => {
    const outputs = [];
    const motionChanges = [];

    let phase = 'initial';
    const handleMessage = (message, socket) => {
      if (message.method === 'info') {
        writeFrame(socket, {
          id: message.id,
          result: { state: 'ready', state_message: 'Printer is ready' },
        });
        return;
      }
      if (message.method === 'objects/list') {
        writeFrame(socket, {
          id: message.id,
          result: {
            objects: [
              'webhooks',
              'toolhead',
              'gcode_move',
              'motion_report',
              'print_stats',
              'virtual_sdcard',
            ],
          },
        });
        return;
      }
      if (message.method === 'objects/subscribe') {
        const initialStatus = phase === 'initial'
          ? {
            webhooks: { state: 'ready', state_message: 'Printer is ready' },
            toolhead: { position: [1, 2, 3, 0] },
            gcode_move: { gcode_position: [1, 2, 3, 0] },
            motion_report: {
              steppers: ['stepper_y', 'stepper_x'],
              trapq: ['extruder', 'toolhead'],
            },
            print_stats: { state: 'standby', info: { current_layer: 0 } },
            virtual_sdcard: { progress: 0 },
          }
          : {
            webhooks: { state: 'ready', state_message: 'Printer is ready' },
            toolhead: { position: [4, 5, 6, 0] },
            gcode_move: { gcode_position: [4, 5, 6, 0] },
            motion_report: {
              steppers: ['extruder_stepper', 'stepper_beta'],
              trapq: ['toolhead'],
            },
            print_stats: { state: 'printing', info: { current_layer: 2 } },
            virtual_sdcard: { progress: 0.5 },
          };

        writeFrame(socket, {
          id: message.id,
          result: {
            eventtime: phase === 'initial' ? 1.25 : 9.5,
            status: initialStatus,
          },
        });

        if (phase === 'initial') {
          setImmediate(() => {
            writeFrame(socket, {
              q: message.params.response_template.q,
              params: {
                eventtime: 2.5,
                status: {
                  webhooks: { state: 'shutdown', state_message: 'Restart required' },
                  motion_report: {
                    steppers: ['stepper_a'],
                    trapq: ['toolhead'],
                  },
                },
              },
            });
          });
        }
        return;
      }
      if (message.method === 'gcode/subscribe_output') {
        writeFrame(socket, {
          id: message.id,
          result: {},
        });
        setImmediate(() => {
          writeFrame(socket, {
            q: message.params.response_template.q,
            params: {
              response: phase === 'initial' ? '// first output' : '// reconnect output',
            },
          });
        });
      }
    };

    let harness = await createServerHarness(socketPath, handleMessage);
    const client = new KlippyApiClient({
      socketPath,
      reconnectDelayMs: 20,
      maxReconnectDelayMs: 40,
      requestTimeoutMs: 1_000,
      connectTimeoutMs: 500,
    });
    const runtime = new KlippyRuntimeState({
      client,
      clientInfo: { program: 'test-runtime' },
    });

    runtime.on('gcode-output', ({ response }) => {
      outputs.push(response);
    });
    runtime.on('motion-sources-changed', ({ steppers, trapq }) => {
      motionChanges.push({ steppers, trapq });
    });

    try {
      await client.start();
      const primed = await runtime.prime();

      expect(primed.info).toMatchObject({ state: 'ready' });
      expect(runtime.getSnapshot().availableObjects).toEqual([
        'gcode_move',
        'motion_report',
        'print_stats',
        'toolhead',
        'virtual_sdcard',
        'webhooks',
      ]);
      expect(runtime.getSnapshot().subscribedObjects).toEqual([
        'gcode_move',
        'motion_report',
        'print_stats',
        'toolhead',
        'virtual_sdcard',
        'webhooks',
      ]);

      await waitFor(() => outputs.includes('// first output'));
      await waitFor(() => runtime.getPrinterState() === 'shutdown');

      const shutdownSnapshot = runtime.getSnapshot();
      expect(shutdownSnapshot.printerStateMessage).toBe('Restart required');
      expect(shutdownSnapshot.status.print_stats).toEqual({
        state: 'standby',
        info: { current_layer: 0 },
      });
      expect(shutdownSnapshot.status.virtual_sdcard).toEqual({ progress: 0 });
      expect(shutdownSnapshot.motionSources).toEqual({
        steppers: ['stepper_a'],
        trapq: ['toolhead'],
      });

      phase = 'reconnect';
      await harness.close();
      harness = await createServerHarness(socketPath, handleMessage);

      await expect(client.request('info', {})).resolves.toMatchObject({ state: 'ready' });
      await waitFor(() => outputs.includes('// reconnect output'));
      await waitFor(() => runtime.getSnapshot().motionSources.steppers.join(',') === 'extruder_stepper,stepper_beta');

      const reconnectSnapshot = runtime.getSnapshot();
      expect(reconnectSnapshot.printerState).toBe('ready');
      expect(reconnectSnapshot.status.print_stats).toEqual({
        state: 'printing',
        info: { current_layer: 2 },
      });
      expect(reconnectSnapshot.status.virtual_sdcard).toEqual({ progress: 0.5 });
      expect(reconnectSnapshot.motionSources).toEqual({
        steppers: ['extruder_stepper', 'stepper_beta'],
        trapq: ['toolhead'],
      });
      expect(motionChanges).toEqual(expect.arrayContaining([
        {
          steppers: ['stepper_a'],
          trapq: ['toolhead'],
        },
        {
          steppers: ['extruder_stepper', 'stepper_beta'],
          trapq: ['toolhead'],
        },
      ]));
    } finally {
      client.close();
      if (harness) {
        await harness.close();
      }
    }
  });

  test('KlippyRuntimeState refreshes motion_report once startup transitions to ready without a motion update', async () => {
    let objectsQueryCount = 0;

    const harness = await createServerHarness(socketPath, (message, socket) => {
      if (message.method === 'info') {
        writeFrame(socket, {
          id: message.id,
          result: { state: 'startup', state_message: 'Booting' },
        });
        return;
      }
      if (message.method === 'objects/list') {
        writeFrame(socket, {
          id: message.id,
          result: {
            objects: ['webhooks', 'toolhead', 'gcode_move', 'motion_report'],
          },
        });
        return;
      }
      if (message.method === 'objects/subscribe') {
        writeFrame(socket, {
          id: message.id,
          result: {
            eventtime: 1.0,
            status: {
              webhooks: { state: 'startup', state_message: 'Booting' },
              toolhead: { position: [0, 0, 0, 0] },
              gcode_move: { gcode_position: [0, 0, 0, 0] },
              motion_report: {
                live_position: [0, 0, 0, 0],
                live_velocity: 0,
                live_extruder_velocity: 0,
                steppers: [],
                trapq: [],
              },
            },
          },
        });
        setImmediate(() => {
          writeFrame(socket, {
            q: message.params.response_template.q,
            params: {
              eventtime: 1.5,
              status: {
                webhooks: { state: 'ready', state_message: 'Printer is ready' },
              },
            },
          });
        });
        return;
      }
      if (message.method === 'gcode/subscribe_output') {
        writeFrame(socket, {
          id: message.id,
          result: {},
        });
        return;
      }
      if (message.method === 'objects/query') {
        objectsQueryCount += 1;
        writeFrame(socket, {
          id: message.id,
          result: {
            eventtime: 1.75,
            status: {
              motion_report: {
                live_position: [0, 0, 0, 0],
                live_velocity: 0,
                live_extruder_velocity: 0,
                steppers: ['stepper_a', 'stepper_b'],
                trapq: ['toolhead'],
              },
              webhooks: { state: 'ready', state_message: 'Printer is ready' },
            },
          },
        });
      }
    });

    const client = new KlippyApiClient({
      socketPath,
      reconnectDelayMs: 20,
      maxReconnectDelayMs: 40,
      requestTimeoutMs: 1_000,
      connectTimeoutMs: 500,
    });
    const runtime = new KlippyRuntimeState({
      client,
      clientInfo: { program: 'test-runtime' },
    });

    try {
      await client.start();
      await runtime.prime();

      await waitFor(() => objectsQueryCount === 1);
      await waitFor(() => runtime.getSnapshot().motionSources.steppers.join(',') === 'stepper_a,stepper_b');

      expect(runtime.getSnapshot().motionSources).toEqual({
        steppers: ['stepper_a', 'stepper_b'],
        trapq: ['toolhead'],
      });
    } finally {
      client.close();
      await harness.close();
    }
  });
});
