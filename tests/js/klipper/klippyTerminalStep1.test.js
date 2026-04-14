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

describe('klipper terminal step 1', () => {
  let KlippyApiClient;
  let buildKlippyApiLaunchSpec;
  let ensureKlippyApiServer;
  let findKlippyApiProcess;
  let terminateKlippyApiProcess;
  let buildFakeGpioChipSetupCommand;
  let isMissingFakeGpioChipStateMessage;
  let shouldDeferFakeGpioSetupPromptUntilInteractiveReadline;
  let parseArgs;
  let tmpDir;
  let socketPath;

  beforeAll(async () => {
    ({ KlippyApiClient } = await import('../../../integrations/klipper/klippyApiClient.js'));
    ({
      buildKlippyApiLaunchSpec,
      ensureKlippyApiServer,
      findKlippyApiProcess,
      terminateKlippyApiProcess,
    } = await import('../../../integrations/klipper/klippy_api_cli_config.mjs'));
    ({ parseArgs } = await import('../../../integrations/klipper/klipperTerminalArgs.js'));
    ({
      buildFakeGpioChipSetupCommand,
      isMissingFakeGpioChipStateMessage,
      shouldDeferFakeGpioSetupPromptUntilInteractiveReadline,
    } = await import('../../../integrations/klipper/klipperTerminalRecovery.js'));
  });

  beforeEach(() => {
    tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), 'klippy-terminal-test-'));
    socketPath = path.join(tmpDir, 'klippy.sock');
  });

  afterEach(() => {
    if (tmpDir) {
      fs.rmSync(tmpDir, { recursive: true, force: true });
    }
  });

  test('parseArgs accepts step-1 klipper terminal flags', () => {
    const args = parseArgs([
      '--socket', '/tmp/custom.sock',
      '--config', './printer.cfg',
      '--log-path', './klippy.log',
      '--start-script', './scripts/custom.sh',
      '--cmd', 'G90',
      '--ws-port', '9900',
      '--quiet',
      '--debug',
      '--keep-alive',
      '--no-ws',
    ]);

    expect(args.socketPath).toBe('/tmp/custom.sock');
    expect(args.configPath).toBe('./printer.cfg');
    expect(args.logPath).toBe('./klippy.log');
    expect(args.startScript).toBe('./scripts/custom.sh');
    expect(args.command).toBe('G90');
    expect(args.wsPort).toBe(9900);
    expect(args.quiet).toBe(true);
    expect(args.debug).toBe(true);
    expect(args.keepAlive).toBe(true);
    expect(args.noWs).toBe(true);
  });

  test('buildKlippyApiLaunchSpec resolves launcher paths through the repo root', () => {
    const spec = buildKlippyApiLaunchSpec({
      cwd: '/repo',
      startScript: './scripts/run_klippy_api_mode.sh',
      socketPath: './tmp/klippy.sock',
      logPath: './tmp/klippy.log',
      configPath: './examples/printer.cfg',
      python: './.venv/bin/python',
      hostMcuBin: './public/klipper/linux_mcu/klipper.elf',
      hostMcuSerial: './tmp/klipper_host_mcu',
      hostMcuRealtime: '0',
      motionQueueStepGenLowTime: '4.500',
      motionQueueStepGenHighTime: '7.000',
    });

    expect(spec.command).toBe('/bin/bash');
    expect(spec.args).toEqual(['/repo/scripts/run_klippy_api_mode.sh']);
    expect(spec.options.env.KLIPPY_SOCKET_PATH).toBe('/repo/tmp/klippy.sock');
    expect(spec.options.env.KLIPPY_LOG_PATH).toBe('/repo/tmp/klippy.log');
    expect(spec.options.env.KLIPPY_CONFIG_PATH).toBe('/repo/examples/printer.cfg');
    expect(spec.options.env.KLIPPY_PYTHON).toBe('/repo/.venv/bin/python');
    expect(spec.options.env.KLIPPY_HOST_MCU_BIN).toBe('/repo/public/klipper/linux_mcu/klipper.elf');
    expect(spec.options.env.KLIPPY_HOST_MCU_SERIAL).toBe('/repo/tmp/klipper_host_mcu');
    expect(spec.options.env.KLIPPY_HOST_MCU_REALTIME).toBe('0');
    expect(spec.options.env.KLIPPY_MOTION_QUEUE_SG_LOW_TIME).toBe('4.500');
    expect(spec.options.env.KLIPPY_MOTION_QUEUE_SG_HIGH_TIME).toBe('7.000');
  });

  test('ensureKlippyApiServer launches the helper script and waits for the socket', async () => {
    const spawnImpl = jest.fn(() => ({
      pid: 123,
      unref: jest.fn(),
    }));
    const waitForSocket = jest.fn(async () => {});

    const child = await ensureKlippyApiServer({
      cwd: '/repo',
      socketPath: '/tmp/klippy.sock',
      startScript: './scripts/run_klippy_api_mode.sh',
      spawnImpl,
      waitForSocket,
    });

    expect(child.pid).toBe(123);
    expect(spawnImpl).toHaveBeenCalledWith(
      '/bin/bash',
      ['/repo/scripts/run_klippy_api_mode.sh'],
      expect.objectContaining({
        cwd: '/repo',
        detached: true,
        stdio: 'ignore',
      }),
    );
    expect(waitForSocket).toHaveBeenCalledWith('/tmp/klippy.sock');
  });

  test('findKlippyApiProcess matches klippy.py by socket path', async () => {
    const processInfo = await findKlippyApiProcess({
      socketPath: './tmp/klippy.sock',
      cwd: '/repo',
      listProcessesImpl: async () => ([
        {
          pid: 10,
          argv: ['python', '/repo/klipper/klippy/klippy.py', '/repo/printer.cfg'],
        },
        {
          pid: 11,
          argv: ['python', '/repo/klipper/klippy/klippy.py', '/repo/printer.cfg', '-a', '/repo/tmp/klippy.sock'],
        },
      ]),
    });

    expect(processInfo).toMatchObject({
      pid: 11,
    });
  });

  test('terminateKlippyApiProcess kills the matching process and waits for exit', async () => {
    const states = [
      [
        {
          pid: 25,
          argv: ['python', '/repo/klipper/klippy/klippy.py', '/repo/printer.cfg', '-a', '/repo/tmp/klippy.sock'],
        },
      ],
      [],
    ];
    const killImpl = jest.fn();

    await expect(terminateKlippyApiProcess({
      socketPath: './tmp/klippy.sock',
      cwd: '/repo',
      pollMs: 0,
      listProcessesImpl: async () => states.shift() || [],
      killImpl,
    })).resolves.toBe(true);

    expect(killImpl).toHaveBeenCalledWith(25, 'SIGTERM');
  });

  test('fake gpio chip helper command resolves through the repo root', () => {
    expect(buildFakeGpioChipSetupCommand('./public/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg', '/repo')).toEqual({
      command: 'sudo',
      args: [
        '/repo/scripts/make-fake-pin-chip.sh',
        '/repo/public/klipper/hp3/printer-hp3-linux-mcu-with-buildup.cfg',
      ],
    });
  });

  test('GPIO chip shutdown detection only matches the missing fake-chip failure', () => {
    expect(isMissingFakeGpioChipStateMessage(
      "MCU 'mcu' shutdown: GPIO chip device not found",
    )).toBe(true);
    expect(isMissingFakeGpioChipStateMessage(
      'Printer is not ready',
    )).toBe(false);
    expect(isMissingFakeGpioChipStateMessage(null)).toBe(false);
  });

  test('fake gpio chip prompt waits for the shared interactive readline in interactive mode', () => {
    expect(shouldDeferFakeGpioSetupPromptUntilInteractiveReadline({
      stdinIsTTY: true,
      hasInteractiveReadline: false,
      hasCommand: false,
    })).toBe(true);

    expect(shouldDeferFakeGpioSetupPromptUntilInteractiveReadline({
      stdinIsTTY: true,
      hasInteractiveReadline: true,
      hasCommand: false,
    })).toBe(false);

    expect(shouldDeferFakeGpioSetupPromptUntilInteractiveReadline({
      stdinIsTTY: false,
      hasInteractiveReadline: false,
      hasCommand: false,
    })).toBe(false);

    expect(shouldDeferFakeGpioSetupPromptUntilInteractiveReadline({
      stdinIsTTY: true,
      hasInteractiveReadline: false,
      hasCommand: true,
    })).toBe(false);
  });

  test('KlippyApiClient handles requests, async subscriptions, and reconnect resubscribe', async () => {
    const seenMethods = [];
    const asyncResponses = [];

    let activeHarness = await createServerHarness(socketPath, (message, socket) => {
      seenMethods.push(message.method);
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
          result: { objects: ['webhooks', 'toolhead'] },
        });
        return;
      }
      if (message.method === 'gcode/subscribe_output') {
        writeFrame(socket, {
          id: message.id,
          result: {},
        });
        writeFrame(socket, {
          q: message.params.response_template.q,
          params: { response: `hello:${message.params.response_template.q}` },
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

    try {
      await client.start();

      await expect(client.request('info', {
        client_info: { program: 'test' },
      })).resolves.toMatchObject({ state: 'ready' });

      await expect(client.request('objects/list', {})).resolves.toMatchObject({
        objects: ['webhooks', 'toolhead'],
      });

      await client.subscribe(
        'gcode/subscribe_output',
        {},
        (params) => asyncResponses.push(params.response),
        { id: 'sub:gcode' },
      );

      expect(asyncResponses).toContain('hello:sub:gcode');

      await activeHarness.close();

      activeHarness = await createServerHarness(socketPath, (message, socket) => {
        seenMethods.push(`re:${message.method}`);
        if (message.method === 'gcode/subscribe_output') {
          writeFrame(socket, {
            id: message.id,
            result: {},
          });
          writeFrame(socket, {
            q: message.params.response_template.q,
            params: { response: 'hello:reconnect' },
          });
          return;
        }
        if (message.method === 'objects/list') {
          writeFrame(socket, {
            id: message.id,
            result: { objects: ['webhooks', 'toolhead', 'motion_report'] },
          });
          return;
        }
        if (message.method === 'info') {
          writeFrame(socket, {
            id: message.id,
            result: { state: 'ready' },
          });
        }
      });

      await expect(client.request('objects/list', {})).resolves.toMatchObject({
        objects: ['webhooks', 'toolhead', 'motion_report'],
      });

      expect(asyncResponses).toContain('hello:reconnect');
      expect(seenMethods).toContain('gcode/subscribe_output');
      expect(seenMethods).toContain('re:gcode/subscribe_output');
    } finally {
      client.close();
      if (activeHarness) {
        await activeHarness.close();
      }
    }
  });
});
