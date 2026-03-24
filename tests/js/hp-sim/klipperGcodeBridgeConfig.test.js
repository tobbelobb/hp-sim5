let parseKlipperGcodeBridgeArgs;
let buildMcuBridgeSpawnSpec;
let buildKlippySpawnSpec;
let buildKlipperMotionRelayOptions;
let extractGpioChipNumbersFromConfigText;
let extractConfiguredGpioChipPaths;
let ensureConfiguredGpioChipAccess;

async function importModule() {
  if (parseKlipperGcodeBridgeArgs) {
    return;
  }
  ({
    parseKlipperGcodeBridgeArgs,
    buildMcuBridgeSpawnSpec,
    buildKlippySpawnSpec,
    buildKlipperMotionRelayOptions,
    extractGpioChipNumbersFromConfigText,
    extractConfiguredGpioChipPaths,
    ensureConfiguredGpioChipAccess,
  } = await import('../../../autocal/control/primitives/klipper_gcode_bridge_config.mjs'));
}

describe('Klipper gcode bridge config', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('uses a per-run raw MCU path by default', () => {
    const args = parseKlipperGcodeBridgeArgs([], { env: {} });

    expect(args.rawPath).toContain(`klipper_host_mcu_raw-${process.pid}`);
  });

  test('extracts configured gpiochip paths from config text', () => {
    const configText = [
      'step_pin: gpiochip1/gpio0',
      'dir_pin: gpiochip1/gpio1',
      'enable_pin: gpiochip2/gpio2',
      'step_pin: gpiochip2/gpio3',
    ].join('\n');

    expect(extractGpioChipNumbersFromConfigText(configText)).toEqual(['1', '2']);
    expect(extractConfiguredGpioChipPaths('ignored.cfg', {
      readFileSync: () => configText,
    })).toEqual(['/dev/gpiochip1', '/dev/gpiochip2']);
  });

  test('prompts for sudo chmod when a gpiochip is inaccessible', async () => {
    const spawnCalls = [];
    await ensureConfiguredGpioChipAccess('ignored.cfg', {
      readFileSync: () => 'step_pin: gpiochip3/gpio0',
      accessSync: () => {
        const err = new Error('permission denied');
        err.code = 'EACCES';
        throw err;
      },
      spawnImpl: (command, args, options) => {
        spawnCalls.push({ command, args, options });
        return {
          on(event, handler) {
            if (event === 'exit') {
              handler(0);
            }
            return this;
          },
        };
      },
    });

    expect(spawnCalls).toEqual([
      {
        command: 'sudo',
        args: ['chmod', '666', '/dev/gpiochip3'],
        options: { stdio: 'inherit' },
      },
    ]);
  });

  test('parses the launcher options and builds the Klippy process specs', () => {
    const args = parseKlipperGcodeBridgeArgs([
      '--socket', '/tmp/custom.sock',
      '--ws-port', '9001',
      '--bridge-ws-port', '9002',
      '--raw-path', '/tmp/raw-mcu',
      '--host-path', '/tmp/host-mcu',
      '--mcu-bin', '/opt/klipper.elf',
      '--dict', '/opt/klipper.dict',
      '--klipper-py', '/opt/klippy',
      '--klippy-python', '/opt/klippy-python',
      '--cmd', 'G1 X100',
      '--quiet',
    ], { env: {} });

    expect(args.socketPath).toBe('/tmp/custom.sock');
    expect(args.wsPort).toBe(9001);
    expect(args.bridgeWsPort).toBe(9002);
    expect(args.rawPath).toBe('/tmp/raw-mcu');
    expect(args.hostPath).toBe('/tmp/host-mcu');
    expect(args.mcuBin).toBe('/opt/klipper.elf');
    expect(args.dictPath).toBe('/opt/klipper.dict');
    expect(args.klipperPyPath).toBe('/opt/klippy');
    expect(args.klippyPython).toBe('/opt/klippy-python');
    expect(args.command).toBe('G1 X100');
    expect(args.quiet).toBe(true);

    const mcuSpec = buildMcuBridgeSpawnSpec(args, { cwd: '/workspace/hp-sim5' });
    expect(mcuSpec.command).toBe('/workspace/hp-sim5/examples/klipper/slideprinter/klipper_linux_mcu_bridge.py');
    expect(mcuSpec.args).toContain('--no-realtime');
    expect(mcuSpec.args).toContain('--ws-port');
    expect(mcuSpec.args).toContain('9002');
    expect(mcuSpec.args).toContain('/opt/klipper.elf');

    const klippySpec = buildKlippySpawnSpec(args, { cwd: '/workspace/hp-sim5' });
    expect(klippySpec.command).toBe('/opt/klippy-python');
    expect(klippySpec.args).toContain('/workspace/hp-sim5/klipper/klippy/klippy.py');
    expect(klippySpec.args).toContain('/tmp/custom.sock');
  });

  test('uses ASAP motion relay settings for the live bridge', () => {
    expect(buildKlipperMotionRelayOptions()).toEqual({
      dt: 1 / 500,
      asapMode: true,
    });
  });
});
