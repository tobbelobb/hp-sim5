let parseKlipperGcodeBridgeArgs;
let buildMcuBridgeSpawnSpec;
let buildKlippySpawnSpec;

async function importModule() {
  if (parseKlipperGcodeBridgeArgs) {
    return;
  }
  ({
    parseKlipperGcodeBridgeArgs,
    buildMcuBridgeSpawnSpec,
    buildKlippySpawnSpec,
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
});
