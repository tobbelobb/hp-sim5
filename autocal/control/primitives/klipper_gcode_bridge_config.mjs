import os from 'node:os';
import path from 'node:path';

const DEFAULT_WS_PORT = 8790;
const DEFAULT_INTERNAL_WS_PORT = 8770;
const DEFAULT_SOCKET_PATH = '/tmp/klippy_uds';
const DEFAULT_HOST_PATH = '/tmp/klipper_host_mcu';
const DEFAULT_RAW_PATH = path.join(os.tmpdir(), `klipper_host_mcu_raw-${process.pid}`);
const DEFAULT_CONFIG = 'examples/klipper/slideprinter/printer-hp3-linux-mcu-with-buildup.cfg';
const DEFAULT_MCU_BIN = 'examples/klipper/linux_mcu/klipper.elf';
const DEFAULT_DICT = 'examples/klipper/linux_mcu/klipper.dict';
const DEFAULT_KLIPPER_PY = './klipper/klippy/';
const DEFAULT_KLIPPY_PYTHON = path.join(os.homedir(), 'klippy-env/bin/python');

function resolvePath(workdirPath, cwd = process.cwd()) {
  return path.isAbsolute(workdirPath) ? workdirPath : path.resolve(cwd, workdirPath);
}

export function parseKlipperGcodeBridgeArgs(argv, { cwd = process.cwd(), env = process.env } = {}) {
  const out = {
    config: DEFAULT_CONFIG,
    socketPath: DEFAULT_SOCKET_PATH,
    wsPort: DEFAULT_WS_PORT,
    bridgeWsPort: DEFAULT_INTERNAL_WS_PORT,
    rawPath: DEFAULT_RAW_PATH,
    hostPath: DEFAULT_HOST_PATH,
    mcuBin: DEFAULT_MCU_BIN,
    dictPath: DEFAULT_DICT,
    klipperPyPath: DEFAULT_KLIPPER_PY,
    klippyPython: env.KLIPPY_PYTHON || DEFAULT_KLIPPY_PYTHON,
    command: null,
    quiet: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--help' || arg === '-h') {
      out.help = true;
      continue;
    }
    if (arg === '--quiet' || arg === '-q') {
      out.quiet = true;
      continue;
    }
    const readValue = () => {
      const next = argv[i + 1];
      i += 1;
      return next;
    };
    if (arg === '--config') {
      out.config = readValue() || out.config;
      continue;
    }
    if (arg.startsWith('--config=')) {
      out.config = arg.slice('--config='.length) || out.config;
      continue;
    }
    if (arg === '--socket') {
      out.socketPath = readValue() || out.socketPath;
      continue;
    }
    if (arg.startsWith('--socket=')) {
      out.socketPath = arg.slice('--socket='.length) || out.socketPath;
      continue;
    }
    if (arg === '--ws-port') {
      out.wsPort = Number(readValue()) || out.wsPort;
      continue;
    }
    if (arg.startsWith('--ws-port=')) {
      out.wsPort = Number(arg.slice('--ws-port='.length)) || out.wsPort;
      continue;
    }
    if (arg === '--bridge-ws-port') {
      out.bridgeWsPort = Number(readValue()) || out.bridgeWsPort;
      continue;
    }
    if (arg.startsWith('--bridge-ws-port=')) {
      out.bridgeWsPort = Number(arg.slice('--bridge-ws-port='.length)) || out.bridgeWsPort;
      continue;
    }
    if (arg === '--raw-path') {
      out.rawPath = readValue() || out.rawPath;
      continue;
    }
    if (arg.startsWith('--raw-path=')) {
      out.rawPath = arg.slice('--raw-path='.length) || out.rawPath;
      continue;
    }
    if (arg === '--host-path') {
      out.hostPath = readValue() || out.hostPath;
      continue;
    }
    if (arg.startsWith('--host-path=')) {
      out.hostPath = arg.slice('--host-path='.length) || out.hostPath;
      continue;
    }
    if (arg === '--mcu-bin') {
      out.mcuBin = readValue() || out.mcuBin;
      continue;
    }
    if (arg.startsWith('--mcu-bin=')) {
      out.mcuBin = arg.slice('--mcu-bin='.length) || out.mcuBin;
      continue;
    }
    if (arg === '--dict') {
      out.dictPath = readValue() || out.dictPath;
      continue;
    }
    if (arg.startsWith('--dict=')) {
      out.dictPath = arg.slice('--dict='.length) || out.dictPath;
      continue;
    }
    if (arg === '--klipper-py') {
      out.klipperPyPath = readValue() || out.klipperPyPath;
      continue;
    }
    if (arg.startsWith('--klipper-py=')) {
      out.klipperPyPath = arg.slice('--klipper-py='.length) || out.klipperPyPath;
      continue;
    }
    if (arg === '--klippy-python') {
      out.klippyPython = readValue() || out.klippyPython;
      continue;
    }
    if (arg.startsWith('--klippy-python=')) {
      out.klippyPython = arg.slice('--klippy-python='.length) || out.klippyPython;
      continue;
    }
    if (arg === '--cmd' || arg === '-c') {
      out.command = readValue() || '';
      continue;
    }
    if (arg.startsWith('--cmd=')) {
      out.command = arg.slice('--cmd='.length);
      continue;
    }
  }

  return out;
}

export function buildMcuBridgeSpawnSpec(args, { cwd = process.cwd() } = {}) {
  const bridgeScript = resolvePath('examples/klipper/slideprinter/klipper_linux_mcu_bridge.py', cwd);
  return {
    command: bridgeScript,
    args: [
      '--raw-path', resolvePath(args.rawPath, cwd),
      '--host-path', resolvePath(args.hostPath, cwd),
      '--mcu-bin', resolvePath(args.mcuBin, cwd),
      '--dict', resolvePath(args.dictPath, cwd),
      '--klipper-py', resolvePath(args.klipperPyPath, cwd),
      '--no-realtime',
      '--ws-port', String(args.bridgeWsPort),
    ],
  };
}

export function buildKlippySpawnSpec(args, { cwd = process.cwd() } = {}) {
  const klippyPy = resolvePath('klipper/klippy/klippy.py', cwd);
  return {
    command: args.klippyPython || DEFAULT_KLIPPY_PYTHON,
    args: [
      klippyPy,
      resolvePath(args.config, cwd),
      '-a', args.socketPath,
      '-l', '/tmp/klipper.log',
    ],
  };
}
