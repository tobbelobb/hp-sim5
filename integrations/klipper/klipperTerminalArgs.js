import {
  DEFAULT_KLIPPY_API_START_SCRIPT,
  DEFAULT_KLIPPY_CONFIG_PATH,
  DEFAULT_KLIPPY_LOG_PATH,
  DEFAULT_KLIPPY_SOCKET_PATH,
} from './klippy_api_cli_config.mjs';
import {
  isBuildupFlag,
  isNoBuildupFlag,
  resolveKlipperConfigFromMachine,
  resolveKlipperConfigSelector,
} from './klipperConfigResolver.js';

function takeValue(argv, index, optionName) {
  const value = argv[index + 1];
  if (!value) {
    throw new Error(`${optionName} requires an argument`);
  }
  return value;
}

export function parseArgs(argv) {
  let machineType = 'hp3';
  let buildup = true;
  let configExplicit = false;
  let hasConfigSelectorOption = false;
  const args = {
    socketPath: process.env.KLIPPY_SOCKET_PATH || DEFAULT_KLIPPY_SOCKET_PATH,
    configPath: process.env.KLIPPY_CONFIG_PATH || DEFAULT_KLIPPY_CONFIG_PATH,
    logPath: process.env.KLIPPY_LOG_PATH || DEFAULT_KLIPPY_LOG_PATH,
    startScript: process.env.KLIPPY_API_START_SCRIPT || DEFAULT_KLIPPY_API_START_SCRIPT,
    wsPort: 8790,
    noWs: false,
    command: null,
    quiet: false,
    debug: false,
    keepAlive: false,
    help: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--socket') {
      args.socketPath = takeValue(argv, i, arg);
      i += 1;
    } else if (arg === '-c' || arg === '--config') {
      args.configPath = resolveKlipperConfigSelector(takeValue(argv, i, arg));
      configExplicit = true;
      hasConfigSelectorOption = true;
      i += 1;
    } else if (arg.startsWith('--config=')) {
      args.configPath = resolveKlipperConfigSelector(arg.slice('--config='.length));
      configExplicit = true;
      hasConfigSelectorOption = true;
    } else if (arg === '-m' || arg === '--machineType') {
      machineType = takeValue(argv, i, arg);
      hasConfigSelectorOption = true;
      i += 1;
    } else if (arg.startsWith('--machineType=')) {
      machineType = arg.slice('--machineType='.length);
      hasConfigSelectorOption = true;
    } else if (isBuildupFlag(arg)) {
      buildup = true;
      hasConfigSelectorOption = true;
    } else if (isNoBuildupFlag(arg)) {
      buildup = false;
      hasConfigSelectorOption = true;
    } else if (arg === '--log-path') {
      args.logPath = takeValue(argv, i, arg);
      i += 1;
    } else if (arg === '--start-script') {
      args.startScript = takeValue(argv, i, arg);
      i += 1;
    } else if (arg === '--ws-port') {
      const value = parseInt(takeValue(argv, i, arg), 10);
      i += 1;
      args.wsPort = Number.isFinite(value) && value > 0 ? value : 0;
    } else if (arg === '--no-ws') {
      args.noWs = true;
    } else if (arg === '--cmd') {
      args.command = takeValue(argv, i, arg);
      i += 1;
    } else if (arg === '--quiet' || arg === '-q') {
      args.quiet = true;
    } else if (arg === '--debug') {
      args.debug = true;
    } else if (arg === '--keep-alive') {
      args.keepAlive = true;
    } else if (arg === '--help' || arg === '-h') {
      args.help = true;
    }
  }

  if (!configExplicit && (hasConfigSelectorOption || !process.env.KLIPPY_CONFIG_PATH)) {
    args.configPath = resolveKlipperConfigFromMachine(machineType, { buildup });
  }

  return args;
}
