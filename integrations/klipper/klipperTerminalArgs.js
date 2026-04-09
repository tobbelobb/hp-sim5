import {
  DEFAULT_KLIPPY_API_START_SCRIPT,
  DEFAULT_KLIPPY_CONFIG_PATH,
  DEFAULT_KLIPPY_LOG_PATH,
  DEFAULT_KLIPPY_SOCKET_PATH,
} from './klippy_api_cli_config.mjs';

export function parseArgs(argv) {
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
      args.socketPath = argv[++i] || args.socketPath;
    } else if (arg === '--config') {
      args.configPath = argv[++i] || args.configPath;
    } else if (arg === '--log-path') {
      args.logPath = argv[++i] || args.logPath;
    } else if (arg === '--start-script') {
      args.startScript = argv[++i] || args.startScript;
    } else if (arg === '--ws-port') {
      const value = parseInt(argv[++i], 10);
      args.wsPort = Number.isFinite(value) && value > 0 ? value : 0;
    } else if (arg === '--no-ws') {
      args.noWs = true;
    } else if (arg === '--cmd' || arg === '-c') {
      args.command = argv[++i] || null;
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

  return args;
}
