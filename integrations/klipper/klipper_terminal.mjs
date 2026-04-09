#!/usr/bin/env node
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import readline from 'node:readline';
import { fileURLToPath } from 'node:url';
import { KlippyApiClient } from './klippyApiClient.js';
import {
  buildKlippyBridgeWsHint,
  createKlipperTerminalBridge,
} from './klipperTerminalBridge.js';
import { parseArgs } from './klipperTerminalArgs.js';
import {
  buildFakeGpioChipSetupCommand,
  configNeedsGpioChipSetup,
  isMissingFakeGpioChipStateMessage,
  shouldDeferFakeGpioSetupPromptUntilInteractiveReadline,
} from './klipperTerminalRecovery.js';
import { KlippyRuntimeState } from './klippyRuntimeState.js';
import {
  DEFAULT_KLIPPY_API_START_SCRIPT,
  DEFAULT_KLIPPY_CONFIG_PATH,
  DEFAULT_KLIPPY_LOG_PATH,
  DEFAULT_KLIPPY_SOCKET_PATH,
  ensureKlippyApiServer,
  stopKlippyApiServer,
  terminateKlippyApiProcess,
  waitForKlippySocket,
} from './klippy_api_cli_config.mjs';

const CLIENT_INFO = {
  program: 'klipper_terminal',
  version: 'step4',
};

const PROMPT_READY = 'gcode> ';
const PROMPT_DISCONNECTED = '\x1b[90mdisconnected>\x1b[0m ';
const DEFAULT_DEBUG_LOG_FILENAME = 'klipper_terminal_debug.log';
const LOST_MCU_STATE_RE = /Lost communication with MCU/i;

function runFakeGpioChipSetup(commandSpec) {
  return new Promise((resolve, reject) => {
    const child = spawn(commandSpec.command, commandSpec.args, {
      stdio: 'inherit',
    });
    child.once('error', reject);
    child.once('exit', (code, signal) => {
      if (signal) {
        reject(new Error(`fake gpio chip setup exited on signal ${signal}`));
        return;
      }
      if (code === 0) {
        resolve();
        return;
      }
      reject(new Error(`fake gpio chip setup exited with code ${code}`));
    });
  });
}

function buildStatePrompt({ connected, printerState }) {
  if (!connected) {
    return PROMPT_DISCONNECTED;
  }
  if (printerState === 'ready') {
    return PROMPT_READY;
  }
  const label = printerState || 'connected';
  const color = printerState === 'shutdown' || printerState === 'error'
    ? '\x1b[31m'
    : '\x1b[33m';
  return `${color}${label}>\x1b[0m `;
}

function splitTerminalLines(response) {
  return String(response)
    .split(/\r?\n/)
    .map((line) => line.trimEnd())
    .filter((line) => line.length > 0);
}

function printHelp() {
  console.log(`Usage: node integrations/klipper/klipper_terminal.mjs [options]

Send G-code to a Klippy API socket, subscribe to motion_report stepper batches,
and fan simulator commands out over WebSocket for hp-sim to visualize.

Options:
  --socket <path>          Klippy API socket path (default: ${DEFAULT_KLIPPY_SOCKET_PATH})
  --config <path>          Klippy printer config for autostart
                           (default: ${DEFAULT_KLIPPY_CONFIG_PATH})
  --log-path <path>        Klippy log path for autostart (default: ${DEFAULT_KLIPPY_LOG_PATH})
  --start-script <path>    Launcher script for autostart
                           (default: ${DEFAULT_KLIPPY_API_START_SCRIPT})
  --ws-port <port>         Port for the hp-sim WebSocket fan-out (default: 8790)
  --no-ws                  Disable WebSocket fan-out entirely
  --cmd, -c <GCODE>        Send G-code and exit
  --quiet, -q              Only print command results
  --debug                  Write API ingress logs to ${DEFAULT_DEBUG_LOG_FILENAME}
  --keep-alive             Do not stop a terminal-managed klippy on exit
  --help, -h               Show this help`);
}
export { parseArgs } from './klipperTerminalArgs.js';

function createTerminalDebugLogger({ enabled = false, logPath = DEFAULT_DEBUG_LOG_FILENAME, cliArgv = [] } = {}) {
  const resolvedPath = path.resolve(process.cwd(), logPath);
  if (enabled) {
    fs.writeFileSync(resolvedPath, '', 'utf8');
  }

  const serializeError = (error) => {
    if (!(error instanceof Error)) {
      return error;
    }
    return {
      name: error.name,
      message: error.message,
      code: error.code ?? null,
      stack: error.stack ?? null,
      cause: error.cause ? serializeError(error.cause) : null,
    };
  };

  const replacer = (_key, value) => {
    if (value instanceof Error) {
      return serializeError(value);
    }
    if (typeof value === 'bigint') {
      return value.toString();
    }
    return value;
  };

  const append = (event, payload = {}) => {
    if (!enabled) {
      return;
    }
    const entry = {
      ts: new Date().toISOString(),
      event,
      ...payload,
    };
    fs.appendFileSync(resolvedPath, `${JSON.stringify(entry, replacer)}\n`, 'utf8');
  };

  append('debug-session-start', {
    pid: process.pid,
    cwd: process.cwd(),
    cliArgv,
  });

  return {
    enabled,
    path: resolvedPath,
    log: append,
    close() {
      append('debug-session-end');
    },
  };
}

function buildRuntime(args, { cliArgv = process.argv.slice(2) } = {}) {
  const debugLogger = createTerminalDebugLogger({
    enabled: args.debug,
    cliArgv,
  });
  const client = new KlippyApiClient({
    socketPath: args.socketPath,
  });
  const klippyState = new KlippyRuntimeState({
    client,
    clientInfo: CLIENT_INFO,
    debugLog: debugLogger.log,
  });

  let rl = null;
  let promptEverRendered = false;
  let managedKlippyProcess = null;
  let autoStartInFlight = null;
  let shuttingDown = false;
  let primePromise = null;
  let lastReportedPrinterState = null;
  let lastReportedMotionKey = null;
  let fakeGpioPromptInFlight = false;
  let fakeGpioPromptShown = false;
  let pendingFakeGpioPromptSnapshot = null;
  let confirmationPromptInFlight = false;
  let terminalHandoffInFlight = false;
  const deferredRuntimeLines = [];
  const sendQueue = [];
  let processingQueue = false;
  const bridgeContext = createKlipperTerminalBridge({
    client,
    klippyState,
    wsPort: args.noWs ? 0 : args.wsPort,
    quiet: args.quiet,
    debugLog: debugLogger.log,
    debugTrapq: args.debug,
    onClientChange: () => updatePromptForRuntimeState(),
  });

  const interactivePromptEnabled = () => rl && process.stdin.isTTY && !args.quiet;

  const updatePromptForRuntimeState = ({ forcePrompt = false } = {}) => {
    if (terminalHandoffInFlight || confirmationPromptInFlight) {
      return;
    }
    if (!interactivePromptEnabled()) {
      return;
    }
    const nextPrompt = buildStatePrompt(klippyState.getSnapshot());
    if (rl.getPrompt() !== nextPrompt) {
      rl.setPrompt(nextPrompt);
    }
    if (forcePrompt || !promptEverRendered) {
      promptEverRendered = true;
      rl.prompt();
      return;
    }
    rl.prompt(true);
  };

  const promptIfInteractive = () => {
    if (confirmationPromptInFlight) {
      return;
    }
    if (!interactivePromptEnabled()) {
      return;
    }
    promptEverRendered = true;
    rl.prompt(true);
  };

  const handleReadlineLine = (line) => {
    enqueueLine(line);
  };

  const processQueue = async () => {
    if (processingQueue) {
      return;
    }
    processingQueue = true;
    try {
      while (sendQueue.length > 0) {
        const next = sendQueue.shift();
        // eslint-disable-next-line no-await-in-loop
        await next();
      }
    } finally {
      processingQueue = false;
    }
  };

  const enqueueLine = (line) => {
    sendQueue.push(() => sendGcodeLine(line));
    processQueue().catch((error) => {
      console.error(`Unexpected terminal error: ${error.message}`);
    });
  };

  const handleReadlineClose = () => {
    shutdown(0);
  };

  const attachInteractiveReadline = () => {
    if (rl) {
      return;
    }
    rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
      terminal: process.stdin.isTTY,
    });
    rl.on('line', handleReadlineLine);
    rl.on('close', handleReadlineClose);
  };

  const detachInteractiveReadline = () => {
    if (!rl) {
      return false;
    }
    rl.off('line', handleReadlineLine);
    rl.off('close', handleReadlineClose);
    rl.pause();
    rl.close();
    rl = null;
    return true;
  };

  const flushDeferredRuntimeLines = () => {
    while (deferredRuntimeLines.length > 0) {
      const { line, stream } = deferredRuntimeLines.shift();
      stream(line);
    }
  };

  const withExclusiveTerminalAccess = async (fn) => {
    const hadInteractiveReadline = detachInteractiveReadline();
    terminalHandoffInFlight = true;
    try {
      return await fn();
    } finally {
      terminalHandoffInFlight = false;
      if (hadInteractiveReadline && !shuttingDown) {
        attachInteractiveReadline();
      }
      flushDeferredRuntimeLines();
      updatePromptForRuntimeState({ forcePrompt: true });
    }
  };

  const askUserConfirmation = async (question) => {
    if (rl) {
      return new Promise((resolve) => {
        rl.question(question, (answer) => resolve(answer));
      });
    }
    const promptRl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
      terminal: process.stdin.isTTY,
    });
    try {
      return await new Promise((resolve) => {
        promptRl.question(question, (answer) => resolve(answer));
      });
    } finally {
      promptRl.close();
    }
  };

  const primeConnection = async () => {
    const nextPrime = (async () => {
      await client.waitForConnection();
      return klippyState.prime();
    })();
    primePromise = nextPrime;
    try {
      return await nextPrime;
    } finally {
      if (primePromise === nextPrime) {
        primePromise = null;
      }
    }
  };

  const waitForPrimedConnection = async () => {
    if (primePromise) {
      return primePromise;
    }
    return primeConnection();
  };

  const waitForKlippyReady = async (timeoutMs = 15_000) => klippyState.waitForReady(timeoutMs);

  const maybeRecoverLostMcuStartupState = async () => {
    const snapshot = klippyState.getSnapshot();
    const stateMessage = snapshot.printerStateMessage || '';
    if (snapshot.printerState !== 'shutdown' || !LOST_MCU_STATE_RE.test(stateMessage)) {
      return null;
    }
    if (!args.quiet) {
      console.log('Detected stale Klippy shutdown caused by a missing host MCU. Restarting the managed Klippy stack...');
    }
    debugLogger.log('startup-recovery', {
      socketPath: args.socketPath,
      printerState: snapshot.printerState,
      printerStateMessage: stateMessage,
      managedKlippyProcess: Boolean(managedKlippyProcess),
    });
    if (managedKlippyProcess) {
      stopKlippyApiServer(managedKlippyProcess);
      managedKlippyProcess = null;
    } else {
      await terminateKlippyApiProcess({
        socketPath: args.socketPath,
      }).catch((error) => {
        debugLogger.log('startup-recovery-terminate-error', {
          socketPath: args.socketPath,
          error,
        });
      });
    }
    primePromise = null;
    lastReportedPrinterState = null;
    lastReportedMotionKey = null;
    await ensureKlippyServerReady({ forceStart: true });
    return waitForPrimedConnection();
  };

  const maybeFlushPendingFakeGpioPrompt = async () => {
    if (!pendingFakeGpioPromptSnapshot || !rl || shuttingDown) {
      return;
    }
    const snapshot = pendingFakeGpioPromptSnapshot;
    pendingFakeGpioPromptSnapshot = null;
    await maybeOfferFakeGpioChipSetup(snapshot);
  };

  const terminateStaleKlippyProcess = async ({ reason }) => {
    if (managedKlippyProcess) {
      return false;
    }
    try {
      const terminated = await terminateKlippyApiProcess({
        socketPath: args.socketPath,
      });
      if (terminated) {
        debugLogger.log('stale-klippy-terminated', {
          socketPath: args.socketPath,
          reason,
        });
        if (!args.quiet) {
          console.log(`Stopped stale klippy.py on ${args.socketPath} before starting the managed stack.`);
        }
      }
      return terminated;
    } catch (error) {
      debugLogger.log('stale-klippy-terminate-error', {
        socketPath: args.socketPath,
        reason,
        error,
      });
      return false;
    }
  };

  const ensureKlippyServerReady = async ({ forceStart = false } = {}) => {
    if (autoStartInFlight) {
      return autoStartInFlight;
    }
    autoStartInFlight = (async () => {
      if (!forceStart) {
        try {
          await waitForKlippySocket(args.socketPath, 750);
          return managedKlippyProcess;
        } catch (_error) {
          await terminateStaleKlippyProcess({ reason: 'socket-not-ready' });
          // Fall through to autostart.
        }
      } else {
        await terminateStaleKlippyProcess({ reason: 'forced-restart' });
      }
      if (!managedKlippyProcess) {
        managedKlippyProcess = await ensureKlippyApiServer({
          socketPath: args.socketPath,
          configPath: args.configPath,
          logPath: args.logPath,
          startScript: args.startScript,
          onInfo: args.quiet ? null : (message) => console.log(message),
        });
        return managedKlippyProcess;
      }
      await waitForKlippySocket(args.socketPath);
      return managedKlippyProcess;
    })();
    try {
      return await autoStartInFlight;
    } finally {
      autoStartInFlight = null;
    }
  };

  const printRuntimeLine = (line, stream = console.log) => {
    if (terminalHandoffInFlight) {
      deferredRuntimeLines.push({ line, stream });
      return;
    }
    stream(line);
    updatePromptForRuntimeState();
  };

  const reportPrinterState = (snapshot) => {
    if (args.quiet) {
      return;
    }
    const state = snapshot.printerState;
    const stateMessage = snapshot.printerStateMessage;
    const stateKey = `${state || ''}\n${stateMessage || ''}`;
    if (!state || lastReportedPrinterState === stateKey) {
      return;
    }
    lastReportedPrinterState = stateKey;
    const suffix = stateMessage && stateMessage !== state ? `: ${stateMessage}` : '';
    printRuntimeLine(`Klippy state: ${state}${suffix}`);
  };

  const reportMotionSources = ({ steppers = [], trapq = [] }) => {
    if (args.quiet) {
      return;
    }
    const motionKey = JSON.stringify({ steppers, trapq });
    if (motionKey === lastReportedMotionKey) {
      return;
    }
    lastReportedMotionKey = motionKey;
    const parts = [];
    if (steppers.length > 0) {
      parts.push(`steppers=${steppers.join(', ')}`);
    }
    if (trapq.length > 0) {
      parts.push(`trapq=${trapq.join(', ')}`);
    }
    if (parts.length > 0) {
      printRuntimeLine(`Motion sources: ${parts.join(' | ')}`);
    }
  };

  const restartManagedKlippyAfterFakeGpioSetup = async () => {
    if (!managedKlippyProcess) {
      return false;
    }
    printRuntimeLine('Restarting the terminal-managed Klippy process now that the fake gpio chip is available...');
    stopKlippyApiServer(managedKlippyProcess);
    managedKlippyProcess = null;
    primePromise = null;
    await new Promise((resolve) => setTimeout(resolve, 500));
    await ensureKlippyServerReady({ forceStart: true });
    await waitForPrimedConnection();
    await waitForKlippyReady();
    return true;
  };

  const maybeOfferFakeGpioChipSetup = async (snapshot) => {
    if (fakeGpioPromptShown || fakeGpioPromptInFlight || pendingFakeGpioPromptSnapshot || shuttingDown) {
      return;
    }
    if (!isMissingFakeGpioChipStateMessage(snapshot?.printerStateMessage)) {
      return;
    }
    const resolvedConfigPath = path.resolve(process.cwd(), args.configPath);
    if (!await configNeedsGpioChipSetup(resolvedConfigPath)) {
      return;
    }

    if (shouldDeferFakeGpioSetupPromptUntilInteractiveReadline({
      stdinIsTTY: process.stdin.isTTY,
      hasInteractiveReadline: Boolean(rl),
      hasCommand: Boolean(args.command),
    })) {
      pendingFakeGpioPromptSnapshot = snapshot;
      debugLogger.log('fake-gpio-setup-prompt', {
        phase: 'deferred-until-interactive-readline',
      });
      return;
    }

    fakeGpioPromptShown = true;
    fakeGpioPromptInFlight = true;
    const commandSpec = buildFakeGpioChipSetupCommand(args.configPath);
    const commandText = `${commandSpec.command} ${commandSpec.args.join(' ')}`;

    try {
      printRuntimeLine('Klipper running on Linux needs a fake gpio chip before this config can start.');
      printRuntimeLine(`It can be created like this: ${commandText}`);

      if (!process.stdin.isTTY) {
        printRuntimeLine('Run that command, then use FIRMWARE_RESTART or restart klipper_terminal.');
        return;
      }

      confirmationPromptInFlight = true;
      debugLogger.log('fake-gpio-setup-prompt', {
        phase: 'question-shown',
      });
      const answer = await askUserConfirmation(
        'Do you allow me to run that command for you? This command will ask you for sudo privileges. [y/N] ',
      );
      debugLogger.log('fake-gpio-setup-prompt', {
        phase: 'question-answered',
        answer: String(answer ?? ''),
      });
      if (!/^(y|yes)$/iu.test(String(answer).trim())) {
        printRuntimeLine('Skipped fake gpio chip setup. Run the command above, then use FIRMWARE_RESTART or restart klipper_terminal.');
        return;
      }

      await withExclusiveTerminalAccess(async () => {
        await runFakeGpioChipSetup(commandSpec);
      });
      const recovered = await restartManagedKlippyAfterFakeGpioSetup().catch((error) => {
        console.error(`Failed to restart Klippy after fake gpio chip setup: ${error.message}`);
        return false;
      });
      if (recovered) {
        printRuntimeLine('Fake gpio chip setup completed and Klippy restarted successfully.');
      } else {
        printRuntimeLine('Fake gpio chip setup completed. Use FIRMWARE_RESTART or restart klipper_terminal to retry Klippy.');
      }
    } catch (error) {
      console.error(`Failed to create fake gpio chip: ${error.message}`);
    } finally {
      confirmationPromptInFlight = false;
      fakeGpioPromptInFlight = false;
      updatePromptForRuntimeState();
    }
  };

  const sendGcodeLine = async (line) => {
    const trimmed = line?.trim?.();
    if (!trimmed) {
      promptIfInteractive();
      return true;
    }
    if (!args.quiet) {
      console.log(`> ${trimmed}`);
    }
    try {
      await waitForPrimedConnection();
      await waitForKlippyReady();
      const result = await bridgeContext.runGcodeCommand(
        trimmed,
        async () => {
          const response = await client.request('gcode/script', { script: trimmed });
          debugLogger.log('api-response', {
            method: 'gcode/script',
            channel: 'command',
            gcode: trimmed,
            result: response,
          });
          return response;
        },
      );
      if (!result?.printedLiveOutput) {
        console.log(result?.reply?.trim?.() || 'ok');
      }
      return true;
    } catch (error) {
      console.error(`Error sending "${trimmed}": ${error.message}`);
      return false;
    } finally {
      promptIfInteractive();
    }
  };

  const shutdown = (exitCode = 0) => {
    if (shuttingDown) {
      return;
    }
    shuttingDown = true;
    debugLogger.close();
    try {
      bridgeContext.close();
    } catch (_error) {
      // Ignore shutdown errors.
    }
    try {
      client.close();
    } catch (_error) {
      // Ignore shutdown errors.
    }
    if (managedKlippyProcess && !args.keepAlive) {
      stopKlippyApiServer(managedKlippyProcess);
      managedKlippyProcess = null;
    }
    process.exit(exitCode);
  };

  klippyState.on('gcode-output', ({ response }) => {
    const linesToPrint = bridgeContext.handleGcodeOutput(response);
    for (const line of Array.isArray(linesToPrint) ? linesToPrint : splitTerminalLines(response)) {
      printRuntimeLine(line);
    }
  });

  klippyState.on('printer-state-changed', ({ snapshot }) => {
    updatePromptForRuntimeState();
    reportPrinterState(snapshot);
    maybeOfferFakeGpioChipSetup(snapshot).catch((error) => {
      if (!shuttingDown) {
        console.error(`Failed to offer fake gpio chip setup: ${error.message}`);
      }
    });
  });

  klippyState.on('motion-sources-changed', ({ steppers, trapq }) => {
    reportMotionSources({ steppers, trapq });
  });

  client.on('connected', () => {
    updatePromptForRuntimeState();
    primeConnection().catch((error) => {
      if (!shuttingDown) {
        console.error(`Failed to prime Klippy runtime state: ${error.message}`);
      }
    });
  });

  client.on('disconnected', () => {
    updatePromptForRuntimeState();
  });

  client.on('socket-error', (error) => {
    debugLogger.log('socket-error', {
      error,
    });
    if (shuttingDown || args.quiet) {
      return;
    }
    if (error?.code === 'ENOENT' || error?.code === 'ECONNREFUSED') {
      return;
    }
    console.error(`Klippy socket error: ${error.message}`);
  });

  const main = async () => {
    await ensureKlippyServerReady();
    client.start();
    let primed = await waitForPrimedConnection();
    const recoveredPrime = await maybeRecoverLostMcuStartupState();
    if (recoveredPrime) {
      primed = await recoveredPrime;
    }

    if (args.command) {
      await bridgeContext.waitForHpSimConnection();
      const success = await sendGcodeLine(args.command);
      shutdown(success ? 0 : 1);
      return;
    }

    attachInteractiveReadline();
    await maybeFlushPendingFakeGpioPrompt();

    if (process.stdin.isTTY && !args.quiet) {
      updatePromptForRuntimeState({ forcePrompt: true });
    }

    if (!args.quiet) {
      const objectCount = primed?.objectsList?.objects?.length ?? 0;
      const snapshot = klippyState.getSnapshot();
      const state = snapshot.printerState || 'unknown';
      console.log(`Connected to Klippy on ${args.socketPath} (${state}, ${objectCount} objects).`);
      if (debugLogger.enabled) {
        console.log(`Debug logging to ${debugLogger.path}`);
      }
      if (bridgeContext.wss) {
        console.log(`WebSocket feed ready on ws://localhost:${args.wsPort}`);
        console.log(`Open hp-sim with ${buildKlippyBridgeWsHint(args.wsPort)} to follow along.`);
      }
      reportPrinterState(snapshot);
      reportMotionSources(snapshot.motionSources);
    }
  };

  return {
    client,
    klippyState,
    main,
    shutdown,
  };
}

export async function main(argv = process.argv.slice(2)) {
  const args = parseArgs(argv);
  if (args.help) {
    printHelp();
    return 0;
  }
  const runtime = buildRuntime(args, { cliArgv: argv });
  activeRuntime = runtime;
  await runtime.main();
  return 0;
}

const entryPath = process.argv[1] ? path.resolve(process.argv[1]) : null;
const modulePath = fileURLToPath(import.meta.url);
let activeRuntime = null;

if (entryPath && entryPath === modulePath) {
  main().catch((error) => {
    console.error(`Failed to start klipper_terminal: ${error.message}`);
    process.exit(1);
  });
}

function handleSignal(exitCode) {
  if (activeRuntime) {
    activeRuntime.shutdown(exitCode);
    return;
  }
  process.exit(exitCode);
}

process.on('SIGINT', () => handleSignal(130));
process.on('SIGTERM', () => handleSignal(143));
process.on('SIGHUP', () => handleSignal(129));
