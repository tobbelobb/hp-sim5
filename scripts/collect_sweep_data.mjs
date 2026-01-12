#!/usr/bin/env node
import path from 'node:path';
import readline from 'node:readline';
import { pathToFileURL } from 'node:url';
import { createGcodeBridge, parseBridgeArgs } from './gcode_bridge.mjs';
import {
  DEFAULT_RRF_PORT,
  sendHpSimPositionTraceMode,
  sendHpSimReset,
  sendHpSimSpeedScale,
  sleep as baseSleep,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from './encoder_utils.mjs';
import {
  collectSweepData,
  MACHINE_CONFIGS,
  MOTOR_IDS_BY_MACHINE,
  SWEEP_DEFAULTS,
} from './sweep_data_collection.mjs';
import { FORCE_TUNING_DEFAULTS } from './force_tuning.mjs';

const SOURCE_FILE_LABEL = 'scripts/collect_sweep_data.mjs';
let stepGcodeMode = false;
let pendingPreSendDelayMs = 0;
let stepReadline = null;

function getSourceLineFromStack(stack, { skipMatches = 0 } = {}) {
  if (!stack) {
    return null;
  }
  const matches = [...stack.matchAll(/(scripts\/[^:\n]+\.mjs):(\d+):\d+/g)];
  if (matches.length === 0) {
    return null;
  }
  const idx = matches.length > skipMatches ? skipMatches : 0;
  const file = matches[idx][1];
  const line = parseInt(matches[idx][2], 10);
  if (!Number.isFinite(line)) {
    return null;
  }
  return { file, line };
}

async function waitForEnter(message) {
  if (!process.stdin.isTTY) {
    console.log(message);
    return;
  }
  if (!stepReadline) {
    stepReadline = readline.createInterface({ input: process.stdin, output: process.stdout });
  }
  await new Promise((resolve) => stepReadline.question(message, resolve));
}

function sleep(ms) {
  const delayMs = Number(ms);
  if (!Number.isFinite(delayMs) || delayMs <= 0) {
    return Promise.resolve();
  }
  if (stepGcodeMode) {
    pendingPreSendDelayMs += delayMs;
    return Promise.resolve();
  }
  return baseSleep(delayMs);
}

function printHelp() {
  console.log(`Usage: node scripts/collect_sweep_data.mjs [options]

Collect circular sweep data where N-1 anchors are fixed, one anchor drives, and another is in force/sensor mode.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --sweepRange <mm>          Sweep half-range in mm (default: ${SWEEP_DEFAULTS.DEFAULT_SWEEP_RANGE_MM})
  --sweepPoints <count>      Number of points per sweep (default: ${SWEEP_DEFAULTS.DEFAULT_SWEEP_POINTS})
  --superSweepRange <mm>     Fixed-anchor half-range in mm (default: ${SWEEP_DEFAULTS.DEFAULT_SUPER_SWEEP_RANGE_MM})
  --superSweepPoints <count> Number of fixed-length samples per sweep config (default: ${SWEEP_DEFAULTS.DEFAULT_SUPER_SWEEP_POINTS})
  --fixed-targets <spec>     Override fixed-anchor targets; e.g. "-600,-200,200,600" or "-100,0;0,100"
  --maxSweeps <count>        Max sweeps when auto-generating configs (default: ${SWEEP_DEFAULTS.DEFAULT_MAX_SWEEPS})
  --feed <mm/min>            Feed rate for drive moves (default: ${SWEEP_DEFAULTS.DEFAULT_FEED})
  --torque <Nm>              Force for sensor motor (default: ${SWEEP_DEFAULTS.DEFAULT_TORQUE})
  --settleMs <ms>            Deprecated (was fixed settle time; now waits for encoder stability)
  --speedup <scale>          hp-sim speed scale (default: 1)
  --continuous               Use continuous sweep mode (records at --sample-rate)
  --sample-rate <Hz>         Sample rate for continuous mode (default: ${SWEEP_DEFAULTS.DEFAULT_SAMPLE_RATE_HZ})
  --sweep-method <name>      Sweep method: position | torque-ramp | force-ramp (default: ${SWEEP_DEFAULTS.DEFAULT_SWEEP_METHOD})
  --torque-low <Nm>          torque-ramp: idle force (default: ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_LOW_NM})
  --torque-min <Nm>          torque-ramp: start force (default: ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MIN_NM})
  --torque-max <Nm>          torque-ramp: end force (default: ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MAX_NM})
  --torque-step <Nm>         torque-ramp: force increment (default: (torque-max - torque-min) / ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_STEP_DIVISOR})
  --auto-tune-torque         torque-ramp: auto-tune force-low/min/max using a test sweep (default when no torque args)
  --no-auto-tune-torque      torque-ramp: skip auto-tuning and use provided/default forces
  --ramp-wait-ms <ms>        Deprecated (was fixed wait; now waits for encoder stability)
  --swap-wait-ms <ms>        Deprecated (was fixed wait; now waits for encoder stability)
  --sweep-config-file <file> Provide explicit sweep configs ([fixed] drive sensor per line)
  --debug-sweep              Print planned sweep permutations before collecting
  --trace                    Tell hp-sim to plot a trace of its movements (default: on)
  --no-trace                 Disable hp-sim trace plotting
  --output-file <path>       Output JSON path (default: sweep_data_<machine>_<timestamp>.json)
  --observability-file <path> Sidecar histogram JSON path (default: <output>.obs.json)
  --server, --rrf <url>      RRF server URL (default: http://localhost:${DEFAULT_RRF_PORT})
  --port <port>              Port for spawned rrf_simulator (default: ${DEFAULT_RRF_PORT})
  --no-spawn-rrf-simulator   Do not start rrf_simulator automatically
  --no-ws                    Disable hp-sim websocket bridge
  --ws-port <port>           WebSocket port (default: 8790)
  --no-hp-sim-reset          Do not reset hp-sim on start
  --wait-ws <ms>             Wait for hp-sim websocket connection (default: 0)
  --debug                    Verbose logging (includes G-code replies)
  --debug-gcode              Echo sent G-code
  --debug-gcode-responses    Echo G-code responses
  --step-gcode               Pause before each G-code; press Enter to send (prints source line + skipped waits)
  --return-to-origin         After collection, return all motors to encoder origin (useful across repeated runs)

Examples:
  node scripts/collect_sweep_data.mjs --machineType slideprinter --sweepRange 100 --sweepPoints 41 --output-file sweep.json
  node scripts/collect_sweep_data.mjs --machineType hangprinter_4 --sweep-config-file scripts/sweep_configs/hangprinter_4.txt --debug-sweep`);
}

async function main() {
  const argv = process.argv.slice(2);
  const args = parseBridgeArgs(argv);
  const hasNoTrace = argv.includes('--no-trace');
  if (hasNoTrace) {
    args.trace = false;
  } else if (!args.trace) {
    args.trace = true;
  }
  if (args.help) {
    printHelp();
    process.exit(0);
  }
  stepGcodeMode = !!args.stepGcode;
  const machineType = (args.machineType || 'slideprinter').toLowerCase();
  const machineConfig = MACHINE_CONFIGS[machineType];
  if (!machineConfig) {
    console.error(`Unknown machine type: ${machineType}`);
    process.exit(1);
  }
  const motorIds = MOTOR_IDS_BY_MACHINE[machineType];
  if (!motorIds || motorIds.length !== machineConfig.numAnchors) {
    console.error(`Motor ID mapping missing or mismatched for ${machineType}`);
    process.exit(1);
  }
  if (motorIds.length !== machineConfig.axes.length) {
    console.error(`Motor ID count (${motorIds.length}) does not match axes count (${machineConfig.axes.length}) for ${machineType}`);
    process.exit(1);
  }

  const speedup = Number.isFinite(parseFloat(args.speedup)) && parseFloat(args.speedup) > 0
    ? parseFloat(args.speedup)
    : 1;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : 0;

  const targetPort = Number.isFinite(parseInt(args.port, 10)) ? parseInt(args.port, 10) : DEFAULT_RRF_PORT;
  const targetServer = args.serverExplicit ? args.server : `http://localhost:${targetPort}`;
  const shouldSpawnRrf = !args.noSpawnRrfSimulator && !args.serverExplicit;
  let rrfProcess = null;

  if (shouldSpawnRrf) {
    try {
      console.log(`Starting rrf_simulator at ${targetServer}...`);
      rrfProcess = await startRrfSimulator({ port: targetPort, debug: args.debug });
      await waitForRrfSimulator(targetServer);
    } catch (err) {
      console.error(`Unable to start rrf_simulator: ${err?.message || err}`);
      process.exit(1);
    }
  }

  const bridgeCtx = createGcodeBridge({
    server: targetServer,
    wsPort: args.noWs ? 0 : args.wsPort,
    quiet: args.quiet,
    encoderTimeoutMs,
  });

  const send = async (line, options = {}) => {
    const trimmed = line?.trim?.();
    if (args.stepGcode && trimmed) {
      const normalPreWaitMs = pendingPreSendDelayMs;
      pendingPreSendDelayMs = 0;
      const source = getSourceLineFromStack(new Error().stack, { skipMatches: 0 });
      const sourceLabel = source ? `${source.file}:${source.line}` : SOURCE_FILE_LABEL;
      await waitForEnter(
        `Send: ${trimmed}\n  Enter to send (normal pre-wait ${Math.round(normalPreWaitMs)}ms; from ${sourceLabel}) `,
      );
    } else if (args.debugGcode && trimmed) {
      console.log(`[rrf_gcode] ${trimmed}`);
    }
    const res = await bridgeCtx.sendGcodeLine(line, options);
    if (args.stepGcode) {
      const reply = res?.reply?.trim?.() || '';
      console.log(reply.length > 0 ? `[rrf_reply] ${reply}` : '[rrf_reply] <empty>');
    } else if ((args.debugGcodeResponses || args.debug) && res?.reply) {
      const reply = res.reply.trim();
      if (reply.length > 0) {
        console.log(`[rrf_reply] ${reply}`);
      }
    }
    return res;
  };

  if (!args.noWs) {
    await bridgeCtx.waitForHpSimConnection(waitForWsMs);
    if (!args.noHpSimReset) {
      await sendHpSimReset(bridgeCtx, { quiet: args.quiet });
    }
    if (speedup !== 1) {
      await sendHpSimSpeedScale(bridgeCtx, speedup, { quiet: args.quiet });
    }
    if (args.trace) {
      await sendHpSimPositionTraceMode(bridgeCtx, true, { quiet: args.quiet });
    }
  }

  let success = false;
  try {
    await collectSweepData(send, {
      args,
      machineType,
      machineConfig,
      motorIds,
      speedup,
      delayFn: sleep,
    });
    success = true;
  } catch (err) {
    console.error(`Failed to collect sweeps: ${err?.message || err}`);
  } finally {
    if (rrfProcess && !args.persistRrfSimulator) {
      stopProcess(rrfProcess);
    }
    if (stepReadline) {
      stepReadline.close();
      stepReadline = null;
    }
    bridgeCtx.close();
    process.exit(success ? 0 : 1);
  }
}

export {
  angleToLength,
  combinations,
  generateSweepConfigs,
  loadSweepConfigFile,
  permutations,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from './sweep_data_collection.mjs';

const isMain = import.meta.url === pathToFileURL(process.argv[1] || '').href;
if (isMain) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
