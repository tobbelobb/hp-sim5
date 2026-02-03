#!/usr/bin/env node
import path from 'node:path';
import readline from 'node:readline';
import { pathToFileURL } from 'node:url';
import { createGcodeBridge, parseBridgeArgs } from '../primitives/gcode_bridge.mjs';
import { attachDebugState } from '../primitives/debug_trace.mjs';
import {
  DEFAULT_RRF_PORT,
  sendHpSimPositionTraceMode,
  sendHpSimReset,
  sendHpSimSpeedScale,
  sleep as baseSleep,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from '../primitives/encoder_utils.mjs';
import {
  collectSweepData,
  MACHINE_CONFIGS,
  MOTOR_IDS_BY_MACHINE,
  SWEEP_DEFAULTS,
} from '../behaviors/sweep_data_collection.mjs';
import { FORCE_TUNING_DEFAULTS } from '../behaviors/force_tuning.mjs';

const SOURCE_FILE_LABEL = 'autocal/control/cli/collect_sweep_data.mjs';
let stepGcodeMode = false;
let pendingPreSendDelayMs = 0;
let stepReadline = null;

function getSourceLineFromStack(stack, { skipMatches = 0 } = {}) {
  if (!stack) {
    return null;
  }
  const matches = [...stack.matchAll(/((?:scripts|autocal\/control)\/[^:\n]+\.mjs):(\d+):\d+/g)];
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
  console.log(`Usage: node autocal/control/cli/collect_sweep_data.mjs [options]

Collect circular sweep data where N-1 anchors are fixed, one anchor drives, and another is in force/sensor mode.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --sweepPoints <count>      Number of points per sweep (default: ${SWEEP_DEFAULTS.DEFAULT_SWEEP_POINTS})
  --fixed-targets <csv>      Fixed-anchor target deltas in mm (comma-separated)
  --max-travel-mm <mm>       Fixed-anchor delta applied to all fixed anchors
  --maxSweeps <count>        Max sweeps when auto-generating configs (default: ${SWEEP_DEFAULTS.DEFAULT_MAX_SWEEPS})
  --feed <mm/min>            Feed rate for drive moves (default: ${SWEEP_DEFAULTS.DEFAULT_FEED})
  --sensor-force <N>         Deprecated (sensor motor uses force-low)
  --settleMs <ms>            Deprecated (was fixed settle time; now waits for encoder stability)
  --speedup <scale>          hp-sim speed scale (default: 1)
  --continuous               Deprecated (force-drive sweeps only)
  --sample-rate <Hz>         Deprecated (continuous mode removed)
  --force-low <N>            idle force (default: ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_LOW_N})
  --force-mid <N>            start force (default: ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MID_N})
  --force-max <N>            end force (default: ${FORCE_TUNING_DEFAULTS.DEFAULT_FORCE_MAX_N})
  --auto-tune-force          auto-tune force-low/mid/max using a test sweep (default when no force args)
  --no-auto-tune-force       skip auto-tuning and use provided/default forces
  --sweep-config-file <file> Provide explicit sweep configs ([fixed] drive sensor per line)
  --debug-sweep              Print planned sweep permutations before collecting
  --trace                    Tell hp-sim to plot a trace of its movements (default: on)
  --no-trace                 Disable hp-sim trace plotting
  --project-zero-tension     Project encoder readings to zero tension during each data point
  --output-file <path>       Output JSON path (default: sweep_data_<machine>_<timestamp>.json)
  --observability-file <path> Sidecar histogram JSON path (default: <output>.obs.json)
  --server, --rrf <url>      RRF server URL (default: http://localhost:${DEFAULT_RRF_PORT})
  --port <port>              Port for spawned rrf_simulator (default: ${DEFAULT_RRF_PORT})
  --no-spawn-rrf-simulator   Do not start rrf_simulator automatically
  --no-ws                    Disable hp-sim websocket bridge
  --ws-port <port>           WebSocket port (default: 8790)
  --hp-sim-reset             Reset hp-sim on start
  --wait-ws <ms>             Wait for hp-sim websocket connection (default: 0)
  --debug                    Verbose logging (includes G-code replies)
  --debug-gcode              Echo sent G-code
  --debug-gcode-responses    Echo G-code responses
  --debug-sweep-actions      Print sweep debug traces (force modes, waits, moves, data points)
  --step-gcode               Pause before each G-code; press Enter to send (prints source line + skipped waits)
  --return-to-origin         Return all motors to encoder origin between sweeps and after collection

Examples:
  node autocal/control/cli/collect_sweep_data.mjs --machineType slideprinter --sweepPoints 41 --output-file sweep.json
  node autocal/control/cli/collect_sweep_data.mjs --machineType hangprinter_4 --sweep-config-file autocal/sweep_configs/hangprinter_4.txt --debug-sweep`);
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
  const debugSweepActions = !!args.debugSweepActions;
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
  attachDebugState(send, {
    enabled: debugSweepActions,
    axes: machineConfig.axes,
    motorIds,
  });

  if (!args.noWs) {
    await bridgeCtx.waitForHpSimConnection(waitForWsMs);
    if (args.hpSimReset) {
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
  combinations,
  generateSweepConfigs,
  loadSweepConfigFile,
  permutations,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from '../behaviors/sweep_data_collection.mjs';

export {
  angleToLength,
} from '../primitives/uncalibrated_actions.mjs';

const isMain = import.meta.url === pathToFileURL(process.argv[1] || '').href;
if (isMain) {
  main().catch((err) => {
    console.error(err);
    process.exit(1);
  });
}
