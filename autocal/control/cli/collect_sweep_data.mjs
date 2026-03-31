#!/usr/bin/env node
import path from 'node:path';
import readline from 'node:readline';
import { pathToFileURL } from 'node:url';
import { createGcodeBridge } from '../../../bridges/rrf/gcode_to_rrf_simulator_to_websocket.mjs';
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
import { normalizeMachineType } from '../primitives/machine_type.mjs';
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


export function parseBridgeArgs(argv) {
  const envServer = process.env.RRF_SERVER_URL;
  const args = {
    server: envServer || 'http://localhost:8080',
    serverExplicit: !!envServer,
    wsPort: 8790,
    quiet: false,
    command: null,
    help: false,
    noWs: false,
    stepGcode: false,
    dx: null,
    dy: null,
    feed: null,
    waitWs: null,
    timeout: null,
    debug: false,
    debugGcode: false,
    debugGcodeResponses: false,
    pointsFile: null,
    outputFile: null,
    settleMs: null,
    persistRrfSimulator: false,
    hpSimReset: false,
    noSpawnRrfSimulator: false,
    speedup: null,
    port: null,
    machineType: null,
    sweepRange: null,
    sweepPoints: null,
    maxSweeps: null,
    fixedTargets: null,
    sweepMethod: null,
    maxTravelMm: null,
    sensorForce: null,
    forceLow: null,
    forceMid: null,
    forceMax: null,
    preserveBuildupFactor: false,
    forceBuildupFactor: null,
    forceBaseRadii: null,
    forceStep: null,
    autoTuneForce: false,
    noAutoTuneForce: false,
    torque: null,
    rampWaitMs: null,
    swapWaitMs: null,
    debugSweep: false,
    debugSweepActions: false,
    trace: false,
    sweepConfigFile: null,
    continuous: false,
    sampleRate: null,
    observabilityFile: null,
    returnToOrigin: false,
  };

  for (let i = 0; i < argv.length; i += 1) {
    const arg = argv[i];
    if (arg === '--server' || arg === '--rrf') {
      args.server = argv[++i] || args.server;
      args.serverExplicit = true;
    } else if (arg === '--port') {
      const value = parseInt(argv[++i], 10);
      if (Number.isFinite(value)) {
        args.port = value;
      }
    } else if (arg === '--ws-port') {
      const value = parseInt(argv[++i], 10);
      if (Number.isFinite(value) && value > 0) {
        args.wsPort = value;
      } else {
        args.wsPort = 0;
      }
    } else if (arg === '--cmd' || arg === '-c') {
      args.command = argv[++i] || null;
    } else if (arg === '--quiet' || arg === '-q') {
      args.quiet = true;
    } else if (arg === '--help' || arg === '-h') {
      args.help = true;
    } else if (arg === '--no-ws') {
      args.noWs = true;
    } else if (arg === '--step-gcode' || arg === '--step-gcodes' || arg === '--interactive-gcode') {
      args.stepGcode = true;
    } else if (arg === '--dx') {
      args.dx = argv[++i] || null;
    } else if (arg === '--dy') {
      args.dy = argv[++i] || null;
    } else if (arg === '--feed' || arg === '-f') {
      args.feed = argv[++i] || null;
    } else if (arg === '--wait-ws') {
      args.waitWs = argv[++i] || null;
    } else if (arg === '--timeout') {
      args.timeout = argv[++i] || null;
    } else if (arg === '--debug') {
      args.debug = true;
    } else if (arg === '--debug-gcode' || arg === '--debug-gcodes' || arg === '--trace-gcode') {
      args.debugGcode = true;
    } else if (arg === '--debug-gcode-responses' || arg === '--trace-gcode-responses' || arg === '--debug-gcodes-responses') {
      args.debugGcodeResponses = true;
    } else if (arg === '--points-file' || arg === '--points') {
      args.pointsFile = argv[++i] || null;
    } else if (arg === '--output-file' || arg === '--output' || arg === '--out') {
      args.outputFile = argv[++i] || null;
    } else if (arg === '--settle-ms') {
      args.settleMs = argv[++i] || null;
    } else if (arg === '--persist-rrf-simulator') {
      args.persistRrfSimulator = true;
    } else if (arg === '--hp-sim-reset') {
      args.hpSimReset = true;
    } else if (arg === '--no-spawn-rrf-simulator') {
      args.noSpawnRrfSimulator = true;
    } else if (arg === '--speedup') {
      args.speedup = argv[++i] || null;
    } else if (arg === '--machineType' || arg === '--machine-type') {
      args.machineType = argv[++i] || null;
    } else if (arg === '--sweepRange') {
      args.sweepRange = argv[++i] || null;
    } else if (arg === '--sweepPoints') {
      args.sweepPoints = argv[++i] || null;
    } else if (arg === '--maxSweeps') {
      args.maxSweeps = argv[++i] || null;
    } else if (arg === '--max-travel-mm' || arg === '--max-travel') {
      args.maxTravelMm = argv[++i] || null;
    } else if (arg === '--fixed-targets' || arg === '--fixedTargets' || arg === '--fixed-target') {
      args.fixedTargets = argv[++i] || null;
    } else if (arg === '--sensor-force' || arg === '--force-sensor' || arg === '--sensorForce') {
      args.sensorForce = argv[++i] || null;
    } else if (arg === '--torque') {
      args.torque = argv[++i] || null;
    } else if (arg === '--sweepMethod' || arg === '--sweep-method') {
      args.sweepMethod = argv[++i] || null;
    } else if (arg === '--force-low' || arg === '--forceLow') {
      args.forceLow = argv[++i] || null;
    } else if (arg === '--force-mid' || arg === '--forceMid' || arg === '--force-min' || arg === '--forceMin') {
      args.forceMid = argv[++i] || null;
    } else if (arg === '--force-max' || arg === '--forceMax') {
      args.forceMax = argv[++i] || null;
    } else if (arg === '--preserve-buildup-factor' || arg === '--preserveBuildupFactor') {
      args.preserveBuildupFactor = true;
    } else if (arg === '--force-buildup-factor' || arg === '--forceBuildupFactor') {
      args.forceBuildupFactor = argv[++i] || null;
    } else if (arg === '--force-base-radii' || arg === '--forceBaseRadii' || arg === '--base-radii') {
      args.forceBaseRadii = argv[++i] || null;
    } else if (arg === '--force-step' || arg === '--forceStep') {
      args.forceStep = argv[++i] || null;
    } else if (arg === '--auto-tune-force' || arg === '--autoTuneForce') {
      args.autoTuneForce = true;
    } else if (arg === '--no-auto-tune-force' || arg === '--noAutoTuneForce') {
      args.noAutoTuneForce = true;
    } else if (arg === '--torque-low') {
      args.forceLow = argv[++i] || null;
    } else if (arg === '--torque-min') {
      args.forceMid = argv[++i] || null;
    } else if (arg === '--torque-max') {
      args.forceMax = argv[++i] || null;
    } else if (arg === '--torque-step') {
      args.forceStep = argv[++i] || null;
    } else if (arg === '--auto-tune-torque' || arg === '--autoTuneTorque') {
      args.autoTuneForce = true;
    } else if (arg === '--no-auto-tune-torque' || arg === '--noAutoTuneTorque') {
      args.noAutoTuneForce = true;
    } else if (arg === '--ramp-wait-ms') {
      args.rampWaitMs = argv[++i] || null;
    } else if (arg === '--swap-wait-ms') {
      args.swapWaitMs = argv[++i] || null;
    } else if (arg === '--debug-sweep' || arg === '--debugSweep') {
      args.debugSweep = true;
    } else if (arg === '--debug-sweep-actions') {
      args.debugSweepActions = true;
    } else if (arg === '--trace' || arg === '--trace-positions') {
      args.trace = true;
    } else if (arg === '--sweep-config-file' || arg === '--sweep-config' || arg === '--sweepFile') {
      args.sweepConfigFile = argv[++i] || null;
    } else if (arg === '--continuous') {
      args.continuous = true;
    } else if (arg === '--sample-rate' || arg === '--sampleRate') {
      args.sampleRate = argv[++i] || null;
    } else if (arg === '--observability-file' || arg === '--obs-file') {
      args.observabilityFile = argv[++i] || null;
    } else if (arg === '--return-to-origin' || arg === '--returnToOrigin') {
      args.returnToOrigin = true;
    } else if (arg === '--project-zero-tension') {
      args.projectZeroTension = true;
    }
  }

  return args;
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
  --preserve-buildup-factor  Keep current M666 Q (default behavior when no Q override is provided)
  --force-buildup-factor <k> Force M666 Q to this value before collection
  --force-base-radii <csv>   Force M666 R radii before collection (comma-separated, e.g. 30,30,30)
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
  const machineType = normalizeMachineType(args.machineType || 'slideprinter');
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
      rrfProcess = await startRrfSimulator({ port: targetPort, debug: args.debug, machineType });
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
