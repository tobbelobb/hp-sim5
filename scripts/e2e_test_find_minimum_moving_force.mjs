#!/usr/bin/env node
import { parseBridgeArgs, createGcodeBridge } from './gcode_bridge.mjs';
import {
  DEFAULT_RRF_PORT,
  computeMmPerDegree,
  parseM666,
  sendHpSimSpeedScale,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from './encoder_utils.mjs';
import {
  buildForceRampValues,
  buildMovementThresholds,
  calibrateEncoderNoise,
  computeStallSpeedThresholdDegPerSec,
  findMinimumMovingForce,
  FORCE_TUNING_CONSTANTS,
  runForceTrial,
} from './force_tuning.mjs';
import { MACHINE_CONFIGS, MOTOR_IDS_BY_MACHINE } from './sweep_data_collection.mjs';
import { applyForceModeState, primeEncoders } from './uncalibrated_actions.mjs';

function parseNumberArg(argv, flag, fallback) {
  const idx = argv.indexOf(flag);
  if (idx >= 0 && idx < argv.length - 1) {
    const value = parseFloat(argv[idx + 1]);
    if (Number.isFinite(value)) {
      return value;
    }
  }
  return fallback;
}

function parseIntegerArg(argv, flag, fallback) {
  const idx = argv.indexOf(flag);
  if (idx >= 0 && idx < argv.length - 1) {
    const value = parseInt(argv[idx + 1], 10);
    if (Number.isFinite(value)) {
      return value;
    }
  }
  return fallback;
}

function printHelp() {
  console.log(`Usage: node scripts/e2e_test_find_minimum_moving_force.mjs [options]

Runs an end-to-end demo that finds the minimum force that triggers movement.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --drive-anchor <index>     Anchor index to drive (default: 0)
  --fixed-anchor <index>     Anchor index to keep fixed in position mode (default: first non-drive)
  --force-low <N>            Idle force for non-driven anchors (default: 0.02)
  --force-cap <N>            Max force to scan up to (default: 20.0)
  --speedup <scale>          Speed scale passed to waitForStableEncoders (default: 1)
  --feed <mm/min>            Feed rate for return moves (default: 1400)
  --sim, --simulation        Query hp-sim for encoder angles (auto-enables WebSocket)
  --ws                       Enable hp-sim websocket bridge
  --server, --rrf <url>      RRF server URL (default: http://localhost:${DEFAULT_RRF_PORT})
  --port <port>              Port for spawned rrf_simulator (default: ${DEFAULT_RRF_PORT})
  --no-spawn-rrf-simulator   Do not start rrf_simulator automatically
  --debug                    Verbose logging (includes G-code replies)
  --debug-gcode              Echo sent G-code`);
}

async function main() {
  const argv = process.argv.slice(2);
  const args = parseBridgeArgs(argv);
  if (args.help) {
    printHelp();
    process.exit(0);
  }

  const isSimulation = argv.includes('--sim') || argv.includes('--simulation');
  const idleForce = parseNumberArg(argv, '--force-low', 0.02);
  const capForceLimit = parseNumberArg(argv, '--force-cap', 20.0);
  const driveAnchor = parseIntegerArg(argv, '--drive-anchor', 0);
  const fixedAnchorArg = parseIntegerArg(argv, '--fixed-anchor', null);
  const feed = Number.isFinite(parseFloat(args.feed)) ? parseFloat(args.feed) : 1400;
  const speedup = Number.isFinite(parseFloat(args.speedup)) && parseFloat(args.speedup) > 0
    ? parseFloat(args.speedup)
    : 1;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : 0;

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

  if (!Number.isFinite(driveAnchor) || driveAnchor < 0 || driveAnchor >= motorIds.length) {
    console.error('Invalid --drive-anchor');
    process.exit(1);
  }

  let fixedAnchor = fixedAnchorArg;
  if (!Number.isFinite(fixedAnchor)) {
    const forbidden = new Set(machineConfig.forbiddenSensors ?? []);
    fixedAnchor = motorIds.findIndex((_, idx) => idx !== driveAnchor && forbidden.has(idx));
    if (fixedAnchor < 0) {
      fixedAnchor = motorIds.findIndex((_, idx) => idx !== driveAnchor);
    }
  }
  if (!Number.isFinite(fixedAnchor) || fixedAnchor < 0 || fixedAnchor >= motorIds.length || fixedAnchor === driveAnchor) {
    console.error('Invalid --fixed-anchor');
    process.exit(1);
  }

  const restAnchors = motorIds
    .map((_, idx) => idx)
    .filter((idx) => idx !== driveAnchor && idx !== fixedAnchor);
  if (restAnchors.length === 0) {
    console.error('Need at least one non-fixed anchor');
    process.exit(1);
  }

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

  const useWs = isSimulation || argv.includes('--ws');
  const bridgeCtx = createGcodeBridge({
    server: targetServer,
    wsPort: useWs ? args.wsPort : 0,
    quiet: args.quiet,
    encoderTimeoutMs,
  });

  if (useWs) {
    await bridgeCtx.waitForHpSimConnection(waitForWsMs);
    if (isSimulation && bridgeCtx.getReadyWsClients().length === 0) {
      throw new Error('hp-sim encoder bridge not connected (use --wait-ws or check ws port).');
    }
    if (isSimulation && speedup !== 1) {
      await sendHpSimSpeedScale(bridgeCtx, speedup, { quiet: args.quiet });
    }
  }

  const send = async (line, options = {}) => {
    const trimmed = line?.trim?.();
    if (args.debugGcode && trimmed) {
      console.log(`[rrf_gcode] ${trimmed}`);
    }
    const res = await bridgeCtx.sendGcodeLine(line, options);
    if ((args.debug || args.debugGcodeResponses) && res?.reply) {
      const reply = res.reply.trim();
      if (reply.length > 0) {
        console.log(`[rrf_reply] ${reply}`);
      }
    }
    return res;
  };

  let success = false;
  try {
    const m666Reply = await send('M666');
    const m666Values = parseM666(m666Reply?.reply);
    const mmPerDeg = machineConfig.axes.map((_, idx) => computeMmPerDegree(m666Values, idx));

    await primeEncoders(send, { motorIds, axes: machineConfig.axes });
    await applyForceModeState(send, {
      motorIds,
      modes: motorIds.map(() => idleForce),
    });

    const noiseStats = await calibrateEncoderNoise(send, {
      motorIds,
      fixedAnchor,
      idleForce,
      speedup,
      forbiddenForceAnchors: machineConfig.forbiddenSensors,
    });
    const thresholds = buildMovementThresholds(noiseStats.sigmaByMotorDeg, {
      activeAnchor: driveAnchor,
      restAnchors,
    });

    const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
    const intervalMs = Math.max(20, FORCE_TUNING_CONSTANTS.AUTO_TUNE_SAMPLE_INTERVAL_MS / timeScale);
    const stallSpeedDegPerSec = computeStallSpeedThresholdDegPerSec(thresholds.sigmaAct, intervalMs / 1000);

    const formatValue = (value, digits = 4) => (Number.isFinite(value) ? value.toFixed(digits) : 'n/a');
    const logTrial = (label, force, result) => {
      const travel = formatValue(result?.travelDeg, 3);
      console.log(`; min-force ${label}: test=${formatValue(force)} moved=${result?.moved ? 'yes' : 'no'} travel=${travel}deg stalled=${result?.stalled ? 'yes' : 'no'}`);
    };

    const runTrial = async (force, label, trialOptions = {}) => {
      const result = await runForceTrial(send, {
        motorIds,
        activeAnchor: driveAnchor,
        fixedAnchor,
        restAnchors,
        idleForce,
        testForce: force,
        speedup,
        thresholds,
        axes: machineConfig.axes,
        mmPerDeg,
        feed,
        forbiddenForceAnchors: machineConfig.forbiddenSensors,
      });
      if (label) {
        logTrial(label, force, result);
      }
      return result;
    };

    const result = await findMinimumMovingForce(send, {
      motorIds,
      axes: machineConfig.axes,
      mmPerDeg,
      feed,
      speedup,
      baseLow: idleForce,
      capForceLimit,
      trialFn: runTrial,
    });

    console.log(`Minimum moving force: ${formatValue(result.forceStart)} N`);
    console.log(JSON.stringify(result, null, 2));
    success = Number.isFinite(result.forceStart);
  } catch (err) {
    console.error(`E2E find-minimum-moving-force failed: ${err?.message || err}`);
  } finally {
    if (rrfProcess && !args.persistRrfSimulator) {
      stopProcess(rrfProcess);
    }
    bridgeCtx.close();
    process.exit(success ? 0 : 1);
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
