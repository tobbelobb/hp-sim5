#!/usr/bin/env node
import { parseBridgeArgs } from '../../cli/collect_sweep_data.mjs';
import { createGcodeBridge } from '../../../../integrations/rrf/rrfSimulatorBridge.mjs';
import {
  DEFAULT_RRF_PORT,
  sendHpSimReset,
  sendHpSimSpeedScale,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from '../../primitives/encoder_utils.mjs';
import { normalizeMachineType } from '../../primitives/machine_type.mjs';
import { calibrateEncoderNoise } from '../../behaviors/force_tuning.mjs';
import { MACHINE_CONFIGS, MOTOR_IDS_BY_MACHINE } from '../../behaviors/sweep_data_collection.mjs';

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
  console.log(`Usage: node autocal/control/tests/e2e/calibrate_encoder_noise.e2e.test.mjs [options]

Runs an end-to-end demo that samples encoder noise and prints the result.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --force <N>                Idle force applied during sampling (default: 0.02)
  --fixed-anchor <index>     Anchor index to keep fixed in position mode (default: none)
  --sample-ms <ms>           Total sample duration (default: 4000)
  --sample-interval-ms <ms>  Sample interval (default: 200)
  --speedup <scale>          Speed scale passed to waitForStableEncoders (default: 1)
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
  const idleForce = parseNumberArg(argv, '--force', 0.02);
  const fixedAnchor = parseIntegerArg(argv, '--fixed-anchor', null);
  const sampleDurationMs = parseNumberArg(argv, '--sample-ms', 4000);
  const sampleIntervalMs = parseNumberArg(argv, '--sample-interval-ms', 200);
  const speedup = Number.isFinite(parseFloat(args.speedup)) && parseFloat(args.speedup) > 0
    ? parseFloat(args.speedup)
    : 1;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : 0;

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
    if (args.hpSimReset) {
      await sendHpSimReset(bridgeCtx, { quiet: args.quiet });
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
    const fixedAnchorIdx = Number.isFinite(fixedAnchor) ? fixedAnchor : null;
    console.log(`Calibrating encoder noise (idle force ${idleForce} N)...`);
    const noise = await calibrateEncoderNoise(send, {
      motorIds,
      fixedAnchor: fixedAnchorIdx,
      idleForce,
      speedup,
      sampleDurationMs,
      sampleIntervalMs,
      forbiddenForceAnchors: machineConfig.mustBeInFixedSet,
    });

    console.log(JSON.stringify(noise, null, 2));
    if (Array.isArray(noise.sigmaByMotorDeg)) {
      console.log('Noise sigma per motor (deg):');
      noise.sigmaByMotorDeg.forEach((value, idx) => {
        const motorId = motorIds[idx] ?? `motor_${idx}`;
        const display = Number.isFinite(value) ? value.toFixed(4) : 'n/a';
        console.log(`  ${motorId}: ${display}`);
      });
    }

    success = true;
  } catch (err) {
    console.error(`E2E calibrate-encoder-noise failed: ${err?.message || err}`);
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
