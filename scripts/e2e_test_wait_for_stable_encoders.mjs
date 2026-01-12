#!/usr/bin/env node
import { parseBridgeArgs, createGcodeBridge } from './gcode_bridge.mjs';
import {
  DEFAULT_RRF_PORT,
  sendHpSimSpeedScale,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from './encoder_utils.mjs';
import { applyForceModeState, waitForStableEncoders } from './uncalibrated_actions.mjs';
import { MACHINE_CONFIGS, MOTOR_IDS_BY_MACHINE } from './sweep_data_collection.mjs';

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

function printHelp() {
  console.log(`Usage: node scripts/e2e_test_wait_for_stable_encoders.mjs [options]

Runs an end-to-end demo that applies force state, then waits for stable encoders.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --force <N>                Force applied to motor 40.0 in applyForceState (default: 2.0)
  --speedup <scale>          Speed scale passed to waitForStableEncoders (default: 1)
  --stable-window-ms <ms>    Stable window for encoder settle (default: 500)
  --poll-ms <ms>             Poll interval for encoder settle (default: 100)
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
  const force = parseNumberArg(argv, '--force', 2.0);
  const stableWindowMs = parseNumberArg(argv, '--stable-window-ms', 500);
  const pollIntervalMs = parseNumberArg(argv, '--poll-ms', 100);
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
    console.log(`Applying force state (${force} N on motor 40.0) on ${motorIds.length} motors...`);
    await applyForceModeState(send, {
      motorIds,
      modes: motorIds.map((_, idx) => (idx === 0 ? force : 0.001))
    });

    console.log(`Waiting for stable encoders (${stableWindowMs}ms window, ${pollIntervalMs}ms poll)...`);
    const stable = await waitForStableEncoders(send, motorIds, {
      speedup,
      stableWindowMs,
      pollIntervalMs,
    });

    console.log(`Stable after ${stable.elapsedMs}ms with ${stable.samples} samples.`);
    console.log(`Angles (deg): ${stable.anglesDeg.map((val) => val.toFixed(3)).join(', ')}`);
    success = true;
  } catch (err) {
    console.error(`E2E wait-for-stable-encoders failed: ${err?.message || err}`);
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
