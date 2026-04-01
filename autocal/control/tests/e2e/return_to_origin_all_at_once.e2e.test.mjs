#!/usr/bin/env node
import { parseBridgeArgs } from '../../cli/collect_sweep_data.mjs';
import { createGcodeBridge } from '../../../../integrations/rrf/rrfSimulatorBridge.mjs';
import {
  DEFAULT_RRF_PORT,
  computeMmPerDegree,
  parseM666,
  sendHpSimSpeedScale,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from '../../primitives/encoder_utils.mjs';
import { normalizeMachineType } from '../../primitives/machine_type.mjs';
import { applyForceModeState, returnMotorsToOriginOneAtATime, returnMotorsToOriginAllAtOnce, waitForStableEncoders } from '../../primitives/uncalibrated_actions.mjs';
import { MACHINE_CONFIGS, MOTOR_IDS_BY_MACHINE, SWEEP_DEFAULTS } from '../../behaviors/sweep_data_collection.mjs';

const LOW_FORCE_N = 0.001;

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
  console.log(`Usage: node autocal/control/tests/e2e/return_to_origin_all_at_once.e2e.test.mjs [options]

Runs an end-to-end demo that applies force state, waits for stable encoders,
then returns motors to origin. First one at a time, then all at once.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --force <N>                Force applied to motor 42.0 in applyForceState (default: 2.0)
  --speedup <scale>          Speed scale passed to waitForStableEncoders (default: 1)
  --feed <mm/min>            Feed rate for return moves (default: ${SWEEP_DEFAULTS.DEFAULT_FEED})
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
  const feed = Number.isFinite(parseFloat(args.feed)) ? parseFloat(args.feed) : SWEEP_DEFAULTS.DEFAULT_FEED;
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

    console.log(`Applying force state (40.0 position, 42.0 ${force} N, others ${LOW_FORCE_N} N)...`);
    await applyForceModeState(send, {
      motorIds,
      modes: motorIds.map((id) => {
        if (id === '40.0') {
          return 'position';
        }
        if (id === '42.0') {
          return force;
        }
        return LOW_FORCE_N;
      })
    });

    console.log(`Waiting for stable encoders (${stableWindowMs}ms window, ${pollIntervalMs}ms poll)...`);
    await waitForStableEncoders(send, motorIds, {
      speedup,
      stableWindowMs,
      pollIntervalMs,
    });

    console.log('Returning motors to origin all at once...');
    let lengths = await returnMotorsToOriginAllAtOnce(send, {
      motorIds,
      axes: machineConfig.axes,
      mmPerDeg,
      feed,
      speedup,
      settleOptions: {
        stableWindowMs,
        pollIntervalMs,
      },
    });
    console.log(lengths);
    if (lengths.some(x => Math.abs(x) > 0.5)) {
      console.log('Returning motors to origin all at once...');
      lengths = await returnMotorsToOriginAllAtOnce(send, {
        motorIds,
        axes: machineConfig.axes,
        mmPerDeg,
        feed,
        speedup,
        settleOptions: {
          stableWindowMs,
          pollIntervalMs,
        },
      });
      console.log(lengths);
    }

    if (lengths.every(x => Math.abs(x) < 1.0)) {
      success = true;
    } else {
      success = false;
    }
  } catch (err) {
    console.error(`E2E return-to-origin failed: ${err?.message || err}`);
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
