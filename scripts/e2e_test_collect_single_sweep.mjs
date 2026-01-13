#!/usr/bin/env node
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { parseBridgeArgs, createGcodeBridge } from './gcode_bridge.mjs';
import {
  DEFAULT_RRF_PORT,
  sendHpSimSpeedScale,
  startRrfSimulator,
  stopProcess,
  waitForRrfSimulator,
} from './encoder_utils.mjs';
import { collectSweepData, MACHINE_CONFIGS, MOTOR_IDS_BY_MACHINE } from './sweep_data_collection.mjs';
import { sleep as baseSleep } from './encoder_utils.mjs';

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

function parseListArg(argv, flag) {
  const idx = argv.indexOf(flag);
  if (idx >= 0 && idx < argv.length - 1) {
    const raw = argv[idx + 1];
    if (typeof raw === 'string') {
      const values = raw
        .split(',')
        .map((part) => parseInt(part.trim(), 10))
        .filter((val) => Number.isFinite(val));
      return values.length > 0 ? values : null;
    }
  }
  return null;
}

function printHelp() {
  console.log(`Usage: node scripts/e2e_test_collect_single_sweep.mjs [options]

Runs a single sweep collection end-to-end using collect_sweep_data.mjs logic.

Options:
  --help, -h                 Show this help and exit
  --machineType <name>       Machine type: slideprinter | hangprinter_4 | hangprinter_5 | cubecorners | skycam (default: slideprinter)
  --drive-anchor <index>     Anchor index to drive (default: 0)
  --sensor-anchor <index>    Anchor index to use as sensor force (default: first non-drive non-fixed)
  --fixed-anchors <list>     Comma-separated fixed anchors (default: first non-drive; others are sensors)
  --sweep-config-file <file> Use explicit sweep config file instead of generated one
  --max-travel-mm <spec>     Override fixed-anchor targets (single value or list spec)
  --sweepPoints <count>      Number of points per sweep (default: 21)
  --speedup <scale>          hp-sim speed scale (default: 1)
  --feed <mm/min>            Feed rate for drive moves (default: 1400)
  --sensor-force <N>         Deprecated (sensor motor uses force-low)
  --force-low <N>            idle force (default: 0.01)
  --force-mid <N>            start force (default: 0.01)
  --force-max <N>            end force (default: 0.1)
  --auto-tune-force          auto-tune force-low/mid/max (default when no force args)
  --no-auto-tune-force       skip auto-tuning and use provided/default forces
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

  const driveAnchor = parseIntegerArg(argv, '--drive-anchor', 0);
  if (!Number.isFinite(driveAnchor) || driveAnchor < 0 || driveAnchor >= motorIds.length) {
    console.error('Invalid --drive-anchor');
    process.exit(1);
  }

  const sensorAnchorArg = parseIntegerArg(argv, '--sensor-anchor', null);
  let fixedAnchors = parseListArg(argv, '--fixed-anchors');
  if (!fixedAnchors) {
    const defaultFixed = motorIds.findIndex(
      (_, idx) => idx !== driveAnchor && (!Number.isFinite(sensorAnchorArg) || idx !== sensorAnchorArg),
    );
    if (!Number.isFinite(defaultFixed) || defaultFixed < 0) {
      console.error('Unable to select a default fixed anchor');
      process.exit(1);
    }
    fixedAnchors = [defaultFixed];
  }
  fixedAnchors = [...new Set(fixedAnchors)];
  if (fixedAnchors.some((idx) => !Number.isFinite(idx) || idx < 0 || idx >= motorIds.length)) {
    console.error('Invalid --fixed-anchors');
    process.exit(1);
  }
  if (fixedAnchors.includes(driveAnchor)) {
    console.error('Invalid --fixed-anchors (includes drive anchor)');
    process.exit(1);
  }
  if (fixedAnchors.length > machineConfig.numAnchors - 2) {
    console.error('Invalid --fixed-anchors (must leave room for drive and sensor)');
    process.exit(1);
  }

  const forbidden = new Set(machineConfig.forbiddenSensors ?? []);
  let sensorAnchor = sensorAnchorArg;
  if (!Number.isFinite(sensorAnchor)) {
    sensorAnchor = motorIds.findIndex(
      (_, idx) => idx !== driveAnchor && !fixedAnchors.includes(idx) && !forbidden.has(idx),
    );
  }
  if (!Number.isFinite(sensorAnchor) || sensorAnchor < 0 || sensorAnchor >= motorIds.length) {
    console.error('Invalid --sensor-anchor');
    process.exit(1);
  }
  if (sensorAnchor === driveAnchor || fixedAnchors.includes(sensorAnchor)) {
    console.error('Invalid --sensor-anchor (must be distinct from drive/fixed anchors)');
    process.exit(1);
  }
  if (forbidden.has(sensorAnchor)) {
    console.error('Invalid --sensor-anchor (forbidden sensor anchor)');
    process.exit(1);
  }

  const targetPort = Number.isFinite(parseInt(args.port, 10)) ? parseInt(args.port, 10) : DEFAULT_RRF_PORT;
  const targetServer = args.serverExplicit ? args.server : `http://localhost:${targetPort}`;
  const shouldSpawnRrf = !args.noSpawnRrfSimulator && !args.serverExplicit;
  let rrfProcess = null;
  let tempSweepCfg = null;
  let tempSweepDir = null;

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

  if (!args.sweepConfigFile) {
    tempSweepDir = await fs.mkdtemp(path.join(os.tmpdir(), 'hp-sim5-sweep-'));
    tempSweepCfg = path.join(tempSweepDir, 'single_sweep_cfg.txt');
    const fixedSpec = fixedAnchors.join(',');
    await fs.writeFile(tempSweepCfg, `${fixedSpec} ${driveAnchor} ${sensorAnchor}\n`, 'utf8');
    args.sweepConfigFile = tempSweepCfg;
  }

  const isSimulation = argv.includes('--sim') || argv.includes('--simulation');
  const useWs = isSimulation || argv.includes('--ws');
  const speedup = Number.isFinite(parseFloat(args.speedup)) && parseFloat(args.speedup) > 0
    ? parseFloat(args.speedup)
    : 1;
  const encoderTimeoutMs = Number.isFinite(parseFloat(args.timeout)) ? parseFloat(args.timeout) : undefined;
  const waitForWsMs = Number.isFinite(parseFloat(args.waitWs)) ? parseFloat(args.waitWs) : 0;

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
    await collectSweepData(send, {
      args,
      machineType,
      machineConfig,
      motorIds,
      speedup,
      delayFn: baseSleep,
    });
    success = true;
  } catch (err) {
    console.error(`E2E collect-single-sweep failed: ${err?.message || err}`);
  } finally {
    if (rrfProcess && !args.persistRrfSimulator) {
      stopProcess(rrfProcess);
    }
    bridgeCtx.close();
    if (tempSweepCfg) {
      try {
        await fs.unlink(tempSweepCfg);
      } catch (_err) {
        /* ignore */
      }
    }
    if (tempSweepDir) {
      try {
        await fs.rmdir(tempSweepDir);
      } catch (_err) {
        /* ignore */
      }
    }
    process.exit(success ? 0 : 1);
  }
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
