// Launcher-owned RRF bridge. Reuse the production collector and firmware planner.
import http from 'node:http';
import fs from 'node:fs/promises';
import { createGcodeBridge } from '../integrations/rrf/rrfSimulatorBridge.mjs';
import { collectSweepData, MACHINE_CONFIGS, MOTOR_IDS_BY_MACHINE } from '../autocal/control/behaviors/sweep_data_collection.mjs';
import { waitForRrfSimulator } from '../autocal/control/primitives/encoder_utils.mjs';

const [rrfUrl, wsPort, apiPort, runtimeUrl] = process.argv.slice(2);
const token = process.env.HP_SIM5_RUNTIME_TOKEN;
await waitForRrfSimulator(rrfUrl, 30000);
const bridge = createGcodeBridge({ server: rrfUrl, wsPort: Number(wsPort), quiet: true, encoderTimeoutMs: 120000 });
const firmware = await bridge.sendGcodeLine('M115');
if (!/RepRapFirmware/i.test(firmware.reply ?? '')) throw new Error('RRF did not identify its firmware after readiness');
let simTimeMs = 0;
let busy = false;
let cancellation = new AbortController();
let partialWrite = Promise.resolve();
function checkCancellation() {
  if (cancellation.signal.aborted) throw new Error('Collection cancelled at collector command boundary');
}
async function cancellable(task) {
  checkCancellation();
  let onAbort;
  const signal = cancellation.signal;
  const aborted = new Promise((_, reject) => {
    onAbort = () => reject(new Error('Collection cancelled at collector command boundary'));
    signal.addEventListener('abort', onAbort, { once: true });
  });
  try { return await Promise.race([task(), aborted]); }
  finally { signal.removeEventListener('abort', onAbort); }
}

async function runtime(operation, args = {}) {
  const response = await fetch(`${runtimeUrl}/${operation}`, {
    signal: cancellation.signal,
    method: 'POST', headers: { Authorization: `Bearer ${token}` }, body: JSON.stringify(args),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  simTimeMs = result.collector_time_s * 1000;
  return result;
}

const send = async (line) => {
  checkCancellation();
  if (/^M569\.3\b/i.test(line)) {
    await cancellable(() => bridge.sendEncoderRequest(['A', 'B', 'C', 'D'], 120000));
  }
  const result = await cancellable(() => bridge.sendGcodeLine(line, { timeout: 120000 }));
  if (/^Error:/im.test(result?.reply ?? '')) throw new Error(result.reply);
  // Empty-axis encoder request acknowledges all preceding WebSocket payloads.
  // Real encoder requests drain motion in Python before answering.
  await cancellable(() => bridge.sendEncoderRequest([], 120000));
  await runtime('clock');
  return result;
};
send.firmware = 'rrf';
send.simulationClock = {
  now: () => simTimeMs,
  sleep: (ms) => runtime('advance', { seconds: ms / 1000 }),
  settlingTimeoutMs: 30000,
};

const server = http.createServer(async (request, response) => {
  response.setHeader('Content-Type', 'application/json');
  if (request.headers.authorization !== `Bearer ${token}`) {
    response.writeHead(403).end(JSON.stringify({ error: 'Unauthorized' }));
    return;
  }
  if (request.url === '/cancel') {
    cancellation.abort();
    await partialWrite;
    response.end(JSON.stringify({ cancelled: true, boundary: 'before next collector command' }));
    return;
  }
  if (request.url === '/status') {
    response.end(JSON.stringify({ connected: bridge.hasReadyWsClients(), busy }));
    return;
  }
  if (busy) {
    response.writeHead(409).end(JSON.stringify({ error: 'Collector is busy' }));
    return;
  }
  busy = true;
  try {
    let body = '';
    for await (const chunk of request) body += chunk;
    const args = JSON.parse(body);
    if (request.url === '/gcode') {
      response.end(JSON.stringify(await send(args.line)));
    } else if (request.url === '/collect') {
      cancellation = new AbortController();
      await fs.writeFile(args.sweepConfigFile, args.configs.map((cfg) => `[${cfg.fixed.join(',')}] ${cfg.drive} ${cfg.sensor}\n`).join(''));
      const collectorArgs = {
        sim: true, firmware: 'rrf', machineType: 'hangprinter_4',
        noAutoTuneForce: true, fixedTargets: '0,0', forceLow: .01, forceMid: .05, forceMax: .5,
        sensorCollectionForce: 1,
        ...args.options,
        outputFile: args.outputFile, sweepConfigFile: args.sweepConfigFile,
      };
      send.simulationClock.settlingTimeoutMs = args.settlingTimeoutMs;
      const result = await collectSweepData(send, {
        args: collectorArgs, machineType: 'hangprinter_4', machineConfig: MACHINE_CONFIGS.hangprinter_4,
        motorIds: MOTOR_IDS_BY_MACHINE.hangprinter_4, speedup: 1,
        delayFn: send.simulationClock.sleep,
        onPoint: async (point, config) => {
          checkCancellation();
          partialWrite = fs.appendFile(args.partialFile, `${JSON.stringify({ backend: args.backend, config, point })}\n`);
          await partialWrite;
        },
      });
      response.end(JSON.stringify(result));
    } else {
      throw new Error('Unknown operation');
    }
  } catch (error) {
    response.writeHead(500).end(JSON.stringify({ error: error.message }));
  } finally {
    busy = false;
  }
});
server.listen(Number(apiPort), '127.0.0.1');
