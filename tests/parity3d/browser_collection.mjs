// Independent collection through Chromium's production command/encoder endpoint.
// Both backends use a controlled fixed clock, so throughput cannot change settling.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { createServer } from 'vite';
import puppeteer from 'puppeteer';
import { createViteServerOptions } from '../viteServerOptions.mjs';
import { createGcodeBridge } from '../../integrations/rrf/rrfSimulatorBridge.mjs';
import { waitForRrfSimulator } from '../../autocal/control/primitives/encoder_utils.mjs';

const [headlessDataset, outputDir] = process.argv.slice(2);
if (!outputDir) throw new Error('Usage: node tests/parity3d/browser_collection.mjs HEADLESS_BOOTSTRAP_DATASET NEW_OUTPUT_DIR');
fs.mkdirSync(outputDir, { recursive: true });
const root = process.cwd();
let vite, browser, server, bridge, rrf, headlessRrf, headlessService;
async function port() {
  const probe = http.createServer(); probe.listen(0, '127.0.0.1'); await once(probe, 'listening');
  const value = probe.address().port; await new Promise(resolve => probe.close(resolve)); return value;
}
try {
  const rrfPort = await port(), wsPort = await port();
  const rrfUrl = `http://127.0.0.1:${rrfPort}`;
  const log = fs.openSync(path.join(outputDir, 'rrf.log'), 'a');
  rrf = spawn('RRF/build/rrf_simulator', ['--vsd', 'RRF/run/vsd', '-c', 'sys/config_hp3_w_line_layers.g', '--server', '-p', String(rrfPort)],
    { stdio: ['ignore', log, log] });
  await waitForRrfSimulator(rrfUrl, 30000);
  bridge = createGcodeBridge({ server: rrfUrl, wsPort, quiet: true, encoderTimeoutMs: 120000 });
  vite = await createServer(createViteServerOptions(root));
  await vite.listen();
  browser = await puppeteer.launch({ headless: true, args: ['--no-sandbox', '--disable-setuid-sandbox'] });
  const page = await browser.newPage();
  await page.goto(`http://127.0.0.1:${vite.httpServer.address().port}/tests/parity3d/collection.html`);
  await page.evaluate(async ({ scene, wsPort }) => {
    const { createHeadlessWorld } = await import('/hp-sim-3d/app/headlessWorld.js');
    const { createExternalCommandController } = await import('/hp-sim-3d/app/externalCommandSocket.js');
    let { world, remote, dt } = createHeadlessWorld(scene);
    let steps = 0, time = 0, speed = 1;
    const controller = createExternalCommandController({ world, url: `ws://127.0.0.1:${wsPort}`,
      commands: {
        getRemoteSystem: () => remote,
        pushExternalCommands(commands) { commands.forEach(command => remote.addCommand(command)); return true; },
        applyTimeScaleChange(value) { speed = value; },
      }, runtime: { resume() {} },
    });
    controller.connect();
    window.collectionClock = {
      advance(seconds = 0, drain = false) {
        const count = Math.max(Math.ceil(seconds * speed / dt - 1e-9), drain ? remote.getQueueLength() : 0);
        for (let i = 0; i < count; i++) { world.update(dt); steps++; time += dt / speed; }
        return { backend: 'chromium-browser-js', collector_time_s: time, step: steps, simulated_s: steps * dt };
      },
    };
  }, { scene: fs.readFileSync('public/usd_scenes/hp3_rigid_body.usda', 'utf8'), wsPort });
  await bridge.waitForHpSimConnection(10000);
  if (!bridge.hasReadyWsClients()) throw new Error('Browser did not connect');
  server = http.createServer(async (request, response) => {
    try {
      let body = ''; for await (const chunk of request) body += chunk;
      const args = body ? JSON.parse(body) : {};
      let result;
      if (request.url === '/gcode') {
        if (/^M569\.3\b/i.test(args.line)) await page.evaluate(() => window.collectionClock.advance(0, true));
        result = await bridge.sendGcodeLine(args.line, { timeout: 120000 });
        await bridge.sendEncoderRequest([], 120000);
      } else if (request.url === '/advance') {
        await page.evaluate(seconds => window.collectionClock.advance(seconds), args.seconds);
      } else if (request.url === '/payload') {
        if (args.payload.type === 'reset') throw new Error('Verification starts from a fresh browser world');
        bridge.broadcast(args.payload); await bridge.sendEncoderRequest([], 120000);
      }
      response.end(JSON.stringify({ ...await page.evaluate(() => window.collectionClock.advance()), result }));
    } catch (error) { response.writeHead(500).end(JSON.stringify({ error: error.message })); }
  });
  server.listen(0, '127.0.0.1'); await once(server, 'listening');
  const dataset = path.join(outputDir, 'sweeps.json');
  // Hold force and span inputs fixed for measurement parity. Independent adaptive
  // autotuning can select slightly different spans; full-auto tests cover tuning.
  const tuning = JSON.parse(fs.readFileSync(headlessDataset)).config.force_tuning;
  const matchedOptions = ['--no-auto-tune-force', '--force-low', String(tuning.force_low_n),
    '--force-mid', String(tuning.force_mid_n), '--force-max', String(tuning.force_max_n), '--max-travel-mm', '300'];
  async function collect(url, firmwareUrl, target, name) {
    const collectorLog = fs.openSync(path.join(outputDir, `${name}-collector.log`), 'a');
    const child = spawn('node', ['autocal/control/cli/collect_sweep_data.mjs', '--machineType', 'hangprinter_4',
      '--sim', '--no-spawn-rrf-simulator', '--server', firmwareUrl, '--headless-url', url, '--sweep-config-file',
      headlessDataset.replace(/\.json$/, '.bootstrap_cfg.txt'), '--output-file', target,
      '--force-base-radii', '30', '--force-buildup-factor', '0.636619', '--return-to-origin', ...matchedOptions],
      { stdio: ['ignore', collectorLog, collectorLog] });
    const [code] = await once(child, 'exit');
    if (code) throw new Error(`${name} collection failed (${code}); inspect ${name}-collector.log`);
  }
  await collect(`http://127.0.0.1:${server.address().port}`, rrfUrl, dataset, 'browser');
  const headlessDir = path.join(outputDir, 'headless'); fs.mkdirSync(headlessDir);
  const firmwarePort = await port(), socketPort = await port(), servicePort = await port();
  const firmwareUrl = `http://127.0.0.1:${firmwarePort}`, serviceUrl = `http://127.0.0.1:${servicePort}`;
  const headlessLog = fs.openSync(path.join(headlessDir, 'services.log'), 'a');
  headlessRrf = spawn('RRF/build/rrf_simulator', ['--vsd', 'RRF/run/vsd', '-c', 'sys/config_hp3_w_line_layers.g',
    '--server', '-p', String(firmwarePort)], { stdio: ['ignore', headlessLog, headlessLog] });
  await waitForRrfSimulator(firmwareUrl, 30000);
  headlessService = spawn('node', ['scripts/autocal_headless.mjs', 'public/usd_scenes/hp3_rigid_body.usda',
    firmwareUrl, String(socketPort), String(servicePort), headlessDir], { stdio: ['ignore', headlessLog, headlessLog] });
  const deadline = Date.now() + 30000;
  while (true) {
    try { if ((await fetch(`${serviceUrl}/status`)).ok) break; } catch {}
    if (Date.now() > deadline || headlessService.exitCode !== null) throw new Error('Headless parity service failed to start');
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  const matchedDataset = path.join(headlessDir, 'sweeps.json');
  await collect(serviceUrl, firmwareUrl, matchedDataset, 'headless');
  const browserData = JSON.parse(fs.readFileSync(dataset));
  const headlessData = JSON.parse(fs.readFileSync(matchedDataset));
  const errors = { angle_deg: 0, length_mm: 0, noise_deg: 0, timestamp_ms: 0, sample_duration_ms: 0 };
  let points = 0;
  function compare(a, b, key = '') {
    if (typeof a === 'number' && typeof b === 'number') {
      const category = key.includes('timestamp') ? 'timestamp_ms' : key.endsWith('duration_ms') ? 'sample_duration_ms'
        : key.startsWith('l_') || key.includes('_mm') ? 'length_mm'
        : key.includes('sigma') || key === 'mu' ? 'noise_deg' : key.includes('deg') ? 'angle_deg' : null;
      if (category) errors[category] = Math.max(errors[category], Math.abs(a - b));
      else if (Math.abs(a - b) > 1e-6) throw new Error(`Measurement metadata differs: ${key}`);
    } else if (a && typeof a === 'object') {
      if (Array.isArray(a) && a.length !== b.length) throw new Error(`Array length differs: ${key}`);
      for (const field of Object.keys(a)) {
        if (!(field in b)) throw new Error(`Missing measurement field ${field}`);
        compare(a[field], b[field], Array.isArray(a) ? key : field);
      }
    } else if (a !== b) throw new Error(`Measurement metadata differs: ${key}`);
  }
  for (let i = 0; i < browserData.sweeps.length; i++) {
    const a = browserData.sweeps[i], b = headlessData.sweeps[i];
    for (const key of ['fixed_anchors', 'drive_anchor', 'sensor_anchor']) {
      if (JSON.stringify(a[key]) !== JSON.stringify(b[key])) throw new Error(`Sweep roles differ: ${key}`);
    }
    if (a.data_points.length !== b.data_points.length) throw new Error('Point counts differ');
    compare(a.data_points, b.data_points); points += a.data_points.length;
    for (const sweep of [a, b]) {
      if (!sweep.data_points.every((point, i, records) => Number.isFinite(point.timestamp_ms)
          && (i === 0 || point.timestamp_ms >= records[i - 1].timestamp_ms))) throw new Error('Invalid simulation timestamps');
    }
  }
  // RRF reports hundredths of degrees. Allow three reporting quanta while
  // requiring collected lengths within 10 micrometres. Absolute timestamps are
  // diagnostic: independent settling can cross a quiet-window boundary later.
  const tolerances = { angle_deg: .030001, length_mm: .01, noise_deg: .030001, sample_duration_ms: 2.001 };
  const passed = points === 60 && Object.keys(tolerances).every(key => errors[key] <= tolerances[key]);
  const result = { backend: 'chromium-browser-js', matched_backend: 'headless-js', passed, points,
    sweeps: browserData.sweeps.length, errors, tolerances, matched_options: matchedOptions,
    clock: await page.evaluate(() => window.collectionClock.advance()) };
  fs.writeFileSync(path.join(outputDir, 'parity.json'), JSON.stringify(result, null, 2));
  console.log(JSON.stringify(result));
  if (!passed) process.exitCode = 1;
} finally {
  if (browser) await browser.close();
  bridge?.close();
  if (server) await new Promise(resolve => server.close(resolve));
  if (vite) await vite.close();
  if (rrf && rrf.exitCode === null) { rrf.kill('SIGTERM'); await once(rrf, 'exit'); }
  for (const process of [headlessService, headlessRrf]) {
    if (process && process.exitCode === null) { process.kill('SIGTERM'); await once(process, 'exit'); }
  }
}
