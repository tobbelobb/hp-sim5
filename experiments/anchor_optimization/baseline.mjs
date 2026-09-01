import { spawn } from 'node:child_process';
import { mkdir, writeFile } from 'node:fs/promises';
import { chromium } from 'playwright';

const OUTPUT_DIR = 'experiments/anchor_optimization/results';
const BASE_URL = 'http://127.0.0.1:5173/hp-sim5';

async function waitForServer(url, timeoutMs = 120000) {
  const deadline = Date.now() + timeoutMs;
  let lastError = null;
  while (Date.now() < deadline) {
    try {
      const response = await fetch(url);
      if (response.ok) return;
    } catch (error) {
      lastError = error;
    }
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
  throw new Error(`Vite did not become ready: ${lastError?.message || 'timeout'}`);
}

async function measureDefault(browser, route, moduleRelativeUrl, label) {
  const page = await browser.newPage({ viewport: { width: 1440, height: 1000 } });
  const consoleErrors = [];
  page.on('console', (message) => {
    if (message.type() === 'error') consoleErrors.push(message.text());
  });
  page.on('pageerror', (error) => consoleErrors.push(error.message));

  const startedAt = Date.now();
  await page.goto(`${BASE_URL}/${route}/`, { waitUntil: 'networkidle', timeout: 120000 });
  await page.waitForFunction(() => {
    const button = document.querySelector('#printSquareBtn');
    return button && !button.disabled;
  }, null, { timeout: 120000 });

  await page.evaluate(async ({ moduleRelativeUrl }) => {
    window.__anchorOptimizationMetrics = [];
    const moduleUrl = new URL(moduleRelativeUrl, window.location.href).href;
    const { QualityMonitor } = await import(moduleUrl);
    if (!QualityMonitor.prototype.__anchorOptimizationPatched) {
      const original = QualityMonitor.prototype.runFinalCheck;
      QualityMonitor.prototype.runFinalCheck = function patchedRunFinalCheck(...args) {
        const result = original.apply(this, args);
        const metrics = this.getMetrics();
        window.__anchorOptimizationMetrics.push({
          machineId: this.machineId,
          machineLabel: this.machineLabel,
          metrics,
        });
        return result;
      };
      QualityMonitor.prototype.__anchorOptimizationPatched = true;
    }
  }, { moduleRelativeUrl });

  await page.click('#printSquareBtn');
  await page.waitForFunction(() => {
    const button = document.querySelector('#finishAsapBtn');
    return button && !button.disabled;
  }, null, { timeout: 120000 });
  for (let index = 0; index < 4; index += 1) {
    await page.click('#speedFasterBtn', { force: true });
  }

  await page.waitForFunction(
    () => Array.isArray(window.__anchorOptimizationMetrics)
      && window.__anchorOptimizationMetrics.length > 0,
    null,
    { timeout: 600000 }
  );
  const records = await page.evaluate(() => window.__anchorOptimizationMetrics);
  const hud = await page.locator('#qualityHud').innerText().catch(() => '');
  await page.close();

  return {
    label,
    route,
    elapsedSeconds: (Date.now() - startedAt) / 1000,
    records,
    hud,
    consoleErrors,
  };
}

await mkdir(OUTPUT_DIR, { recursive: true });
const vite = spawn(process.execPath, ['node_modules/vite/bin/vite.js', '--host', '127.0.0.1', '--port', '5173'], {
  stdio: ['ignore', 'pipe', 'pipe'],
});
vite.stdout.on('data', (chunk) => process.stdout.write(chunk));
vite.stderr.on('data', (chunk) => process.stderr.write(chunk));

let browser;
try {
  await waitForServer(`${BASE_URL}/hp-sim/`);
  browser = await chromium.launch({
    headless: true,
    args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader'],
  });
  const results = [];
  results.push(await measureDefault(browser, 'hp-sim', './app/quality-monitor.js', '2D default'));
  results.push(await measureDefault(browser, 'hp-sim-3d', './app/quality-monitor.js', '3D default'));
  const payload = {
    generatedAt: new Date().toISOString(),
    commit: process.env.GITHUB_SHA || null,
    results,
  };
  await writeFile(`${OUTPUT_DIR}/baseline.json`, JSON.stringify(payload, null, 2));
  console.log('ANCHOR_OPT_BASELINE=' + JSON.stringify(payload));
} finally {
  if (browser) await browser.close();
  vite.kill('SIGTERM');
  await new Promise((resolve) => {
    let settled = false;
    const finish = () => {
      if (!settled) {
        settled = true;
        resolve();
      }
    };
    vite.once('close', finish);
    setTimeout(() => {
      if (!settled) vite.kill('SIGKILL');
      finish();
    }, 5000);
  });
}
