#!/usr/bin/env node
import puppeteer from 'puppeteer';
import http from 'node:http';
import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

function parseArgs(argv) {
  const args = {
    scenePath: 'examples/js/flipper/spool_energy_debug.html',
    steps: 4000,
    analysisStartStep: 500,
    timeoutMs: 120000,
    speedScale: 12.0,
    maxSubSteps: 500,
    reportPath: 'tests/benchmark/spool_energy_report_latest.json',
    tracePath: '',
    configJson: '',
    verbosePageLogs: false,
    failOnUnsolved: false,
    criteria: {
      jumpDeltaEThreshold: 5e-4,
      coupledBorderDeltaLambdaThreshold: 2e-4,
      maxPositiveDeltaE: 1.5e-3,
      maxJumpEpisodes: 0,
      maxCoupledJumpEpisodes: 0,
      minStoredGain: null
    }
  };

  const readNumber = (value, fallback) => {
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  };

  for (let i = 0; i < argv.length; i += 1) {
    const a = argv[i];
    if (a === '--scene-path') args.scenePath = argv[++i] || args.scenePath;
    else
    if (a === '--steps') args.steps = Math.max(100, readNumber(argv[++i], args.steps) | 0);
    else if (a === '--analysis-start-step') args.analysisStartStep = Math.max(0, readNumber(argv[++i], args.analysisStartStep) | 0);
    else if (a === '--timeout-ms') args.timeoutMs = Math.max(10000, readNumber(argv[++i], args.timeoutMs) | 0);
    else if (a === '--speed-scale') args.speedScale = readNumber(argv[++i], args.speedScale);
    else if (a === '--max-substeps') args.maxSubSteps = Math.max(10, readNumber(argv[++i], args.maxSubSteps) | 0);
    else if (a === '--report') args.reportPath = argv[++i] || args.reportPath;
    else if (a === '--trace') args.tracePath = argv[++i] || '';
    else if (a === '--config-json') args.configJson = argv[++i] || '';
    else if (a === '--verbose-page-logs') args.verbosePageLogs = true;
    else if (a === '--fail-on-unsolved') args.failOnUnsolved = true;
    else if (a === '--jump-delta-e-threshold') args.criteria.jumpDeltaEThreshold = readNumber(argv[++i], args.criteria.jumpDeltaEThreshold);
    else if (a === '--coupled-border-dlambda-threshold') args.criteria.coupledBorderDeltaLambdaThreshold = readNumber(argv[++i], args.criteria.coupledBorderDeltaLambdaThreshold);
    else if (a === '--max-positive-delta-e') args.criteria.maxPositiveDeltaE = readNumber(argv[++i], args.criteria.maxPositiveDeltaE);
    else if (a === '--max-jump-episodes') args.criteria.maxJumpEpisodes = Math.max(0, readNumber(argv[++i], args.criteria.maxJumpEpisodes) | 0);
    else if (a === '--max-coupled-jump-episodes') args.criteria.maxCoupledJumpEpisodes = Math.max(0, readNumber(argv[++i], args.criteria.maxCoupledJumpEpisodes) | 0);
    else if (a === '--min-stored-gain') args.criteria.minStoredGain = readNumber(argv[++i], 0);
  }
  return args;
}

function quantile(values, q) {
  if (!values || values.length === 0) return 0;
  const sorted = [...values].sort((a, b) => a - b);
  const idx = Math.max(0, Math.min(sorted.length - 1, Math.floor((sorted.length - 1) * q)));
  return sorted[idx];
}

function toEpisodes(samples, predicate) {
  const episodes = [];
  let current = null;
  for (const sample of samples) {
    if (!predicate(sample)) {
      current = null;
      continue;
    }
    if (!current) {
      current = {
        startStep: sample.step,
        endStep: sample.step,
        maxDeltaE: sample.delta_e,
        maxBorderDeltaLambda: sample.max_border_delta_lambda
      };
      episodes.push(current);
      continue;
    }
    current.endStep = sample.step;
    if (sample.delta_e > current.maxDeltaE) {
      current.maxDeltaE = sample.delta_e;
    }
    if (sample.max_border_delta_lambda > current.maxBorderDeltaLambda) {
      current.maxBorderDeltaLambda = sample.max_border_delta_lambda;
    }
  }
  return episodes;
}

function buildSummary(trace, criteria, analysisStartStep) {
  const allSamples = Array.isArray(trace)
    ? trace.filter((s) => Number.isFinite(s?.delta_e))
    : [];
  const samples = allSamples.filter((s) => (s.step | 0) >= analysisStartStep);

  const positiveDeltaE = samples
    .map((s) => s.delta_e)
    .filter((v) => v > 0);
  const maxPositiveDeltaE = positiveDeltaE.length > 0 ? Math.max(...positiveDeltaE) : 0;

  const jumpEpisodes = toEpisodes(
    samples,
    (s) => s.delta_e > criteria.jumpDeltaEThreshold
  );

  const coupledJumpEpisodes = toEpisodes(
    samples,
    (s) =>
      s.delta_e > criteria.jumpDeltaEThreshold &&
      Number.isFinite(s.max_border_delta_lambda) &&
      s.max_border_delta_lambda > criteria.coupledBorderDeltaLambdaThreshold
  );

  const first = samples[0] ?? null;
  const last = samples.length > 0 ? samples[samples.length - 1] : null;
  const totalEnergyDrift = (first && last) ? (last.e_total - first.e_total) : 0;
  const avgEnergyDriftPerStep = samples.length > 1 ? totalEnergyDrift / (samples.length - 1) : 0;

  const storedSeries = allSamples
    .map((s) => s.stored_0)
    .filter((v) => Number.isFinite(v));
  const storedStart = storedSeries.length > 0 ? storedSeries[0] : 0;
  const storedEnd = storedSeries.length > 0 ? storedSeries[storedSeries.length - 1] : 0;
  const storedMin = storedSeries.length > 0 ? Math.min(...storedSeries) : 0;
  const storedMax = storedSeries.length > 0 ? Math.max(...storedSeries) : 0;
  const storedGain = storedMax - storedStart;

  const topJumpSamples = [...samples]
    .sort((a, b) => b.delta_e - a.delta_e)
    .slice(0, 10)
    .map((s) => ({
      step: s.step,
      delta_e: s.delta_e,
      e_total: s.e_total,
      max_border_delta_lambda: s.max_border_delta_lambda,
      effective_radius_bottom: s.effective_radius_bottom
    }));

  return {
    allSampleCount: allSamples.length,
    sampleCount: samples.length,
    analysisStartStep,
    maxPositiveDeltaE,
    p95PositiveDeltaE: quantile(positiveDeltaE, 0.95),
    p99PositiveDeltaE: quantile(positiveDeltaE, 0.99),
    totalEnergyDrift,
    avgEnergyDriftPerStep,
    storedStart,
    storedEnd,
    storedMin,
    storedMax,
    storedGain,
    jumpEpisodeCount: jumpEpisodes.length,
    coupledJumpEpisodeCount: coupledJumpEpisodes.length,
    jumpEpisodes,
    coupledJumpEpisodes,
    topJumpSamples
  };
}

function evaluateSolved(summary, criteria) {
  const hasStoredGainRequirement = Number.isFinite(criteria.minStoredGain);
  const storedGainSolved = !hasStoredGainRequirement || summary.storedGain >= criteria.minStoredGain;
  return (
    summary.maxPositiveDeltaE <= criteria.maxPositiveDeltaE &&
    summary.jumpEpisodeCount <= criteria.maxJumpEpisodes &&
    summary.coupledJumpEpisodeCount <= criteria.maxCoupledJumpEpisodes &&
    storedGainSolved
  );
}

async function ensureDirForFile(filePath) {
  const dir = path.dirname(filePath);
  await fs.mkdir(dir, { recursive: true });
}

function contentTypeFor(filePath) {
  const ext = path.extname(filePath).toLowerCase();
  switch (ext) {
    case '.html': return 'text/html; charset=utf-8';
    case '.js':
    case '.mjs': return 'text/javascript; charset=utf-8';
    case '.css': return 'text/css; charset=utf-8';
    case '.json': return 'application/json; charset=utf-8';
    case '.svg': return 'image/svg+xml';
    case '.png': return 'image/png';
    case '.jpg':
    case '.jpeg': return 'image/jpeg';
    case '.ico': return 'image/x-icon';
    case '.wasm': return 'application/wasm';
    default: return 'application/octet-stream';
  }
}

function normalizeWithinRoot(root, urlPathname) {
  const decoded = decodeURIComponent(urlPathname);
  const noQuery = decoded.split('?')[0];
  let relativePath = noQuery.startsWith('/') ? noQuery.slice(1) : noQuery;
  if (!relativePath) {
    relativePath = 'index.html';
  }
  const candidate = path.resolve(root, relativePath);
  if (!candidate.startsWith(root)) {
    return null;
  }
  return candidate;
}

async function startStaticServer(root) {
  const server = http.createServer(async (req, res) => {
    try {
      const requestUrl = new URL(req.url || '/', 'http://127.0.0.1');
      let filePath = normalizeWithinRoot(root, requestUrl.pathname);
      if (!filePath) {
        res.statusCode = 403;
        res.end('Forbidden');
        return;
      }

      try {
        const stat = await fs.stat(filePath);
        if (stat.isDirectory()) {
          filePath = path.join(filePath, 'index.html');
        }
      } catch {
        // Continue to readFile, which will produce a 404 path.
      }

      const data = await fs.readFile(filePath);
      res.statusCode = 200;
      res.setHeader('Content-Type', contentTypeFor(filePath));
      res.end(data);
    } catch {
      res.statusCode = 404;
      res.end('Not Found');
    }
  });

  await new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', resolve);
  });
  const addr = server.address();
  const port = addr && typeof addr === 'object' ? addr.port : NaN;
  return { server, port };
}

async function runProbe(args) {
  const thisFile = fileURLToPath(import.meta.url);
  const testsDir = path.resolve(path.dirname(thisFile), '..');
  const projectRoot = path.resolve(testsDir, '..');

  const { server, port } = await startStaticServer(projectRoot);

  let browser = null;
  try {
    browser = await puppeteer.launch({
      headless: 'new',
      args: ['--no-sandbox', '--disable-setuid-sandbox']
    });
    const page = await browser.newPage();
    await page.setViewport({ width: 1200, height: 800 });

    if (args.verbosePageLogs) {
      page.on('console', (msg) => {
        const type = msg.type();
        if (type === 'error' || type === 'warn' || type === 'log') {
          console.log(`[PAGE ${type}] ${msg.text()}`);
        }
      });
    }
    page.on('pageerror', (err) => {
      console.error(`[PAGE ERROR] ${err?.message || err}`);
    });

    await page.evaluateOnNewDocument((speedScale, maxSubSteps) => {
      window._flipperSpeedScale = speedScale;
      window._flipperMaxSubSteps = maxSubSteps;
    }, args.speedScale, args.maxSubSteps);

    await page.goto(`http://127.0.0.1:${port}/${args.scenePath}`, {
      waitUntil: 'networkidle0',
      timeout: args.timeoutMs
    });
    await page.waitForFunction(
      () => typeof window.getSpoolEnergyTrace === 'function' && typeof window.setSpoolEnergyConfig === 'function',
      { timeout: 20000 }
    );

    let sceneConfig = null;
    if (args.configJson) {
      let parsed = null;
      try {
        parsed = JSON.parse(args.configJson);
      } catch (error) {
        throw new Error(`Invalid --config-json payload: ${error.message}`);
      }
      sceneConfig = await page.evaluate((cfg) => window.setSpoolEnergyConfig(cfg), parsed);
      await page.click('#resetBtn');
    } else {
      sceneConfig = await page.evaluate(() => window.getSpoolEnergyConfig());
    }

    await page.evaluate(() => window.clearSpoolEnergyTrace());
    await page.click('#pauseBtn');
    await page.waitForFunction((target) => window.getSpoolEnergyTrace().length >= target, {
      timeout: args.timeoutMs
    }, args.steps);
    await page.click('#pauseBtn');

    const trace = await page.evaluate(() => window.getSpoolEnergyTrace());
    const summary = buildSummary(trace, args.criteria, args.analysisStartStep);
    const solved = evaluateSolved(summary, args.criteria);

    const report = {
      timestamp: new Date().toISOString(),
      scene: args.scenePath,
      args: {
        steps: args.steps,
        analysisStartStep: args.analysisStartStep,
        timeoutMs: args.timeoutMs,
        speedScale: args.speedScale,
        maxSubSteps: args.maxSubSteps,
        criteria: args.criteria
      },
      sceneConfig,
      solved,
      summary
    };

    const reportPathAbs = path.isAbsolute(args.reportPath)
      ? args.reportPath
      : path.resolve(projectRoot, args.reportPath);
    await ensureDirForFile(reportPathAbs);
    await fs.writeFile(reportPathAbs, `${JSON.stringify(report, null, 2)}\n`, 'utf8');

    if (args.tracePath) {
      const tracePathAbs = path.isAbsolute(args.tracePath)
        ? args.tracePath
        : path.resolve(projectRoot, args.tracePath);
      await ensureDirForFile(tracePathAbs);
      await fs.writeFile(tracePathAbs, `${JSON.stringify(trace, null, 2)}\n`, 'utf8');
    }

    console.log(`SPOOL_ENERGY_REPORT ${reportPathAbs}`);
    if (args.tracePath) {
      const tracePathAbs = path.isAbsolute(args.tracePath)
        ? args.tracePath
        : path.resolve(projectRoot, args.tracePath);
      console.log(`SPOOL_ENERGY_TRACE ${tracePathAbs}`);
    }
    console.log(`SPOOL_ENERGY_SAMPLES ${summary.sampleCount}`);
    console.log(`SPOOL_ENERGY_MAX_POSITIVE_DELTA_E ${summary.maxPositiveDeltaE}`);
    console.log(`SPOOL_ENERGY_JUMP_EPISODES ${summary.jumpEpisodeCount}`);
    console.log(`SPOOL_ENERGY_COUPLED_JUMP_EPISODES ${summary.coupledJumpEpisodeCount}`);
    console.log(`SPOOL_ENERGY_TOTAL_DRIFT ${summary.totalEnergyDrift}`);
    console.log(`SPOOL_ENERGY_STORED_GAIN ${summary.storedGain}`);
    console.log(`SPOOL_ENERGY_SOLVED ${solved ? 'true' : 'false'}`);

    if (args.failOnUnsolved && !solved) {
      process.exitCode = 2;
    }
  } finally {
    if (browser) {
      await browser.close();
    }
    await new Promise((resolve) => server.close(resolve));
  }
}

const args = parseArgs(process.argv.slice(2));
runProbe(args).catch((error) => {
  console.error(`[spool-energy-probe] ${error?.stack || error?.message || String(error)}`);
  process.exitCode = 1;
});
