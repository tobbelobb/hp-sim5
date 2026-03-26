const path = require('path');
const { spawn } = require('child_process');
const puppeteer = require('puppeteer');

function shouldLogFlipperConsoleMessage(type, text) {
  if (!['log', 'warn', 'error'].includes(type)) {
    return false;
  }
  if (type === 'warn' && text.includes('Automatic fallback to software WebGL has been deprecated')) {
    return false;
  }
  if (type === 'warn' && text.includes('GPU stall due to ReadPixels')) {
    return false;
  }
  return true;
}

function getFlipperBrowserLaunchArgs() {
  return ['--no-sandbox', '--disable-setuid-sandbox', '--enable-unsafe-swiftshader'];
}

async function launchFlipperPage({
  pagePath,
  speedScale = 10.0,
  viewport = null,
  newDocumentVars = {},
  waitForRenderSystem = false
}) {
  const projectRoot = path.resolve(__dirname, '../../..');
  const serverScript = path.join(projectRoot, 'tests/startViteServer.mjs');
  const server = spawn(process.execPath, [serverScript, projectRoot], {
    stdio: ['pipe', 'pipe', 'inherit']
  });

  const port = await new Promise((resolve, reject) => {
    server.stdout.once('data', (data) => {
      const match = /PORT:(\d+)/.exec(data.toString());
      resolve(match ? Number(match[1]) : NaN);
    });
    server.once('error', reject);
  });

  const browser = await puppeteer.launch({
    headless: 'new',
    args: getFlipperBrowserLaunchArgs()
  });

  const page = await browser.newPage();
  if (viewport) {
    await page.setViewport(viewport);
  }

  page.on('console', (msg) => {
    const type = msg.type();
    const text = msg.text();
    if (shouldLogFlipperConsoleMessage(type, text)) {
      console.log(`PAGE CONSOLE [${type.toUpperCase()}]: ${text}`);
    }
  });
  page.on('pageerror', (error) => {
    console.error(`PAGE ERROR: ${error.message}`);
  });

  await page.evaluateOnNewDocument((vars) => {
    Object.assign(window, vars);
  }, {
    _flipperDisableAutoLoop: true,
    _flipperSpeedScale: speedScale,
    _flipperMaxSubSteps: 500,
    ...newDocumentVars
  });

  await page.goto(`http://127.0.0.1:${port}${pagePath}`, {
    waitUntil: 'domcontentloaded',
    timeout: 60000
  });

  await page.waitForFunction(
    () => {
      if ((typeof window.world === 'undefined' || !window.world) || typeof window.getGameStateForTest !== 'function') {
        return false;
      }
      const state = window.getGameStateForTest();
      return (
        state &&
        Array.isArray(state.balls) &&
        state.balls.length > 0 &&
        Number.isFinite(state.score) &&
        state.score >= 0 &&
        Number.isFinite(window.world.getResource('dt')) &&
        window.world.getResource('dt') > 0
      );
    },
    { timeout: 30000 }
  );

  if (waitForRenderSystem) {
    await page.waitForFunction(
      () => {
        if (!window.world || !window.world.systems) return false;
        return window.world.systems.some((system) => system?.constructor?.name === 'RenderSystem3D');
      },
      { timeout: 30000 }
    );
  }

  return { browser, page, server, port };
}

async function closeFlipperPage(harness) {
  if (!harness) {
    return;
  }
  if (harness.server) {
    harness.server.kill('SIGTERM');
  }
  if (harness.browser) {
    await harness.browser.close();
  }
}

async function resetGame(page) {
  await page.click('#resetBtn');
  await page.waitForFunction(
    () => window.world.getResource('pauseState')?.paused === true,
    { timeout: 10000 }
  );
}

async function runAutonomousScoreExpectation(page, {
  expectedScore = 25,
  maxSimulationSteps = 250000,
  stepChunk = 5000,
  flipperYLine = 0.05,
  stableScoreChunksRequired = 0
} = {}) {
  await page.waitForFunction(
    () => {
      if ((typeof window.world === 'undefined' || !window.world) || typeof window.getGameStateForTest !== 'function') {
        return false;
      }
      const state = window.getGameStateForTest();
      return (
        state &&
        Array.isArray(state.balls) &&
        state.balls.length > 0 &&
        Number.isFinite(state.score) &&
        state.score >= 0 &&
        Number.isFinite(window.world.getResource('dt')) &&
        window.world.getResource('dt') > 0
      );
    },
    { timeout: 10000 }
  );

  let settled = false;
  let stepsTaken = 0;
  let stableScoreChunks = 0;

  while (stepsTaken < maxSimulationSteps) {
    const gameState = await page.evaluate(({ chunkSize }) => {
      const world = window.world;
      const pauseState = world.getResource('pauseState');
      const prevPaused = pauseState ? pauseState.paused : false;
      if (pauseState) {
        pauseState.paused = false;
      }

      const dt = world.getResource('dt');
      for (let i = 0; i < chunkSize; i += 1) {
        world.update(dt);
      }

      if (pauseState) {
        pauseState.paused = prevPaused;
      }

      return window.getGameStateForTest();
    }, { chunkSize: stepChunk });
    stepsTaken += stepChunk;

    if (!gameState) {
      throw new Error('Failed to get game state from the page.');
    }

    const { balls, score } = gameState;
    if (score > expectedScore) {
      throw new Error(`Test failed: Score exceeded ${expectedScore}. Current score: ${score}`);
    }

    if (score === expectedScore) {
      stableScoreChunks += 1;
      if (stableScoreChunksRequired > 0 && stableScoreChunks >= stableScoreChunksRequired) {
        settled = true;
        break;
      }
    } else {
      stableScoreChunks = 0;
    }

    let allBallsBelowFlippers = balls.length > 0;
    if (balls.length > 0) {
      for (const ball of balls) {
        if (ball.y === -Infinity) {
          throw new Error(`Invalid ball data encountered for ball ID ${ball.id}`);
        }
        if (ball.y >= flipperYLine) {
          allBallsBelowFlippers = false;
          break;
        }
      }
    }

    if (allBallsBelowFlippers) {
      console.log(`All balls detected below flipper line (Y < ${flipperYLine}). Current score: ${score}.`);
      settled = true;
      if (score !== expectedScore) {
        throw new Error(`Test failed: Balls settled below flippers, but score is ${score} (expected ${expectedScore}).`);
      }
      break;
    }
  }

  if (!settled) {
    const finalGameState = await page.evaluate(() => window.getGameStateForTest());
    throw new Error(
      `Test failed: Balls did not settle below flippers or hold the expected score within ${maxSimulationSteps} simulation steps. Final score: ${finalGameState.score}`
    );
  }

  const finalScore = (await page.evaluate(() => window.getGameStateForTest())).score;
  expect(finalScore).toBe(expectedScore);
}

module.exports = {
  launchFlipperPage,
  closeFlipperPage,
  resetGame,
  runAutonomousScoreExpectation,
  shouldLogFlipperConsoleMessage,
  getFlipperBrowserLaunchArgs
};
