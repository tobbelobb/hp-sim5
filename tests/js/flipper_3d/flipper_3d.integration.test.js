const puppeteer = require('puppeteer');
const path = require('path');

describe('Flipper 3D Integration', () => {
  let browser;
  let page;
  let server;
  let port;

  jest.setTimeout(120000);

  beforeAll(async () => {
    const projectRoot = path.resolve(__dirname, '../../..');
    const { spawn } = require('child_process');
    const serverScript = path.join(projectRoot, 'tests/startViteServer.mjs');

    server = spawn(process.execPath, [serverScript, projectRoot], {
      stdio: ['pipe', 'pipe', 'inherit']
    });

    port = await new Promise((resolve, reject) => {
      server.stdout.once('data', (data) => {
        const match = /PORT:(\d+)/.exec(data.toString());
        resolve(match ? Number(match[1]) : NaN);
      });
      server.once('error', reject);
    });

    browser = await puppeteer.launch({
      headless: 'new',
      args: ['--no-sandbox', '--disable-setuid-sandbox']
    });

    page = await browser.newPage();
    await page.setViewport({ width: 1200, height: 900 });

    page.on('pageerror', (error) => {
      // Surface page crashes to CI logs.
      console.error(`PAGE ERROR: ${error.message}`);
    });

    await page.evaluateOnNewDocument(() => {
      window._flipper3dDebug = true;
      window._flipperSpeedScale = 2.0;
      window._flipperMaxSubSteps = 500;
    });

    await page.goto(`http://127.0.0.1:${port}/examples/js/flipper_3d/index.html`, {
      waitUntil: 'domcontentloaded',
      timeout: 60000
    });

    await page.waitForFunction(
      () => {
        if (!window.world || !window.world.systems) return false;
        return window.world.systems.some((system) => system?.constructor?.name === 'RenderSystem3D');
      },
      { timeout: 30000 }
    );
  });

  afterAll(async () => {
    if (server) {
      server.kill('SIGTERM');
    }
    if (browser) {
      await browser.close();
    }
  });

  async function readFrameState() {
    return page.evaluate(() => {
      const world = window.world;
      const renderSystem = world.systems.find((system) => system?.constructor?.name === 'RenderSystem3D');
      const camera = renderSystem?.camera;

      let maxAbsZ = 0;
      const posStore = world.components.get([...world.components.keys()].find((key) => key?.name === 'PositionComponent'));
      if (posStore) {
        for (const component of posStore.values()) {
          const z = Math.abs(component.pos.z || 0);
          if (z > maxAbsZ) maxAbsZ = z;
        }
      }

      return {
        paused: world.getResource('pauseState')?.paused,
        camera: camera
          ? { x: camera.position.x, y: camera.position.y, z: camera.position.z }
          : null,
        maxAbsZ,
        entities: world.entities.size
      };
    });
  }

  test('camera stays fixed and simulation stays planar (no Z drift)', async () => {
    const initial = await readFrameState();
    expect(initial.camera).not.toBeNull();
    expect(initial.paused).toBe(true);

    await new Promise((resolve) => setTimeout(resolve, 1200));
    const pausedLater = await readFrameState();

    expect(pausedLater.camera.x).toBeCloseTo(initial.camera.x, 6);
    expect(pausedLater.camera.y).toBeCloseTo(initial.camera.y, 6);
    expect(pausedLater.camera.z).toBeCloseTo(initial.camera.z, 6);
    expect(pausedLater.maxAbsZ).toBeLessThan(1e-6);

    await page.click('#pauseBtn');

    await page.waitForFunction(() => window.world.getResource('pauseState')?.paused === false, { timeout: 5000 });
    await new Promise((resolve) => setTimeout(resolve, 2500));

    const running = await readFrameState();

    expect(running.camera.x).toBeCloseTo(initial.camera.x, 6);
    expect(running.camera.y).toBeCloseTo(initial.camera.y, 6);
    expect(running.camera.z).toBeCloseTo(initial.camera.z, 6);
    expect(running.maxAbsZ).toBeLessThan(1e-5);
  });
});
