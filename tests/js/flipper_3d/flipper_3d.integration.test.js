const {
  launchFlipperPage,
  closeFlipperPage,
  resetGame,
  runAutonomousScoreExpectation
} = require('../flipper/flipperIntegrationTestUtils.cjs');

describe.skip('Flipper 3D Integration', () => {
  let harness;

  jest.setTimeout(120000);

  beforeAll(async () => {
    harness = await launchFlipperPage({
      pagePath: '/examples/js/flipper_3d/index.html',
      speedScale: 10.0,
      viewport: { width: 1200, height: 900 },
      newDocumentVars: { _flipper3dDebug: true },
      waitForRenderSystem: true
    });
  });

  afterAll(async () => {
    await closeFlipperPage(harness);
  });

  async function readFrameState() {
    return harness.page.evaluate(() => {
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

    await harness.page.click('#pauseBtn');

    await harness.page.waitForFunction(() => window.world.getResource('pauseState')?.paused === false, { timeout: 5000 });
    await new Promise((resolve) => setTimeout(resolve, 2500));

    const running = await readFrameState();

    expect(running.camera.x).toBeCloseTo(initial.camera.x, 6);
    expect(running.camera.y).toBeCloseTo(initial.camera.y, 6);
    expect(running.camera.z).toBeCloseTo(initial.camera.z, 6);
    expect(running.maxAbsZ).toBeLessThan(1e-5);
  });

  test('should run autonomously and reach a score of 4 when balls settle below flippers', async () => {
    await resetGame(harness.page);
    await runAutonomousScoreExpectation(harness.page, { expectedScore: 4 });
  });
});
