const {
  launchFlipperPage,
  closeFlipperPage,
  runAutonomousScoreExpectation
} = require('./flipperIntegrationTestUtils.cjs');

describe('Flipper Integration Test', () => {
  let harness;

  jest.setTimeout(120000);

  beforeAll(async () => {
    harness = await launchFlipperPage({
      pagePath: '/example_apps/js/flipper/index.html',
      speedScale: 10.0
    });
  });

  afterAll(async () => {
    await closeFlipperPage(harness);
  });

  test('should run autonomously and reach a score of 3 when balls settle below flippers', async () => {
    await runAutonomousScoreExpectation(harness.page, { expectedScore: 3, stableScoreChunksRequired: 2 });
  });
});
