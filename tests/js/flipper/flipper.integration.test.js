const {
  launchFlipperPage,
  closeFlipperPage,
  runAutonomousScoreExpectation
} = require('./flipperIntegrationTestUtils.cjs');

describe.skip('Flipper Integration Test', () => {
  let harness;

  jest.setTimeout(120000);

  beforeAll(async () => {
    harness = await launchFlipperPage({
      pagePath: '/examples/js/flipper/index.html',
      speedScale: 10.0
    });
  });

  afterAll(async () => {
    await closeFlipperPage(harness);
  });

  test('should run autonomously and reach a score of 4 when balls settle below flippers', async () => {
    await runAutonomousScoreExpectation(harness.page, { expectedScore: 4 });
  });
});
