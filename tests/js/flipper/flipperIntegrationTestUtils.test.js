const {
  getFlipperBrowserLaunchArgs,
  shouldLogFlipperConsoleMessage
} = require('./flipperIntegrationTestUtils.cjs');

describe('flipper integration test harness logging', () => {
  test('suppresses known WebGL noise warnings', () => {
    expect(
      shouldLogFlipperConsoleMessage(
        'warn',
        'Automatic fallback to software WebGL has been deprecated'
      )
    ).toBe(false);

    expect(
      shouldLogFlipperConsoleMessage(
        'warn',
        '[.WebGL-0x11bc00ff0700]GL Driver Message (OpenGL, Performance, GL_CLOSE_PATH_NV, High): GPU stall due to ReadPixels'
      )
    ).toBe(false);
  });

  test('keeps normal logs and errors visible', () => {
    expect(shouldLogFlipperConsoleMessage('log', 'Advancing simulation')).toBe(true);
    expect(shouldLogFlipperConsoleMessage('error', 'PAGE ERROR: boom')).toBe(true);
  });

  test('opts in to unsafe swiftshader for headless WebGL', () => {
    expect(getFlipperBrowserLaunchArgs()).toEqual(
      expect.arrayContaining(['--enable-unsafe-swiftshader'])
    );
  });
});
