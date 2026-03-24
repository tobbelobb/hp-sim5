let normalizeSpeedScale;
let resolveExternalKlipperRawTimeScale;
let scaleDurationMs;
let scaleDeadlineMs;
let createTimelineNormalizer;

async function importModule() {
  if (normalizeSpeedScale) {
    return;
  }
  ({
    normalizeSpeedScale,
    resolveExternalKlipperRawTimeScale,
    scaleDurationMs,
    scaleDeadlineMs,
    createTimelineNormalizer,
  } = await import('../../../autocal/control/primitives/klipper_live_timing.mjs'));
}

describe('klipper live timing helpers', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('normalizes and scales raw live playback timing', () => {
    expect(normalizeSpeedScale(0)).toBe(1);
    expect(normalizeSpeedScale('4')).toBe(4);
    expect(resolveExternalKlipperRawTimeScale(1)).toBe(20);
    expect(resolveExternalKlipperRawTimeScale(40)).toBe(40);
    expect(scaleDurationMs(20_000, 20)).toBe(1_000);
    expect(scaleDeadlineMs(100, 20_100, 20)).toBe(1_100);
    const normalizer = createTimelineNormalizer();
    expect(normalizer.normalizeAt(20_100, 100)).toBe(100);
    expect(normalizer.normalizeAt(20_110, 100)).toBe(110);
  });
});
