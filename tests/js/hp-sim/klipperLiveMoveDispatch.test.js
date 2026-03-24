let resolveRawMoveDispatchTime;

async function importModule() {
  if (resolveRawMoveDispatchTime) {
    return;
  }
  ({ resolveRawMoveDispatchTime } = await import('../../../autocal/control/primitives/klipper_live_timing.mjs'));
}

describe('klipper live move dispatch timing', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('dispatches a move when its first step is due, not after the batch ends', () => {
    const now = 1000;
    expect(resolveRawMoveDispatchTime(1200, now)).toBe(1200);
    expect(resolveRawMoveDispatchTime(900, now)).toBe(1000);
    expect(resolveRawMoveDispatchTime(null, now)).toBe(1000);
  });
});
