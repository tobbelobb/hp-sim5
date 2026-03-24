let isKlipperRawBridgeMessage;

async function importModule() {
  if (isKlipperRawBridgeMessage) {
    return;
  }
  ({ isKlipperRawBridgeMessage } = await import('../../../autocal/control/primitives/klipper_bridge_stream.mjs'));
}

describe('klipper bridge stream helpers', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('recognizes Klipper raw stream messages', () => {
    expect(isKlipperRawBridgeMessage({ action: 'klipper_parsed' })).toBe(true);
    expect(isKlipperRawBridgeMessage({ action: 'klipper_clock' })).toBe(true);
    expect(isKlipperRawBridgeMessage({ action: 'klipper_serial' })).toBe(true);
    expect(isKlipperRawBridgeMessage({ action: 'reply' })).toBe(false);
    expect(isKlipperRawBridgeMessage(null)).toBe(false);
  });
});
