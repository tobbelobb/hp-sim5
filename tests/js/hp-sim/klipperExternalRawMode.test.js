let createKlipperExternalRawModeHandlers;

async function importModule() {
  if (createKlipperExternalRawModeHandlers) {
    return;
  }
  ({ createKlipperExternalRawModeHandlers } = await import('../../../autocal/control/primitives/klipper_external_raw_mode.mjs'));
}

describe('Klipper external raw mode helpers', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('pushes commands as external batches and schedules reconnect on close', () => {
    const pushed = [];
    const reconnects = [];
    const { onCommand, onClose } = createKlipperExternalRawModeHandlers({
      pushExternalCommands: (commands) => {
        pushed.push(commands);
      },
      scheduleReconnect: (reason) => {
        reconnects.push(reason);
      },
    });

    onCommand({ type: 'Move', A: 1 });
    onClose();

    expect(pushed).toEqual([
      [{ type: 'Move', A: 1 }],
    ]);
    expect(reconnects).toEqual(['connection closed']);
  });
});
