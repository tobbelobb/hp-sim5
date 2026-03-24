let createKlipperDumpStepperRelay;

async function importModule() {
  if (createKlipperDumpStepperRelay) {
    return;
  }
  ({ createKlipperDumpStepperRelay } = await import('../../../autocal/control/primitives/klipper_dump_stepper_relay.mjs'));
}

describe('Klipper dump-stepper relay', () => {
  beforeAll(async () => {
    await importModule();
  });

  test('turns dump_stepper batches into bucketed Move commands', () => {
    const emitted = [];
    const relay = createKlipperDumpStepperRelay({
      nowFn: () => 1000,
      bucketDurationMs: 2,
      asapMode: true,
      onCommand: (command) => {
        emitted.push({ ...command });
      },
    });

    relay.handleParsedLines([
      'config_stepper oid=0 step_pin=gpiochip1/gpio0 dir_pin=gpiochip1/gpio1 invert_step=0 step_pulse_ticks=100',
      'set_next_step_dir oid=0 dir=1',
    ]);
    relay.handleMotionReportStatus({
      status: {
        toolhead: {
          print_time: 10,
          estimated_print_time: 10,
        },
        motion_report: {
          steppers: ['stepper_a'],
        },
      },
    });
    relay.handleClockMessage({ clock_hz: 10_000_000 });
    relay.feedStepperDump('stepper_a', {
      start_position: 0,
      step_distance: 0.125,
      first_step_time: 10,
      data: [[1000, 2, 0]],
    });

    relay.flushDueCommands(1000, true);

    expect(emitted).toEqual([
      { type: 'Move', A: 0.25 },
    ]);
  });
});
