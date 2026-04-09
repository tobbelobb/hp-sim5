describe('KlipperApiMotionAdapter', () => {
  let KlipperApiMotionAdapter;

  const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);

  beforeEach(async () => {
    jest.resetModules();
    ({ KlipperApiMotionAdapter } = await import('../../../integrations/klipper/klipperApiMotionAdapter.js'));
  });

  test('converts dump_stepper batches into the same bucketed Move sequence as raw uploads', () => {
    const adapter = new KlipperApiMotionAdapter({
      now: () => 1000,
    });

    adapter.updateClock({ clock_hz: 50_000_000, mcu_clock: 250000 }, 1000);
    adapter.consumeStepperBatch({
      name: 'stepper_a',
      first_clock: 0,
      start_mcu_position: 0,
      data: [[60000, 2, 0]],
    });

    const commands = adapter.drainReadyCommands();
    const moves = commands.filter((command) => command?.type === 'Move');

    expect(moves).toHaveLength(2);
    expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(5 / 3, 6);
    expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('emits Add to reference before batch motion when the MCU batch starts from a non-zero position', () => {
    const adapter = new KlipperApiMotionAdapter({
      now: () => 1000,
    });

    adapter.updateClock({ clock_hz: 50_000_000, mcu_clock: 250000 }, 1000);
    adapter.consumeStepperBatch({
      name: 'stepper_a',
      first_clock: 0,
      start_mcu_position: 2,
      data: [[60000, 2, 0]],
    });

    const commands = adapter.drainReadyCommands();

    expect(commands.map((command) => command?.type)).toEqual([
      'Add to reference',
      'Move',
      'Move',
    ]);
    expect(commands[0].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
    expect(commands[0].at).toBe(commands[1].at);
    expect(commands[1].A / STEP_ANGLE_RAD).toBeCloseTo(5 / 3, 6);
    expect(commands[2].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('combines simultaneous dump_stepper batches into shared Move payloads', () => {
    const adapter = new KlipperApiMotionAdapter({
      now: () => 1000,
    });

    adapter.updateClock({ clock_hz: 50_000_000, mcu_clock: 250000 }, 1000);
    adapter.consumeStepperBatch({
      name: 'stepper_a',
      first_clock: 0,
      start_mcu_position: 0,
      data: [[60000, 2, 0]],
    });
    adapter.consumeStepperBatch({
      name: 'stepper_b',
      first_clock: 0,
      start_mcu_position: 0,
      data: [[60000, 2, 0]],
    });

    const commands = adapter.drainReadyCommands();
    const moves = commands.filter((command) => command?.type === 'Move');

    expect(moves).toHaveLength(2);
    expect(moves[0].A / STEP_ANGLE_RAD).toBeCloseTo(5 / 3, 6);
    expect(moves[0].B / STEP_ANGLE_RAD).toBeCloseTo(5 / 3, 6);
    expect(moves[1].A / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
    expect(moves[1].B / STEP_ANGLE_RAD).toBeCloseTo(2, 6);
  });

  test('does not inject add-to-reference corrections at later dump_stepper batch boundaries', () => {
    const adapter = new KlipperApiMotionAdapter({
      now: () => 1000,
    });

    adapter.updateClock({ clock_hz: 50_000_000, mcu_clock: 10_000_000 }, 1000);
    adapter.consumeStepperBatch({
      name: 'stepper_a',
      first_clock: 0,
      start_mcu_position: 0,
      data: [[60_000, 2, 0]],
    });
    adapter.consumeStepperBatch({
      name: 'stepper_a',
      first_clock: 120_000,
      start_mcu_position: 2,
      data: [[60_000, 2, 0]],
    });

    const commands = adapter.drainReadyCommands({ force: true });

    expect(commands.filter((command) => command?.type === 'Add to reference')).toHaveLength(0);
    const moves = commands.filter((command) => command?.type === 'Move');
    expect(moves.at(-1).A / STEP_ANGLE_RAD).toBeCloseTo(4, 6);
  });
});
