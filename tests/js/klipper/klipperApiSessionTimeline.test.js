describe('Klipper API session timeline rebasing', () => {
  let KlipperApiSessionTimelineBuffer;
  let rebaseApiStepperBatch;

  beforeEach(async () => {
    jest.resetModules();
    ({
      KlipperApiSessionTimelineBuffer,
      rebaseApiStepperBatch,
    } = await import('../../../integrations/klipper/klipperApiSessionTimeline.js'));
  });

  test('rebases the first dump_stepper interval against the trapq session start', () => {
    const rebased = rebaseApiStepperBatch({
      name: 'stepper_a',
      first_time: 97.17113626,
      last_time: 97.47718972,
      start_mcu_position: 0,
      data: [
        [563589517, 1, 0],
        [2669614, 1, 0],
        [1838064, 1, 0],
        [1494072, 1, 0],
        [1291931, 1, 0],
        [1154740, 1, 0],
        [1053741, 1, 0],
        [975368, 1, 0],
        [912252, 1, 0],
        [860007, 1, 0],
        [815829, 1, 0],
        [776851, 3, -31166],
      ],
    }, {
      sessionStartTime: 97.0704243,
      clockHz: 50_000_000,
      previousLastStepTick: null,
    });

    expect(rebased.timeline_start_tick).toBe(0);
    expect(rebased.data[0]).toEqual([5035598, 1, 0]);
    expect(rebased.data[1]).toEqual([2669614, 1, 0]);
    expect(rebased.data.at(-1)).toEqual([776851, 3, -31166]);
    expect(rebased.timeline_last_tick).toBe(20338271);
  });

  test('stitches later dump_stepper batches onto the prior rebased last tick', () => {
    const buffer = new KlipperApiSessionTimelineBuffer({
      clockHz: 50_000_000,
      startupBufferMs: 1000,
    });

    buffer.consumeTrapqBatch({
      name: 'toolhead',
      data: [
        [97.0704243, 0.05555555555555555, 0, 3000, [0, 0, 0], [1, 0, 0]],
        [97.12597985555556, 0.5444444444444444, 166.66666666666666, 0, [4.629629629629629, 0, 0], [1, 0, 0]],
        [97.67042430000001, 0.05555555555555555, 166.66666666666666, -3000, [95.37037037037037, 0, 0], [1, 0, 0]],
      ],
    });
    buffer.consumeStepperBatch({
      name: 'stepper_a',
      first_time: 97.17113626,
      last_time: 97.47718972,
      start_mcu_position: 0,
      data: [
        [563589517, 1, 0],
        [2669614, 1, 0],
        [1838064, 1, 0],
        [1494072, 1, 0],
        [1291931, 1, 0],
        [1154740, 1, 0],
        [1053741, 1, 0],
        [975368, 1, 0],
        [912252, 1, 0],
        [860007, 1, 0],
        [815829, 1, 0],
        [776851, 3, -31166],
      ],
    });
    buffer.consumeStepperBatch({
      name: 'stepper_a',
      first_time: 97.49097322,
      last_time: 97.70806218,
      start_mcu_position: 14,
      data: [
        [689175, 3, -22047],
        [624571, 4, -16061],
        [562243, 5, -11834],
        [506000, 5, -8961],
        [488007, 1, 0],
        [598913, 1, 0],
        [920136, 1, 0],
      ],
    });

    const [firstBatch, secondBatch] = buffer.flushReady({ forceStart: true });

    expect(firstBatch.timeline_start_tick).toBe(0);
    expect(firstBatch.timeline_last_tick).toBe(20338271);
    expect(secondBatch.timeline_start_tick).toBe(20338271);
    expect(secondBatch.data[0]).toEqual([689175, 1, 0]);
    expect(secondBatch.data[1]).toEqual([667128, 2, -22047]);
    expect(secondBatch.timeline_last_tick).toBe(31881894);
  });

  test('uses the earliest stepper first_time as a fallback anchor with a short lead-in', () => {
    const buffer = new KlipperApiSessionTimelineBuffer({
      clockHz: 50_000_000,
      startupBufferMs: 1000,
    });

    buffer.consumeStepperBatch({
      name: 'stepper_b',
      first_time: 10.8,
      last_time: 13.0,
      start_mcu_position: 0,
      data: [
        [10_000_000, 1, 0],
      ],
    });
    buffer.consumeStepperBatch({
      name: 'stepper_a',
      first_time: 10.2,
      last_time: 13.1,
      start_mcu_position: 0,
      data: [
        [99_999_999, 1, 0],
        [60_000, 1, 0],
      ],
    });

    expect(buffer.canStart(false)).toBe(true);

    const batches = buffer.flushReady({ forceStart: false });
    const firstBatch = batches.find((batch) => batch.name === 'stepper_a');

    expect(firstBatch.timeline_start_tick).toBe(0);
    expect(firstBatch.data[0]).toEqual([25_000_000, 1, 0]);
    expect(firstBatch.data[1]).toEqual([60_000, 1, 0]);
  });
});
