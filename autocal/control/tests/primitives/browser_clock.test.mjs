import { createBrowserClock } from '../../primitives/browser_clock.mjs';

function harness(rate, options = {}) {
  let wallMs = 0, generation = 1;
  const clock = createBrowserClock(async () => ({ simulationClock: {
    time_ms: wallMs * rate, generation, wall_time_ms: wallMs,
  } }), { wallNow: () => wallMs, poll: async ms => { wallMs += ms; }, ...options });
  return { clock, wall: () => wallMs, reset: () => { generation += 1; } };
}

test.each([.57, 25])('waits for observed simulation time at %sx achieved playback', async rate => {
  const { clock, wall } = harness(rate);
  await clock.sleep(1000);
  expect(clock.now()).toBeGreaterThanOrEqual(1000);
  expect(wall()).toBeGreaterThanOrEqual(1000 / rate);
  expect(clock.observedWallMs).toBe(wall());
});

test('a stopped browser reaches the independent wall deadline', async () => {
  const { clock, wall } = harness(0, { wallTimeoutMs: 100 });
  await expect(clock.sleep(1000)).rejects.toThrow('wall-clock deadline');
  expect(wall()).toBe(100);
});

test('a nested wait honors its callers earlier wall deadline', async () => {
  const { clock, wall } = harness(0);
  await expect(clock.sleep(1000, { wallDeadlineMs: 60 })).rejects.toThrow('wall-clock deadline');
  expect(wall()).toBe(60);
});

test('rejects a scene reset during a wait', async () => {
  let generation = 1;
  const clock = createBrowserClock(async () => ({ simulationClock: {
    time_ms: 0, generation,
  } }), { poll: async () => { generation += 1; } });
  await expect(clock.sleep(1000)).rejects.toThrow('reset');
});

test('an old browser cannot silently fall back to requested speed', async () => {
  const clock = createBrowserClock(async () => ({}));
  await expect(clock.sleep(1000)).rejects.toThrow('reload');
});
