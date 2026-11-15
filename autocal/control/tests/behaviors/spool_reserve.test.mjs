import { probeSpoolInterval } from '../../behaviors/spool_reserve.mjs';

function trial(mode = 'coupled') {
  const visited = [];
  const sample = async target => {
    visited.push(target);
    let sensor = -4 * target;
    if (mode === 'flat') sensor = 0;
    if (mode === 'noise') sensor = -0.0001 * target;
    if (mode === 'reversed' && target === 50) sensor = -480;
    if (mode === 'hysteresis' && visited.length > 1 && target === 100) sensor += 1;
    if (mode === 'transport') throw new Error('Disconnected');
    return { muByMotorDeg: [mode === 'lost motion' ? 100 : target, sensor], sigmaByMotorDeg: [.01, .01] };
  };
  return { visited, run: () => probeSpoolInterval(sample, {
    start: 100, away: 120, toward: 50, driveAnchor: 0, sensorAnchor: 1,
    mmPerDeg: [1, 1], sigmaFloorDeg: .01,
  }) };
}

test('promotes only the reversibly probed interval and restores the reserved start', async () => {
  const t = trial();
  expect(await t.run()).toMatchObject({ accepted: true, tested_drive_mm: [100, 50] });
  expect(t.visited).toEqual([100, 120, 100, 50, 100]);
});

test.each(['flat', 'noise', 'reversed', 'lost motion'])(
  'retains reserve for %s even when encoders are stationary', async mode => {
    const t = trial(mode);
    expect((await t.run()).accepted).toBe(false);
    expect(t.visited.at(-1)).toBe(100);
  },
);

test('stops when reversal cannot restore the previously reserved state', async () => {
  const t = trial('hysteresis');
  await expect(t.run()).rejects.toThrow('did not return');
  expect(t.visited.at(-1)).toBe(100);
});

test('does not reinterpret transport failure as a rejected probe or retry it', async () => {
  const t = trial('transport');
  await expect(t.run()).rejects.toThrow('Disconnected');
  expect(t.visited).toEqual([100]);
});
