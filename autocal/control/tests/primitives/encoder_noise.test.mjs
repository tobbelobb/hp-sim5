import { applySigmaFloor, robustStdMad, sampleEncoderNoise } from '../../primitives/encoder_noise.mjs';

describe('encoder_noise helpers', () => {
  test('robustStdMad uses MAD scaling', () => {
    const sigma = robustStdMad([1, 2, 3, 4, 100]);
    expect(sigma).toBeCloseTo(1.4826, 4);
  });

  test('applySigmaFloor floors and flags', () => {
    const { sigmaEff, belowFloor, nonFinite } = applySigmaFloor([0.005, 0.02, NaN], 0.01);
    expect(sigmaEff).toEqual([0.01, 0.02, 0.01]);
    expect(belowFloor).toEqual([true, false, true]);
    expect(nonFinite).toEqual([false, false, true]);
  });
});

describe('sampleEncoderNoise mean precision', () => {
  async function sample(values) {
    let index = 0;
    let now = 0;
    const send = async () => ({ reply: String(values[index++]) });
    return sampleEncoderNoise(send, ['40.0'], {
      sampleCount: values.length, sampleIntervalMs: 1,
      sleepFn: async ms => { now += ms; }, nowFn: () => now,
    });
  }

  test('constant observations retain their reported angle exactly', async () => {
    const stats = await sample(Array(16).fill(987.3));
    expect(stats.muByMotorDeg).toEqual([987.3]);
    expect(stats.sigmaByMotorDeg).toEqual([0]);
    expect(stats.sampleCount).toBe(16);
  });

  test('small deviations are averaged at a large unwrapped angle', async () => {
    const values = [...Array(8).fill(1000000), ...Array(8).fill(1000000.125)];
    const stats = await sample(values);
    expect(stats.muByMotorDeg).toEqual([1000000.0625]);
    expect((await sample(values.reverse())).muByMotorDeg).toEqual(stats.muByMotorDeg);
  });
});
