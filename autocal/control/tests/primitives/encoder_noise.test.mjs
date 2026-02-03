import { applySigmaFloor, robustStdMad } from '../../primitives/encoder_noise.mjs';

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
