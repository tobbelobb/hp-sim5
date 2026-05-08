import { parseEncoderReply, sleep as baseSleep } from './encoder_utils.mjs';

const MAD_SCALE = 1.4826;
const DEFAULT_NOISE_SAMPLE_COUNT = 16;
const DEFAULT_NOISE_SAMPLE_RATE_HZ = 40;
const DEFAULT_NOISE_MIN_SAMPLES = 8;
const DEFAULT_NOISE_SIGMA_FLOOR_DEG = 0.02;

export const ENCODER_NOISE_DEFAULTS = {
  DEFAULT_NOISE_SAMPLE_COUNT,
  DEFAULT_NOISE_SAMPLE_RATE_HZ,
  DEFAULT_NOISE_MIN_SAMPLES,
  DEFAULT_NOISE_SIGMA_FLOOR_DEG,
  MAD_SCALE,
};

function median(values) {
  if (!Array.isArray(values) || values.length === 0) {
    return NaN;
  }
  const sorted = values.slice().sort((a, b) => a - b);
  const mid = Math.floor(sorted.length / 2);
  if (sorted.length % 2 === 1) {
    return sorted[mid];
  }
  return 0.5 * (sorted[mid - 1] + sorted[mid]);
}

export function robustStdMad(values) {
  if (!Array.isArray(values)) {
    return NaN;
  }
  const finite = values.filter((v) => Number.isFinite(v));
  if (finite.length === 0) {
    return NaN;
  }
  const med = median(finite);
  if (!Number.isFinite(med)) {
    return NaN;
  }
  const absDev = finite.map((v) => Math.abs(v - med));
  const mad = median(absDev);
  if (!Number.isFinite(mad)) {
    return NaN;
  }
  return MAD_SCALE * mad;
}

function meanFinite(values) {
  if (!Array.isArray(values)) {
    return NaN;
  }
  let sum = 0;
  let count = 0;
  for (const v of values) {
    if (!Number.isFinite(v)) {
      continue;
    }
    sum += v;
    count += 1;
  }
  if (count === 0) {
    return NaN;
  }
  return sum / count;
}

export function applySigmaFloor(sigmas, sigmaFloor) {
  const floor = Number.isFinite(sigmaFloor) && sigmaFloor >= 0 ? sigmaFloor : 0;
  const sigmaEff = [];
  const belowFloor = [];
  const nonFinite = [];
  for (const s of Array.isArray(sigmas) ? sigmas : []) {
    if (!Number.isFinite(s)) {
      sigmaEff.push(floor);
      belowFloor.push(true);
      nonFinite.push(true);
      continue;
    }
    if (s < floor) {
      sigmaEff.push(floor);
      belowFloor.push(true);
      nonFinite.push(false);
    } else {
      sigmaEff.push(s);
      belowFloor.push(false);
      nonFinite.push(false);
    }
  }
  return { sigmaEff, belowFloor, nonFinite, sigmaFloor: floor };
}

export async function sampleEncoderNoise(sendFn, motorIds, options = {}) {
  if (!Array.isArray(motorIds) || motorIds.length === 0) {
    return {
      muByMotorDeg: [],
      sigmaByMotorDeg: [],
      sampleCount: 0,
      durationMs: 0,
      intervalMs: 0,
      samplingHz: null,
    };
  }
  const {
    sampleCount = DEFAULT_NOISE_SAMPLE_COUNT,
    sampleRateHz = DEFAULT_NOISE_SAMPLE_RATE_HZ,
    sampleIntervalMs = null,
    speedup,
    sleepFn = baseSleep,
    nowFn = () => Date.now(),
  } = options;

  const timeScale = Number.isFinite(speedup) && speedup > 0 ? speedup : 1;
  const intervalRaw = Number.isFinite(sampleIntervalMs)
    ? sampleIntervalMs
    : (1000 / Math.max(1e-6, Number.isFinite(sampleRateHz) ? sampleRateHz : DEFAULT_NOISE_SAMPLE_RATE_HZ));
  const intervalMsEff = Math.max(1, intervalRaw / timeScale);
  const targetSamples = Math.max(1, Math.floor(sampleCount));

  const samplesByMotor = Array.from({ length: motorIds.length }, () => []);
  let samples = 0;
  const startMs = nowFn();

  for (let i = 0; i < targetSamples; i += 1) {
    // eslint-disable-next-line no-await-in-loop
    const reply = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const angles = parseEncoderReply(reply?.reply);
    if (angles.length === motorIds.length && angles.every((v) => Number.isFinite(v))) {
      for (let idx = 0; idx < angles.length; idx += 1) {
        samplesByMotor[idx].push(angles[idx]);
      }
      samples += 1;
    }
    // eslint-disable-next-line no-await-in-loop
    await sleepFn(intervalMsEff);
  }

  const durationMs = nowFn() - startMs;
  const muByMotorDeg = samplesByMotor.map((values) => meanFinite(values));
  const sigmaByMotorDeg = samplesByMotor.map((values) => robustStdMad(values));
  const samplingHz = durationMs > 0 ? (samples / durationMs) * 1000 : null;

  return {
    muByMotorDeg,
    sigmaByMotorDeg,
    sampleCount: samples,
    durationMs,
    intervalMs: intervalMsEff,
    samplingHz,
  };
}
