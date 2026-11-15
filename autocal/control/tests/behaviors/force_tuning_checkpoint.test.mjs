import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { collectSweepData, MACHINE_CONFIGS } from '../../behaviors/sweep_data_collection.mjs';
import { tuneForce } from '../../behaviors/force_tuning.mjs';
import { collectWithSlipRecovery } from '../../behaviors/slip_recovery.mjs';
import { UnmeasurableSweepError } from '../../behaviors/spool_reserve.mjs';
import { returnMotorsToOriginOneAtATime } from '../../primitives/uncalibrated_actions.mjs';

jest.mock('../../behaviors/force_tuning.mjs', () => ({
  ...jest.requireActual('../../behaviors/force_tuning.mjs'),
  tuneForce: jest.fn(),
}));
jest.mock('../../behaviors/slip_recovery.mjs', () => ({
  ...jest.requireActual('../../behaviors/slip_recovery.mjs'),
  collectWithSlipRecovery: jest.fn(),
}));
jest.mock('../../primitives/uncalibrated_actions.mjs', () => ({
  ...jest.requireActual('../../primitives/uncalibrated_actions.mjs'),
  waitForStableEncoders: jest.fn().mockResolvedValue(undefined),
  getCurrentLengths: jest.fn().mockResolvedValue([1, 2, 3, 0]),
  returnMotorsToOriginOneAtATime: jest.fn().mockResolvedValue(undefined),
}));

function interruptedCollection(outputFile, args = {}, interrupt = true) {
  const send = async line => {
    if (line.startsWith('M569.4') && interrupt) throw new Error('collection interrupted after tuning');
    if (line === 'M666') return { reply: 'M666 R30:30:30:30 Q0 U1:1:1:1 O1:1:1:1' };
    return { reply: '' };
  };
  return collectSweepData(send, {
    args: { outputFile, maxSweeps: 1, sensorCollectionForce: 1, ...args },
    machineType: 'hangprinter_3', machineConfig: MACHINE_CONFIGS.hangprinter_3,
    motorIds: ['40.0', '41.0', '42.0', '43.0'], speedup: 1,
  });
}

beforeEach(() => {
  tuneForce.mockReset();
  collectWithSlipRecovery.mockReset();
  returnMotorsToOriginOneAtATime.mockReset().mockResolvedValue(undefined);
});

test.each([false, true])('only resolved later boundary failures become rejected-probe evidence, later=%s', async later => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'probe-rejection-'));
  const outputFile = path.join(directory, 'sweeps.json');
  tuneForce.mockResolvedValue({ forceLow: .01, forceMid: .2, forceMax: 2 });
  collectWithSlipRecovery.mockRejectedValue(new UnmeasurableSweepError('No coupled interval', { retained_points: 0 }));
  try {
    const pending = interruptedCollection(outputFile, { fixedTargets: '0,0', completedSweeps: later ? 3 : 0 }, false);
    if (later) {
      await pending;
      const dataset = JSON.parse(readFileSync(outputFile, 'utf8'));
      expect(dataset.sweeps).toEqual([]);
      expect(dataset.acquisition_rejections).toHaveLength(1);
      expect(dataset.acquisition_rejections[0]).toMatchObject({ fixed_anchors: [0,3], fixed_lengths: [0,0], reason: 'No coupled interval' });
      expect(returnMotorsToOriginOneAtATime).toHaveBeenCalledTimes(1);
    } else {
      await expect(pending).rejects.toThrow('No coupled interval');
      expect(existsSync(outputFile)).toBe(false);
    }
  } finally { rmSync(directory, {recursive: true, force: true}); }
});

test('failed return to origin aborts without publishing a boundary', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'probe-return-'));
  const outputFile = path.join(directory, 'sweeps.json');
  tuneForce.mockResolvedValue({ forceLow: .01, forceMid: .2, forceMax: 2 });
  collectWithSlipRecovery.mockRejectedValue(new UnmeasurableSweepError('No coupled interval'));
  returnMotorsToOriginOneAtATime.mockRejectedValueOnce(new Error('Return failed'));
  try {
    await expect(interruptedCollection(outputFile, { fixedTargets: '0,0', completedSweeps: 3 }, false)).rejects.toThrow('Return failed');
    expect(existsSync(outputFile)).toBe(false);
  } finally { rmSync(directory, {recursive: true, force: true}); }
});

test('ordinary collector errors never masquerade as learned reach boundaries', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'probe-transport-'));
  const outputFile = path.join(directory, 'sweeps.json');
  tuneForce.mockResolvedValue({ forceLow: .01, forceMid: .2, forceMax: 2 });
  collectWithSlipRecovery.mockRejectedValue(new Error('Transport disconnected'));
  try {
    await expect(interruptedCollection(outputFile, { fixedTargets: '0,0', completedSweeps: 3 }, false)).rejects.toThrow('Transport disconnected');
    expect(existsSync(outputFile)).toBe(false);
  } finally { rmSync(directory, {recursive: true, force: true}); }
});

test.each([1, 2])('ordinary return failures abort collection with maxSweeps=%s', async maxSweeps => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'ordinary-return-'));
  const outputFile = path.join(directory, 'sweeps.json');
  tuneForce.mockResolvedValue({ forceLow: .01, forceMid: .2, forceMax: 2 });
  collectWithSlipRecovery.mockResolvedValue({ forces: { forceLow: .01, forceMid: .2, forceMax: 2 },
    dataPoints: [{ l_drive: 1, l_sensor: 2 }], recoveries: [] });
  returnMotorsToOriginOneAtATime.mockRejectedValue(new Error('Return movement failed'));
  try {
    await expect(interruptedCollection(outputFile, { fixedTargets: '0,0', maxSweeps, returnToOrigin: true }, false))
      .rejects.toThrow('Return movement failed');
    // A final-return failure retains measurement evidence; a between-sweep
    // failure never publishes a completed multi-sweep dataset.
    expect(existsSync(outputFile)).toBe(maxSweeps === 1);
  } finally { rmSync(directory, {recursive: true, force: true}); }
});

test.each([false, true])('preserves completed tuning before a later failure, tuning_failed=%s', async tuningFailed => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'force-checkpoint-'));
  const outputFile = path.join(directory, 'nested', 'sweeps.json');
  tuneForce.mockResolvedValue({
    forceLow: .0124, forceMid: .248, forceMax: 2.61,
    tuningMeta: { method: 'incremental-compliance', tuning_failed: tuningFailed, noise_sigma_deg: [.02, .03, .02, 0] },
  });
  try {
    await expect(interruptedCollection(outputFile, { sensorCollectionForce: null })).rejects.toThrow('collection interrupted after tuning');
    const checkpoint = JSON.parse(readFileSync(`${outputFile}.force-tuning.json`, 'utf8'));
    expect(checkpoint.stage).toBe(tuningFailed
      ? 'force_tuning_failed_fallback_selected' : 'force_tuning_complete_before_sweeps');
    expect(checkpoint.machine_type).toBe('hangprinter_3');
    expect(checkpoint.m666.R).toEqual([30, 30, 30, 30]);
    expect(checkpoint.requested_sensor_collection_force_n).toBeNull();
    expect(checkpoint.sensor_collection_force_n).toBe(1.24);
    expect(checkpoint.force_tuning).toMatchObject({
      force_low_n: .0124, force_mid_n: .248, force_max_n: 2.61,
      method: 'incremental-compliance', tuning_failed: tuningFailed,
    });
    expect(existsSync(outputFile)).toBe(false);
    expect(existsSync(`${outputFile}.force-tuning.json.tmp`)).toBe(false);
    expect(tuneForce).toHaveBeenCalledTimes(1);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('manual forces replace stale automatic metadata without running tuning', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'force-checkpoint-'));
  const outputFile = path.join(directory, 'sweeps.json');
  writeFileSync(`${outputFile}.force-tuning.json`, JSON.stringify({ force_tuning: { auto_tuned: true } }));
  try {
    await expect(interruptedCollection(outputFile, { forceLow: .01, forceMid: .05, forceMax: .5 }))
      .rejects.toThrow('collection interrupted after tuning');
    expect(tuneForce).not.toHaveBeenCalled();
    const checkpoint = JSON.parse(readFileSync(`${outputFile}.force-tuning.json`, 'utf8'));
    expect(checkpoint.stage).toBe('manual_force_selection_before_sweeps');
    expect(checkpoint.force_tuning).toMatchObject({ auto_tuned: false, method: 'manual', force_low_n: .01, force_mid_n: .05, force_max_n: .5 });
    expect(checkpoint.force_tuning.tuned_at).toBeUndefined();
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('stops before collection when the checkpoint cannot be written', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'force-checkpoint-'));
  const blocked = path.join(directory, 'blocked');
  writeFileSync(blocked, 'existing file');
  tuneForce.mockResolvedValue({ forceLow: .01, forceMid: .25, forceMax: 2 });
  try {
    await expect(interruptedCollection(path.join(blocked, 'sweeps.json'))).rejects.toMatchObject({ code: 'EEXIST' });
    expect(tuneForce).toHaveBeenCalledTimes(1);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('final dataset records recovered forces while checkpoint retains the earlier selection', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'force-checkpoint-'));
  const outputFile = path.join(directory, 'sweeps.json');
  tuneForce.mockResolvedValue({ forceLow: .0124, forceMid: .248, forceMax: 2.61 });
  collectWithSlipRecovery.mockResolvedValue({
    forces: { forceLow: .0124, forceMid: .3, forceMax: 3, sensorCollectionForce: 1.5 },
    dataPoints: [{ l_drive: 1, l_sensor: 2 }], recoveries: [],
  });
  try {
    await interruptedCollection(outputFile, { fixedTargets: '0,0' }, false);
    const checkpoint = JSON.parse(readFileSync(`${outputFile}.force-tuning.json`, 'utf8'));
    const dataset = JSON.parse(readFileSync(outputFile, 'utf8'));
    expect(checkpoint.force_tuning).toMatchObject({ force_mid_n: .248, force_max_n: 2.61, pullout_force_n: 2.61 });
    expect(dataset.config.force_tuning).toMatchObject({ force_mid_n: .3, force_max_n: 3, pullout_force_n: 3 });
    expect(dataset.config.force_tuning.tuned_at).toBe(checkpoint.force_tuning.tuned_at);
    expect(dataset.sweeps[0].metadata.sensor_force_n).toBe(1.5);
    expect(checkpoint.requested_sensor_collection_force_n).toBe(1);
    expect(checkpoint.sensor_collection_force_n).toBe(1);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('final dataset records unprojected collection', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'projection-mode-'));
  const outputFile = path.join(directory, 'sweeps.json');
  tuneForce.mockResolvedValue({ forceLow: .0124, forceMid: .248, forceMax: 2.61 });
  collectWithSlipRecovery.mockResolvedValue({
    forces: { forceLow: .0124, forceMid: .248, forceMax: 2.61 },
    dataPoints: [{ l_drive: 1, l_sensor: 2 }], recoveries: [],
  });
  try {
    await interruptedCollection(outputFile, { fixedTargets: '0,0' }, false);
    const dataset = JSON.parse(readFileSync(outputFile, 'utf8'));
    expect(dataset.config.noise_model.project_zero_tension).toBe(false);
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});

test('selects relative coordinates before RRF motion starts', async () => {
  const directory = mkdtempSync(path.join(os.tmpdir(), 'relative-moves-'));
  const sent = [];
  tuneForce.mockResolvedValue({ forceLow: .0124, forceMid: .248, forceMax: 2.61 });
  collectWithSlipRecovery.mockResolvedValue({
    forces: { forceLow: .0124, forceMid: .248, forceMax: 2.61 },
    dataPoints: [{ l_drive: 1, l_sensor: 2 }], recoveries: [],
  });
  const send = async line => {
    sent.push(line);
    if (line === 'M666') return { reply: 'M666 R30:30:30:30 Q0 U1:1:1:1 O1:1:1:1' };
    return { reply: '' };
  };
  try {
    await collectSweepData(send, {
      args: { outputFile: path.join(directory, 'sweeps.json'), maxSweeps: 1, sensorCollectionForce: 1, fixedTargets: '0,0' },
      machineType: 'hangprinter_3', machineConfig: MACHINE_CONFIGS.hangprinter_3,
      motorIds: ['40.0', '41.0', '42.0', '43.0'], speedup: 1,
    });
    expect(sent.indexOf('G91')).toBeGreaterThanOrEqual(0);
    expect(sent.indexOf('G91')).toBeLessThan(sent.findIndex(line => line.startsWith('M569.4')));
  } finally {
    rmSync(directory, { recursive: true, force: true });
  }
});
