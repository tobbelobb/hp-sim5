import { existsSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { collectSweepData, MACHINE_CONFIGS } from '../../behaviors/sweep_data_collection.mjs';
import { tuneForce } from '../../behaviors/force_tuning.mjs';
import { collectWithSlipRecovery } from '../../behaviors/slip_recovery.mjs';

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
