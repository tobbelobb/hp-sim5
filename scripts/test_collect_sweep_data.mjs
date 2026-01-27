#!/usr/bin/env node
import { strict as assert } from 'node:assert';
import {
  angleToLength,
} from './uncalibrated_actions.mjs';
import {
  combinations,
  generateSweepConfigs,
  MACHINE_CONFIGS,
  selectRepresentativeConfigs,
  validateSweepConfig,
} from './sweep_data_collection.mjs';

function testGenerateSweepConfigs() {
  const slideConfigs = generateSweepConfigs('slideprinter');
  assert.equal(slideConfigs.length, 3, 'Slideprinter should produce 3 canonical sweep configs');

  const hp4Configs = generateSweepConfigs('hangprinter_4');
  assert.equal(hp4Configs.length, 6, 'Hangprinter_4 should produce 6 sweep configs when sensor 3 is blocked');
}

function testCombinations() {
  const result = combinations([0, 1, 2, 3], 2);
  assert.equal(result.length, 6);
}

function testSelectRepresentative() {
  const allConfigs = generateSweepConfigs('hangprinter_4');
  const selected = selectRepresentativeConfigs(allConfigs, MACHINE_CONFIGS.hangprinter_4, 6);
  assert.equal(selected.length, 6);
  const drives = new Set(selected.map((c) => c.driveAnchor));
  assert(drives.size >= 3, 'Expected at least three distinct drive anchors in representative set');
}

function testValidateSweepConfig() {
  const config = { numAnchors: 3, dimensions: 2, forbiddenSensors: [], axes: ['X', 'Y', 'Z'] };
  validateSweepConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 2 }, config);
  assert.throws(
    () => validateSweepConfig({ fixedAnchors: [0, 0], driveAnchor: 1, sensorAnchor: 2 }, config),
    /duplicate/i,
  );
  assert.throws(
    () => validateSweepConfig({ fixedAnchors: [0], driveAnchor: 1, sensorAnchor: 3 }, config),
    /range/i,
  );
}

function testAngleToLength() {
  const mmPerDeg = [0.5, 1.0];
  assert.equal(angleToLength(10, 0, mmPerDeg), 5);
  assert.equal(angleToLength(10, 1, mmPerDeg), 10);
  assert.equal(angleToLength(10, 2, mmPerDeg), 0);
}

testGenerateSweepConfigs();
testCombinations();
testSelectRepresentative();
testValidateSweepConfig();
testAngleToLength();

console.log('collect_sweep_data.mjs tests: PASSED');
