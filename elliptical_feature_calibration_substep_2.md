# Substep 2: Sweep Data Collector Script

## Overview

Create or modify the data collection script to perform "circular sweep" measurements where N-1 cables are held at fixed length while one cable is driven and another is passively measured. This produces the (l_drive, l_sensor) pairs needed for ellipse fitting. Carry over the masterplan constraint: on Hangprinter variants the top "carrying" anchor should never be assigned the Sensor/torque role during sweeps.

We cannot record absolute cable lengths during collection because anchor locations are unknown. The script must therefore zero encoders at the origin and log *relative* lengths (`ΔL` from origin) for `fixed_lengths`, `l_drive`, and `l_sensor`. Phase 2 will reconstruct absolute lengths as `L_abs = ||anchor - origin|| + ΔL` using the current anchor guess before comparing against theoretical ellipses or re-fitting if needed. Keep the raw encoder angles in the log (alongside the derived lengths) so later physics models—sag/flex/buildup—can reinterpret the data without recollecting, and so QC tooling can visualize where along the arc the sampler spent the most time and how noise varies.

## Implementation Details

### 2.1 Script Design: `collect_sweep_data.mjs`

This script extends the existing `collect_encoder_data.mjs` with sweep-specific functionality.

```javascript
#!/usr/bin/env node
/**
 * collect_sweep_data.mjs
 *
 * Collects sweep data for elliptical feature calibration.
 * Supports: Slideprinter (3 anchors, 2D), Hangprinter (4-5 anchors, 3D),
 *           CubeCorners (8 anchors, 3D), SkyCam (4 anchors, 3D)
 */

import fs from 'node:fs/promises';
import { spawn } from 'node:child_process';
import { createGcodeBridge, parseBridgeArgs } from './gcode_bridge.mjs';
import { STEP_CLOCK_HZ } from '../examples/js/slideprinter/rrfMotionUtils.js';

// Machine configurations
const MACHINE_CONFIGS = {
  slideprinter: { numAnchors: 3, dimensions: 2, axes: ['X', 'Y', 'Z'], forbiddenSensors: [] },
  hangprinter_4: { numAnchors: 4, dimensions: 3, axes: ['A', 'B', 'C', 'D'], forbiddenSensors: [3] }, // Anchor 3 carries, never Sensor
  hangprinter_5: { numAnchors: 5, dimensions: 3, axes: ['A', 'B', 'C', 'D', 'I'], forbiddenSensors: [4] }, // Anchor 4 carries, never Sensor
  cubecorners: { numAnchors: 8, dimensions: 3, axes: ['A', 'B', 'C', 'D', 'I', 'J', 'K', 'L'], forbiddenSensors: [] },
  skycam: { numAnchors: 4, dimensions: 3, axes: ['A', 'B', 'C', 'D'], forbiddenSensors: [] },
};

// Motor IDs for Slideprinter (extend for other machines)
const MOTOR_IDS_BY_MACHINE = {
  slideprinter: ['40.0', '41.0', '42.0'],
  hangprinter_4: ['40.0', '41.0', '42.0', '43.0'],
  hangprinter_5: ['40.0', '41.0', '42.0', '43.0', '44.0'],
  // Add others as needed
};

const DEFAULT_SWEEP_RANGE_MM = 50;  // ±50mm sweep
const DEFAULT_SWEEP_POINTS = 21;    // Points per sweep
const DEFAULT_FEED = 2000;
const DEFAULT_TORQUE = 0.05;
const DEFAULT_SETTLE_MS = 200;

/**
 * Generate sweep configurations for a machine type.
 */
function generateSweepConfigs(machineType, forbiddenSensors = null) {
  const config = MACHINE_CONFIGS[machineType];
  if (!config) throw new Error(`Unknown machine type: ${machineType}`);

  const n = config.numAnchors;
  const k = config.dimensions - 1;  // Number of fixed anchors for 1-DOF
  const sensorBlock = new Set(forbiddenSensors ?? config.forbiddenSensors ?? []);

  const configs = [];

  // Generate all combinations of k fixed anchors
  const fixedCombinations = combinations(range(n), k);

  for (const fixed of fixedCombinations) {
    const fixedSet = new Set(fixed);
    const freeAnchors = range(n).filter(i => !fixedSet.has(i));

    // Generate all permutations of (drive, sensor) from free anchors
    for (const [drive, sensor] of permutations(freeAnchors, 2)) {
      if (sensorBlock.has(sensor)) continue;  // Never assign carrying anchor to Sensor
      configs.push({
        fixedAnchors: fixed,
        driveAnchor: drive,
        sensorAnchor: sensor,
      });
    }
  }

  return configs;
}

/**
 * Helper: Generate combinations C(arr, k)
 */
function combinations(arr, k) {
  if (k === 0) return [[]];
  if (arr.length < k) return [];

  const [first, ...rest] = arr;
  const withFirst = combinations(rest, k - 1).map(c => [first, ...c]);
  const withoutFirst = combinations(rest, k);
  return [...withFirst, ...withoutFirst];
}

/**
 * Helper: Generate permutations P(arr, k)
 */
function permutations(arr, k) {
  if (k === 0) return [[]];
  const result = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = [...arr.slice(0, i), ...arr.slice(i + 1)];
    for (const perm of permutations(rest, k - 1)) {
      result.push([arr[i], ...perm]);
    }
  }
  return result;
}

function range(n) {
  return Array.from({ length: n }, (_, i) => i);
}

/**
 * Perform a single sweep.
 */
async function performSweep(sendFn, machineConfig, sweepConfig, options) {
  const {
    axes,
    motorIds,
    sweepRangeMm,
    sweepPoints,
    feed,
    torque,
    settleMs,
    speedup,
  } = options;

  const { fixedAnchors, driveAnchor, sensorAnchor } = sweepConfig;

  const dataPoints = [];

  // Put sensor motor in torque mode
  const sensorMotorId = motorIds[sensorAnchor];
  await sendFn(`M569.4 P${sensorMotorId} T${torque}`);

  // Calculate sweep positions
  const stepSize = (2 * sweepRangeMm) / (sweepPoints - 1);
  const startPos = -sweepRangeMm;

  // Record initial lengths from encoders
  const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
  const initialAngles = parseEncoderReply(encoderReply?.reply);

  // Move to start position
  const driveAxis = axes[driveAnchor];
  await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${startPos} F${feed}`, speedup);
  await sleep(settleMs / speedup);

  // Sweep across range
  for (let i = 0; i < sweepPoints; i++) {
    const targetDelta = startPos + i * stepSize;

    if (i > 0) {
      // Move to next position
      await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${stepSize} F${feed}`, speedup);
      await sleep(settleMs / speedup);
    }

    // Read all encoder positions
    const encoderMid = await sendFn(`M569.3 P${motorIds.join(':')}`);
    const anglesDeg = parseEncoderReply(encoderMid?.reply);

    // Convert angles to lengths (simplified - actual conversion depends on machine)
    const lengths = anglesDeg.map((angle, idx) =>
      angleToLength(angle, idx, options.mmPerDeg)
    );

    dataPoints.push({
      l_drive: lengths[driveAnchor],
      l_sensor: lengths[sensorAnchor],
      timestamp_ms: Date.now(),
      raw_angles: anglesDeg, // Preserve raw encoders for future physics models and coverage/noise visualization
    });

    console.log(`  Point ${i + 1}/${sweepPoints}: drive=${lengths[driveAnchor].toFixed(2)}, sensor=${lengths[sensorAnchor].toFixed(2)}`);
  }

  // Return sensor motor to position mode
  await sendFn(`M569.4 P${sensorMotorId} T0.0`);

  // Return to start
  await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${-sweepRangeMm - (sweepPoints - 1) * stepSize} F${feed}`, speedup);

  return dataPoints;
}

/**
 * Get current cable lengths from encoders.
 */
async function getCurrentLengths(sendFn, motorIds, mmPerDeg) {
  const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
  const anglesDeg = parseEncoderReply(encoderReply?.reply);
  return anglesDeg.map((angle, idx) => angleToLength(angle, idx, mmPerDeg));
}

/**
 * Main function.
 */
async function main() {
  const args = parseBridgeArgs(process.argv.slice(2));

  const machineType = args.machineType || 'slideprinter';
  const machineConfig = MACHINE_CONFIGS[machineType];
  if (!machineConfig) {
    console.error(`Unknown machine type: ${machineType}`);
    process.exit(1);
  }

  const motorIds = MOTOR_IDS_BY_MACHINE[machineType];
  const axes = machineConfig.axes;

  const sweepRangeMm = parseFloat(args.sweepRange) || DEFAULT_SWEEP_RANGE_MM;
  const sweepPoints = parseInt(args.sweepPoints) || DEFAULT_SWEEP_POINTS;
  const feed = parseFloat(args.feed) || DEFAULT_FEED;
  const torque = parseFloat(args.torque) || DEFAULT_TORQUE;
  const settleMs = parseFloat(args.settleMs) || DEFAULT_SETTLE_MS;
  const speedup = parseFloat(args.speedup) || 1;
  const maxSweeps = parseInt(args.maxSweeps) || 6;

  // Generate sweep configurations
  let sweepConfigs = generateSweepConfigs(machineType);
  if (sweepConfigs.length > maxSweeps) {
    sweepConfigs = selectRepresentativeConfigs(
      sweepConfigs,
      machineConfig.numAnchors,
      maxSweeps,
      machineConfig.forbiddenSensors
    );
  }

  console.log(`Machine: ${machineType} (${machineConfig.numAnchors} anchors, ${machineConfig.dimensions}D)`);
  console.log(`Sweep range: ±${sweepRangeMm}mm, ${sweepPoints} points per sweep`);
  console.log(`Number of sweeps: ${sweepConfigs.length}`);

  // Connect to RRF
  const bridgeCtx = createGcodeBridge({
    server: args.server || `http://localhost:${args.port || 8081}`,
    wsPort: args.noWs ? 0 : args.wsPort,
    quiet: args.quiet,
  });

  const send = async (line) => {
    const res = await bridgeCtx.sendGcodeLine(line);
    return res;
  };

  // Get mm per degree conversion factors
  const m666Reply = await send('M666');
  const m666Values = parseM666(m666Reply?.reply);
  const mmPerDeg = axes.map((_, i) => computeMmPerDegree(m666Values, i));

  // Initialize
  await send('G92 X0 Y0 Z0');
  await send('G91');  // Relative mode
  await send('M666 Q0');  // Disable line length compensation
  await send(`M569.3 P${motorIds.join(':')} S`);  // Reset encoder angles

  const sweeps = [];

  try {
    for (let i = 0; i < sweepConfigs.length; i++) {
      const sweepConfig = sweepConfigs[i];
      console.log(`\nSweep ${i + 1}/${sweepConfigs.length}:`);
      console.log(`  Fixed anchors: [${sweepConfig.fixedAnchors.join(', ')}]`);
      console.log(`  Drive: ${sweepConfig.driveAnchor}, Sensor: ${sweepConfig.sensorAnchor}`);

      // Get current fixed lengths (encoder deltas relative to origin; absolute length will be reconstructed later)
      const currentLengths = await getCurrentLengths(send, motorIds, mmPerDeg);
      const fixedLengths = sweepConfig.fixedAnchors.map(idx => currentLengths[idx]);

      const dataPoints = await performSweep(send, machineConfig, sweepConfig, {
        axes,
        motorIds,
        sweepRangeMm,
        sweepPoints,
        feed,
        torque,
        settleMs,
        speedup,
        mmPerDeg,
      });

      sweeps.push({
        id: `sweep_${String(i + 1).padStart(3, '0')}`,
        fixed_anchors: sweepConfig.fixedAnchors,
        fixed_lengths: fixedLengths,
        drive_anchor: sweepConfig.driveAnchor,
        sensor_anchor: sweepConfig.sensorAnchor,
        drive_range: { start: -sweepRangeMm, end: sweepRangeMm, unit: 'mm' },
        data_points: dataPoints.map(p => ({
          l_drive: p.l_drive,
          l_sensor: p.l_sensor,
          timestamp_ms: p.timestamp_ms,
          raw_angles_deg: p.raw_angles,
        })),
        metadata: {
          feed_rate: feed,
          torque: torque,
          settle_ms: settleMs,
        },
      });

      console.log(`  Collected ${dataPoints.length} points`);
    }

    // Build output dataset
    const dataset = {
      version: '1.0',
      machine_type: machineType,
      num_anchors: machineConfig.numAnchors,
      dimensions: machineConfig.dimensions,
      timestamp: new Date().toISOString(),
      sweeps: sweeps,
      fitted_ellipses: [],  // To be filled by Python fitting script
    };

    // Save output
    const outputFile = args.outputFile || `sweep_data_${machineType}_${Date.now()}.json`;
    await fs.writeFile(outputFile, JSON.stringify(dataset, null, 2));
    console.log(`\nSaved ${sweeps.length} sweeps to ${outputFile}`);

  } finally {
    bridgeCtx.close();
  }
}

/**
 * Select representative sweep configurations.
 */
function selectRepresentativeConfigs(allConfigs, numAnchors, maxSweeps, forbiddenSensors = []) {
  const selected = [];
  const usedAsDrive = new Set();
  const usedAsSensor = new Set();
  const sensorBlock = new Set(forbiddenSensors);

  // First pass: ensure coverage
  for (const cfg of allConfigs) {
    if (sensorBlock.has(cfg.sensorAnchor)) continue;
    if (cfg.driveAnchor in usedAsDrive && cfg.sensorAnchor in usedAsSensor) {
      continue;
    }
    selected.push(cfg);
    usedAsDrive.add(cfg.driveAnchor);
    usedAsSensor.add(cfg.sensorAnchor);

    if (selected.length >= maxSweeps) break;
  }

  // Second pass: fill remaining
  if (selected.length < maxSweeps) {
    const remaining = allConfigs.filter(c => !selected.includes(c));
    selected.push(...remaining.slice(0, maxSweeps - selected.length));
  }

  return selected;
}

// Helper functions (imported or defined)
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function parseEncoderReply(reply) {
  if (typeof reply !== 'string') return [];
  const matches = reply.match(/-?[0-9]+(?:\.[0-9]+)?/g);
  return matches ? matches.map(v => parseFloat(v)) : [];
}

function parseM666(reply) {
  // Same as in collect_encoder_data.mjs
  if (typeof reply !== 'string') return {};
  const values = {};
  const regex = /([A-Z])\s*([-0-9.:]+)/g;
  let match;
  while ((match = regex.exec(reply))) {
    const key = match[1];
    const rawVal = match[2];
    values[key] = rawVal.includes(':')
      ? rawVal.split(':').map(v => parseFloat(v))
      : parseFloat(rawVal);
  }
  return values;
}

function computeMmPerDegree(m666Values, axisIdx) {
  // Same calculation as in collect_encoder_data.mjs
  const radii = m666Values.R;
  const mechAdv = m666Values.U;
  const linesPerSpool = m666Values.O;
  const motorGear = m666Values.L;
  const spoolGear = m666Values.H;

  const r = Array.isArray(radii) ? radii[axisIdx] : radii;
  if (!Number.isFinite(r) || r === 0) return null;

  const ma = Math.max(Array.isArray(mechAdv) ? mechAdv[axisIdx] : mechAdv || 1, 1e-6);
  const lines = Math.max(Array.isArray(linesPerSpool) ? linesPerSpool[axisIdx] : linesPerSpool || 1, 1e-6);
  const motorGearTeeth = Math.max(Array.isArray(motorGear) ? motorGear[axisIdx] : motorGear || 1, 1e-6);
  const spoolGearTeeth = Math.max(Array.isArray(spoolGear) ? spoolGear[axisIdx] : spoolGear || 1, 1e-6);
  const gearFactor = spoolGearTeeth / motorGearTeeth;

  return (2 * Math.PI * r) / (gearFactor * ma * lines * 360.0);
}

function angleToLength(angleDeg, axisIdx, mmPerDeg) {
  // Convert encoder angle to cable length delta
  const factor = Array.isArray(mmPerDeg) ? mmPerDeg[axisIdx] : mmPerDeg;
  if (!Number.isFinite(factor)) return 0;
  return angleDeg * factor;
}

async function runMoveWithWait(sendFn, gcode, speedup = 1) {
  const result = await sendFn(gcode);
  // Estimate move duration
  const feedMatch = gcode.match(/\bF([0-9]+(?:\.[0-9]+)?)/i);
  const feed = feedMatch ? parseFloat(feedMatch[1]) : DEFAULT_FEED;
  const coordMatch = gcode.match(/([XYZ])(-?[0-9]+(?:\.[0-9]+)?)/gi) || [];
  let dist = 0;
  for (const m of coordMatch) {
    const val = parseFloat(m.slice(1));
    dist += val * val;
  }
  dist = Math.sqrt(dist);

  if (feed > 0 && dist > 0) {
    await sleep(((dist / (feed / 60)) + 0.1) * 1000 / speedup);
  } else {
    await sleep(500 / speedup);
  }
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
```

### 2.2 Command Line Interface

```
Usage: node collect_sweep_data.mjs [options]

Options:
  --machineType     Machine configuration (slideprinter|hangprinter_4|hangprinter_5|cubecorners|skycam)
  --sweepRange      Sweep range in mm (default: 50)
  --sweepPoints     Number of points per sweep (default: 21)
  --maxSweeps       Maximum number of sweeps to collect (default: 6)
  --feed            Feed rate in mm/min (default: 2000)
  --torque          Sensor motor torque in Nm (default: 0.05)
  --settleMs        Settle time between points in ms (default: 200)
  --speedup         Simulation speedup factor (default: 1)
  --outputFile      Output JSON file path
  --server          RRF server URL
  --port            RRF server port (default: 8081)
  --noWs            Disable WebSocket connection
  --quiet           Suppress progress output

Example:
  node collect_sweep_data.mjs --machineType slideprinter --sweepRange 100 --sweepPoints 41 --outputFile sweep_data.json
```

### 2.3 Sweep Point File Format (Alternative Input)

For manual control over sweep configurations, a sweep points file can be used:

```
# sweep_configs.txt
# Format: [fixed_anchors] drive_anchor sensor_anchor
# Lines starting with # are comments

# Slideprinter configurations
[0] 1 2
[0] 2 1
[1] 0 2
[1] 2 0
[2] 0 1
[2] 1 0
```

### 2.4 Continuous Sweep Mode (High-Frequency Sampling)

For smoother ellipse data, implement continuous sweep mode:

```javascript
async function performContinuousSweep(sendFn, sweepConfig, options) {
  const { driveAnchor, sensorAnchor } = sweepConfig;
  const { axes, motorIds, sweepRangeMm, feed, torque, sampleRateHz } = options;

  const dataPoints = [];
  let collecting = true;

  // Start sampling in background
  const samplingInterval = 1000 / sampleRateHz;
  const samplingPromise = (async () => {
    while (collecting) {
      const encoderReply = await sendFn(`M569.3 P${motorIds.join(':')}`);
      const angles = parseEncoderReply(encoderReply?.reply);
      const lengths = angles.map((a, i) => angleToLength(a, i, options.mmPerDeg));

      dataPoints.push({
        l_drive: lengths[driveAnchor],
        l_sensor: lengths[sensorAnchor],
        timestamp_ms: Date.now(),
        raw_angles_deg: angles,
      });

      await sleep(samplingInterval);
    }
  })();

  // Perform continuous move
  const driveAxis = axes[driveAnchor];
  const sensorMotorId = motorIds[sensorAnchor];

  await sendFn(`M569.4 P${sensorMotorId} T${torque}`);
  await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${-sweepRangeMm} F${feed}`, 1);
  await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${2 * sweepRangeMm} F${feed}`, 1);
  await runMoveWithWait(sendFn, `G1 H2 ${driveAxis}${-sweepRangeMm} F${feed}`, 1);

  // Stop sampling
  collecting = false;
  await samplingPromise;
  await sendFn(`M569.4 P${sensorMotorId} T0.0`);

  return dataPoints;
}
```

### 2.5 Observability Outputs

- Persist `timestamp_ms`, `raw_angles_deg`, and drive setpoints for each sample so QC plots can show sampling density along the arc and noise-versus-arc-position.
- Emit a lightweight histogram per sweep (e.g. bucket by drive delta or fitted φ) plus basic per-bucket variance in a sidecar JSON for quick inspection without reprocessing the whole dataset.

## Testing

### Unit Tests

```javascript
// test_collect_sweep_data.mjs
import { strict as assert } from 'assert';

// Test sweep configuration generation
function testGenerateSweepConfigs() {
  // Slideprinter: 3 anchors, fix 1, permute 2 -> 3 * 2 = 6
  const slideConfigs = generateSweepConfigs('slideprinter');
  assert.equal(slideConfigs.length, 6);

  // Hangprinter 4: C(4,2) = 6, but anchor 3 must never be Sensor -> 9 configs
  const hp4Configs = generateSweepConfigs('hangprinter_4');
  assert.equal(hp4Configs.length, 9);

  console.log('testGenerateSweepConfigs: PASSED');
}

// Test combinations helper
function testCombinations() {
  const result = combinations([0, 1, 2, 3], 2);
  assert.equal(result.length, 6);  // C(4,2) = 6
  console.log('testCombinations: PASSED');
}

// Test representative selection
function testSelectRepresentative() {
  const allConfigs = generateSweepConfigs('hangprinter_4');  // 9 configs
  const selected = selectRepresentativeConfigs(
    allConfigs,
    4,
    6,
    MACHINE_CONFIGS.hangprinter_4.forbiddenSensors
  );
  assert.equal(selected.length, 6);

  // Check all anchors appear as drive at least once
  const drives = new Set(selected.map(c => c.driveAnchor));
  assert(drives.size >= Math.min(4, 6 / 2));

  console.log('testSelectRepresentative: PASSED');
}

testGenerateSweepConfigs();
testCombinations();
testSelectRepresentative();
```

### Integration Tests

```javascript
// Mock RRF responses for testing
const mockBridge = {
  responses: {
    'M666': 'R75.0:75.0:75.0 U2.0:2.0:2.0 O1.0:1.0:1.0 L20 H255',
    'M569.3': '0.0 0.0 0.0',
    'G92': 'ok',
    'G91': 'ok',
    'G1': 'ok',
    'M569.4': 'ok',
  },
  sendGcodeLine: async (line) => {
    for (const [cmd, reply] of Object.entries(mockBridge.responses)) {
      if (line.startsWith(cmd)) {
        return { reply };
      }
    }
    return { reply: 'ok' };
  },
};

async function testSweepDataCollection() {
  // This would run against mockBridge
  console.log('Integration test: sweep data collection');
  // ... implementation
}
```

## Validation Criteria

1. **Sweep Count**: Generated number of sweeps matches the expected count after excluding carrying anchors from Sensor role
2. **Anchor Coverage**: Each anchor appears as drive in at least one sweep; each non-carrying anchor appears as sensor in at least one sweep
3. **Data Completeness**: All `sweepPoints` data points collected for each sweep
4. **JSON Validity**: Output JSON validates against schema from Substep 1
5. **Length Plausibility**: Collected lengths change monotonically during sweep (sanity check)
6. **No Motor Conflicts**: Drive and sensor motors are never the same, fixed motors remain fixed

## Dependencies

- Node.js 16+
- `gcode_bridge.mjs` (existing)
- `rrfMotionUtils.js` (existing)
- RRF firmware with encoder support

## Estimated Complexity

**Effort**: Medium (4-6 hours)

The core sweep logic is straightforward, but edge cases around motor mode switching, error handling, and different machine configurations add complexity. Testing with actual hardware may reveal additional issues.

## Files to Create/Modify

| File | Action |
|------|--------|
| `scripts/collect_sweep_data.mjs` | Create |
| `scripts/test_collect_sweep_data.mjs` | Create |
| `scripts/sweep_configs/slideprinter.txt` | Create (optional) |
| `scripts/sweep_configs/hangprinter_4.txt` | Create (optional) |
