import { MACHINE_CONFIGS, meshMoveFollowed, performSphereMesh, validateMeshPlan } from '../../behaviors/sweep_data_collection.mjs';
import { UnmeasurableSweepError } from '../../behaviors/spool_reserve.mjs';
import { FixedAnchorDriftError } from '../../primitives/uncalibrated_actions.mjs';

// HP4-like geometry. Lengths are reported directly as encoder angles (1 mm/deg).
const HP4 = [[0, -1900, -280], [1645.45, 950, -280], [-1645.45, 950, -280], [0, 0, 1900]];
const HP5 = [[0, -1900, -280], [1900, 0, -280], [0, 1900, -280], [-1900, 0, -280], [0, 0, 1900]];
const AXES = ['X', 'Y', 'Z', 'U', 'V'];
const norm = v => Math.hypot(...v);
const sub = (a, b) => a.map((value, i) => value - b[i]);

// A rigid effector: position-controlled lines fix the pose (least squares), and
// force-controlled lines report the distance to it.
function geometricMachine({ anchors: ANCHORS = HP4, frozenSensor = null, frozenAfterMoves = 0, blockedDrive = null,
  slipAfterMoves = null, slipAnchor = null } = {}) {
  const home = ANCHORS.map(norm);
  let modes = ANCHORS.map(() => 0);
  const lengths = ANCHORS.map(() => 0);
  const modeLog = [];
  let moves = 0;
  let position = [0, 0, 0];
  let now = 0;
  const solve = () => {
    const held = modes.map((mode, i) => (mode === 0 ? i : -1)).filter(i => i >= 0);
    for (let iteration = 0; iteration < 30; iteration += 1) {
      // Gauss-Newton with a small ridge, so fewer than three held lines still move minimally.
      const jtj = [[1e-9, 0, 0], [0, 1e-9, 0], [0, 0, 1e-9]];
      const jtr = [0, 0, 0];
      for (const i of held) {
        const offset = sub(position, ANCHORS[i]);
        const distance = norm(offset);
        const direction = offset.map(value => value / distance);
        const residual = distance - (home[i] + lengths[i]);
        for (let r = 0; r < 3; r += 1) {
          jtr[r] += direction[r] * residual;
          for (let c = 0; c < 3; c += 1) jtj[r][c] += direction[r] * direction[c];
        }
      }
      const [a, b, c] = jtj;
      const det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
      const inverse = [
        [b[1] * c[2] - b[2] * c[1], a[2] * c[1] - a[1] * c[2], a[1] * b[2] - a[2] * b[1]],
        [b[2] * c[0] - b[0] * c[2], a[0] * c[2] - a[2] * c[0], a[2] * b[0] - a[0] * b[2]],
        [b[0] * c[1] - b[1] * c[0], a[1] * c[0] - a[0] * c[1], a[0] * b[1] - a[1] * b[0]],
      ].map(row => row.map(value => value / det));
      position = position.map((value, r) => value - inverse[r].reduce((sum, entry, k) => sum + entry * jtr[k], 0));
    }
    modes.forEach((mode, i) => {
      if (mode !== 0 && !(i === frozenSensor && moves >= frozenAfterMoves)) {
        lengths[i] = norm(sub(position, ANCHORS[i])) - home[i];
      }
    });
  };
  const send = async line => {
    if (line.startsWith('M569.4')) {
      modes = line.split(' T')[1].split(':').map(Number);
      modeLog.push(modes.slice());
      solve();
    } else if (line.startsWith('M569.3')) {
      return { reply: lengths.join(' ') };
    } else if (line.startsWith('G1 H2')) {
      moves += 1;
      for (const [, axis, delta] of line.matchAll(/([XYZUV])([-\d.]+)/g)) {
        const index = AXES.indexOf(axis);
        if (index !== blockedDrive) lengths[index] += Number(delta);
      }
      if (slipAfterMoves !== null && moves === slipAfterMoves) lengths[slipAnchor] += 5;
      solve();
    }
    return { reply: '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  return { send, lengths, home, position: () => position, modeLog };
}

// Points on the sphere around the top anchor, 200 mm shorter than at home.
function meshPlan({ anchors = HP4, count = 12, held = anchors.length - 1, drives = [0, 1], sensor = 2, extras = [] } = {}) {
  const radius = norm(anchors[held]) - 200;
  const predict = position => anchors.map(anchor => norm(sub(position, anchor)) - norm(anchor));
  const points = Array.from({ length: count }, (_, i) => {
    const angle = (i / count) * 2 * Math.PI;
    const offset = [300 * Math.cos(angle), 300 * Math.sin(angle)];
    const height = anchors[held][2] - Math.sqrt(radius ** 2 - offset[0] ** 2 - offset[1] ** 2);
    const position = [offset[0], offset[1], height];
    const predicted = predict(position);
    return { drive_mm: drives.map(axis => predicted[axis]), sensor_force_n: 2 + i * 0.1, predicted_mm: predicted, position_mm: position };
  });
  const first = points[0].drive_mm[1];
  return { held_anchor: held, held_length_mm: -200, drive_anchors: drives, sensor_anchor: sensor, extra_anchors: extras,
    extra_sensor_force_n: extras.length ? 3 : null, strand_tension_n: 2, strain_model: 'reeving-aware-v2',
    max_sensor_force_n: 5, min_points: 6, probe: { away_mm: first + 10, toward_mm: first - 10 }, points };
}

const run = (machine, plan, overrides = {}) => performSphereMesh(machine.send, plan, {
  axes: AXES.slice(0, plan.points[0].predicted_mm.length),
  motorIds: plan.points[0].predicted_mm.map((_, i) => `${40 + i}.0`), mmPerDeg: plan.points[0].predicted_mm.map(() => 1),
  feed: 1400, speedup: 1, datasetStartMs: 0, forceLow: 0.01, forceMid: 0.23, forceMax: 2.4, noiseSampleCount: 8,
  noiseSampleIntervalMs: 40, noiseMinSamples: 4, noiseSigmaFloorDeg: 0.01, forbiddenForceAnchors: [plan.held_anchor],
  meshId: 'mesh_004', ...overrides,
});

test('a sphere mesh records one held-pair point per planned target, at the true geometry', async () => {
  const machine = geometricMachine();
  const plan = meshPlan();
  const { records } = await run(machine, plan);
  expect(records).toHaveLength(plan.points.length);
  records.forEach((record, index) => {
    const [point] = record.data_points;
    expect(record.fixed_anchors).toEqual([3, 0]);
    expect(record.fixed_lengths[1]).toBeCloseTo(plan.points[index].drive_mm[0], 6);
    expect(record.drive_anchor).toBe(1);
    expect(record.sensor_anchor).toBe(2);
    expect(record.metadata.mesh).toMatchObject({ id: 'mesh_004', held_anchor: 3, index, count: plan.points.length });
    expect(record.metadata.sensor_force_profile).toBe(true);
    expect(point.sensor_force_n).toBe(plan.points[index].sensor_force_n);
    // Moves are commanded to 1 µm.
    expect(point.l_drive).toBeCloseTo(plan.points[index].drive_mm[1], 3);
    const truth = plan.points[index].predicted_mm[2];
    expect(point.l_sensor).toBeCloseTo(truth, 2);
    expect(point.fixed_error_angles_deg[3]).toBeCloseTo(0, 6);
  });
});

test('records state how sensor coupling was established', async () => {
  const { records } = await run(geometricMachine(), meshPlan());
  expect(records[0].data_points[0].spool_reserve_method).toBe('local_encoder_coupling');
  expect(records.slice(1).map(r => r.data_points[0].spool_reserve_method)).toContain('planned_mesh_motion');
});

test('a sensor that freezes later stops the mesh, or rejects it below min_points', async () => {
  // Moves include positioning, held-line corrections and the four probe moves.
  const late = await run(geometricMachine({ frozenSensor: 2, frozenAfterMoves: 16 }), meshPlan());
  expect(late.records.length).toBeGreaterThanOrEqual(6);
  expect(late.records.length).toBeLessThan(12);
  expect(late.records[0].metadata.mesh.stopped.reason).toMatch(/sensor 2 moved/);
  await expect(run(geometricMachine({ frozenSensor: 2, frozenAfterMoves: 8 }), meshPlan()))
    .rejects.toBeInstanceOf(UnmeasurableSweepError);
});

test('a held-line slip during the mesh is reported, not corrected', async () => {
  await expect(run(geometricMachine({ slipAfterMoves: 8, slipAnchor: 3 }), meshPlan()))
    .rejects.toBeInstanceOf(FixedAnchorDriftError);
});

test('HP5 spare ranges use the spare force and count only after following a planned move', async () => {
  const plan = meshPlan({ anchors: HP5, drives: [0, 1], sensor: 2, extras: [3] });
  const machine = geometricMachine({ anchors: HP5 });
  const { records } = await run(machine, plan);
  expect(records[0].data_points[0].extra_sensor_ranges[0].accepted).toBe(false);
  expect(records.slice(1).some(r => r.data_points[0].extra_sensor_ranges[0].accepted)).toBe(true);
  // While measuring, the spare holds the planned spare force and the top line stays held.
  expect(machine.modeLog.some(modes => modes[3] === 3 && modes[4] === 0 && modes[2] > 1.9)).toBe(true);
  expect(machine.modeLog.every(modes => modes[4] === 0)).toBe(true);
});

test('planned motion following accepts only same-direction changes of plausible size', () => {
  expect(meshMoveFollowed(5, 5.2)).toBe(true);
  expect(meshMoveFollowed(0, 5)).toBe(false);
  expect(meshMoveFollowed(-4, 5)).toBe(false);
  expect(meshMoveFollowed(20, 5)).toBe(false);
  expect(meshMoveFollowed(0.1, 1)).toBeNull();
});

test('an uncoupled sensor rejects the mesh before any data is taken', async () => {
  await expect(run(geometricMachine({ frozenSensor: 2 }), meshPlan())).rejects.toBeInstanceOf(UnmeasurableSweepError);
});

test('a drive that does not reach its target rejects the mesh', async () => {
  await expect(run(geometricMachine({ blockedDrive: 1 }), meshPlan())).rejects.toThrow(/did not track/);
});

test('mesh plans are validated against the machine', () => {
  const config = MACHINE_CONFIGS.hangprinter_4;
  expect(() => validateMeshPlan(meshPlan(), config)).not.toThrow();
  expect(() => validateMeshPlan({ ...meshPlan(), held_anchor: 0, drive_anchors: [3, 1] }, config)).toThrow(/must hold anchor 3/);
  expect(() => validateMeshPlan({ ...meshPlan(), held_length_mm: 50 }, config)).toThrow(/fixed-target bounds/);
  expect(() => validateMeshPlan({ ...meshPlan(), sensor_anchor: 0 }, config)).toThrow(/distinct/);
  expect(() => validateMeshPlan({ ...meshPlan(), points: [{ drive_mm: [1], sensor_force_n: 1 }], min_points: 1 }, config)).toThrow(/two finite/);
  const heavy = meshPlan();
  heavy.points[3].sensor_force_n = 9;
  expect(() => validateMeshPlan(heavy, config)).toThrow(/exceeds the applied maximum/);
  expect(() => validateMeshPlan({ ...meshPlan(), probe: undefined }, config)).toThrow(/probe/);
});
