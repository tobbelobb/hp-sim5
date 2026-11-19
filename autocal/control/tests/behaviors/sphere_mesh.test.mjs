import { MACHINE_CONFIGS, performSphereMesh, validateMeshPlan } from '../../behaviors/sweep_data_collection.mjs';
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
  blockedMoves = null, slipAfterMoves = null, slipAnchor = null } = {}) {
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
        const blocked = index === blockedDrive && (blockedMoves === null || blockedMoves.includes(moves));
        if (!blocked) lengths[index] += Number(delta);
      }
      if (slipAfterMoves !== null && moves === slipAfterMoves) lengths[slipAnchor] += 5;
      solve();
    }
    return { reply: '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  return { send, lengths, home, position: () => position, modeLog };
}

// Rows of a sweep of sweeps: each row holds the level drive at one length, so its
// points lie on the circle where the held anchor's and level anchor's spheres meet.
function circlePoint(a, ra, b, rb, angle) {
  const axis = sub(b, a); const d = norm(axis); const u = axis.map(v => v / d);
  const along = (d ** 2 + ra ** 2 - rb ** 2) / (2 * d); const radius = Math.sqrt(ra ** 2 - along ** 2);
  const helper = Math.abs(u[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const cross = (p, q) => [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];
  let e1 = cross(u, helper); const n1 = norm(e1); e1 = e1.map(v => v / n1); const e2 = cross(u, e1);
  return a.map((v, i) => v + along * u[i] + radius * (Math.cos(angle) * e1[i] + Math.sin(angle) * e2[i]));
}

function meshPlan({ anchors = HP4, rows = 3, perRow = 4, held = anchors.length - 1, drives = [0, 1], sensor = 2, extras = [] } = {}) {
  const heldLength = -200;
  const predict = position => anchors.map(anchor => norm(sub(position, anchor)) - norm(anchor));
  // Choose row levels from points below the held anchor, then step the second drive along each row's circle.
  const centre = [0, 0, 400];
  const baseLevel = norm(sub(centre, anchors[drives[0]])) - norm(anchors[drives[0]]);
  const points = [];
  for (let row = 0; row < rows; row += 1) {
    const level = baseLevel + (row - 1) * 60;
    for (let step = 0; step < perRow; step += 1) {
      // Bottom of the circle is nearest the workspace; sweep a short arc around it.
      const candidates = Array.from({ length: 72 }, (_, i) => circlePoint(anchors[held], norm(anchors[held]) + heldLength,
        anchors[drives[0]], norm(anchors[drives[0]]) + level, (i / 72) * 2 * Math.PI));
      const bottom = candidates.reduce((best, p) => (p[2] < best[2] ? p : best));
      const bottomIndex = candidates.indexOf(bottom);
      const position = candidates[(bottomIndex + (step - perRow / 2) * 3 + 72) % 72];
      const predicted = predict(position);
      points.push({ row, drive_mm: [level, predicted[drives[1]]], sensor_force_n: 2 + step * 0.1,
        predicted_mm: predicted, position_mm: position });
    }
  }
  return { held_anchor: held, held_length_mm: heldLength, drive_anchors: drives, sensor_anchor: sensor, extra_anchors: extras,
    extra_sensor_force_n: extras.length ? 3 : null, strand_tension_n: 2, strain_model: 'reeving-aware-v2',
    max_sensor_force_n: 16, min_points: 6, probe_radius_mm: 10, points };
}

const run = (machine, plan, overrides = {}) => performSphereMesh(machine.send, plan, {
  axes: AXES.slice(0, plan.points[0].predicted_mm.length),
  motorIds: plan.points[0].predicted_mm.map((_, i) => `${40 + i}.0`), mmPerDeg: plan.points[0].predicted_mm.map(() => 1),
  feed: 1400, speedup: 1, datasetStartMs: 0, forceLow: 0.01, forceMid: 0.23, forceMax: 2.4, noiseSampleCount: 8,
  noiseSampleIntervalMs: 40, noiseMinSamples: 4, noiseSigmaFloorDeg: 0.01, forbiddenForceAnchors: [plan.held_anchor],
  meshId: 'mesh_004', ...overrides,
});

test('a sphere mesh records each row as a sweep, at the true geometry', async () => {
  const plan = meshPlan();
  const { records, skipped } = await run(geometricMachine(), plan);
  expect(skipped).toEqual([]);
  expect(records).toHaveLength(3);
  records.forEach((record, row) => {
    const planned = plan.points.filter(point => point.row === row);
    expect(record.fixed_anchors).toEqual([3, 0]);
    expect(record.fixed_lengths).toEqual([-200, planned[0].drive_mm[0]]);
    expect(record.drive_anchor).toBe(1);
    expect(record.sensor_anchor).toBe(2);
    expect(record.metadata.mesh).toMatchObject({ id: 'mesh_004', row, rows: 3, held_anchor: 3 });
    expect(record.metadata.sensor_force_profile).toBe(true);
    expect(record.data_points.map(point => point.step_index)).toEqual([0, 1, 2, 3]);
    record.data_points.forEach((point, step) => {
      // Moves are commanded to 1 µm.
      expect(point.l_drive).toBeCloseTo(planned[step].drive_mm[1], 2);
      expect(point.l_sensor).toBeCloseTo(planned[step].predicted_mm[2], 2);
      expect(point.sensor_force_n).toBe(planned[step].sensor_force_n);
      expect(point.fixed_error_angles_deg[3]).toBeCloseTo(0, 6);
    });
  });
});

test('a point whose drive is blocked is skipped and the rest of the mesh kept', async () => {
  const plan = meshPlan();
  // The first point and its four probe moves come first; block a later move of drive 1.
  const { records, skipped } = await run(geometricMachine({ blockedDrive: 1, blockedMoves: [9] }), plan);
  expect(skipped).toHaveLength(1);
  expect(skipped[0].reason).toMatch(/did not track/);
  expect(records.reduce((total, record) => total + record.data_points.length, 0)).toBe(plan.points.length - 1);
});

test('a held-line slip skips only that point', async () => {
  const plan = meshPlan();
  const { records, skipped } = await run(geometricMachine({ slipAfterMoves: 9, slipAnchor: 3 }), plan);
  expect(skipped.length).toBeGreaterThanOrEqual(1);
  expect(records.reduce((total, record) => total + record.data_points.length, 0)).toBeGreaterThanOrEqual(plan.min_points);
});

test('a sensor that never couples rejects the mesh after a few probes', async () => {
  await expect(run(geometricMachine({ frozenSensor: 2 }), meshPlan())).rejects.toBeInstanceOf(UnmeasurableSweepError);
});

test('a mesh left with too few points is rejected', async () => {
  const plan = meshPlan();
  await expect(run(geometricMachine({ blockedDrive: 1, blockedMoves: Array.from({ length: 40 }, (_, i) => i + 7) }), plan))
    .rejects.toThrow(/below its minimum/);
});

test('HP5 spare ranges use the spare force and count after consecutive row steps', async () => {
  const plan = meshPlan({ anchors: HP5, drives: [0, 1], sensor: 2, extras: [3] });
  const machine = geometricMachine({ anchors: HP5 });
  const { records } = await run(machine, plan);
  for (const record of records) {
    expect(record.data_points[0].extra_sensor_ranges[0].accepted).toBe(false);
  }
  expect(records.some(record => record.data_points.some(point => point.extra_sensor_ranges[0].accepted))).toBe(true);
  expect(machine.modeLog.some(modes => modes[3] === 3 && modes[4] === 0)).toBe(true);
  expect(machine.modeLog.every(modes => modes[4] === 0)).toBe(true);
});

test('mesh plans are validated against the machine', () => {
  const config = MACHINE_CONFIGS.hangprinter_4;
  expect(() => validateMeshPlan(meshPlan(), config)).not.toThrow();
  expect(() => validateMeshPlan({ ...meshPlan(), held_anchor: 0, drive_anchors: [3, 1] }, config)).toThrow(/must hold anchor 3/);
  expect(() => validateMeshPlan({ ...meshPlan(), held_length_mm: 50 }, config)).toThrow(/fixed-target bounds/);
  expect(() => validateMeshPlan({ ...meshPlan(), sensor_anchor: 0 }, config)).toThrow(/distinct/);
  const heavy = meshPlan();
  heavy.points[3].sensor_force_n = 20;
  expect(() => validateMeshPlan(heavy, config)).toThrow(/exceeds the ceiling/);
  const rowless = meshPlan();
  delete rowless.points[0].row;
  expect(() => validateMeshPlan(rowless, config)).toThrow(/row/);
});
