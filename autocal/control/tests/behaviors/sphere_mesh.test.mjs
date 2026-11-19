import { MACHINE_CONFIGS, performSphereMesh, validateMeshPlan } from '../../behaviors/sweep_data_collection.mjs';
import { UnmeasurableSweepError } from '../../behaviors/spool_reserve.mjs';

// HP4-like geometry. Lengths are reported directly as encoder angles (1 mm/deg).
const ANCHORS = [[0, -1900, -280], [1645.45, 950, -280], [-1645.45, 950, -280], [0, 0, 1900]];
const AXES = ['X', 'Y', 'Z', 'U'];
const norm = v => Math.hypot(...v);
const sub = (a, b) => a.map((value, i) => value - b[i]);

// A rigid effector: position-controlled lines fix the pose (least squares), and
// force-controlled lines report the distance to it.
function geometricMachine({ frozenSensor = null, blockedDrive = null } = {}) {
  const home = ANCHORS.map(norm);
  let modes = ANCHORS.map(() => 0);
  const lengths = ANCHORS.map(() => 0);
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
      if (mode !== 0 && i !== frozenSensor) lengths[i] = norm(sub(position, ANCHORS[i])) - home[i];
    });
  };
  const send = async line => {
    if (line.startsWith('M569.4')) {
      modes = line.split(' T')[1].split(':').map(Number);
      solve();
    } else if (line.startsWith('M569.3')) {
      return { reply: lengths.join(' ') };
    } else if (line.startsWith('G1 H2')) {
      for (const [, axis, delta] of line.matchAll(/([XYZU])([-\d.]+)/g)) {
        const index = AXES.indexOf(axis);
        if (index !== blockedDrive) lengths[index] += Number(delta);
      }
      solve();
    }
    return { reply: '' };
  };
  send.simulationClock = { now: () => now, sleep: async ms => { now += ms; } };
  return { send, lengths, home, position: () => position };
}

// Points on the sphere around anchor 3 that is 200 mm shorter than at home.
function meshPlan(count = 4) {
  const radius = norm(ANCHORS[3]) - 200;
  const points = Array.from({ length: count }, (_, i) => {
    const angle = (i / count) * 2 * Math.PI;
    const offset = [300 * Math.cos(angle), 300 * Math.sin(angle)];
    const height = ANCHORS[3][2] - Math.sqrt(radius ** 2 - offset[0] ** 2 - offset[1] ** 2);
    const position = [offset[0], offset[1], height];
    return {
      drive_mm: [0, 1].map(axis => norm(sub(position, ANCHORS[axis])) - norm(ANCHORS[axis])),
      sensor_force_n: 2 + i * 0.1,
      position_mm: position,
    };
  });
  return { held_anchor: 3, held_length_mm: -200, drive_anchors: [0, 1], sensor_anchor: 2, extra_anchors: [],
    strand_tension_n: 2, strain_model: 'reeving-aware-v2', points };
}

const run = (machine, plan, overrides = {}) => performSphereMesh(machine.send, plan, {
  axes: AXES, motorIds: ['40.0', '41.0', '42.0', '43.0'], mmPerDeg: [1, 1, 1, 1], feed: 1400, speedup: 1,
  datasetStartMs: 0, forceLow: 0.01, forceMid: 0.23, forceMax: 2.4, noiseSampleCount: 8, noiseSampleIntervalMs: 40,
  noiseMinSamples: 4, noiseSigmaFloorDeg: 0.01, forbiddenForceAnchors: [3], meshId: 'mesh_004', baseRadiusMm: 30,
  ...overrides,
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
    const truth = norm(sub(plan.points[index].position_mm, ANCHORS[2])) - norm(ANCHORS[2]);
    expect(point.l_sensor).toBeCloseTo(truth, 3);
    expect(point.fixed_error_angles_deg[3]).toBeCloseTo(0, 6);
  });
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
  expect(() => validateMeshPlan({ ...meshPlan(), points: [{ drive_mm: [1], sensor_force_n: 1 }] }, config)).toThrow(/two finite/);
});
