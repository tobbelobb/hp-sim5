import {
  cloneExtrusionList,
  collectActiveExtrusionMachineIds,
  filterExtrusionsForMachines,
  restoreReplayExtrusions,
} from '../../../hp-sim-3d/app/replay_state.js';

describe('hp-sim-3d replay extrusion state', () => {
  test('clones extrusion snapshots with full 3D positions intact', () => {
    const original = [
      { machineId: 'machine-0', pos: [0.1, 0.2, -0.003], length: 0.004, qualityColor: '#ffffff' },
    ];

    const cloned = cloneExtrusionList(original);

    expect(cloned).toEqual(original);
    expect(cloned).not.toBe(original);
    expect(cloned[0].pos).not.toBe(original[0].pos);
  });

  test('restores replay extrusions with Z preserved and filters removed machines', () => {
    const extruderComp = {
      centerSources: {
        'machine-1': [1, 2, 3],
      },
      extrusions: [],
    };
    const snapshot = [
      { machineId: 'machine-0', pos: [0.0, 0.1, -0.001], length: 0.002 },
      { machineId: 'machine-1', pos: [0.2, 0.3, -0.004], length: 0.003 },
      { pos: [0.4, 0.5, -0.006], length: 0.001 },
    ];

    expect(collectActiveExtrusionMachineIds(extruderComp)).toEqual(['machine-1']);
    expect(filterExtrusionsForMachines(snapshot, ['machine-1'])).toEqual([
      { machineId: 'machine-1', pos: [0.2, 0.3, -0.004], length: 0.003 },
      { pos: [0.4, 0.5, -0.006], length: 0.001 },
    ]);

    const restored = restoreReplayExtrusions(extruderComp, snapshot);

    expect(restored).toEqual([
      { machineId: 'machine-1', pos: [0.2, 0.3, -0.004], length: 0.003 },
      { pos: [0.4, 0.5, -0.006], length: 0.001 },
    ]);
    expect(extruderComp.extrusions).toEqual(restored);
    expect(extruderComp.extrusions[0].pos[2]).toBeCloseTo(-0.004, 9);
    expect(extruderComp.extrusions[1].pos[2]).toBeCloseTo(-0.006, 9);
  });
});
