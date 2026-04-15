import {
  World,
  PositionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { ExtruderComponent, ExtruderSystem } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';

describe('ExtruderSystem (3D)', () => {
  test('rotates the authored extruder frame with the effector plane', () => {
    const world = new World();
    const sourceA = world.createEntity();
    const sourceB = world.createEntity();
    const sourceC = world.createEntity();
    world.addComponent(sourceA, new PositionComponent(-1.0, 0.0, -1.0 / 3.0));
    world.addComponent(sourceB, new PositionComponent(1.0, 0.0, -1.0 / 3.0));
    world.addComponent(sourceC, new PositionComponent(0.0, 0.0, 2.0 / 3.0));

    const extruderEntity = world.createEntity();
    const extruder = new ExtruderComponent();
    extruder.centerSources.default = [sourceA, sourceB, sourceC];
    extruder.centerSourceOffsets.default = [
      new PositionComponent(-1.0, -1.0 / 3.0, 0.0).pos,
      new PositionComponent(1.0, -1.0 / 3.0, 0.0).pos,
      new PositionComponent(0.0, 2.0 / 3.0, 0.0).pos,
    ];
    extruder.centerOffsets.default = new PositionComponent(0.0, 0.0, -0.1).pos;
    extruder.tipOffsets.default = new PositionComponent(0.0, 0.0, 0.0).pos;
    extruder.coldEndOffsets.default = new PositionComponent(0.0, 0.0, 0.1).pos;
    world.addComponent(extruderEntity, extruder);

    new ExtruderSystem().update(world, 0);

    expect(extruder.effectorCenterPos.x).toBeCloseTo(0.0, 6);
    expect(extruder.effectorCenterPos.y).toBeCloseTo(0.0, 6);
    expect(extruder.effectorCenterPos.z).toBeCloseTo(0.0, 6);
    expect(extruder.centerPos.x).toBeCloseTo(0.0, 6);
    expect(extruder.centerPos.y).toBeCloseTo(0.1, 6);
    expect(extruder.centerPos.z).toBeCloseTo(0.0, 6);
    expect(extruder.tipPos.y).toBeCloseTo(0.1, 6);
    expect(extruder.coldEndPos.y).toBeCloseTo(0.0, 6);
    expect(extruder.coldEndPos.z).toBeCloseTo(0.0, 6);
  });

  test('RemoteSpoolSystem deposits extrusions at the hot-end tip', () => {
    const world = new World();
    const extruderEntity = world.createEntity();
    const extruder = new ExtruderComponent();
    extruder.centerPos = new PositionComponent(0.0, 0.0, 0.0).pos;
    extruder.tipPos = new PositionComponent(0.2, -0.3, 0.4).pos;
    extruder.machineTips.default = extruder.tipPos.clone();
    world.addComponent(extruderEntity, extruder);

    const system = new RemoteSpoolSystem();
    system._processCommand(world, { type: 'Move', E: 0.025 }, { recordHistory: false, emitEvents: false });

    expect(extruder.extrusions).toHaveLength(1);
    expect(extruder.extrusions[0].pos).toEqual([0.2, -0.3, 0.4]);
  });
});
