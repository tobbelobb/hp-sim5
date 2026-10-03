import {
  World,
  PositionComponent,
  OrientationComponent,
  MassComponent,
  MomentOfInertiaComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
  DistanceConstraintComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { RigidBodySyncSystem, XPBDDistanceConstraintSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';
import { computeWorldAttachment } from '../../../src/js/cable_joints_3d/rigid_bodies.js';
import { SpoolTagComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';
import { ExtruderComponent, ExtruderSystem } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';

describe('ExtruderSystem (3D)', () => {
  test.each([true, false])('uses final rigid-body poses after constraints (authored sources: %s)', (authored) => {
    const world = new World();
    const body = world.createEntity();
    world.addComponent(body, new PositionComponent());
    world.addComponent(body, new OrientationComponent());
    world.addComponent(body, new MassComponent(1));
    world.addComponent(body, new MomentOfInertiaComponent(1));
    const offsets = [[-1, -1 / 3, 0], [1, -1 / 3, 0], [0, 2 / 3, 0]]
      .map(p => new PositionComponent(...p).pos);
    const members = offsets.map(offset => {
      const id = world.createEntity();
      world.addComponent(id, new PositionComponent(offset.x, offset.y, offset.z));
      world.addComponent(id, new RigidBodyMemberComponent(body, offset));
      world.addComponent(id, new SpoolTagComponent());
      return id;
    });
    world.addComponent(body, new RigidBodyComponent(members));
    const anchor = world.createEntity();
    world.addComponent(anchor, new PositionComponent(0, 5, 2));
    world.addComponent(anchor, new MassComponent(0));
    world.addComponent(world.createEntity(), new DistanceConstraintComponent(members[0], anchor, 1));
    const extruder = new ExtruderComponent();
    const tipOffset = new PositionComponent(0.1, 0, -0.1).pos;
    extruder.tipOffsets.default = tipOffset;
    if (authored) {
      extruder.centerSources.default = members;
      extruder.centerSourceOffsets.default = offsets;
    }
    world.addComponent(world.createEntity(), extruder);
    world.registerSystem(new RigidBodySyncSystem());
    world.registerSystem(new XPBDDistanceConstraintSystem());
    world.registerSystem(new ExtruderSystem());
    world.update(0.002);

    // A hidden sync would conceal the bug and change the simulation pipeline.
    expect(world.getComponent(members[0], PositionComponent).pos).toEqual(offsets[0]);
    const finalCenter = world.getComponent(body, PositionComponent).pos;
    expect(finalCenter.length()).toBeGreaterThan(1);
    for (const key of ['x', 'y', 'z']) expect(extruder.centerPos[key]).toBeCloseTo(finalCenter[key], 12);
    const finalTip = authored ? computeWorldAttachment(world, body, tipOffset) : finalCenter.clone().add(tipOffset);
    for (const key of ['x', 'y', 'z']) expect(extruder.tipPos[key]).toBeCloseTo(finalTip[key], 12);
  });

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
    extruder.centerOffsets.default = new PositionComponent(0.0, 0.0, 0.0).pos;
    extruder.tipOffsets.default = new PositionComponent(0.0, 0.0, -0.1).pos;
    extruder.coldEndOffsets.default = new PositionComponent(0.0, 0.0, 0.0).pos;
    world.addComponent(extruderEntity, extruder);

    new ExtruderSystem().update(world, 0);

    expect(extruder.effectorCenterPos.x).toBeCloseTo(0.0, 6);
    expect(extruder.effectorCenterPos.y).toBeCloseTo(0.0, 6);
    expect(extruder.effectorCenterPos.z).toBeCloseTo(0.0, 6);
    expect(extruder.centerPos.x).toBeCloseTo(0.0, 6);
    expect(extruder.centerPos.y).toBeCloseTo(0.0, 6);
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
