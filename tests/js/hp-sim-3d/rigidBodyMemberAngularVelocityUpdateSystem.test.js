import {
  World,
  AngularVelocityComponent,
  OrientationComponent,
  RigidBodyMemberComponent,
  PrevRigidBodyLocalOrientationComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { RigidBodyMemberAngularVelocityUpdateSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';

describe('RigidBodyMemberAngularVelocityUpdateSystem', () => {
  test('does not overwrite motor-managed spool angular velocity', () => {
    const world = new World();
    const bodyEntity = world.createEntity();
    const spoolEntity = world.createEntity();

    world.addComponent(bodyEntity, new OrientationComponent());

    const localOrientation = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 4.0);
    world.addComponent(spoolEntity, new SpoolStateComponent('A'));
    world.addComponent(spoolEntity, new AngularVelocityComponent(0.0, 0.0, 0.25));
    world.addComponent(
      spoolEntity,
      new RigidBodyMemberComponent(bodyEntity, new Vector3(0.0, 0.0, 0.0), localOrientation),
    );
    world.addComponent(spoolEntity, new PrevRigidBodyLocalOrientationComponent());

    new RigidBodyMemberAngularVelocityUpdateSystem().update(world, 0.1);

    const angularVelocity = world.getComponent(spoolEntity, AngularVelocityComponent).omega;
    expect(angularVelocity.x).toBeCloseTo(0.0, 12);
    expect(angularVelocity.y).toBeCloseTo(0.0, 12);
    expect(angularVelocity.z).toBeCloseTo(0.25, 12);
  });

  test('still reconstructs angular velocity for non-spool rigid-body members', () => {
    const world = new World();
    const bodyEntity = world.createEntity();
    const memberEntity = world.createEntity();

    world.addComponent(bodyEntity, new OrientationComponent());

    const localOrientation = new Quaternion().setFromAxisAngle(new Vector3(0.0, 0.0, 1.0), Math.PI / 4.0);
    world.addComponent(memberEntity, new AngularVelocityComponent(0.0, 0.0, 0.0));
    world.addComponent(
      memberEntity,
      new RigidBodyMemberComponent(bodyEntity, new Vector3(0.0, 0.0, 0.0), localOrientation),
    );
    world.addComponent(memberEntity, new PrevRigidBodyLocalOrientationComponent());

    new RigidBodyMemberAngularVelocityUpdateSystem().update(world, 0.1);

    const angularVelocity = world.getComponent(memberEntity, AngularVelocityComponent).omega;
    expect(angularVelocity.x).toBeCloseTo(0.0, 12);
    expect(angularVelocity.y).toBeCloseTo(0.0, 12);
    expect(angularVelocity.z).toBeCloseTo((Math.PI / 4.0) / 0.1, 12);
  });
});
