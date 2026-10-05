import { World, OrientationComponent, PrevFinalOrientationComponent,
  AngularVelocityComponent, MomentOfInertiaComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import { PBDAngularVelocityUpdateSystem } from '../../../src/js/cable_joints_3d/commonSystems.js';

test('PBD recovers a small rotation even when quaternion w rounds to one', () => {
  const world = new World();
  const entity = world.createEntity();
  world.addComponent(entity, new OrientationComponent(0, 0, 5e-9, 1));
  world.addComponent(entity, new PrevFinalOrientationComponent());
  world.addComponent(entity, new AngularVelocityComponent());
  world.addComponent(entity, new MomentOfInertiaComponent(1));
  const system = new PBDAngularVelocityUpdateSystem();
  system.update(world, .002);
  expect(world.getComponent(entity, AngularVelocityComponent).omega.z).toBeCloseTo(5e-6, 14);
  world.getComponent(entity, OrientationComponent).quaternion.z = 2e-10;
  system.update(world, .002);
  expect(world.getComponent(entity, AngularVelocityComponent).omega.z).toBe(0);
});
