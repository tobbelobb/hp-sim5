import { readFileSync } from 'node:fs';
import {
  AngularVelocityComponent, BearingFrictionComponent, DistanceConstraintComponent, MassComponent,
  MomentOfInertiaComponent, OrientationComponent, PositionComponent, RadiusComponent,
  RenderableComponent, SceneEntityInfoComponent, VelocityComponent, World,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent } from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';
import { OpenText } from '../../../src/js/usd/stage.js';
import { createHeadlessWorld } from '../../../hp-sim-3d/app/headlessWorld.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import {
  applyEntityPlan, buildEntityPlan, parseStage, readMachineSceneSpec, validateMachineSceneSpec,
} from '../../../hp-sim-3d/app/scene/machineScenePipeline.js';

const scenePrimPath = '/World/Scene';
const fixture = readFileSync('tests/fixtures/engine_parity/usd_scene_minimal.usda', 'utf8')
  .replace('def CableJoint "J1" {', `def CableJoint "J1" {
            point3d localPos0 = (0.02, 0.03, 0.04)
            point3d localPos1 = (0.01, 0, 0)
            double restLength = 0.95`)
  .replace('double height = 0.04', 'double height = 0.04\n            double bearing:frictionTorque = 0.005')
  .replace('cablePath:stiffnessPerUnitLength = 0', 'cablePath:stiffnessPerUnitLength = 300')
  .replace('cablePath:dampingPerUnitLength = 0', 'cablePath:dampingPerUnitLength = 3')
  .replace('double cablePath:halfWidth', 'double cablePath:totalLength = 5\n            double cablePath:halfWidth');

function withUnits(source, units) {
  return source.replace('timeCodesPerSecond = 500', `metersPerUnit = ${units}\n    timeCodesPerSecond = 500`);
}

function inMillimetres(source) {
  return withUnits(source, 0.001).replace(
    /\b(xformOp:translate|radius|height|physics:gravityMagnitude|physics:velocity|physics:inertiaTensor|physics:minDistance|physics:maxDistance|localPos[01]|restLength|cablePath:stored|cablePath:halfWidth|cablePath:totalLength)( = )([^\n]+)/g,
    (_, field, equals, values) => field + equals + values.replace(
      /-?\d+(?:\.\d+)?(?:e[+-]?\d+)?/gi, value => Number(value) * (field === 'physics:inertiaTensor' ? 1e6 : 1e3),
    ),
  );
}

function loadStage(source) {
  const world = new World();
  const spec = validateMachineSceneSpec(readMachineSceneSpec(parseStage(OpenText(source)), scenePrimPath));
  expect(spec.valid).toBe(true);
  applyEntityPlan(world, buildEntityPlan(spec));
  return world;
}

function readPhysicalState(world) {
  const get = (id, Type) => world.getComponent(id, Type);
  const read = Type => world.query([Type]).map(id => get(id, Type));
  const bodies = Object.fromEntries(world.query([SceneEntityInfoComponent, PositionComponent]).map(id => [
    get(id, SceneEntityInfoComponent).name,
    {
      position: get(id, PositionComponent).pos,
      radius: get(id, RadiusComponent)?.radius,
      height: get(id, RenderableComponent)?.height,
      mass: get(id, MassComponent)?.mass,
      velocity: get(id, VelocityComponent)?.vel,
      omega: get(id, AngularVelocityComponent)?.omega,
      inertia: get(id, MomentOfInertiaComponent)?.tensor,
      orientation: get(id, OrientationComponent)?.quaternion,
      bearingTorque: get(id, BearingFrictionComponent)?.torque,
      motor: get(id, StepperMotorComponent),
    },
  ]));
  return JSON.parse(JSON.stringify({
    bodies,
    dt: world.getResource('dt'),
    gravity: world.getResource('gravity'),
    distances: read(DistanceConstraintComponent).map(joint => joint.restLength),
    cableJoints: read(CableJointComponent),
    cablePaths: read(CablePathComponent),
    extruder: read(ExtruderComponent),
  }, (_, value) => typeof value === 'number' ? Number(value.toPrecision(12)) : value));
}

test.each([true, false])('metre and millimetre machines have equivalent baked physics (layered=%s)', lineLayeringEnabled => {
  const options = { scenePrimPath, lineLayeringEnabled };
  const metres = createHeadlessWorld(withUnits(fixture, 1), options);
  const millimetres = createHeadlessWorld(inMillimetres(fixture), options);
  const expected = readPhysicalState(metres.world);
  expect(readPhysicalState(millimetres.world)).toEqual(expected);
  expect(readPhysicalState(createHeadlessWorld(fixture, options).world)).toEqual(expected);

  expect(expected).toMatchObject({
    bodies: {
      Anchor: { position: { x: 2, y: 1, z: 0 }, radius: 0.01 },
      Wheel: { radius: 0.1, height: 0.04, bearingTorque: 0.005,
        inertia: [[0.02, 0.001, 0], [0.001, 0.03, 0], [0, 0, 0.04]] },
      Effector: { mass: 0.8 },
      SpoolA: { motor: { holdingTorque: 0.7, dampingCoeff: 0.02, maxSpeedRad: 70 } },
      Pinhole: { radius: 0.002 },
    },
    dt: 0.002,
    gravity: { x: 0, y: 0, z: -0.1 },
    distances: [1.5],
    cablePaths: [{ cableHalfWidth: lineLayeringEnabled ? 0.001 : 0,
      stiffnessPerUnitLength: 300, dampingPerUnitLength: 3 }],
  });
  expect(expected.cableJoints).toHaveLength(3);
  if (lineLayeringEnabled) {
    expect(expected.cableJoints[1].restLength).toBe(0.95);
    expect(expected.cablePaths[0].stored[0]).toBe(1);
    expect(expected.cablePaths[0].stored[1]).toBeGreaterThan(0);
  }

  expect(millimetres.bakedScene).toMatch(/metersPerUnit = 1\n/);
  expect(millimetres.bakedScene).toMatch(/cablePath:totalLength = 5\n/);
  const rebaked = bakeCableSceneUsdaSource(millimetres.bakedScene).source;
  expect(rebaked).toBe(millimetres.bakedScene);
  expect(readPhysicalState(loadStage(rebaked))).toEqual(expected);

  for (const { world, dt } of [metres, millimetres]) world.update(dt);
  expect(readPhysicalState(millimetres.world)).toEqual(readPhysicalState(metres.world));
});

test('loading a manually authored millimetre scene directly uses metre physics', () => {
  const baked = bakeCableSceneUsdaSource(fixture).source;
  expect(readPhysicalState(loadStage(inMillimetres(baked)))).toEqual(readPhysicalState(loadStage(baked)));
});

test.each([0, -1, '"millimetres"'])('rejects invalid metersPerUnit metadata (%s)', units => {
  const source = withUnits(fixture, units);
  expect(() => OpenText(source)).toThrow(/metersPerUnit must be a finite positive number/);
  expect(() => bakeCableSceneUsdaSource(source)).toThrow(/metersPerUnit must be a finite positive number/);
});
