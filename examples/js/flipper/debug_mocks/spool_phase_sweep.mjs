import Vector2 from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/vector2.js';
import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  GravityAffectedComponent,
  OrientationComponent,
  AngularVelocityComponent,
  CoefficientOfFrictionComponent,
  MomentOfInertiaComponent,
  PrevFinalOrientationComponent,
  RenderableComponent,
  SimulationErrorStateComponent,
  layeringEnabled
} from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/ecs.js';
import { tangentFromCircleToPoint } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/geometry.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver,
} from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/cable_joints_core.js';
import { PBDResolveCableOverCorrections } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/pbdResolveCableOverCorrections.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
} from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/commonSystems.js';
import { CableAttachmentCacheSystem } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/cable_attachment_cache_system.js';
import { CableSlackSystem } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/cable_slack_system.js';
import { CableFrictionSystem } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/cable_friction_system.js';
import {
  BallTagComponent,
  PauseStateComponent,
  BorderComponent,
  PBDUnifiedContactManifoldSystem,
  OverlayRadiusAndCircleSectorSystem,
} from '/home/torbjorn/repos/hp-sim5/examples/js/flipper/flipper_common.js';
import { BallBorderOrFlipperVelocityContactSystem } from '/home/torbjorn/repos/hp-sim5/examples/js/flipper/ball_border_or_flipper_velocity_contact_system.js';
import { ensureCableFeatureFlags, setCableFeatureFlags } from '/home/torbjorn/repos/hp-sim5/examples/js/flipper/cable_feature_flags.js';

function runOnce(initialLinearSpeed = 0.15, initialStoredTurns = 1.0, steps = 700) {
  const world = new World();
  const simHeight = 0.5;
  const simWidth = 16 / 9 * simHeight;
  const cfg = {
    initialStoredTurns,
    cableHalfWidth: 0.005,
    cableStiffness: 1200.0,
    initialLinearSpeed,
    restLengthScale: 1.0,
    ballRadius: 0.019,
    ballMass: 0.008,
    borderFriction: 0.6,
    cw: true
  };

  world.setResource('gravity', new Vector2(0.0, -9.81));
  world.setResource('dt', 1.0 / 500.0);
  world.setResource('simWidth', simWidth);
  world.setResource('simHeight', simHeight);
  world.setResource('cScale', 1000);
  world.setResource('pauseState', new PauseStateComponent(false));
  world.setResource('errorState', new SimulationErrorStateComponent(false));
  world.setResource('debugRenderPoints', {});
  world.setResource('ball_border_contacts', []);
  world.setResource('ball_flipper_contacts', []);
  world.setResource('ball_obstacle_contacts', []);
  world.setResource('ball_ball_contacts', []);
  world.setResource('grabbedBall', null);
  ensureCableFeatureFlags(world);
  setCableFeatureFlags(world, {
    enableLayering: true,
    layeringClampJointRestLength: true,
    layeringAttachmentUpdatePoints: true,
    layeringMergeJoints: true,
    layeringSplitJoints: true,
    layeringHybridLinkStates: true,
  });

  const left = 0.03;
  const right = simWidth - 0.03;
  const bottom = 0.04;
  const top = simHeight - 0.03;
  const borderId = world.createEntity();
  world.addComponent(borderId, new BorderComponent([
    new Vector2(right, bottom),
    new Vector2(right, top),
    new Vector2(left, top),
    new Vector2(left, bottom)
  ]));
  world.addComponent(borderId, new RenderableComponent('border', '#0b0b0b'));
  world.addComponent(borderId, new RestitutionComponent(0.0));
  world.addComponent(borderId, new CoefficientOfFrictionComponent(cfg.borderFriction));

  const rawRadius = cfg.ballRadius;
  const ballMass = cfg.ballMass;
  const ballX = left + 0.05;
  const halfW = cfg.cableHalfWidth;
  const ballY = bottom + rawRadius + 4 * halfW;
  const v0 = cfg.initialLinearSpeed;
  const w0 = -v0 / Math.max(rawRadius, 1e-9);
  const ballI = 0.5 * ballMass * rawRadius * rawRadius;

  const ballId = world.createEntity();
  world.addComponent(ballId, new BallTagComponent());
  world.addComponent(ballId, new PositionComponent(ballX, ballY));
  world.addComponent(ballId, new PrevFinalPosComponent(ballX, ballY));
  world.addComponent(ballId, new VelocityComponent(v0, 0.0));
  world.addComponent(ballId, new RadiusComponent(rawRadius));
  world.addComponent(ballId, new MassComponent(ballMass));
  world.addComponent(ballId, new RestitutionComponent(0.0));
  world.addComponent(ballId, new GravityAffectedComponent());
  world.addComponent(ballId, new OrientationComponent(0.0));
  world.addComponent(ballId, new PrevFinalOrientationComponent(0.0));
  world.addComponent(ballId, new AngularVelocityComponent(w0));
  world.addComponent(ballId, new MomentOfInertiaComponent(ballI));
  world.addComponent(ballId, new CoefficientOfFrictionComponent(0.6));
  world.addComponent(ballId, new RenderableComponent('circle', '#a9adb6'));
  world.addComponent(ballId, new CableLinkComponent(ballX, ballY, 0.0));

  const obstacleRadius = rawRadius * 0.72;
  const obstacleX = ballX + 0.09;
  const obstacleY = bottom - obstacleRadius;
  const obstacleId = world.createEntity();
  world.addComponent(obstacleId, new BallTagComponent());
  world.addComponent(obstacleId, new PositionComponent(obstacleX, obstacleY));
  world.addComponent(obstacleId, new RadiusComponent(obstacleRadius));
  world.addComponent(obstacleId, new MassComponent(-1.0));
  world.addComponent(obstacleId, new RestitutionComponent(0.0));
  world.addComponent(obstacleId, new CoefficientOfFrictionComponent(0.6));
  world.addComponent(obstacleId, new RenderableComponent('circle', '#4bb9ff'));
  world.addComponent(obstacleId, new CableLinkComponent(obstacleX, obstacleY, 0.0));

  const anchorX = right - 0.03;
  const anchorY = bottom + 2 * cfg.cableHalfWidth;
  const anchorId = world.createEntity();
  world.addComponent(anchorId, new PositionComponent(anchorX, anchorY));
  world.addComponent(anchorId, new MassComponent(-1.0));
  world.addComponent(anchorId, new RadiusComponent(0.010));
  world.addComponent(anchorId, new RenderableComponent('circle', '#ff3c2e'));
  world.addComponent(anchorId, new CableLinkComponent(anchorX, anchorY, 0.0));

  const ballPos = world.getComponent(ballId, PositionComponent).pos;
  const anchorPos = world.getComponent(anchorId, PositionComponent).pos;
  const cwBall = cfg.cw === true;
  const tangent = tangentFromCircleToPoint(anchorPos, ballPos, rawRadius, cwBall);
  const attachBall = tangent.a_circle;
  const attachAnchor = tangent.a_attach;
  const restLength = Math.max(1e-9, attachBall.distanceTo(attachAnchor) * cfg.restLengthScale);

  const jointId = world.createEntity();
  world.addComponent(jointId, CableJointComponent.fromWorld(ballId, anchorId, restLength, attachBall, attachAnchor));
  world.addComponent(jointId, new RenderableComponent('line', '#ff0'));

  const storedLength = Math.max(0.0, cfg.initialStoredTurns) * 2 * Math.PI * (rawRadius + halfW);
  const pathId = world.createEntity();
  const pathComp = new CablePathComponent(world, [jointId], ['hybrid', 'attachment'], [cwBall, true], cfg.cableStiffness, [storedLength, 0.0], halfW);
  world.addComponent(pathId, pathComp);

  const systems = [
    new PrevFinalPosSystem(),
    new PrevFinalOrientationSystem(),
    new GravitySystem(),
    new MovementSystem(),
    new AngularMovementSystem(),
    new CableAttachmentUpdateSystem(),
    new OverlayRadiusAndCircleSectorSystem(),
    new CableAttachmentCacheSystem(),
    new CableSlackSystem(),
    new PBDCableConstraintSolver(),
    new PBDResolveCableOverCorrections(),
    new PBDUnifiedContactManifoldSystem(),
    new CableFrictionSystem(),
    new PBDVelocityUpdateSystem(),
    new PBDAngularVelocityUpdateSystem(),
    new BallBorderOrFlipperVelocityContactSystem(),
  ];

  const dt = world.getResource('dt');
  const events = [];

  for (let step = 1; step <= steps; step++) {
    for (let i = 0; i < 9; i++) systems[i].update(world, dt);
    const anglePre = world.getComponent(ballId, OrientationComponent)?.angle ?? 0.0;
    const storedPre = pathComp.stored[0] ?? 0.0;
    systems[9].update(world, dt);
    systems[10].update(world, dt);
    systems[11].update(world, dt);
    const anglePost = world.getComponent(ballId, OrientationComponent)?.angle ?? 0.0;
    const storedPost = pathComp.stored[0] ?? 0.0;
    for (let i = 12; i < systems.length; i++) systems[i].update(world, dt);

    const dA = anglePost - anglePre;
    const dS = storedPost - storedPre;
    if (Math.abs(dA) > 1e-7 || Math.abs(dS) > 1e-12) {
      events.push({ step, dA, dS });
    }
  }

  return events;
}

const speeds = [0.0,0.05,0.1,0.15,0.2,0.25,0.3,0.35];
const turns = [0.5, 1.0, 1.5, 2.0];
const results = [];
for (const s of speeds) {
  for (const t of turns) {
    const ev = runOnce(s, t, 700);
    const near = ev.filter((e) => e.step >= 560 && e.step <= 620);
    results.push({ speed: s, turns: t, eventCount: ev.length, nearCount: near.length, nearSteps: near.map((e)=>e.step).slice(0,10), firstEvent: ev[0]?.step ?? null });
  }
}
console.log(JSON.stringify(results, null, 2));
