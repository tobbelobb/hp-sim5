const origLog = console.log;
console.log = (...args) => {
  if (typeof args[0] === 'string' && args[0].startsWith('timestep:')) {
    return;
  }
  origLog(...args);
};

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
  HybridKnotAngleComponent,
  layeringEnabled
} from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/ecs.js';
import { tangentFromCircleToPoint } from '/home/torbjorn/repos/hp-sim5/src/js/cable_joints/geometry.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver,
  calculateAttachmentPoints
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
import {
  ensureCableFeatureFlags,
  setCableFeatureFlags
} from '/home/torbjorn/repos/hp-sim5/examples/js/flipper/cable_feature_flags.js';

const EPSILON = 1e-9;
const KNOT_SPAN = Math.PI / 30.0;

function _normalizeAngleSigned(angle) {
  let wrapped = angle;
  const twoPi = 2.0 * Math.PI;
  while (wrapped > Math.PI) wrapped -= twoPi;
  while (wrapped < -Math.PI) wrapped += twoPi;
  return wrapped;
}

function _isHybrid(value) {
  return value === 'hybrid' || value === 'hybrid-attachment';
}

function _effectiveCW(pathComp, linkIndex, travellingFromCircle) {
  if (linkIndex === 0 && travellingFromCircle) {
    return !pathComp.cw[linkIndex];
  }
  return pathComp.cw[linkIndex];
}

function _layerWrapParams(r0, dr, rampLength, layerIndex) {
  const twoPi = 2.0 * Math.PI;
  const rn = r0 + dr * layerIndex;

  let dPhiRamp = 0.0;
  if (rampLength > EPSILON) {
    dPhiRamp = rampLength / (rn + 0.5 * dr);
    if (dPhiRamp > twoPi) dPhiRamp = twoPi;
    if (dPhiRamp < 0.0) dPhiRamp = 0.0;
  }

  const phiConst = twoPi - dPhiRamp;
  const Lconst = rn * phiConst;
  const Lwrap = Lconst + dPhiRamp * (rn + 0.5 * dr);
  return { rn, dPhiRamp, phiConst, Lconst, Lwrap };
}

function _storedToRadiusAndTheta(storedLength, baseRadius, halfWidth, rampLength) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  const twoPi = 2.0 * Math.PI;

  const r0 = baseRadius + halfWidth;
  const dr = 2.0 * halfWidth;
  const LrampTarget = Math.max(0.0, rampLength ?? 0.0);

  if (!(r0 > EPSILON) || !(dr > EPSILON)) {
    return { radius: Math.max(baseRadius, 0.0), theta: 0.0 };
  }

  let s = stored;
  let thetaBase = 0.0;
  let n = 0;
  const MAX_LAYERS = 2048;

  while (n < MAX_LAYERS) {
    const wrap = _layerWrapParams(r0, dr, LrampTarget, n);
    if (s > wrap.Lwrap + EPSILON) {
      s -= wrap.Lwrap;
      thetaBase += twoPi;
      n += 1;
      continue;
    }

    if (s <= wrap.Lconst + EPSILON || !(wrap.dPhiRamp > EPSILON)) {
      const phi = (wrap.rn > EPSILON) ? (Math.min(wrap.phiConst, s / wrap.rn)) : 0.0;
      return { radius: wrap.rn, theta: thetaBase + phi };
    }

    const sRamp = Math.max(0.0, s - wrap.Lconst);
    const a = dr / (2.0 * wrap.dPhiRamp);
    const b = wrap.rn;
    const disc = b * b + 4.0 * a * sRamp;
    const x = (-b + Math.sqrt(Math.max(0.0, disc))) / (2.0 * a);

    const xClamped = Math.max(0.0, Math.min(wrap.dPhiRamp, x));
    const alpha = xClamped / wrap.dPhiRamp;
    const radius = wrap.rn + dr * alpha;
    const phi = wrap.phiConst + xClamped;
    return { radius, theta: thetaBase + phi };
  }

  const rn = r0 + dr * MAX_LAYERS;
  return { radius: rn, theta: thetaBase };
}

function _effectiveRollingRadius(world, pathComp, linkIndex, baseRadius) {
  if (!layeringEnabled(world) || !Number.isFinite(baseRadius) || !pathComp || !Array.isArray(pathComp.linkTypes) || !Array.isArray(pathComp.stored)) {
    return { radius : baseRadius, theta: 0.0 };
  }

  const halfWidth = pathComp.cableHalfWidth ?? 0.0;
  if (!(halfWidth > EPSILON)) {
    return { radius : baseRadius, theta: 0.0 };
  }

  const effectiveRadius = baseRadius + halfWidth;
  const isEndpoint = linkIndex === 0 || linkIndex === pathComp.linkTypes.length - 1;
  if (!isEndpoint || !_isHybrid(pathComp.linkTypes[linkIndex])) {
    return { radius : effectiveRadius, theta: 0.0 };
  }

  const stored = Math.max(0.0, pathComp.stored[linkIndex] ?? 0.0);
  if (!(stored > EPSILON)) {
    return { radius : effectiveRadius, theta: 0.0 };
  }

  const { radius, theta } = _storedToRadiusAndTheta(stored, baseRadius, halfWidth, baseRadius * KNOT_SPAN);
  return { radius: Math.max(effectiveRadius, radius), theta };
}

function computeDiffA(world, pathComp, jointComp) {
  const A = 0;
  const B = 1;
  if (!_isHybrid(pathComp.linkTypes[A])) {
    return null;
  }

  const entityA = jointComp.entityA;
  const entityB = jointComp.entityB;
  const knotCompA = world.getComponent(entityA, HybridKnotAngleComponent);
  if (!knotCompA) {
    return null;
  }

  const posA = world.getComponent(entityA, PositionComponent)?.pos;
  const angleA = world.getComponent(entityA, OrientationComponent)?.angle ?? 0.0;
  const baseRadiusA = world.getComponent(entityA, RadiusComponent)?.radius;
  const baseRadiusB = world.getComponent(entityB, RadiusComponent)?.radius;
  const { radius: radiusA, theta: thetaA } = _effectiveRollingRadius(world, pathComp, A, baseRadiusA);
  const { radius: radiusB } = _effectiveRollingRadius(world, pathComp, B, baseRadiusB);
  const cwA = _effectiveCW(pathComp, A, true);

  const { attachmentA_current } = calculateAttachmentPoints(world, jointComp, pathComp, 0, radiusA, radiusB);
  if (!posA || !attachmentA_current || !Number.isFinite(thetaA)) {
    return null;
  }

  const attachmentAngleWorldA = Math.atan2(attachmentA_current.y - posA.y, attachmentA_current.x - posA.x);
  const attachmentRelOrientationA = _normalizeAngleSigned(attachmentAngleWorldA - angleA);
  const thetaSignedA = (cwA ? -1.0 : 1.0) * thetaA;
  const knotAngleFromAttachmentA = _normalizeAngleSigned(attachmentRelOrientationA - thetaSignedA);
  const diffA = _normalizeAngleSigned(knotAngleFromAttachmentA - knotCompA.angle);
  return { diffA, attachmentRelOrientationA, thetaSignedA };
}

function setupWorld() {
  const world = new World();
  const simHeight = 0.5;
  const simWidth = 16 / 9 * simHeight;
  const cfg = {
    initialStoredTurns: 0.5,
    cableHalfWidth: 0.005,
    cableStiffness: 1200.0,
    initialLinearSpeed: 0.0,
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
    layeringAttachmentUpdatePoints: true,
    layeringMergeJoints: true,
    layeringSplitJoints: true,
    layeringHybridLinkStates: true,
    layeringClampJointRestLength: true
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
  const ballY = bottom + rawRadius + 4 * cfg.cableHalfWidth;
  const ballI = 0.5 * ballMass * rawRadius * rawRadius;

  const ballId = world.createEntity();
  world.addComponent(ballId, new BallTagComponent());
  world.addComponent(ballId, new PositionComponent(ballX, ballY));
  world.addComponent(ballId, new PrevFinalPosComponent(ballX, ballY));
  world.addComponent(ballId, new VelocityComponent(cfg.initialLinearSpeed, 0.0));
  world.addComponent(ballId, new RadiusComponent(rawRadius));
  world.addComponent(ballId, new MassComponent(ballMass));
  world.addComponent(ballId, new RestitutionComponent(0.0));
  world.addComponent(ballId, new GravityAffectedComponent());
  world.addComponent(ballId, new OrientationComponent(0.0));
  world.addComponent(ballId, new PrevFinalOrientationComponent(0.0));
  world.addComponent(ballId, new AngularVelocityComponent(0.0));
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
  const tangent = tangentFromCircleToPoint(anchorPos, ballPos, rawRadius, cfg.cw);
  const attachBall = tangent.a_circle;
  const attachAnchor = tangent.a_attach;
  const restLength = Math.max(1e-9, attachBall.distanceTo(attachAnchor) * cfg.restLengthScale);

  const jointId = world.createEntity();
  world.addComponent(jointId, CableJointComponent.fromWorld(ballId, anchorId, restLength, attachBall, attachAnchor));
  world.addComponent(jointId, new RenderableComponent('line', '#ff0'));

  const storedLength = cfg.initialStoredTurns * 2.0 * Math.PI * (rawRadius + cfg.cableHalfWidth);
  const pathId = world.createEntity();
  const pathComp = new CablePathComponent(
    world,
    [jointId],
    ['hybrid', 'attachment'],
    [cfg.cw, true],
    cfg.cableStiffness,
    [storedLength, 0.0],
    cfg.cableHalfWidth
  );
  world.addComponent(pathId, pathComp);

  // Build initial knot angle so diffA has a reference, mirroring transition-initialized behavior.
  const cwA = _effectiveCW(pathComp, 0, true);
  const { theta: thetaA0 } = _effectiveRollingRadius(world, pathComp, 0, rawRadius);
  const attachmentAngleWorld = Math.atan2(attachBall.y - ballPos.y, attachBall.x - ballPos.x);
  const attachmentRelOrientation = _normalizeAngleSigned(attachmentAngleWorld - 0.0);
  const thetaSignedA0 = (cwA ? -1.0 : 1.0) * thetaA0;
  const knotAngle = _normalizeAngleSigned(attachmentRelOrientation - thetaSignedA0);
  world.addComponent(ballId, new HybridKnotAngleComponent(knotAngle));

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

  return {
    world,
    systems,
    ballId,
    pathComp,
    jointComp: world.getComponent(jointId, CableJointComponent)
  };
}

const { world, systems, ballId, pathComp, jointComp } = setupWorld();
const dt = world.getResource('dt');
const rows = [];

for (let step = 1; step <= 700; step += 1) {
  for (let i = 0; i <= 4; i += 1) {
    systems[i].update(world, dt);
  }

  systems[5].update(world, dt); // CableAttachmentUpdateSystem
  const postAttach = computeDiffA(world, pathComp, jointComp);

  systems[6].update(world, dt);
  systems[7].update(world, dt); // cache
  systems[8].update(world, dt);

  const cacheAngle = world.getComponent(ballId, CableLinkComponent).prevCableAttachmentTimeAngle;
  const anglePreSolver = world.getComponent(ballId, OrientationComponent).angle;
  const storedPreSolver = pathComp.stored[0];

  systems[9].update(world, dt);
  systems[10].update(world, dt);
  systems[11].update(world, dt);

  const anglePostSolver = world.getComponent(ballId, OrientationComponent).angle;
  const storedPostSolver = pathComp.stored[0];
  const postSolver = computeDiffA(world, pathComp, jointComp);

  systems[12].update(world, dt);
  systems[13].update(world, dt);
  systems[14].update(world, dt);
  systems[15].update(world, dt);

  if (step >= 590 && step <= 610) {
    rows.push({
      step,
      postAttachDiff: postAttach?.diffA ?? null,
      postSolverDiff: postSolver?.diffA ?? null,
      dDiffSolver: (postSolver && postAttach) ? (postSolver.diffA - postAttach.diffA) : null,
      dAngleSolver: anglePostSolver - anglePreSolver,
      dStoredSolver: storedPostSolver - storedPreSolver,
      cacheVsPostSolver: anglePostSolver - cacheAngle
    });
  }
}

console.log(JSON.stringify(rows, null, 2));
