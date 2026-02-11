import Vector2 from '../../../src/js/cable_joints/vector2.js';
import { tangentFromCircleToCircle } from '../../../src/js/cable_joints/geometry.js';
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
  SimulationErrorStateComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  CableAttachmentUpdateSystem,
  PBDCableConstraintSolver
} from '../../../src/js/cable_joints/cable_joints_core.js';
import { PBDResolveCableOverCorrections } from '../../../src/js/cable_joints/pbdResolveCableOverCorrections.js';
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
} from '../../../src/js/cable_joints/commonSystems.js';
import { CableAttachmentCacheSystem } from '../../../src/js/cable_joints/cable_attachment_cache_system.js';
import { CableSlackSystem } from '../../../src/js/cable_joints/cable_slack_system.js';
import { CableFrictionSystem } from '../../../src/js/cable_joints/cable_friction_system.js';
import {
  BallTagComponent,
  PauseStateComponent,
  BorderComponent,
  FlipperMotionSystem,
  FlipperTipLinkSystem,
  OverlayRadiusAndCircleSectorSystem,
  PBDUnifiedContactManifoldSystem
} from '../../../examples/js/flipper/flipper_common.js';
import { BallObstacleBumpSystem } from '../../../examples/js/flipper/ball_obstacle_bump_system.js';
import { BallBorderOrFlipperVelocityContactSystem } from '../../../examples/js/flipper/ball_border_or_flipper_velocity_contact_system.js';
import {
  CABLE_FEATURE_DEFAULTS,
  ensureCableFeatureFlags,
  setCableFeatureFlags
} from '../../../examples/js/flipper/cable_feature_flags.js';

const DEFAULT_CONFIG = Object.freeze({
  pairCount: 6,
  dt: 1.0 / 600.0,
  initialStoredTurns: 1.0,
  cableHalfWidth: 0.0025,
  cableStiffness: 1600.0,
  restLengthScale: 1.0,
  ballRadius: 0.020,
  ballMass: 0.003,
  ballFriction: 0.72,
  borderFriction: 0.82,
  initialLinearSpeed: 0.90,
  initialSpin: 34.0,
  initialPairGap: 0.056,
  initialRowStep: 0.046,
  initialHorizontalSpread: 0.18,
  initialHeightJitter: 0.008,
  funnelTopHalfWidth: 0.25,
  funnelNeckHalfWidth: 0.074,
  cwAlternating: true
});

function _readFinite(value, fallback) {
  return Number.isFinite(value) ? value : fallback;
}

function _average(values) {
  if (!Array.isArray(values) || values.length < 1) {
    return 0.0;
  }
  return values.reduce((sum, value) => sum + value, 0.0) / values.length;
}

function _stddev(values) {
  if (!Array.isArray(values) || values.length < 2) {
    return 0.0;
  }
  const mean = _average(values);
  const variance = _average(values.map((value) => {
    const d = value - mean;
    return d * d;
  }));
  return Math.sqrt(Math.max(0.0, variance));
}

function _linearRegression(points) {
  if (!Array.isArray(points) || points.length < 2) {
    return null;
  }
  const n = points.length;
  let sumX = 0.0;
  let sumY = 0.0;
  let sumXY = 0.0;
  let sumXX = 0.0;
  for (const point of points) {
    const x = _readFinite(point.x, 0.0);
    const y = _readFinite(point.y, 0.0);
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumXX += x * x;
  }
  const denom = n * sumXX - sumX * sumX;
  if (Math.abs(denom) < 1e-12) {
    return {
      n,
      slope: 0.0,
      intercept: sumY / n,
      r2: 0.0
    };
  }
  const slope = (n * sumXY - sumX * sumY) / denom;
  const intercept = (sumY - slope * sumX) / n;
  const yMean = sumY / n;
  let ssRes = 0.0;
  let ssTot = 0.0;
  for (const point of points) {
    const x = _readFinite(point.x, 0.0);
    const y = _readFinite(point.y, 0.0);
    const fitted = slope * x + intercept;
    const res = y - fitted;
    ssRes += res * res;
    const dev = y - yMean;
    ssTot += dev * dev;
  }
  const r2 = ssTot > 1e-12 ? Math.max(0.0, 1.0 - ssRes / ssTot) : 0.0;
  return { n, slope, intercept, r2 };
}

function _sliceByStep(series, stepMin, stepMax) {
  return series.filter((entry) => entry.step >= stepMin && entry.step <= stepMax);
}

function _movementScore(metrics) {
  if (metrics.nanStep !== null) {
    return 1e12 + metrics.nanStep;
  }
  if (metrics.growthAbortStep !== null) {
    return 1e11 + metrics.growthAbortStep;
  }
  if (metrics.spikeStep !== null) {
    return 1e6 + metrics.spikeRatio;
  }
  return metrics.tailAvgMovement;
}

function _configureHybridTransitionTrace(world, traceConfig = {}) {
  if (!traceConfig || traceConfig.enabled === false) {
    world.setResource('cableHybridTransitionTrace', false);
    world.setResource('cableHybridTransitionTraceBuffer', []);
    return;
  }

  const stepMin = Math.floor(_readFinite(traceConfig.stepMin, 235));
  const stepMax = Math.floor(_readFinite(traceConfig.stepMax, 255));
  const limit = Math.max(8, Math.floor(_readFinite(traceConfig.limit, 2048)));
  const toConsole = traceConfig.console === true;

  world.setResource('cableHybridTransitionStep', 0);
  world.setResource('cableHybridTransitionTrace', true);
  world.setResource('cableHybridTransitionTraceStepMin', Math.min(stepMin, stepMax));
  world.setResource('cableHybridTransitionTraceStepMax', Math.max(stepMin, stepMax));
  world.setResource('cableHybridTransitionTraceLimit', limit);
  world.setResource('cableHybridTransitionTraceConsole', toConsole);
  world.setResource('cableHybridTransitionTraceTruncated', false);
  world.setResource('cableHybridTransitionTraceBuffer', []);
}

function _configureCableEventTrace(world, traceConfig = {}) {
  if (!traceConfig || traceConfig.enabled === false) {
    world.setResource('cableEventTrace', false);
    world.setResource('cableEventTraceBuffer', []);
    return;
  }

  const stepMin = Math.floor(_readFinite(traceConfig.stepMin, 220));
  const stepMax = Math.floor(_readFinite(traceConfig.stepMax, 260));
  const limit = Math.max(32, Math.floor(_readFinite(traceConfig.limit, 20000)));
  const toConsole = traceConfig.console === true;

  world.setResource('cableEventTrace', true);
  world.setResource('cableEventTraceStepMin', Math.min(stepMin, stepMax));
  world.setResource('cableEventTraceStepMax', Math.max(stepMin, stepMax));
  world.setResource('cableEventTraceLimit', limit);
  world.setResource('cableEventTraceConsole', toConsole);
  world.setResource('cableEventTraceTruncated', false);
  world.setResource('cableEventTraceBuffer', []);
}

function _registerSystem(world, system, disabledSystems, key, aliases = []) {
  if (disabledSystems.has(key)) {
    return;
  }
  for (const alias of aliases) {
    if (disabledSystems.has(alias)) {
      return;
    }
  }
  if (disabledSystems.has('*')) {
    return;
  }
  world.registerSystem(system);
}

function setupFunnelWorld({
  flagPatch = {},
  disabledSystems = [],
  config = DEFAULT_CONFIG
} = {}) {
  const world = new World();
  const disabled = new Set(disabledSystems);

  const canvasWidth = 960;
  const canvasHeight = 540;
  const simHeight = 0.5;
  const cScale = canvasHeight / simHeight;
  const simWidth = canvasWidth / cScale;

  world.setResource('gravity', new Vector2(0.0, -9.81));
  world.setResource('dt', _readFinite(config.dt, 1.0 / 600.0));
  world.setResource('simWidth', simWidth);
  world.setResource('simHeight', simHeight);
  world.setResource('cScale', cScale);
  world.setResource('pauseState', new PauseStateComponent(false));
  world.setResource('errorState', new SimulationErrorStateComponent(false));
  world.setResource('debugRenderPoints', {});
  world.setResource('ball_obstacle_contacts', []);
  world.setResource('ball_border_contacts', []);
  world.setResource('ball_flipper_contacts', []);
  world.setResource('ball_ball_contacts', []);
  world.setResource('cableDebugLogs', false);
  world.setResource('grabbedBall', null);

  ensureCableFeatureFlags(world);
  setCableFeatureFlags(world, flagPatch);

  const bottom = 0.04;
  const top = simHeight - 0.03;
  const centerX = simWidth * 0.5;
  const topHalf = Math.min(simWidth * 0.45, Math.max(0.12, _readFinite(config.funnelTopHalfWidth, 0.25)));
  const neckHalf = Math.max(0.04, Math.min(topHalf - 0.02, _readFinite(config.funnelNeckHalfWidth, 0.074)));
  const borderPoints = [
    new Vector2(centerX + neckHalf, bottom),
    new Vector2(centerX + topHalf, top),
    new Vector2(centerX - topHalf, top),
    new Vector2(centerX - neckHalf, bottom)
  ];

  const borderId = world.createEntity();
  world.addComponent(borderId, new BorderComponent(borderPoints));
  world.addComponent(borderId, new RenderableComponent('border', '#0b0b0b'));
  world.addComponent(borderId, new RestitutionComponent(0.0));
  world.addComponent(borderId, new CoefficientOfFrictionComponent(_readFinite(config.borderFriction, 0.82)));

  const rawRadius = Math.max(0.008, _readFinite(config.ballRadius, 0.02));
  const ballMass = Math.max(1e-4, _readFinite(config.ballMass, 0.003));
  const ballFriction = Math.max(0.0, _readFinite(config.ballFriction, 0.72));
  const ballI = 0.5 * ballMass * rawRadius * rawRadius;

  const activeFlags = { ...CABLE_FEATURE_DEFAULTS, ...flagPatch };
  const configuredCableHalfWidth = Math.max(0.0, _readFinite(config.cableHalfWidth, 0.0025));
  const useLayeredBaseRadius = activeFlags.enableLayering === true && activeFlags.layeringCableBaseRadius === true;
  const cableHalfWidth = useLayeredBaseRadius ? configuredCableHalfWidth : 0.0;

  const storedTurns = Math.max(0.0, _readFinite(config.initialStoredTurns, 1.0));
  const storedLength = storedTurns * 2.0 * Math.PI * (rawRadius + cableHalfWidth);
  const cableStiffness = Math.max(1.0, _readFinite(config.cableStiffness, 1600.0));
  const restLengthScale = Math.max(0.5, _readFinite(config.restLengthScale, 1.0));
  const pairCount = Math.max(1, Math.floor(_readFinite(config.pairCount, 6)));
  const speed0 = _readFinite(config.initialLinearSpeed, 0.9);
  const spin0 = _readFinite(config.initialSpin, 34.0);

  const pairGap = Math.max(rawRadius * 2.3, _readFinite(config.initialPairGap, 0.056));
  const rowStep = Math.max(rawRadius * 2.0, _readFinite(config.initialRowStep, 0.046));
  const horizontalSpread = Math.max(pairGap, Math.min(topHalf - rawRadius * 1.5, _readFinite(config.initialHorizontalSpread, 0.18)));
  const heightJitter = Math.max(0.0, _readFinite(config.initialHeightJitter, 0.008));
  const wallSpawnMargin = Math.max(1e-3, 0.25 * rawRadius);

  const startY = top - rawRadius * 2.5;
  const ballIds = [];
  const pairDescriptors = [];

  const funnelHalfWidthAtY = (y) => {
    const yClamped = Math.max(bottom, Math.min(top, y));
    const t = (yClamped - bottom) / Math.max(1e-9, top - bottom);
    return neckHalf + (topHalf - neckHalf) * t;
  };

  const createBall = (x, y, vx, vy, omega) => {
    const entity = world.createEntity();
    world.addComponent(entity, new BallTagComponent());
    world.addComponent(entity, new PositionComponent(x, y));
    world.addComponent(entity, new PrevFinalPosComponent(x, y));
    world.addComponent(entity, new VelocityComponent(vx, vy));
    world.addComponent(entity, new RadiusComponent(rawRadius));
    world.addComponent(entity, new MassComponent(ballMass));
    world.addComponent(entity, new RestitutionComponent(0.0));
    world.addComponent(entity, new GravityAffectedComponent());
    world.addComponent(entity, new OrientationComponent(0.0));
    world.addComponent(entity, new PrevFinalOrientationComponent(0.0));
    world.addComponent(entity, new AngularVelocityComponent(omega));
    world.addComponent(entity, new MomentOfInertiaComponent(ballI));
    world.addComponent(entity, new CoefficientOfFrictionComponent(ballFriction));
    world.addComponent(entity, new RenderableComponent('circle', '#a9adb6'));
    world.addComponent(entity, new CableLinkComponent(x, y, 0.0));
    return entity;
  };

  for (let i = 0; i < pairCount; i++) {
    const lane = (i + 0.5) / pairCount;
    const y = Math.max(bottom + rawRadius * 4.0, startY - i * rowStep);
    const jitter = ((i % 2) === 0 ? 1.0 : -1.0) * heightJitter;
    const ay = y + jitter;
    const by = y - jitter;

    const maxHalfAtPair = Math.min(funnelHalfWidthAtY(ay), funnelHalfWidthAtY(by));
    const maxPairGapForRow = Math.max(
      2.0 * rawRadius * 1.2,
      2.0 * (maxHalfAtPair - rawRadius - wallSpawnMargin)
    );
    const pairGapRow = Math.min(pairGap, maxPairGapForRow);
    const centerHalfLimit = Math.max(
      0.0,
      maxHalfAtPair - (0.5 * pairGapRow + rawRadius + wallSpawnMargin)
    );
    const laneXRaw = centerX + (lane - 0.5) * 2.0 * horizontalSpread;
    const laneX = Math.max(centerX - centerHalfLimit, Math.min(centerX + centerHalfLimit, laneXRaw));

    const ax = laneX - 0.5 * pairGapRow;
    const bx = laneX + 0.5 * pairGapRow;
    const vx = ((i % 2) === 0 ? 1.0 : -1.0) * 0.35 * speed0;
    const vy = -0.18 * speed0;
    const spinSign = ((i % 2) === 0 ? 1.0 : -1.0);

    const ballA = createBall(ax, ay, vx, vy, spinSign * spin0);
    const ballB = createBall(bx, by, -vx, vy, -spinSign * spin0);
    ballIds.push(ballA, ballB);
    pairDescriptors.push({ pairIndex: i, ballA, ballB });

    const posA = world.getComponent(ballA, PositionComponent).pos;
    const posB = world.getComponent(ballB, PositionComponent).pos;
    const cwA = config.cwAlternating === false ? true : ((i % 2) === 0);
    const cwB = !cwA;
    const tangent = tangentFromCircleToCircle(posA, rawRadius, cwA, posB, rawRadius, cwB);

    const attachA = tangent.a_circle;
    const attachB = tangent.b_circle;
    const initialLen = attachA.distanceTo(attachB);
    const restLength = Math.max(1e-9, initialLen * restLengthScale);

    const jointId = world.createEntity();
    world.addComponent(
      jointId,
      CableJointComponent.fromWorld(ballA, ballB, restLength, attachA, attachB)
    );
    world.addComponent(jointId, new RenderableComponent('line', '#FFFF00'));

    const pathId = world.createEntity();
    const pathComp = new CablePathComponent(
      world,
      [jointId],
      ['hybrid', 'hybrid'],
      [cwA, cwB],
      cableStiffness,
      [storedLength, storedLength],
      cableHalfWidth
    );
    world.addComponent(pathId, pathComp);
  }

  _registerSystem(world, new PrevFinalPosSystem(), disabled, 'prevFinalPos');
  _registerSystem(world, new PrevFinalOrientationSystem(), disabled, 'prevFinalOrientation');
  _registerSystem(world, new FlipperMotionSystem(), disabled, 'flipperMotion');
  _registerSystem(world, new GravitySystem(), disabled, 'gravity');
  _registerSystem(world, new MovementSystem(), disabled, 'movement');
  _registerSystem(world, new AngularMovementSystem(), disabled, 'angularMovement');
  _registerSystem(world, new FlipperTipLinkSystem(), disabled, 'flipperTipLink');
  _registerSystem(world, new OverlayRadiusAndCircleSectorSystem(), disabled, 'overlayPre', ['overlay']);
  _registerSystem(world, new CableAttachmentUpdateSystem(), disabled, 'attachmentUpdate');
  _registerSystem(world, new OverlayRadiusAndCircleSectorSystem(), disabled, 'overlayPost', ['overlay']);
  _registerSystem(world, new CableAttachmentCacheSystem(), disabled, 'attachmentCache');
  _registerSystem(world, new CableSlackSystem(), disabled, 'slack');
  _registerSystem(world, new PBDCableConstraintSolver(), disabled, 'cableConstraint');
  _registerSystem(world, new PBDResolveCableOverCorrections(), disabled, 'resolveOverCorrections');
  _registerSystem(
    world,
    new PBDUnifiedContactManifoldSystem(),
    disabled,
    'contactManifold',
    [
      'ballBorder',
      'borderSector',
      'ballBall',
      'ballBallSector',
      'ballObstacle',
      'obstacleSector',
      'ballFlipper',
      'flipperSector'
    ]
  );
  _registerSystem(world, new CableFrictionSystem(), disabled, 'cableFriction');
  _registerSystem(world, new PBDVelocityUpdateSystem(), disabled, 'velocityUpdate');
  _registerSystem(world, new PBDAngularVelocityUpdateSystem(), disabled, 'angularVelocityUpdate');
  _registerSystem(world, new BallObstacleBumpSystem(), disabled, 'obstacleBump');
  _registerSystem(world, new BallBorderOrFlipperVelocityContactSystem(), disabled, 'borderFlipperVelocityContact');

  return { world, ballIds, pairDescriptors };
}

function _pathEndpoints(world, pathComp) {
  if (!pathComp || !Array.isArray(pathComp.jointEntities) || pathComp.jointEntities.length < 1) {
    return null;
  }
  const firstJoint = world.getComponent(pathComp.jointEntities[0], CableJointComponent);
  const lastJoint = world.getComponent(
    pathComp.jointEntities[pathComp.jointEntities.length - 1],
    CableJointComponent
  );
  if (!firstJoint || !lastJoint) {
    return null;
  }
  return {
    startEntity: firstJoint.entityA,
    endEntity: lastJoint.entityB
  };
}

function _findPathByEndpoints(world, entityA, entityB) {
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const pathComp = world.getComponent(pathId, CablePathComponent);
    const endpoints = _pathEndpoints(world, pathComp);
    if (!endpoints) {
      continue;
    }
    const direct = endpoints.startEntity === entityA && endpoints.endEntity === entityB;
    const reverse = endpoints.startEntity === entityB && endpoints.endEntity === entityA;
    if (direct || reverse) {
      return { pathId, pathComp, reversed: reverse };
    }
  }
  return null;
}

function _collectPathStats(world, pathComp) {
  if (!pathComp || !Array.isArray(pathComp.jointEntities) || pathComp.jointEntities.length < 1) {
    return {
      jointCount: 0,
      minJointRest: null,
      minStored: null,
      maxStored: null,
      tinyRestCount: 0,
      negativeRestCount: 0,
      maxJointStretch: 0.0,
      chainEntities: [],
      middleEntities: [],
      linkTypes: []
    };
  }

  let minJointRest = Infinity;
  let tinyRestCount = 0;
  let negativeRestCount = 0;
  let maxJointStretch = 0.0;
  const middleEntities = [];
  const chainEntities = [];

  for (let i = 0; i < pathComp.jointEntities.length; i++) {
    const jointId = pathComp.jointEntities[i];
    const joint = world.getComponent(jointId, CableJointComponent);
    if (!joint) {
      continue;
    }
    if (i === 0) {
      chainEntities.push(joint.entityA);
    }
    chainEntities.push(joint.entityB);
    if (Number.isFinite(joint.restLength)) {
      if (joint.restLength < minJointRest) {
        minJointRest = joint.restLength;
      }
      if (joint.restLength < 1e-6) {
        tinyRestCount++;
      }
      if (joint.restLength < 0.0) {
        negativeRestCount++;
      }
    }
    const segLen = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
    const stretch = Math.max(0.0, segLen - _readFinite(joint.restLength, 0.0));
    if (stretch > maxJointStretch) {
      maxJointStretch = stretch;
    }

    if (i < pathComp.jointEntities.length - 1) {
      middleEntities.push(joint.entityB);
    }
  }

  const stored = Array.isArray(pathComp.stored)
    ? pathComp.stored.filter((value) => Number.isFinite(value))
    : [];
  const minStored = stored.length > 0 ? Math.min(...stored) : null;
  const maxStored = stored.length > 0 ? Math.max(...stored) : null;

  return {
    jointCount: pathComp.jointEntities.length,
    minJointRest: Number.isFinite(minJointRest) ? minJointRest : null,
    minStored,
    maxStored,
    tinyRestCount,
    negativeRestCount,
    maxJointStretch,
    chainEntities,
    middleEntities,
    linkTypes: Array.isArray(pathComp.linkTypes) ? [...pathComp.linkTypes] : []
  };
}

function runScenario({
  flagPatch = {},
  disabledSystems = [],
  steps = 420,
  hybridTrace = null,
  eventTrace = null,
  trackedPairIndex = null
} = {}) {
  const { world, ballIds, pairDescriptors } = setupFunnelWorld({ flagPatch, disabledSystems });
  const dt = world.getResource('dt');
  _configureHybridTransitionTrace(world, hybridTrace);
  _configureCableEventTrace(world, eventTrace);

  const prevPos = new Map();
  for (const ballId of ballIds) {
    prevPos.set(ballId, world.getComponent(ballId, PositionComponent).pos.clone());
  }

  const perStep = [];
  const trackedPair = Number.isInteger(trackedPairIndex) && trackedPairIndex >= 0
    ? (pairDescriptors.find((pair) => pair.pairIndex === trackedPairIndex) ?? null)
    : null;
  const trackedPairPerStep = [];
  let trackedPairTopologyChanges = 0;
  let previousTrackedPathSignature = null;
  let previousTrackedPairAbs = null;
  let prevPairAngular = new Map();
  let nanStep = null;
  let growthAbortStep = null;
  for (let step = 1; step <= steps; step++) {
    world.update(dt);

    if (step % 5 === 0) {
      const jointCount = world.query([CableJointComponent]).length;
      const pathCount = world.query([CablePathComponent]).length;
      if (jointCount > 300 || pathCount > 120) {
        growthAbortStep = step;
        break;
      }
    }

    let totalMovement = 0.0;
    let maxMovement = 0.0;
    let maxSpeed = 0.0;
    let maxVy = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;
    let totalBallAngularAbs = 0.0;
    let maxBallAngularAbs = 0.0;
    const pairAngular = new Map();

    for (const ballId of ballIds) {
      const pos = world.getComponent(ballId, PositionComponent)?.pos;
      const vel = world.getComponent(ballId, VelocityComponent)?.vel;
      const angularVel = world.getComponent(ballId, AngularVelocityComponent)?.angularVelocity;
      if (
        !pos ||
        !vel ||
        !Number.isFinite(angularVel) ||
        !Number.isFinite(pos.x) ||
        !Number.isFinite(pos.y) ||
        !Number.isFinite(vel.x) ||
        !Number.isFinite(vel.y)
      ) {
        nanStep = step;
        break;
      }

      const prev = prevPos.get(ballId);
      const movement = prev ? pos.distanceTo(prev) : 0.0;
      totalMovement += movement;
      if (movement > maxMovement) maxMovement = movement;
      const speed = vel.length();
      if (speed > maxSpeed) maxSpeed = speed;
      if (vel.y > maxVy) maxVy = vel.y;
      if (pos.y < minY) minY = pos.y;
      if (pos.y > maxY) maxY = pos.y;
      const absOmega = Math.abs(angularVel);
      totalBallAngularAbs += absOmega;
      if (absOmega > maxBallAngularAbs) {
        maxBallAngularAbs = absOmega;
      }
      prevPos.set(ballId, pos.clone());
    }

    const jointEntities = world.query([CableJointComponent]);
    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      if (!joint) {
        continue;
      }
      const ballA = world.getComponent(joint.entityA, BallTagComponent);
      const ballB = world.getComponent(joint.entityB, BallTagComponent);
      if (!ballA || !ballB) {
        continue;
      }
      const omegaA = world.getComponent(joint.entityA, AngularVelocityComponent)?.angularVelocity;
      const omegaB = world.getComponent(joint.entityB, AngularVelocityComponent)?.angularVelocity;
      if (!Number.isFinite(omegaA) || !Number.isFinite(omegaB)) {
        continue;
      }
      const idA = Math.min(joint.entityA, joint.entityB);
      const idB = Math.max(joint.entityA, joint.entityB);
      const key = `${idA}:${idB}`;
      const pairAbs = Math.abs(omegaA) + Math.abs(omegaB);
      const pairSigned = omegaA + omegaB;
      const existing = pairAngular.get(key);
      if (!existing || pairAbs > existing.pairAbs) {
        pairAngular.set(key, {
          key,
          idA,
          idB,
          jointId,
          pairAbs,
          pairSigned
        });
      }
    }

    let totalPairAngularAbs = 0.0;
    let maxPairAngularAbs = 0.0;
    let maxPairAngularJump = 0.0;
    let maxPairAngularJumpKey = null;
    for (const [key, pair] of pairAngular.entries()) {
      totalPairAngularAbs += pair.pairAbs;
      if (pair.pairAbs > maxPairAngularAbs) {
        maxPairAngularAbs = pair.pairAbs;
      }
      const prevPair = prevPairAngular.get(key);
      const jump = pair.pairAbs - (prevPair ? prevPair.pairAbs : pair.pairAbs);
      if (jump > maxPairAngularJump) {
        maxPairAngularJump = jump;
        maxPairAngularJumpKey = key;
      }
    }
    prevPairAngular = pairAngular;

    perStep.push({
      step,
      totalMovement,
      maxMovement,
      maxSpeed,
      maxVy,
      minY,
      maxY,
      totalBallAngularAbs,
      maxBallAngularAbs,
      totalPairAngularAbs,
      maxPairAngularAbs,
      maxPairAngularJump,
      maxPairAngularJumpKey
    });

    if (trackedPair) {
      const omegaA = world.getComponent(trackedPair.ballA, AngularVelocityComponent)?.angularVelocity;
      const omegaB = world.getComponent(trackedPair.ballB, AngularVelocityComponent)?.angularVelocity;
      const pairAbsAngular = (Number.isFinite(omegaA) && Number.isFinite(omegaB))
        ? (Math.abs(omegaA) + Math.abs(omegaB))
        : null;
      const pairSignedAngular = (Number.isFinite(omegaA) && Number.isFinite(omegaB))
        ? (omegaA + omegaB)
        : null;
      const pairAngularJump = (
        Number.isFinite(pairAbsAngular) &&
        Number.isFinite(previousTrackedPairAbs)
      )
        ? (pairAbsAngular - previousTrackedPairAbs)
        : 0.0;
      if (Number.isFinite(pairAbsAngular)) {
        previousTrackedPairAbs = pairAbsAngular;
      }

      const matchedPath = _findPathByEndpoints(world, trackedPair.ballA, trackedPair.ballB);
      const pathStats = matchedPath
        ? _collectPathStats(world, matchedPath.pathComp)
        : _collectPathStats(world, null);
      let pathAngularAbsTotal = null;
      let pathAngularSignedTotal = null;
      if (matchedPath && Array.isArray(pathStats.chainEntities) && pathStats.chainEntities.length > 0) {
        let absTotal = 0.0;
        let signedTotal = 0.0;
        let finiteCount = 0;
        for (const entityId of pathStats.chainEntities) {
          const omega = world.getComponent(entityId, AngularVelocityComponent)?.angularVelocity;
          if (!Number.isFinite(omega)) {
            continue;
          }
          absTotal += Math.abs(omega);
          signedTotal += omega;
          finiteCount++;
        }
        if (finiteCount > 0) {
          pathAngularAbsTotal = absTotal;
          pathAngularSignedTotal = signedTotal;
        }
      }
      const pathSignature = matchedPath
        ? `${matchedPath.pathId}:${pathStats.jointCount}:${pathStats.middleEntities.join(',')}:${pathStats.linkTypes.join('|')}`
        : 'none';
      if (previousTrackedPathSignature !== null && pathSignature !== previousTrackedPathSignature) {
        trackedPairTopologyChanges++;
      }
      previousTrackedPathSignature = pathSignature;

      trackedPairPerStep.push({
        step,
        ballA: trackedPair.ballA,
        ballB: trackedPair.ballB,
        omegaA: Number.isFinite(omegaA) ? omegaA : null,
        omegaB: Number.isFinite(omegaB) ? omegaB : null,
        pairAbsAngular,
        pairSignedAngular,
        pairAngularJump,
        pathId: matchedPath?.pathId ?? null,
        pathReversed: matchedPath?.reversed ?? false,
        pathSignature,
        pathJointCount: pathStats.jointCount,
        pathMiddleEntities: pathStats.middleEntities,
        pathLinkTypes: pathStats.linkTypes,
        pathMinJointRest: pathStats.minJointRest,
        pathTinyRestCount: pathStats.tinyRestCount,
        pathNegativeRestCount: pathStats.negativeRestCount,
        pathMinStored: pathStats.minStored,
        pathMaxStored: pathStats.maxStored,
        pathMaxJointStretch: pathStats.maxJointStretch,
        pathAngularAbsTotal,
        pathAngularSignedTotal,
        pathEntities: pathStats.chainEntities
      });
    }

    if (nanStep !== null) {
      break;
    }

    if (maxMovement > 0.5 || maxSpeed > 30.0) {
      growthAbortStep = step;
      break;
    }
  }

  const quietWindow = _sliceByStep(perStep, 180, 230);
  const quietAvg = _average(quietWindow.map((entry) => entry.totalMovement));
  const quietMaxSpeed = Math.max(1e-9, ...quietWindow.map((entry) => entry.maxSpeed));
  const quietAngularAvg = _average(quietWindow.map((entry) => entry.totalBallAngularAbs));
  const quietAngularStd = _stddev(quietWindow.map((entry) => entry.totalBallAngularAbs));
  const quietPairJumpPositive = quietWindow
    .map((entry) => Math.max(0.0, entry.maxPairAngularJump))
    .filter((value) => value > 0.0);
  const quietPairJumpAvg = _average(quietPairJumpPositive);
  const spikeWindow = _sliceByStep(perStep, 230, 320);
  const tailWindow = perStep.slice(Math.max(0, perStep.length - 80));
  const tailAvgMovement = _average(tailWindow.map((entry) => entry.totalMovement));
  const peakAngularAbs = Math.max(0.0, ...spikeWindow.map((entry) => entry.totalBallAngularAbs));
  const peakPairAngularJump = Math.max(0.0, ...spikeWindow.map((entry) => entry.maxPairAngularJump));

  let spikeStep = null;
  for (const entry of spikeWindow) {
    const movementSpike = entry.totalMovement > Math.max(0.02, quietAvg * 12.0);
    const speedSpike = entry.maxSpeed > Math.max(1.5, quietMaxSpeed * 8.0);
    const upwardKick = entry.maxVy > 1.0;
    if ((movementSpike || speedSpike) && upwardKick) {
      spikeStep = entry.step;
      break;
    }
  }

  let angularSpikeStep = null;
  for (const entry of spikeWindow) {
    const angularLevelSpike = entry.totalBallAngularAbs > Math.max(
      quietAngularAvg * 1.2,
      quietAngularAvg + 2.5 * quietAngularStd
    );
    const pairJumpSpike = entry.maxPairAngularJump > Math.max(
      0.02,
      quietPairJumpAvg * 8.0
    );
    if (angularLevelSpike && pairJumpSpike) {
      angularSpikeStep = entry.step;
      break;
    }
  }

  const peakMovement = Math.max(0.0, ...spikeWindow.map((entry) => entry.totalMovement));
  const spikeRatio = peakMovement / Math.max(1e-9, quietAvg);
  const isStable = (
    growthAbortStep === null &&
    nanStep === null &&
    spikeStep === null &&
    tailAvgMovement < Math.max(0.003, quietAvg * 2.0)
  );

  return {
    isStable,
    growthAbortStep,
    nanStep,
    spikeStep,
    angularSpikeStep,
    spikeRatio,
    quietAvg,
    tailAvgMovement,
    quietAngularAvg,
    quietAngularStd,
    peakAngularAbs,
    peakPairAngularJump,
    quietPairJumpAvg,
    score: _movementScore({
      growthAbortStep,
      nanStep,
      spikeStep,
      spikeRatio,
      tailAvgMovement
    }),
    perStep,
    hybridTransitions: Array.isArray(world.getResource('cableHybridTransitionTraceBuffer'))
      ? world.getResource('cableHybridTransitionTraceBuffer')
      : [],
    hybridTraceTruncated: world.getResource('cableHybridTransitionTraceTruncated') === true,
    cableEvents: Array.isArray(world.getResource('cableEventTraceBuffer'))
      ? world.getResource('cableEventTraceBuffer')
      : [],
    cableEventTraceTruncated: world.getResource('cableEventTraceTruncated') === true,
    trackedPair: trackedPair
      ? {
          pairIndex: trackedPair.pairIndex,
          ballA: trackedPair.ballA,
          ballB: trackedPair.ballB,
          topologyChanges: trackedPairTopologyChanges,
          perStep: trackedPairPerStep
        }
      : null
  };
}

describe('Spool Funnel Stability Sweep', () => {
  const oldUiLayeringFlags = [
    'enableLayering',
    'layeringCableBaseRadius',
    'layeringCableStoredLayerRadius',
    'layeringFrictionEffectiveRadius',
    'layeringCollisionOverlayRadius',
    'layeringCollisionCircleSectors',
    'layeringCollisionSectorSolvers',
    'layeringCollisionPinchShare',
    'layeringVelocityContactOffset',
    'layeringObstacleRawHitFilter',
    'layeringRenderWraps'
  ];

  const coreCableFlags = [
    'layeringAttachmentUpdatePoints',
    'layeringMergeJoints',
    'layeringSplitJoints',
    'layeringHybridLinkStates'
  ];

  function buildPatch(keys, value) {
    const patch = {};
    for (const key of keys) {
      patch[key] = value;
    }
    return patch;
  }

  test('greedy cable-step disabling isolates spike source around step 240', () => {
    const originalWarn = console.warn;
    const originalError = console.error;
    console.warn = jest.fn();
    console.error = jest.fn();

    const legacyAllOffPatch = buildPatch(oldUiLayeringFlags, false);
    const reproPatch = {
      ...legacyAllOffPatch,
      layeringClampJointRestLength: false
    };
    const fixedPatch = {
      ...legacyAllOffPatch,
      layeringClampJointRestLength: true
    };

    try {
      const baseline = runScenario({
        flagPatch: reproPatch,
        steps: 320,
        hybridTrace: {
          stepMin: 205,
          stepMax: 255,
          limit: 2048
        },
        eventTrace: {
          stepMin: 200,
          stepMax: 260,
          limit: 48000
        }
      });
      expect(
        baseline.growthAbortStep !== null ||
        baseline.nanStep !== null ||
        baseline.spikeStep !== null
      ).toBe(true);

      const fixed = runScenario({
        flagPatch: fixedPatch,
        steps: 320,
        eventTrace: {
          stepMin: 200,
          stepMax: 260,
          limit: 24000
        }
      });
      expect(fixed.isStable).toBe(true);

      let traceResult = baseline;
      let traceWindow = { stepMin: 205, stepMax: 255 };
      let eventTraceWindow = { stepMin: 200, stepMax: 260 };
      if (traceResult.hybridTransitions.length === 0) {
        const spikeCenter = baseline.spikeStep ?? baseline.growthAbortStep ?? 245;
        const fallbackMin = Math.max(1, spikeCenter - 120);
        const fallbackMax = Math.min(320, spikeCenter + 24);
        traceWindow = { stepMin: fallbackMin, stepMax: fallbackMax };
        eventTraceWindow = { stepMin: Math.max(1, spikeCenter - 40), stepMax: Math.min(320, spikeCenter + 8) };
        traceResult = runScenario({
          flagPatch: reproPatch,
          steps: 320,
          hybridTrace: {
            stepMin: fallbackMin,
            stepMax: fallbackMax,
            limit: 4096
          },
          eventTrace: {
            stepMin: eventTraceWindow.stepMin,
            stepMax: eventTraceWindow.stepMax,
            limit: 48000
          }
        });
      }

      const transitionCandidates = traceResult.hybridTransitions
        .filter((event) => event.transition === 'hybrid-attachment->hybrid')
        .sort((a, b) => Math.abs(b.restLengthDelta ?? 0.0) - Math.abs(a.restLengthDelta ?? 0.0));
      const worstTransition = transitionCandidates[0] ?? null;
      const spikeReferenceStep = baseline.spikeStep ?? baseline.growthAbortStep ?? null;
      let nearestTransition = null;
      if (Number.isFinite(spikeReferenceStep) && traceResult.hybridTransitions.length > 0) {
        nearestTransition = [...traceResult.hybridTransitions].sort((a, b) => {
          const da = Math.abs((a.step ?? 0) - spikeReferenceStep);
          const db = Math.abs((b.step ?? 0) - spikeReferenceStep);
          if (da !== db) {
            return da - db;
          }
          return Math.abs(b.restLengthDelta ?? 0.0) - Math.abs(a.restLengthDelta ?? 0.0);
        })[0];
      }

      let eventTraceResult = traceResult;
      if (eventTraceResult.cableEvents.length === 0) {
        const spikeCenter = baseline.spikeStep ?? baseline.growthAbortStep ?? 245;
        const fallbackEventMin = Math.max(1, spikeCenter - 80);
        const fallbackEventMax = Math.min(320, spikeCenter + 12);
        eventTraceWindow = { stepMin: fallbackEventMin, stepMax: fallbackEventMax };
        eventTraceResult = runScenario({
          flagPatch: reproPatch,
          steps: 320,
          eventTrace: {
            stepMin: fallbackEventMin,
            stepMax: fallbackEventMax,
            limit: 64000
          }
        });
      }

      const cableEvents = eventTraceResult.cableEvents;
      const summaryEvents = cableEvents.filter((event) => event.type === 'summary');
      const splitEvents = cableEvents.filter((event) => event.type === 'split');
      const mergeEvents = cableEvents.filter((event) => event.type === 'merge');
      const hybridEvents = cableEvents.filter((event) => event.type === 'hybrid-transition');
      const restAnomalyEvents = cableEvents.filter((event) => event.type === 'rest-length-anomaly');
      const restClampEvents = cableEvents.filter((event) => event.type === 'rest-length-clamp');
      const hybridRubEvents = cableEvents.filter((event) => event.type === 'hybrid-rub-check');

      let firstTinyOrNegativeSummary = null;
      for (const event of summaryEvents) {
        if (
          (event.tinyRestCount ?? 0) > 0 ||
          (event.negativeRestCount ?? 0) > 0 ||
          (event.nonFiniteRestCount ?? 0) > 0
        ) {
          firstTinyOrNegativeSummary = event;
          break;
        }
      }

      const perStepCounts = new Map();
      for (const event of cableEvents) {
        if (event.type === 'summary') {
          continue;
        }
        const step = Math.floor(_readFinite(event.step, 0));
        if (!perStepCounts.has(step)) {
          perStepCounts.set(step, { split: 0, merge: 0, hybrid: 0, abort: 0 });
        }
        const counts = perStepCounts.get(step);
        if (event.type === 'split') counts.split++;
        else if (event.type === 'merge') counts.merge++;
        else if (event.type === 'split-abort') counts.abort++;
        else if (event.type === 'hybrid-transition') counts.hybrid++;
      }

      const busiestSteps = [...perStepCounts.entries()]
        .map(([step, counts]) => ({
          step,
          ...counts,
          total: counts.split + counts.merge + counts.hybrid + counts.abort
        }))
        .sort((a, b) => b.total - a.total || a.step - b.step)
        .slice(0, 6);

      const smallestSplit = splitEvents.length > 0
        ? [...splitEvents].sort((a, b) => (a.minNewRestLength ?? Infinity) - (b.minNewRestLength ?? Infinity))[0]
        : null;
      const nearSpikeSplit = (Number.isFinite(spikeReferenceStep) && splitEvents.length > 0)
        ? [...splitEvents].sort((a, b) => {
          const da = Math.abs((a.step ?? 0) - spikeReferenceStep);
          const db = Math.abs((b.step ?? 0) - spikeReferenceStep);
          if (da !== db) return da - db;
          return (a.minNewRestLength ?? Infinity) - (b.minNewRestLength ?? Infinity);
        })[0]
        : null;
      const firstRestAnomaly = restAnomalyEvents.length > 0 ? restAnomalyEvents[0] : null;
      const worstRestAnomaly = restAnomalyEvents.length > 0
        ? [...restAnomalyEvents].sort((a, b) => (a.restAfter ?? Infinity) - (b.restAfter ?? Infinity))[0]
        : null;
      const nearSpikeRestAnomaly = (Number.isFinite(spikeReferenceStep) && restAnomalyEvents.length > 0)
        ? [...restAnomalyEvents].sort((a, b) => {
          const da = Math.abs((a.step ?? 0) - spikeReferenceStep);
          const db = Math.abs((b.step ?? 0) - spikeReferenceStep);
          if (da !== db) return da - db;
          return (a.restAfter ?? Infinity) - (b.restAfter ?? Infinity);
        })[0]
        : null;
      const pinchRubEvents = hybridRubEvents.filter(
        (event) => event.sameJointPath === true && event.bothEndpointsHybridLike === true
      );
      const overlappingPinchRubEvents = pinchRubEvents.filter(
        (event) => Number.isFinite(event.centerOverlap) && event.centerOverlap > 0.0
      );
      const insideCirclePinchEvents = pinchRubEvents.filter(
        (event) => event.neighborAttachmentInsideCircle === true
      );
      const pinchTransitionEvents = pinchRubEvents.filter(
        (event) => event.reason === 'transition'
      );
      const pinchDegenerateSkips = pinchRubEvents.filter(
        (event) => event.reason === 'attachment-degenerate-skip'
      );
      const nearestPinchRubEvent = (Number.isFinite(spikeReferenceStep) && pinchRubEvents.length > 0)
        ? [...pinchRubEvents].sort((a, b) => {
          const da = Math.abs((a.step ?? 0) - spikeReferenceStep);
          const db = Math.abs((b.step ?? 0) - spikeReferenceStep);
          if (da !== db) return da - db;
          const aa = Number.isFinite(a.centerOverlap) ? a.centerOverlap : -Infinity;
          const bb = Number.isFinite(b.centerOverlap) ? b.centerOverlap : -Infinity;
          return bb - aa;
        })[0]
        : null;
      const firstAnomalyStep = firstRestAnomaly?.step ?? null;
      const pinchBeforeFirstAnomaly = (Number.isFinite(firstAnomalyStep) && pinchRubEvents.length > 0)
        ? pinchRubEvents.find((event) => (event.step ?? Infinity) <= firstAnomalyStep) ?? null
        : null;
      const fixedRestAnomalies = fixed.cableEvents.filter((event) => event.type === 'rest-length-anomaly');
      const fixedOrientationProjectionEvents = fixed.cableEvents.filter(
        (event) => event.type === 'rest-length-orientation-projection'
      );
      const fixedOrientationProjectionDrifts = fixedOrientationProjectionEvents
        .flatMap((event) => [event.localAttachmentDriftA, event.localAttachmentDriftB])
        .filter((value) => Number.isFinite(value));
      const fixedMaxLocalAttachmentDrift = fixedOrientationProjectionDrifts.length > 0
        ? Math.max(...fixedOrientationProjectionDrifts)
        : 0.0;
      const fixedAvgLocalAttachmentDrift = fixedOrientationProjectionDrifts.length > 0
        ? _average(fixedOrientationProjectionDrifts)
        : 0.0;
      const summaryAfterAttachmentByStep = new Map(
        summaryEvents
          .filter((event) => event.phase === 'afterAttachment')
          .map((event) => [Math.floor(_readFinite(event.step, 0)), event])
      );
      const angularSpikeEntry = baseline.perStep.find((entry) => entry.step === baseline.angularSpikeStep) ?? null;
      const angularSpikeSummary = Number.isFinite(baseline.angularSpikeStep)
        ? summaryAfterAttachmentByStep.get(baseline.angularSpikeStep) ?? null
        : null;
      const angularSpikeRestAnomalies = Number.isFinite(baseline.angularSpikeStep)
        ? restAnomalyEvents.filter((event) => event.step === baseline.angularSpikeStep)
        : [];
      const worstPairJumpEntry = baseline.perStep.length > 0
        ? [...baseline.perStep].sort((a, b) => b.maxPairAngularJump - a.maxPairAngularJump)[0]
        : null;
      const worstPairJumpSummary = worstPairJumpEntry
        ? summaryAfterAttachmentByStep.get(worstPairJumpEntry.step) ?? null
        : null;
      const negRestVsAngularRegression = _linearRegression(
        baseline.perStep
          .map((entry) => {
            const summary = summaryAfterAttachmentByStep.get(entry.step);
            if (!summary) {
              return null;
            }
            return {
              x: Math.max(0.0, -(summary.minRestLength ?? 0.0)),
              y: entry.totalBallAngularAbs
            };
          })
          .filter((sample) => sample !== null)
      );
      const timeVsAngularOffRegression = _linearRegression(
        _sliceByStep(baseline.perStep, 180, 245)
          .map((entry) => ({ x: entry.step, y: entry.totalBallAngularAbs }))
      );
      const timeVsAngularOnRegression = _linearRegression(
        _sliceByStep(fixed.perStep, 180, 245)
          .map((entry) => ({ x: entry.step, y: entry.totalBallAngularAbs }))
      );

      const rootCauseCandidate = {
        spikeReferenceStep,
        eventTraceWindow,
        eventCount: cableEvents.length,
        eventTraceTruncated: eventTraceResult.cableEventTraceTruncated,
        summaryCount: summaryEvents.length,
        splitCount: splitEvents.length,
        mergeCount: mergeEvents.length,
        hybridCount: hybridEvents.length,
        restAnomalyCount: restAnomalyEvents.length,
        restClampCount: restClampEvents.length,
        hybridRubCount: hybridRubEvents.length,
        pinchRubCount: pinchRubEvents.length,
        overlappingPinchRubCount: overlappingPinchRubEvents.length,
        insideCirclePinchCount: insideCirclePinchEvents.length,
        pinchTransitionCount: pinchTransitionEvents.length,
        pinchDegenerateSkipCount: pinchDegenerateSkips.length,
        firstTinyOrNegativeSummary,
        firstRestAnomaly,
        worstRestAnomaly,
        nearSpikeRestAnomaly,
        firstRestClamp: restClampEvents[0] ?? null,
        nearestPinchRubEvent,
        pinchBeforeFirstAnomaly,
        busiestSteps,
        smallestSplit,
        nearSpikeSplit,
        angularDiagnostics: {
          angularSpikeStep: baseline.angularSpikeStep,
          angularSpikeEntry,
          angularSpikeSummary,
          angularSpikeRestAnomalyCount: angularSpikeRestAnomalies.length,
          angularSpikeRestAnomaly: angularSpikeRestAnomalies[0] ?? null,
          worstPairJumpEntry,
          worstPairJumpSummary,
          quietAngularAvg: baseline.quietAngularAvg,
          quietAngularStd: baseline.quietAngularStd,
          peakAngularAbs: baseline.peakAngularAbs,
          peakPairAngularJump: baseline.peakPairAngularJump,
          quietPairJumpAvg: baseline.quietPairJumpAvg,
          negRestVsAngularRegression,
          timeVsAngularOffRegression,
          timeVsAngularOnRegression
        },
        fixControl: {
          isStable: fixed.isStable,
          spikeStep: fixed.spikeStep,
          angularSpikeStep: fixed.angularSpikeStep,
          growthAbortStep: fixed.growthAbortStep,
          quietAngularAvg: fixed.quietAngularAvg,
          peakAngularAbs: fixed.peakAngularAbs,
          peakPairAngularJump: fixed.peakPairAngularJump,
          orientationProjectionCount: fixedOrientationProjectionEvents.length,
          maxLocalAttachmentDrift: fixedMaxLocalAttachmentDrift,
          avgLocalAttachmentDrift: fixedAvgLocalAttachmentDrift,
          restAnomalyCount: fixedRestAnomalies.length,
          firstRestAnomaly: fixedRestAnomalies[0] ?? null
        }
      };

      // eslint-disable-next-line no-console
      console.log('SPOOL_FUNNEL_EVENT_TRACE', JSON.stringify(rootCauseCandidate));

      // eslint-disable-next-line no-console
      console.log(
        'SPOOL_FUNNEL_HYBRID_TRACE',
        JSON.stringify({
          spikeReferenceStep,
          traceWindow,
          totalTransitions: traceResult.hybridTransitions.length,
          truncated: traceResult.hybridTraceTruncated,
          firstTransitions: traceResult.hybridTransitions.slice(0, 6),
          nearestTransition,
          worstTransition
        })
      );

      const disabledCoreFlags = [];
      const sweepLog = [{
        disabledCoreFlags: [...disabledCoreFlags],
        result: baseline
      }];

      let currentBest = baseline;
      while (!currentBest.isStable && disabledCoreFlags.length < coreCableFlags.length) {
        let bestCandidate = null;
        for (const flag of coreCableFlags) {
          if (disabledCoreFlags.includes(flag)) {
            continue;
          }
          const candidateDisabled = [...disabledCoreFlags, flag];
          const candidatePatch = {
            ...reproPatch,
            ...buildPatch(candidateDisabled, false)
          };
          const candidateResult = runScenario({
            flagPatch: candidatePatch,
            steps: 320
          });

          if (
            bestCandidate === null ||
            candidateResult.score < bestCandidate.result.score
          ) {
            bestCandidate = {
              flag,
              disabledCoreFlags: candidateDisabled,
              result: candidateResult
            };
          }
        }

        if (!bestCandidate || bestCandidate.result.score >= currentBest.score) {
          break;
        }

        disabledCoreFlags.push(bestCandidate.flag);
        currentBest = bestCandidate.result;
        sweepLog.push({
          disabledCoreFlags: [...disabledCoreFlags],
          result: currentBest
        });
      }

      // Emit a compact report for debugging sessions.
      const report = sweepLog.map((entry) => ({
        disabled: entry.disabledCoreFlags,
        stable: entry.result.isStable,
        growthAbortStep: entry.result.growthAbortStep,
        spikeStep: entry.result.spikeStep,
        nanStep: entry.result.nanStep,
        spikeRatio: Number(entry.result.spikeRatio.toFixed(2)),
        quietAvg: Number(entry.result.quietAvg.toFixed(6)),
        tailAvgMovement: Number(entry.result.tailAvgMovement.toFixed(6))
      }));
      // eslint-disable-next-line no-console
      console.log('SPOOL_FUNNEL_STABILITY_SWEEP', JSON.stringify(report));

      expect(currentBest.score).toBeLessThan(baseline.score);
      expect(currentBest.isStable).toBe(true);
      expect(fixedMaxLocalAttachmentDrift).toBeLessThan(1e-6);
    } finally {
      console.warn = originalWarn;
      console.error = originalError;
    }
  });

  const longHorizonTest = process.env.SPOOL_LONG === '1' ? test : test.skip;
  longHorizonTest('long-horizon green-pair routed-path diagnostic', () => {
    const originalWarn = console.warn;
    const originalError = console.error;
    console.warn = jest.fn();
    console.error = jest.fn();

    const windowMin = 17450;
    const windowMax = 17850;
    const longSteps = 18200;
    const greenPairIndex = 3;

    const summarizeTracked = (tracked) => {
      const perStep = tracked?.perStep ?? [];
      const window = perStep.filter((entry) => entry.step >= windowMin && entry.step <= windowMax);
      const firstMultiJoint = perStep.find((entry) => entry.pathJointCount > 1) ?? null;
      const maxJointCount = perStep.length > 0
        ? Math.max(...perStep.map((entry) => entry.pathJointCount))
        : 0;
      let peakPathAbsJump = null;
      let previousPathAbs = null;
      for (const entry of window) {
        if (!Number.isFinite(entry.pathAngularAbsTotal)) {
          continue;
        }
        const jump = Number.isFinite(previousPathAbs)
          ? (entry.pathAngularAbsTotal - previousPathAbs)
          : 0.0;
        if (
          peakPathAbsJump === null ||
          jump > _readFinite(peakPathAbsJump.pathAngularAbsJump, -Infinity)
        ) {
          peakPathAbsJump = {
            step: entry.step,
            pathAngularAbsJump: jump,
            pathAngularAbsTotal: entry.pathAngularAbsTotal,
            pathAngularSignedTotal: entry.pathAngularSignedTotal,
            pathJointCount: entry.pathJointCount,
            pathSignature: entry.pathSignature,
            pathEntities: entry.pathEntities
          };
        }
        previousPathAbs = entry.pathAngularAbsTotal;
      }
      const peakJump = window.length > 0
        ? [...window].sort((a, b) => (_readFinite(b.pairAngularJump, -Infinity) - _readFinite(a.pairAngularJump, -Infinity)))[0]
        : null;
      const routedWindow = window.filter((entry) => entry.pathJointCount > 1);
      const peakJumpWhenRouted = routedWindow.length > 0
        ? [...routedWindow].sort((a, b) => (_readFinite(b.pairAngularJump, -Infinity) - _readFinite(a.pairAngularJump, -Infinity)))[0]
        : null;
      const peakAbs = window.length > 0
        ? [...window].sort((a, b) => (_readFinite(b.pairAbsAngular, -Infinity) - _readFinite(a.pairAbsAngular, -Infinity)))[0]
        : null;
      return {
        pairIndex: tracked?.pairIndex ?? null,
        ballA: tracked?.ballA ?? null,
        ballB: tracked?.ballB ?? null,
        totalTrackedSteps: perStep.length,
        topologyChanges: tracked?.topologyChanges ?? 0,
        maxJointCount,
        firstMultiJointStep: firstMultiJoint?.step ?? null,
        firstMultiJointSignature: firstMultiJoint?.pathSignature ?? null,
        multiJointWindowCount: routedWindow.length,
        peakPathAbsJump,
        peakJump,
        peakJumpWhenRouted,
        peakAbs
      };
    };

    try {
      const baseline = runScenario({
        steps: longSteps,
        trackedPairIndex: greenPairIndex,
        eventTrace: {
          stepMin: windowMin,
          stepMax: windowMax,
          limit: 160000
        },
        hybridTrace: {
          stepMin: windowMin,
          stepMax: windowMax,
          limit: 16000
        }
      });
      const splitOff = runScenario({
        steps: longSteps,
        trackedPairIndex: greenPairIndex,
        flagPatch: {
          layeringSplitJoints: false
        },
        eventTrace: {
          stepMin: windowMin,
          stepMax: windowMax,
          limit: 160000
        }
      });

      const baselineTrackedSummary = summarizeTracked(baseline.trackedPair);
      const splitOffTrackedSummary = summarizeTracked(splitOff.trackedPair);
      const baselinePeakStep = baselineTrackedSummary.peakJumpWhenRouted?.step
        ?? baselineTrackedSummary.peakJump?.step
        ?? null;
      const baselinePeakPathAbsStep = baselineTrackedSummary.peakPathAbsJump?.step ?? null;
      const baselinePeakPathId = baselineTrackedSummary.peakJumpWhenRouted?.pathId
        ?? baselineTrackedSummary.peakJump?.pathId
        ?? null;
      const baselineEventsNearPeak = Number.isFinite(baselinePeakStep)
        ? baseline.cableEvents.filter((event) => {
            const step = Math.floor(_readFinite(event.step, 0));
            const inWindow = step >= baselinePeakStep - 2 && step <= baselinePeakStep + 2;
            if (!inWindow) {
              return false;
            }
            if (!Number.isFinite(baselinePeakPathId)) {
              return true;
            }
            return event.pathId === baselinePeakPathId || event.jointId === baselinePeakPathId;
          })
        : [];
      const baselineEventsNearPathAbsPeak = (
        Number.isFinite(baselinePeakPathAbsStep) && Number.isFinite(baselinePeakPathId)
      )
        ? baseline.cableEvents.filter((event) => {
            const step = Math.floor(_readFinite(event.step, 0));
            const inWindow = step >= baselinePeakPathAbsStep - 2 && step <= baselinePeakPathAbsStep + 2;
            return inWindow && event.pathId === baselinePeakPathId;
          })
        : [];
      const baselinePathEventsInWindow = (
        Number.isFinite(baselinePeakPathId)
          ? baseline.cableEvents.filter((event) => {
              const step = Math.floor(_readFinite(event.step, 0));
              return (
                step >= windowMin &&
                step <= windowMax &&
                event.pathId === baselinePeakPathId
              );
            })
          : []
      );
      const baselinePathSplitEvents = baselinePathEventsInWindow.filter((event) => event.type === 'split');
      const baselinePathMergeEvents = baselinePathEventsInWindow.filter((event) => event.type === 'merge');
      const baselinePathClampEvents = baselinePathEventsInWindow.filter((event) => event.type === 'rest-length-clamp');
      const baselinePathRestAnomalyEvents = baselinePathEventsInWindow.filter((event) => event.type === 'rest-length-anomaly');
      const baselineLastSplitBeforePeak = (
        Number.isFinite(baselinePeakStep)
          ? [...baselinePathSplitEvents]
              .filter((event) => Math.floor(_readFinite(event.step, 0)) <= baselinePeakStep)
              .sort((a, b) => Math.floor(_readFinite(b.step, 0)) - Math.floor(_readFinite(a.step, 0)))[0] ?? null
          : null
      );
      const baselineLastMergeBeforePeak = (
        Number.isFinite(baselinePeakStep)
          ? [...baselinePathMergeEvents]
              .filter((event) => Math.floor(_readFinite(event.step, 0)) <= baselinePeakStep)
              .sort((a, b) => Math.floor(_readFinite(b.step, 0)) - Math.floor(_readFinite(a.step, 0)))[0] ?? null
          : null
      );
      const baselineLastClampBeforePeak = (
        Number.isFinite(baselinePeakStep)
          ? [...baselinePathClampEvents]
              .filter((event) => Math.floor(_readFinite(event.step, 0)) <= baselinePeakStep)
              .sort((a, b) => Math.floor(_readFinite(b.step, 0)) - Math.floor(_readFinite(a.step, 0)))[0] ?? null
          : null
      );
      const baselineLastRestAnomalyBeforePeak = (
        Number.isFinite(baselinePeakStep)
          ? [...baselinePathRestAnomalyEvents]
              .filter((event) => Math.floor(_readFinite(event.step, 0)) <= baselinePeakStep)
              .sort((a, b) => Math.floor(_readFinite(b.step, 0)) - Math.floor(_readFinite(a.step, 0)))[0] ?? null
          : null
      );
      const baselinePathSplitsNearPeak = baselineEventsNearPeak.filter((event) => event.type === 'split');
      const baselinePathMergesNearPeak = baselineEventsNearPeak.filter((event) => event.type === 'merge');
      const baselinePathRestAnomaliesNearPeak = baselineEventsNearPeak.filter((event) => event.type === 'rest-length-anomaly');

      // eslint-disable-next-line no-console
      console.log(
        'SPOOL_FUNNEL_LONG_HORIZON_TRACE',
        JSON.stringify({
          windowMin,
          windowMax,
          longSteps,
          baseline: {
            isStable: baseline.isStable,
            growthAbortStep: baseline.growthAbortStep,
            nanStep: baseline.nanStep,
            tracked: baselineTrackedSummary,
            peakStep: baselinePeakStep,
            peakPathAbsStep: baselinePeakPathAbsStep,
            peakPathId: baselinePeakPathId,
            peakEventsNearStep: baselineEventsNearPeak.slice(0, 40),
            peakPathAbsEventsNearStep: baselineEventsNearPathAbsPeak.slice(0, 40),
            pathEventCountInWindow: baselinePathEventsInWindow.length,
            splitEventCountInWindow: baselinePathSplitEvents.length,
            mergeEventCountInWindow: baselinePathMergeEvents.length,
            clampEventCountInWindow: baselinePathClampEvents.length,
            restAnomalyEventCountInWindow: baselinePathRestAnomalyEvents.length,
            lastSplitBeforePeak: baselineLastSplitBeforePeak,
            lastMergeBeforePeak: baselineLastMergeBeforePeak,
            lastClampBeforePeak: baselineLastClampBeforePeak,
            lastRestAnomalyBeforePeak: baselineLastRestAnomalyBeforePeak,
            splitEventsNearPeakCount: baselinePathSplitsNearPeak.length,
            mergeEventsNearPeakCount: baselinePathMergesNearPeak.length,
            restAnomalyNearPeakCount: baselinePathRestAnomaliesNearPeak.length
          },
          splitOff: {
            isStable: splitOff.isStable,
            growthAbortStep: splitOff.growthAbortStep,
            nanStep: splitOff.nanStep,
            tracked: splitOffTrackedSummary
          },
          comparison: {
            baselinePeakJumpWhenRouted: baselineTrackedSummary.peakJumpWhenRouted?.pairAngularJump ?? null,
            baselinePeakJump: baselineTrackedSummary.peakJump?.pairAngularJump ?? null,
            splitOffPeakJump: splitOffTrackedSummary.peakJump?.pairAngularJump ?? null
          }
        })
      );

      expect(baseline.trackedPair).not.toBeNull();
      expect(splitOff.trackedPair).not.toBeNull();
      expect(splitOffTrackedSummary.maxJointCount).toBeLessThanOrEqual(1);
    } finally {
      console.warn = originalWarn;
      console.error = originalError;
    }
  });
});
