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
  PBDBallBorderCollisions,
  PBDBorderCircleSectorCollisions,
  PBDBallBallCollisions,
  PBDBallCircleSectorCollisions,
  PBDBallObstacleCollisions,
  PBDObstacleCircleSectorCollisions,
  PBDBallFlipperCollisions,
  FlipperCircleSectorCollisions
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

function _registerSystem(world, system, disabledSystems, key) {
  if (disabledSystems.has(key)) {
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
  _registerSystem(world, new OverlayRadiusAndCircleSectorSystem(), disabled, 'overlay');
  _registerSystem(world, new CableAttachmentUpdateSystem(), disabled, 'attachmentUpdate');
  _registerSystem(world, new CableAttachmentCacheSystem(), disabled, 'attachmentCache');
  _registerSystem(world, new CableSlackSystem(), disabled, 'slack');
  _registerSystem(world, new PBDCableConstraintSolver(), disabled, 'cableConstraint');
  _registerSystem(world, new PBDResolveCableOverCorrections(), disabled, 'resolveOverCorrections');
  _registerSystem(world, new PBDBallBorderCollisions(), disabled, 'ballBorder');
  _registerSystem(world, new PBDBorderCircleSectorCollisions(), disabled, 'borderSector');
  _registerSystem(world, new PBDBallBallCollisions(), disabled, 'ballBall');
  _registerSystem(world, new PBDBallCircleSectorCollisions(), disabled, 'ballBallSector');
  _registerSystem(world, new PBDBallObstacleCollisions(), disabled, 'ballObstacle');
  _registerSystem(world, new PBDObstacleCircleSectorCollisions(), disabled, 'obstacleSector');
  _registerSystem(world, new PBDBallFlipperCollisions(), disabled, 'ballFlipper');
  _registerSystem(world, new FlipperCircleSectorCollisions(), disabled, 'flipperSector');
  _registerSystem(world, new CableFrictionSystem(), disabled, 'cableFriction');
  _registerSystem(world, new PBDVelocityUpdateSystem(), disabled, 'velocityUpdate');
  _registerSystem(world, new PBDAngularVelocityUpdateSystem(), disabled, 'angularVelocityUpdate');
  _registerSystem(world, new BallObstacleBumpSystem(), disabled, 'obstacleBump');
  _registerSystem(world, new BallBorderOrFlipperVelocityContactSystem(), disabled, 'borderFlipperVelocityContact');

  return { world, ballIds };
}

function runScenario({
  flagPatch = {},
  disabledSystems = [],
  steps = 420,
  hybridTrace = null,
  eventTrace = null
} = {}) {
  const { world, ballIds } = setupFunnelWorld({ flagPatch, disabledSystems });
  const dt = world.getResource('dt');
  _configureHybridTransitionTrace(world, hybridTrace);
  _configureCableEventTrace(world, eventTrace);

  const prevPos = new Map();
  for (const ballId of ballIds) {
    prevPos.set(ballId, world.getComponent(ballId, PositionComponent).pos.clone());
  }

  const perStep = [];
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

    for (const ballId of ballIds) {
      const pos = world.getComponent(ballId, PositionComponent)?.pos;
      const vel = world.getComponent(ballId, VelocityComponent)?.vel;
      if (
        !pos ||
        !vel ||
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
      prevPos.set(ballId, pos.clone());
    }

    perStep.push({
      step,
      totalMovement,
      maxMovement,
      maxSpeed,
      maxVy,
      minY,
      maxY
    });

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
  const spikeWindow = _sliceByStep(perStep, 230, 320);
  const tailWindow = perStep.slice(Math.max(0, perStep.length - 80));
  const tailAvgMovement = _average(tailWindow.map((entry) => entry.totalMovement));

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
    spikeRatio,
    quietAvg,
    tailAvgMovement,
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
    cableEventTraceTruncated: world.getResource('cableEventTraceTruncated') === true
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
        fixControl: {
          isStable: fixed.isStable,
          spikeStep: fixed.spikeStep,
          growthAbortStep: fixed.growthAbortStep,
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
    } finally {
      console.warn = originalWarn;
      console.error = originalError;
    }
  });
});
