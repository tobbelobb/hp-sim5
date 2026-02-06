import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  AngularVelocityComponent,
  CoefficientOfFrictionComponent,
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import {
  closestPointOnSegment,
  rightOfPlane
} from '../../../src/js/cable_joints_3d/geometry3.js';

export { BallTagComponent, ObstacleTagComponent, ObstaclePushComponent };

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

function normalizePlaneNormal(planeNormal) {
  const n = (planeNormal || DEFAULT_PLANE_NORMAL).clone();
  if (n.lengthSq() <= 1e-12) {
    n.set(DEFAULT_PLANE_NORMAL);
  } else {
    n.normalize();
  }
  return n;
}

function projectPointToPlane(point, planeNormal, planeOffset) {
  const signedDistance = point.dot(planeNormal) - planeOffset;
  point.subtract(planeNormal, signedDistance);
}

function projectVectorToPlane(vector, planeNormal) {
  const normalComponent = vector.dot(planeNormal);
  vector.subtract(planeNormal, normalComponent);
}

function buildPlaneBasis(planeNormal) {
  const n = planeNormal.clone();
  const eps = 1e-9;
  if (n.lengthSq() <= eps) {
    return {
      n: new Vector3(0, 0, 1),
      u: new Vector3(1, 0, 0),
      v: new Vector3(0, 1, 0)
    };
  }

  n.normalize();
  let reference = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  const nDotRef = n.dot(reference);
  let u = reference.clone().subtract(n, nDotRef);
  if (u.lengthSq() <= eps) {
    reference = new Vector3(0, 0, 1);
    const nDotRef2 = n.dot(reference);
    u = reference.clone().subtract(n, nDotRef2);
  }
  u.normalize();
  const v = n.cross(u);
  return { n, u, v };
}

export class FlipperTagComponent {}

export class PlanarConstraintSystem3D {
  runInPause = false;

  constructor(planeNormal = DEFAULT_PLANE_NORMAL, planeOffset = 0.0) {
    this.basis = buildPlaneBasis(planeNormal);
    this.planeOffset = planeOffset;
    this._axisQuat = new Quaternion();
  }

  _projectPointToPlane(point) {
    const signedDistance = point.dot(this.basis.n) - this.planeOffset;
    point.subtract(this.basis.n, signedDistance);
  }

  _projectVectorToPlane(vector) {
    const normalComponent = vector.dot(this.basis.n);
    vector.subtract(this.basis.n, normalComponent);
  }

  _projectAngularVelocityToNormal(vector) {
    const scalar = vector.dot(this.basis.n);
    vector.x = this.basis.n.x * scalar;
    vector.y = this.basis.n.y * scalar;
    vector.z = this.basis.n.z * scalar;
  }

  _clampOrientationToPlaneAxis(quaternion) {
    const rotatedU = quaternion.transformVector(this.basis.u);
    this._projectVectorToPlane(rotatedU);

    if (rotatedU.lengthSq() <= 1e-12) {
      quaternion.set(this._axisQuat.setFromAxisAngle(this.basis.n, 0.0));
      return;
    }

    rotatedU.normalize();
    const angle = Math.atan2(rotatedU.dot(this.basis.v), rotatedU.dot(this.basis.u));
    quaternion.set(this._axisQuat.setFromAxisAngle(this.basis.n, angle));
  }

  update(world, _dt_unused) {
    for (const entityId of world.query([PositionComponent])) {
      const pos = world.getComponent(entityId, PositionComponent).pos;
      this._projectPointToPlane(pos);
    }

    for (const entityId of world.query([PrevFinalPosComponent])) {
      const prevPos = world.getComponent(entityId, PrevFinalPosComponent).pos;
      this._projectPointToPlane(prevPos);
    }

    for (const entityId of world.query([VelocityComponent])) {
      const vel = world.getComponent(entityId, VelocityComponent).vel;
      this._projectVectorToPlane(vel);
    }

    for (const entityId of world.query([AngularVelocityComponent])) {
      const omega = world.getComponent(entityId, AngularVelocityComponent).omega;
      this._projectAngularVelocityToNormal(omega);
    }

    for (const entityId of world.query([OrientationComponent])) {
      const quat = world.getComponent(entityId, OrientationComponent).quaternion;
      this._clampOrientationToPlaneAxis(quat);
    }

    for (const entityId of world.query([PrevFinalOrientationComponent])) {
      const prevQuat = world.getComponent(entityId, PrevFinalOrientationComponent).quaternion;
      this._clampOrientationToPlaneAxis(prevQuat);
    }
  }
}

export class FlipperStateComponent {
  constructor(length, restAngle, maxRotation, angularVelocity, planeNormal = DEFAULT_PLANE_NORMAL) {
    this.length = length;
    this.restAngle = restAngle;
    this.maxRotation = Math.abs(maxRotation);
    this.sign = Math.sign(maxRotation);
    this.angularVelocity = angularVelocity;
    this.planeNormal = planeNormal.clone().normalize();

    this.rotation = 0.0;
    this.currentAngularVelocity = 0.0;
    this.pressed = false;
  }
}

export class FlipperTipComponent {
  constructor(flipperEntityId) {
    this.flipperEntityId = flipperEntityId;
  }
}

export class BorderComponent {
  constructor(points = [], planeNormal = DEFAULT_PLANE_NORMAL) {
    this.points = points.map((p) => p.clone());
    this.planeNormal = planeNormal.clone().normalize();
  }
}

export class ScoredTagComponent {}

export class ScoreComponent {
  constructor(score = 0) {
    this.value = score;
  }
}

export class PauseStateComponent {
  constructor(paused = true) {
    this.paused = paused;
  }
}

export class InputSystem {
  runInPause = true;

  constructor(canvas, world, projectPointerToSim = null, grabSpringParams = { restLength: 0.1, springConstant: () => 10.0 }) {
    this.canvas = canvas;
    this.world = world;
    this.projectPointerToSim = projectPointerToSim;
    this.grabSpringParams = grabSpringParams;

    this.clicks = [];
    this.releases = [];
    this.eventLog = [];
    this.frame = 0;
    this.grabSpring = null;

    this.canvas.setAttribute('tabindex', '0');
    this.canvas.style.outline = 'none';
    this.canvas.focus();

    document.addEventListener('pointerdown', this.handlePointerDown.bind(this));
    document.addEventListener('pointerup', this.handlePointerUp.bind(this));
    this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
    this.canvas.addEventListener('keydown', this.handleKeydown.bind(this));
    this.canvas.addEventListener('keyup', this.handleKeyup.bind(this));
  }

  _shouldHandlePointerEvent(event) {
    if (this.grabSpring) {
      return true;
    }
    const target = event.target;
    return target === this.canvas || this.canvas.contains(target);
  }

  reset() {
    this.clicks.length = 0;
    this.releases.length = 0;
    this.eventLog.length = 0;
    this.frame = 0;
    this.grabSpring = null;
  }

  dumpDebugScenario() {
    const resStr = JSON.stringify(this.world.resources);
    const logEntries = this.eventLog.map((frame) => {
      const clicksStr = frame.clicks.map((p) => `new Vector3(${p.x}, ${p.y}, ${p.z})`).join(', ');
      const releasesStr = frame.releases.map((p) => `new Vector3(${p.x}, ${p.y}, ${p.z})`).join(', ');
      return `{ \"frame\": ${frame.frame}, \"clicks\": [${clicksStr}], \"releases\": [${releasesStr}] }`;
    }).join(', ');
    console.log('DEBUG_SCENARIO_DUMP', `{ \"resources\": ${resStr}, \"inputLog\": [${logEntries}] }`);
  }

  _pointerToSim(event) {
    if (typeof this.projectPointerToSim === 'function') {
      const projected = this.projectPointerToSim(event.clientX, event.clientY);
      if (projected) {
        return projected;
      }
    }

    const rect = this.canvas.getBoundingClientRect();
    const simHeight = this.world.getResource('simHeight') || 1.7;
    const scale = this.canvas.height / simHeight;
    const simX = (event.clientX - rect.left) / scale;
    const simY = (this.canvas.height - (event.clientY - rect.top)) / scale;
    return new Vector3(simX, simY, 0);
  }

  _queueFlipperEdgeInput(isLeft, isPress) {
    const flipperEntities = this.world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);
    if (flipperEntities.length < 2) {
      return;
    }

    const firstPos = this.world.getComponent(flipperEntities[0], PositionComponent).pos;
    const secondPos = this.world.getComponent(flipperEntities[1], PositionComponent).pos;

    let targetPos = null;
    if (firstPos.x < secondPos.x) {
      targetPos = isLeft ? firstPos : secondPos;
    } else {
      targetPos = isLeft ? secondPos : firstPos;
    }

    if (isPress) {
      this.clicks.push(targetPos.clone());
    } else {
      this.releases.push(targetPos.clone());
    }
  }

  handleKeyup(event) {
    if (event.key === 'ArrowLeft') {
      this._queueFlipperEdgeInput(true, false);
    }
    if (event.key === 'ArrowRight') {
      this._queueFlipperEdgeInput(false, false);
    }
  }

  handleKeydown(event) {
    if (event.code === 'Space') {
      this.dumpDebugScenario();
    }
    if (event.key === 'ArrowLeft') {
      this._queueFlipperEdgeInput(true, true);
    }
    if (event.key === 'ArrowRight') {
      this._queueFlipperEdgeInput(false, true);
    }
  }

  handlePointerDown(event) {
    if (event.button !== undefined && event.button !== 0) {
      return;
    }
    if (!this._shouldHandlePointerEvent(event)) {
      return;
    }
    event.preventDefault();
    const clickPos = this._pointerToSim(event);
    if (!clickPos) {
      return;
    }

    if (this.canvas.setPointerCapture) {
      try {
        this.canvas.setPointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore capture failures.
      }
    }

    const cmOnScreen = 0.5;
    const dpi = 96;
    const pixelsPerCm = dpi / 2.54;
    const extraPixels = cmOnScreen * pixelsPerCm;
    const scale = this.world.getResource('cScale') || (this.canvas.height / (this.world.getResource('simHeight') || 1.7));
    const extraClickableRadius = extraPixels / Math.max(scale, 1e-9);

    let closestBall = null;
    let closestDistSq = Infinity;

    for (const ballId of this.world.query([BallTagComponent, PositionComponent, RadiusComponent])) {
      const pos = this.world.getComponent(ballId, PositionComponent).pos;
      const radius = this.world.getComponent(ballId, RadiusComponent).radius + extraClickableRadius;
      const distSq = clickPos.clone().subtract(pos).lengthSq();
      if (distSq <= radius * radius && distSq < closestDistSq) {
        closestBall = ballId;
        closestDistSq = distSq;
      }
    }

    if (closestBall !== null) {
      const ptrE = this.world.createEntity();
      this.world.addComponent(ptrE, new PositionComponent(clickPos.x, clickPos.y, clickPos.z));
      this.world.addComponent(ptrE, new CableLinkComponent(clickPos.x, clickPos.y, clickPos.z));

      const ballPos = this.world.getComponent(closestBall, PositionComponent).pos.clone();
      const ballMass = this.world.getComponent(closestBall, MassComponent).mass;
      const ptrPos = clickPos.clone();

      const jointE = this.world.createEntity();
      this.world.addComponent(
        jointE,
        new CableJointComponent(
          closestBall,
          ptrE,
          this.grabSpringParams.restLength,
          ballPos,
          ptrPos
        )
      );

      const pathE = this.world.createEntity();
      this.world.addComponent(
        pathE,
        new CablePathComponent(
          this.world,
          [jointE],
          ['attachment', 'attachment'],
          [true],
          this.grabSpringParams.springConstant(ballMass)
        )
      );

      this.grabSpring = { ptrE, jointE, pathE, ballE: closestBall };
      this.world.setResource('grabbedBall', closestBall);

      const pauseState = this.world.getResource('pauseState');
      if (pauseState) {
        pauseState.paused = false;
      }
      const pauseBtn = document.getElementById('pauseBtn');
      if (pauseBtn) {
        pauseBtn.textContent = 'Pause';
      }

      return;
    }

    this.clicks.push(clickPos);
  }

  handlePointerUp(event) {
    if (!this.grabSpring && event.button !== undefined && event.button !== 0) {
      return;
    }
    if (!this._shouldHandlePointerEvent(event)) {
      return;
    }
    event.preventDefault();

    if (this.canvas.releasePointerCapture) {
      try {
        this.canvas.releasePointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore release failures.
      }
    }

    const releasePos = this._pointerToSim(event);
    if (!releasePos) {
      return;
    }

    if (this.grabSpring) {
      const { ptrE, jointE, pathE, ballE } = this.grabSpring;
      const velComp = this.world.getComponent(ballE, VelocityComponent);
      const posComp = this.world.getComponent(ballE, PositionComponent);
      const prevFinalPosComp = this.world.getComponent(ballE, PrevFinalPosComponent);

      const dt = this.world.getResource('dt');
      if (velComp && posComp && prevFinalPosComp && dt > 1e-9) {
        velComp.vel.set(posComp.pos.clone().subtract(prevFinalPosComp.pos).scale(1.0 / dt));
      }

      this.world.destroyEntity(pathE);
      this.world.destroyEntity(jointE);
      this.world.destroyEntity(ptrE);
      this.grabSpring = null;
      this.world.setResource('grabbedBall', null);
      return;
    }

    this.releases.push(releasePos);
  }

  handlePointerMove(event) {
    if (!this.grabSpring) {
      return;
    }
    event.preventDefault();

    const simPos = this._pointerToSim(event);
    if (!simPos) {
      return;
    }

    const ptrPos = this.world.getComponent(this.grabSpring.ptrE, PositionComponent)?.pos;
    if (ptrPos) {
      ptrPos.set(simPos);
    }
  }

  update(world, _dt_unused) {
    const clicksFrame = this.clicks.slice();
    const releasesFrame = this.releases.slice();

    if (clicksFrame.length > 0 || releasesFrame.length > 0) {
      this.eventLog.push({ frame: this.frame, clicks: clicksFrame, releases: releasesFrame });
    }
    this.frame += 1;

    if (clicksFrame.length > 0) {
      const clickPos = this.clicks.shift();
      const flipperEntities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);

      const borderEntities = world.query([BorderComponent]);
      if (borderEntities.length > 0) {
        const border = world.getComponent(borderEntities[0], BorderComponent);
        const borderPoints = border.points;
        const planeNormal = border.planeNormal || DEFAULT_PLANE_NORMAL;

        const rightClick =
          rightOfPlane(clickPos, borderPoints[0], borderPoints[1], planeNormal) &&
          rightOfPlane(clickPos, borderPoints[1], borderPoints[2], planeNormal);

        const leftClick =
          rightOfPlane(clickPos, borderPoints[5], borderPoints[6], planeNormal) &&
          rightOfPlane(clickPos, borderPoints[6], borderPoints[7], planeNormal);

        if (rightClick) {
          const flippers = world.query([FlipperStateComponent, PositionComponent]);
          const pos0 = world.getComponent(flippers[0], PositionComponent).pos;
          const pos1 = world.getComponent(flippers[1], PositionComponent).pos;
          if (pos0.x > pos1.x) {
            world.getComponent(flippers[0], FlipperStateComponent).pressed = true;
          } else {
            world.getComponent(flippers[1], FlipperStateComponent).pressed = true;
          }
        } else if (leftClick) {
          const flippers = world.query([FlipperStateComponent, PositionComponent]);
          const pos0 = world.getComponent(flippers[0], PositionComponent).pos;
          const pos1 = world.getComponent(flippers[1], PositionComponent).pos;
          if (pos0.x < pos1.x) {
            world.getComponent(flippers[0], FlipperStateComponent).pressed = true;
          } else {
            world.getComponent(flippers[1], FlipperStateComponent).pressed = true;
          }
        } else {
          for (const id of flipperEntities) {
            const pos = world.getComponent(id, PositionComponent).pos;
            const state = world.getComponent(id, FlipperStateComponent);
            if (clickPos.clone().subtract(pos).lengthSq() < state.length ** 2) {
              state.pressed = true;
            }
          }
        }
      }
    }

    if (this.releases.length > 0) {
      const releasePos = this.releases.shift();
      const flipperEntities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);

      let closestId = null;
      let minDistSq = Infinity;

      for (const id of flipperEntities) {
        const state = world.getComponent(id, FlipperStateComponent);
        if (!state.pressed) continue;

        const pos = world.getComponent(id, PositionComponent).pos;
        const d2 = releasePos.clone().subtract(pos).lengthSq();
        if (d2 < minDistSq) {
          minDistSq = d2;
          closestId = id;
        }
      }

      if (closestId !== null) {
        world.getComponent(closestId, FlipperStateComponent).pressed = false;
      }
    }
  }
}

export class InputReplaySystem {
  runInPause = false;

  constructor(inputLog, inputSystem) {
    this.inputLog = inputLog;
    this.currentIndex = 0;
    this.frame = 0;
    this.inputSystem = inputSystem;
  }

  update(_world, _dt_unused) {
    if (this.inputLog.length > 0) {
      if (this.inputSystem.frame === this.inputLog[0].frame) {
        const frame = this.inputLog.shift();
        this.inputSystem.clicks = frame.clicks.slice();
        this.inputSystem.releases = frame.releases.slice();
      }
    }
  }

  reset() {
    this.inputLog = [];
    this.currentIndex = 0;
    this.frame = 0;
  }
}

export class ScoreSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const scoreEntity = world.query([ScoreComponent])[0];
    const scoreComp = scoreEntity !== undefined ? world.getComponent(scoreEntity, ScoreComponent) : null;
    if (!scoreComp) return;

    const scoredEntities = world.query([ScoredTagComponent]);
    for (const scoredId of scoredEntities) {
      world.removeComponent(scoredId, ScoredTagComponent);
      scoreComp.value += 1;
    }
  }
}

export class ScoreDisplaySystem {
  runInPause = true;

  constructor(elementId) {
    this.scoreElement = document.getElementById(elementId);
  }

  update(world, _dt_unused) {
    const scoreEntity = world.query([ScoreComponent])[0];
    if (scoreEntity !== undefined && this.scoreElement) {
      const scoreComp = world.getComponent(scoreEntity, ScoreComponent);
      this.scoreElement.textContent = scoreComp.value.toString();
    }
  }
}

export class FlipperMotionSystem {
  runInPause = true;

  update(world, dt) {
    const flipperEntities = world.query([FlipperStateComponent]);
    for (const entityId of flipperEntities) {
      const state = world.getComponent(entityId, FlipperStateComponent);

      const prevRotation = state.rotation;
      if (state.pressed) {
        state.rotation = Math.min(state.rotation + dt * state.angularVelocity, state.maxRotation);
      } else {
        state.rotation = Math.max(state.rotation - dt * state.angularVelocity, 0.0);
      }

      state.currentAngularVelocity = dt > 1e-6
        ? state.sign * (state.rotation - prevRotation) / dt
        : 0.0;
    }
  }
}

export class FlipperTipLinkSystem {
  runInPause = false;

  update(world, _dt_unused) {
    for (const tipId of world.query([FlipperTipComponent, PositionComponent])) {
      const tipComp = world.getComponent(tipId, FlipperTipComponent);
      const flipId = tipComp.flipperEntityId;
      const pivotPos = world.getComponent(flipId, PositionComponent).pos;
      const state = world.getComponent(flipId, FlipperStateComponent);

      const angle = state.restAngle + state.sign * state.rotation;
      const dir = new Vector3(Math.cos(angle), Math.sin(angle), 0);
      const tipPos = pivotPos.clone().add(dir, state.length);

      world.getComponent(tipId, PositionComponent).pos.set(tipPos);
    }
  }
}

export class PBDBallBorderCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const borderEntities = world.query([BorderComponent]);
    if (borderEntities.length === 0) return;

    const borderId = borderEntities[0];
    const borderComp = world.getComponent(borderId, BorderComponent);
    const borderPoints = borderComp.points;

    const borderRestitutionComp = world.getComponent(borderId, RestitutionComponent);
    const borderFrictionComp = world.getComponent(borderId, CoefficientOfFrictionComponent);

    const restitution = borderRestitutionComp ? borderRestitutionComp.restitution : null;
    const friction = borderFrictionComp ? borderFrictionComp.mu : null;

    let contacts = world.getResource('ball_border_contacts');
    if (!contacts) {
      contacts = [];
      world.setResource('ball_border_contacts', contacts);
    }
    contacts.length = 0;

    if (borderPoints.length < 2) {
      return;
    }

    const planeNormal = normalizePlaneNormal(borderComp.planeNormal || DEFAULT_PLANE_NORMAL);
    const planeOffset = borderPoints.length > 0 ? planeNormal.dot(borderPoints[0]) : 0.0;

    for (const ballId of ballEntities) {
      const ballPos = world.getComponent(ballId, PositionComponent).pos;
      const radius = world.getComponent(ballId, RadiusComponent).radius;
      const massComp = world.getComponent(ballId, MassComponent);
      const invMass = massComp && massComp.mass > 0 ? 1.0 / massComp.mass : 0.0;
      projectPointToPlane(ballPos, planeNormal, planeOffset);

      let minDistSq = Infinity;
      let closestSegPoint = null;
      let edgeStart = null;
      let edgeEnd = null;

      for (let i = 0; i < borderPoints.length; i++) {
        const a = borderPoints[i].clone();
        const b = borderPoints[(i + 1) % borderPoints.length].clone();
        projectPointToPlane(a, planeNormal, planeOffset);
        projectPointToPlane(b, planeNormal, planeOffset);
        const closest = closestPointOnSegment(ballPos, a, b);
        const distSq = ballPos.distanceToSq(closest);

        if (distSq < minDistSq) {
          minDistSq = distSq;
          closestSegPoint = closest;
          edgeStart = a;
          edgeEnd = b;
        }
      }

      if (!closestSegPoint || minDistSq > radius * radius) {
        continue;
      }

      const ballToClosest = ballPos.clone().subtract(closestSegPoint);
      const edgeVec = edgeEnd.clone().subtract(edgeStart);
      const edgeNormal = planeNormal.clone().cross(edgeVec).normalize();

      let collisionNormal;
      if (ballToClosest.lengthSq() < 1e-9) {
        collisionNormal = edgeNormal.clone();
      } else {
        collisionNormal = ballToClosest.clone().normalize();
      }

      if (ballToClosest.dot(edgeNormal) < 0) {
        collisionNormal = edgeNormal.clone();
      }

      const dist = Math.sqrt(minDistSq);
      const penetration = radius - dist;

      let deltaLambda = 0;
      if (penetration > 0 && invMass > 0) {
        ballPos.add(collisionNormal, penetration);
        projectPointToPlane(ballPos, planeNormal, planeOffset);
        deltaLambda = penetration / invMass;
      }

      contacts.push({
        ball_id: ballId,
        normal: collisionNormal.clone(),
        delta_lambda: deltaLambda,
        restitution,
        friction
      });
    }
  }
}

export class PBDBallBallCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    for (let i = 0; i < ballEntities.length; i++) {
      for (let j = i + 1; j < ballEntities.length; j++) {
        const e1 = ballEntities[i];
        const e2 = ballEntities[j];

        const p1Comp = world.getComponent(e1, PositionComponent);
        const r1 = world.getComponent(e1, RadiusComponent).radius;
        const m1Comp = world.getComponent(e1, MassComponent);

        const p2Comp = world.getComponent(e2, PositionComponent);
        const r2 = world.getComponent(e2, RadiusComponent).radius;
        const m2Comp = world.getComponent(e2, MassComponent);

        const p1 = p1Comp.pos;
        const p2 = p2Comp.pos;
        const m1 = m1Comp ? m1Comp.mass : 0.0;
        const m2 = m2Comp ? m2Comp.mass : 0.0;

        const dir = new Vector3(p2.x - p1.x, p2.y - p1.y, 0.0);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq === 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        const penetration = rSum - d;
        const invMass1 = m1 > 0 ? 1.0 / m1 : 0.0;
        const invMass2 = m2 > 0 ? 1.0 / m2 : 0.0;
        const totalInvMass = invMass1 + invMass2;

        if (totalInvMass <= 1e-9) continue;

        const corr = dir.clone().scale(penetration / totalInvMass);
        p1.add(corr, -invMass1);
        p2.add(corr, invMass2);
      }
    }
  }
}

export class PBDBallObstacleCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent]);

    let contacts = world.getResource('ball_obstacle_contacts');
    if (!contacts) {
      contacts = [];
      world.setResource('ball_obstacle_contacts', contacts);
    }
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;

        const dir = new Vector3(p1.x - p2.x, p1.y - p2.y, 0.0);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq === 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        contacts.push({ ball_id: ballId, obs_id: obsId, direction: dir.clone() });

        const corr = rSum - d;
        p1.add(dir, corr);

        const grabbed = world.getResource('grabbedBall');
        if (ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
      }
    }
  }
}

export class PBDBallFlipperCollisions {
  runInPause = false;

  _getFlipperTip(flipperPos, flipperState) {
    const angle = flipperState.restAngle + flipperState.sign * flipperState.rotation;
    const dir = new Vector3(Math.cos(angle), Math.sin(angle), 0);
    return flipperPos.clone().add(dir, flipperState.length);
  }

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);

    let contacts = world.getResource('ball_flipper_contacts');
    if (!contacts) {
      contacts = [];
      world.setResource('ball_flipper_contacts', contacts);
    }
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const ballPos = world.getComponent(ballId, PositionComponent).pos;
      const ballRadius = world.getComponent(ballId, RadiusComponent).radius;
      const massComp = world.getComponent(ballId, MassComponent);
      const invMass = massComp && massComp.mass > 0 ? 1.0 / massComp.mass : 0.0;

      for (const flipperId of flipperEntities) {
        const pivot = world.getComponent(flipperId, PositionComponent).pos;
        const flipperRadius = world.getComponent(flipperId, RadiusComponent).radius;
        const flipperState = world.getComponent(flipperId, FlipperStateComponent);

        const tip = this._getFlipperTip(pivot, flipperState);
        const closest = closestPointOnSegment(ballPos, pivot, tip);

        const dir = ballPos.clone().subtract(closest);
        const dSq = dir.lengthSq();
        const rSum = ballRadius + flipperRadius;

        if (dSq === 0.0 || dSq > rSum * rSum) {
          continue;
        }

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        const correction = rSum - d;
        if (invMass > 0) {
          ballPos.add(dir, correction);
        }

        let deltaLambda = 0;
        if (invMass > 0) {
          deltaLambda = correction / invMass;
        }

        contacts.push({
          ball_id: ballId,
          flip_id: flipperId,
          normal: dir.clone(),
          contact_point_on_flipper: closest.clone(),
          delta_lambda: deltaLambda
        });
      }
    }
  }
}
