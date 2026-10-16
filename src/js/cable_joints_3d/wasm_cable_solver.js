import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';
import {
  PositionComponent, OrientationComponent, PrevFinalPosComponent,
  PrevFinalOrientationComponent, MassComponent, MomentOfInertiaComponent,
  RigidBodyMemberComponent, RadiusComponent, EncoderComponent,
} from './ecs.js';
import {
  CablePathComponent, CableJointComponent, CableLinkComponent,
  cableSolverEffectiveRadius, cableSolverEffectiveCW,
  cableSolverMotorStiffness, cableSolverHoldingTorqueLimit,
} from './cable_joints_core.js';
import { computeLocalAttachment, createAttachmentTransformScratch } from './rigid_bodies.js';
import { hasAnyInverseInertia } from './inertia_tensor.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

// f64 records, mirrored by Rust's E/J/P constants and slot accesses.
// Entity: pos 0, quat 3, prevPos 7, prevQuat 10, localPos 14, localQuat 17,
// prevLocalQuat 21, inertia 25, inverseInertia 34, invMass 43, axis 44,
// stiffness 47, stiffness*dt² 48, local flags 49, parent 50,
// hasPos/Quat/prevPos/prevQuat/angular/spin/torqueMode 51..57,
// loadDamping 58, loadTorque 59, loadStiffness 60, holdingLimit 61,
// hasPrevLocalQuat 62, dirtyPos/Quat/localQuat 63..65, encoder 66, hasEncoder 67,
// load map presence mask 68 (including loads that cancel to zero).
// Joint: endpoints 0..3, locals 4..9, rest 10, path/index 11..12,
// spinA(id,local,coupled) 13..17, spinB 18..22, storedGrads 23..24,
// spinFlags 25..26, transferredJoints 27..28, pinholeNeighbors 29..30,
// lambda 31, force 32..34, magnitude 35, transfer 36, solved 37.
// Path: first/count/iterations 0..2, zeroStiffness/alpha/gamma/dampingStep 3..6,
// invDt² 7, springConstant 8, damping 9.
const E = 160, J = 38, P = 10, EPS = 1e-9;
const hybrid = type => type === 'hybrid' || type === 'hybrid-attachment';
const backdrivable = type => type === 'rolling' || hybrid(type);
const validQuat = q => q && [q.x, q.y, q.z, q.w].every(Number.isFinite);
const putV = (a, i, v) => { if (v) { a[i] = v.x; a[i + 1] = v.y; a[i + 2] = v.z; } };
const putQ = (a, i, q) => { if (q) { putV(a, i, q); a[i + 3] = q.w; } };
const readV = (v, a, i) => { v.x = a[i]; v.y = a[i + 1]; v.z = a[i + 2]; };
const readQ = (q, a, i) => { readV(q, a, i); q.w = a[i + 3]; };

export async function compileCableSolverWasm(bytes) {
  return WebAssembly.compile(bytes);
}

// One backend belongs to one solver. Instantiate a compiled Module for each
// world; capacity grows only when topology grows, and views refresh after growth.
export class WasmCableSolver {
  constructor(module) {
    this.exports = new WebAssembly.Instance(module, {}).exports;
    if (this.exports.abi_version() !== 2) throw new Error('Unsupported cable solver WASM ABI');
    this.capacity = 0;
    this.pointer = 0;
    this.scratch = createAttachmentTransformScratch();
    this.attachmentPointer = this.exports.alloc_buffer(46);
    this.attachmentData = new Float64Array(this.exports.memory.buffer, this.attachmentPointer, 46);
  }

  calculateAttachmentPoints(joint, path, i, radiusA, radiusB, endpointA, endpointB) {
    if (this.attachmentData.buffer !== this.exports.memory.buffer) {
      this.attachmentData = new Float64Array(this.exports.memory.buffer, this.attachmentPointer, 46);
    }
    const data = this.attachmentData;
    data.fill(0);
    for (let side = 0; side < 2; side++) {
      const o = side * 19, endpoint = side === 0 ? endpointA : endpointB;
      const radius = side === 0 ? radiusA : radiusB, type = path.linkTypes[i + side];
      putV(data, o, endpoint.pos); putV(data, o + 3, endpoint.prevPos);
      putV(data, o + 6, endpoint.planeNormal);
      putV(data, o + 9, side === 0 ? joint.attachmentPointA_world : joint.attachmentPointB_world);
      data[o + 12] = endpoint.deltaAngle;
      data[o + 13] = (endpoint.pos ? 1 : 0) | (endpoint.prevPos ? 2 : 0) | (radius !== undefined ? 4 : 0);
      data[o + 14] = radius ?? 0;
      data[o + 15] = +Boolean(cableSolverEffectiveCW(path, i + side, side === 0));
      data[o + 16] = +(type === 'rolling' || type === 'hybrid');
      data[o + 17] = +hybrid(type); data[o + 18] = +(type === 'hybrid-attachment');
    }
    this.exports.attachment_points(this.attachmentPointer);
    const a = data[44] ? new Vector3(data[38], data[39], data[40]) : null;
    const b = data[45] ? new Vector3(data[41], data[42], data[43]) : null;
    return { attachmentA_current: a, attachmentB_current: b };
  }

  update(world) {
    const pathIds = world.query([CablePathComponent]);
    const get = (id, Type) => world.getComponent(id, Type);
    const paths = pathIds.map(id => get(id, CablePathComponent));
    const joints = paths.flatMap(path => path.jointEntities.map(id => get(id, CableJointComponent)));
    const jointIndex = new Map(paths.flatMap(path => path.jointEntities).map((id, i) => [id, i]));
    const ids = [], entityIndex = new Map();
    const include = id => {
      if (entityIndex.has(id)) return;
      entityIndex.set(id, ids.length); ids.push(id);
      const member = get(id, RigidBodyMemberComponent);
      if (member) include(member.bodyEntity);
    };
    for (const joint of joints) { include(joint.entityA); include(joint.entityB); }
    // Unloaded mounted rotors participate in the carrier inertia too.
    for (const id of world.query([RigidBodyMemberComponent, SpoolStateComponent])) {
      if (entityIndex.has(get(id, RigidBodyMemberComponent).bodyEntity)) include(id);
    }
    const size = ids.length * E + joints.length * J + paths.length * P;
    if (!size) {
      for (const key of ['torqueModeCableLoadTorques', 'torqueModeCableLoadStiffnesses', 'torqueModeCableLoadDampings']) {
        world.setResource(key, new Map());
      }
      return;
    }
    if (size > this.capacity) {
      if (this.capacity) this.exports.free_buffer(this.pointer, this.capacity);
      this.capacity = Math.max(size, this.capacity * 2);
      this.pointer = this.exports.alloc_buffer(this.capacity);
    }
    const data = new Float64Array(this.exports.memory.buffer, this.pointer, size);
    data.fill(0);
    const dt = world.getResource('dt'), validDt = Number.isFinite(dt) && dt > EPS;
    const members = [], encoders = [];
    for (let n = 0; n < ids.length; n++) {
      const id = ids[n], o = n * E;
      const pos = get(id, PositionComponent)?.pos, quat = get(id, OrientationComponent)?.quaternion;
      const prevPos = get(id, PrevFinalPosComponent)?.pos, prevQuat = get(id, PrevFinalOrientationComponent)?.quaternion;
      const member = get(id, RigidBodyMemberComponent), inertia = get(id, MomentOfInertiaComponent);
      const mass = get(id, MassComponent)?.mass, axis = get(id, CableLinkComponent)?.cablePlaneNormalLocal;
      const spool = get(id, SpoolStateComponent), motor = get(id, StepperMotorComponent);
      const encoder = get(id, EncoderComponent);
      putV(data, o, pos); putQ(data, o + 3, quat); putV(data, o + 7, prevPos); putQ(data, o + 10, prevQuat);
      putV(data, o + 14, member?.localPosition); putQ(data, o + 17, member?.localOrientation);
      let prevLocal = prevQuat;
      if (member) {
        const bodyPrev = get(member.bodyEntity, PrevFinalOrientationComponent)?.quaternion;
        prevLocal = bodyPrev && prevQuat
          ? new Quaternion().multiplyQuaternions(bodyPrev.clone().conjugate().normalize(), prevQuat).normalize() : null;
      }
      putQ(data, o + 21, prevLocal);
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
        data[o + 25 + r * 3 + c] = inertia?.inertiaTensor?.[r]?.[c] ?? inertia?.tensor?.[r]?.[c] ?? 0;
        data[o + 34 + r * 3 + c] = inertia?.invInertiaTensor?.[r]?.[c]
          ?? inertia?.inverseInertiaTensor?.[r]?.[c] ?? (r === c ? inertia?.invInertia ?? 0 : 0);
      }
      data[o + 43] = mass > 0 ? 1 / mass : 0;
      putV(data, o + 44, axis);
      const stiffness = cableSolverMotorStiffness(world, motor);
      data[o + 47] = validDt ? stiffness : 0;
      data[o + 48] = stiffness * dt * dt;
      data[o + 49] = (member?.localPosition ? 1 : 0) | (validQuat(member?.localOrientation) ? 2 : 0);
      data[o + 50] = member ? entityIndex.get(member.bodyEntity) : -1;
      data[o + 51] = Boolean(pos); data[o + 52] = validQuat(quat) ? 1 : 0;
      data[o + 53] = Boolean(prevPos); data[o + 54] = Boolean(prevQuat);
      data[o + 55] = Boolean(!(spool && axis) && inertia && quat);
      data[o + 56] = Boolean(axis && inertia && hasAnyInverseInertia(inertia));
      data[o + 57] = Boolean(motor?.torqueMode);
      data[o + 61] = cableSolverHoldingTorqueLimit(world, motor);
      data[o + 62] = Boolean(prevLocal);
      data[o + 66] = encoder?.angle ?? 0;
      data[o + 67] = Boolean(encoder && Number.isFinite(encoder.angle));
      members.push(member); encoders.push(encoder);
    }
    const locals = joints.map(joint => ({
      a: computeLocalAttachment(world, joint.entityA, joint.attachmentPointA_world, new Vector3(), this.scratch),
      b: computeLocalAttachment(world, joint.entityB, joint.attachmentPointB_world, new Vector3(), this.scratch),
    }));
    const mapEndpoint = (id, other) => {
      const member = get(id, RigidBodyMemberComponent), counterpart = get(other, RigidBodyMemberComponent);
      const internal = Boolean(member && counterpart && member.bodyEntity === counterpart.bodyEntity);
      return { id: member && !internal ? member.bodyEntity : id, internal };
    };
    const candidate = id => get(id, CableLinkComponent)?.cablePlaneNormalLocal
      && get(id, OrientationComponent)?.quaternion && hasAnyInverseInertia(get(id, MomentOfInertiaComponent));
    const jointStart = ids.length * E, pathStart = jointStart + joints.length * J;
    let first = 0;
    for (let p = 0; p < paths.length; p++) {
      const path = paths[p], po = pathStart + p * P, compliance = path.compliance ?? 0;
      const zero = path.compliance === Infinity, damping = Math.max(0, path.damping ?? 0);
      data.set([first, path.jointEntities.length, path.solverIterations ?? 1, +zero,
        !zero && validDt ? compliance / (dt * dt) : 0,
        !zero && validDt ? Math.max(0, compliance * (path.damping ?? 0) / dt) : 0,
        validDt ? damping * dt : 0, validDt ? 1 / (dt * dt) : 0,
        Number.isFinite(path.spring_constant) ? Math.max(0, path.spring_constant) : 0,
        Number.isFinite(path.damping) ? Math.max(0, path.damping) : 0], po);
      for (let i = 0; i < path.jointEntities.length; i++) {
        const n = first + i, joint = joints[n], o = jointStart + n * J;
        const mapped = [mapEndpoint(joint.entityA, joint.entityB), mapEndpoint(joint.entityB, joint.entityA)];
        data.set([entityIndex.get(joint.entityA), entityIndex.get(joint.entityB),
          entityIndex.get(mapped[0].id), entityIndex.get(mapped[1].id)], o);
        putV(data, o + 4, locals[n].a); putV(data, o + 7, locals[n].b);
        data[o + 10] = joint.restLength; data[o + 11] = p; data[o + 12] = i;
        data[o + 13] = data[o + 18] = data[o + 27] = data[o + 28] = data[o + 29] = data[o + 30] = -1;
        for (let side = 0; side < 2; side++) {
          let id = side === 0 ? joint.entityA : joint.entityB, linkIndex = i + side;
          let spinJoint = n, coupled = false;
          const direct = Boolean(get(id, SpoolStateComponent) && candidate(id) && path.linkTypes[linkIndex] !== undefined);
          if (!direct) {
            if (mapped[side].internal || path.linkTypes[linkIndex] !== 'pinhole') continue;
            const neighbor = side === 0 ? i - 1 : i + 1;
            const neighborLink = side === 0 ? linkIndex - 1 : linkIndex + 1;
            if (neighbor < 0 || neighbor >= path.jointEntities.length || !backdrivable(path.linkTypes[neighborLink])) continue;
            spinJoint = first + neighbor;
            id = side === 0 ? joints[spinJoint].entityA : joints[spinJoint].entityB;
            if (!candidate(id)) continue;
            linkIndex = neighbor + side; coupled = true;
            data[o + 27 + side] = jointIndex.get(path.jointEntities[neighbor]);
          }
          const so = o + (side === 0 ? 13 : 18);
          data[so] = entityIndex.get(id); putV(data, so + 1, side === 0 ? locals[spinJoint].a : locals[spinJoint].b);
          data[so + 4] = +coupled;
          let stored = 0;
          if (hybrid(path.linkTypes[linkIndex])) {
            const base = get(id, RadiusComponent)?.radius, radius = cableSolverEffectiveRadius(world, path, linkIndex, base).radius;
            const effective = Number.isFinite(radius) ? radius : (Number.isFinite(base) ? base : 0);
            if (effective > EPS) {
              stored = (cableSolverEffectiveCW(path, linkIndex, side === 0) ? 1 : -1) * effective;
              if (side === 1) stored = -stored;
            }
          }
          data[o + 23 + side] = stored;
          if (Math.abs(stored) > EPS) {
            let flags = 2 | 4;
            if (direct) {
              flags |= 1;
              if ((side === 0 && path.linkTypes[i + 1] === 'pinhole' && i + 1 < path.jointEntities.length)
                || (side === 1 && path.linkTypes[i] === 'pinhole' && i > 0)) flags |= 8;
            }
            data[o + 25 + side] = flags;
          }
        }
        if (path.linkTypes[i] === 'pinhole' && i > 0) data[o + 29] = first + i - 1;
        if (path.linkTypes[i + 1] === 'pinhole' && i + 1 < path.jointEntities.length) data[o + 30] = first + i + 1;
      }
      first += path.jointEntities.length;
    }
    this.exports.solve(this.pointer, ids.length, joints.length, paths.length);
    const torques = new Map(), stiffnesses = new Map(), dampings = new Map();
    for (let n = 0; n < ids.length; n++) {
      const id = ids[n], o = n * E;
      if (data[o + 63]) readV(get(id, PositionComponent).pos, data, o);
      if (data[o + 64]) readQ(get(id, OrientationComponent).quaternion, data, o + 3);
      if (data[o + 65]) readQ(members[n].localOrientation, data, o + 17);
      if (data[o + 67]) encoders[n].angle = data[o + 66];
      if (data[o + 68] & 1) torques.set(id, data[o + 59]);
      if (data[o + 68] & 2) stiffnesses.set(id, data[o + 60]);
      if (data[o + 68] & 4) dampings.set(id, data[o + 58]);
    }
    world.setResource('torqueModeCableLoadTorques', torques);
    world.setResource('torqueModeCableLoadStiffnesses', stiffnesses);
    world.setResource('torqueModeCableLoadDampings', dampings);
    for (let n = 0; n < joints.length; n++) {
      const joint = joints[n], o = jointStart + n * J;
      joint.constraintLambda = data[o + 31];
      if (joint.constraintForce || data[o + 37]) {
        joint.constraintForce ??= new Vector3(); readV(joint.constraintForce, data, o + 32);
      }
      joint.constraintForceMagnitude = data[o + 35];
      joint.transferredConstraintForceMagnitude = data[o + 36];
    }
  }
}
