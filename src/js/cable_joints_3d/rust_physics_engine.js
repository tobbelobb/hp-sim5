import * as ecs from './ecs.js';
import { CableLinkComponent, CableJointComponent, CablePathComponent } from './cable_joints_core.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

// See engine.rs. Only scene preparation crosses this boundary; live numeric
// components are views of persistent Rust records, including interactive edits.
const E = 160, H = 9, L = 5, J = 16;
const kinds = ['attachment', 'rolling', 'pinhole', 'hybrid', 'hybrid-attachment'];
const putV = (a, o, v) => { if (v) a.set([v.x, v.y, v.z], o); };
const putQ = (a, o, q) => { if (q) a.set([q.x, q.y, q.z, q.w], o); };
export class RustPhysicsEngine {
  fullEngine = true;
  update(world, dt = world.getResource('dt')) { this.stepPhysics(world, dt); }
  constructor(transport) {
    this.transport = transport;
    this.bindings = [];
    this.regions = [];
    this.revision = -1;
    this.dirty = false;
  }
  refreshMemory() { this.transport.refresh?.(this.regions); }
  region(data, read) {
    const region = { data, read };
    this.regions.push(region);
    return region;
  }
  bind(object, key, offset, group = null, decode = x => x, encode = x => x, region = this.bindingRegion) {
    const getter = () => { this.refreshMemory(); return decode(region.data[offset]); };
    Object.defineProperty(object, key, { enumerable: true, configurable: true,
      get: getter, set: value => {
        const encoded = encode(value);
        if (Object.getOwnPropertyDescriptor(object, key)?.get === getter) { this.refreshMemory(); region.data[offset] = encoded; }
      } });
    this.bindings.push({ object, key, getter, group });
  }
  vector(object, key, offset, group = null) {
    const value = object?.[key];
    if (!value) return;
    for (const [i, field] of (key.toLowerCase().includes('orientation') || key === 'quaternion'
      ? ['x', 'y', 'z', 'w'] : ['x', 'y', 'z']).entries()) this.bind(value, field, offset + i, group);
    // Replacing a vector is an edit too (common for reference orientations).
    const getter = () => value;
    Object.defineProperty(object, key, { configurable: true, enumerable: true, get: getter,
      set: input => { if (input) value.set(input); else { this.invalidate(); Object.defineProperty(object, key, { configurable: true, enumerable: true, writable: true, value: input }); } } });
    this.bindings.push({ object, key, getter, group });
  }
  unbind(group = undefined, copy = true) {
    const keep = [];
    for (const b of this.bindings) {
      if (group !== undefined && b.group !== group) { keep.push(b); continue; }
      if (Object.getOwnPropertyDescriptor(b.object, b.key)?.get !== b.getter) continue;
      const value = copy ? b.getter() : undefined;
      Object.defineProperty(b.object, b.key, { enumerable: true, configurable: true, writable: true, value });
    }
    this.bindings = keep;
  }
  invalidate() { this.unbind(); this.dirty = true; }
  dispose() {
    this.unbind();
    this.transport.destroy();
    this.regions = []; this.revision = -1; this.world = null;
  }
  prepare(world) {
    this.dispose(); this.world = world;
    // Cable/path entities have no body record. All physical entities, including
    // bodies, sensors and potential splitters, belong to the Rust world.
    this.ids = [...world.entities.keys()].filter(id => world.hasComponent(id, ecs.PositionComponent)
      || world.hasComponent(id, ecs.OrientationComponent) || world.hasComponent(id, CableLinkComponent));
    this.index = new Map(this.ids.map((id, i) => [id, i]));
    const bodies = new Float64Array(this.ids.length * E);
    const get = (id, Type) => world.getComponent(id, Type);
    const machines = new Map();
    this.entityObjects = this.ids.map((id, i) => {
      const a = bodies.subarray(i * E, (i + 1) * E);
      const types = { position: ecs.PositionComponent, orientation: ecs.OrientationComponent,
        prevPosition: ecs.PrevFinalPosComponent, prevOrientation: ecs.PrevFinalOrientationComponent,
        velocity: ecs.VelocityComponent, angular: ecs.AngularVelocityComponent, mass: ecs.MassComponent,
        inertia: ecs.MomentOfInertiaComponent, encoder: ecs.EncoderComponent, member: ecs.RigidBodyMemberComponent,
        body: ecs.RigidBodyComponent, radius: ecs.RadiusComponent, friction: ecs.CoefficientOfFrictionComponent,
        link: CableLinkComponent, spool: SpoolStateComponent, motor: StepperMotorComponent };
      const c = Object.fromEntries(Object.entries(types).map(([key, Type]) => [key, get(id, Type)]));
      putV(a, 0, c.position?.pos); putQ(a, 3, c.orientation?.quaternion);
      putV(a, 7, c.prevPosition?.pos); putQ(a, 10, c.prevOrientation?.quaternion);
      putV(a, 14, c.member?.localPosition); putQ(a, 17, c.member?.localOrientation);
      for (let r = 0; r < 3; r++) for (let t = 0; t < 3; t++) {
        a[25 + r * 3 + t] = c.inertia?.inertiaTensor?.[r]?.[t] ?? 0;
        a[34 + r * 3 + t] = c.inertia?.invInertiaTensor?.[r]?.[t] ?? 0;
      }
      a[43] = c.mass?.mass > 0 ? 1 / c.mass.mass : 0;
      putV(a, 44, c.link?.cablePlaneNormalLocal);
      a[49] = (c.member?.localPosition ? 1 : 0) | (c.member?.localOrientation ? 2 : 0);
      a[50] = c.member ? this.index.get(c.member.bodyEntity) : -1;
      if (!Number.isFinite(a[50])) throw new Error(`Missing rigid body for entity ${id}`);
      a[51] = +Boolean(c.position); a[52] = +Boolean(c.orientation);
      a[53] = +Boolean(c.prevPosition); a[54] = +Boolean(c.prevOrientation);
      a[55] = +Boolean(c.inertia && c.orientation && !(c.spool && c.link?.cablePlaneNormalLocal));
      a[56] = +Boolean(c.link?.cablePlaneNormalLocal && c.inertia?.invInertiaTensor.flat().some(x => Math.abs(x) > 1e-12));
      a[57] = +Boolean(c.motor?.torqueMode); a[66] = c.encoder?.angle ?? 0; a[67] = +Boolean(c.encoder);
      putV(a, 69, c.velocity?.vel); putV(a, 72, c.angular?.omega);
      a[75] = (c.velocity ? 1 : 0) | (c.angular ? 2 : 0) | (get(id, ecs.GravityAffectedComponent) ? 4 : 0)
        | (c.spool ? 8 : 0) | (c.motor ? 16 : 0) | (c.body ? 32 : 0) | (c.inertia ? 64 : 0)
        | (c.mass ? 128 : 0) | (c.link ? 256 : 0) | (c.link?.cablePlaneNormalLocal ? 512 : 0) | (c.radius ? 1024 : 0);
      putV(a, 76, c.spool?.axisLocal); putQ(a, 79, c.spool?.referenceOrientation);
      putV(a, 83, c.body?.syncedPosition); putQ(a, 86, c.body?.syncedOrientation);
      putV(a, 90, c.link?.prevCableAttachmentTimePos); putQ(a, 93, c.link?.prevCableAttachmentTimeOrientation);
      putQ(a, 97, c.link?.prevCableAttachmentTimeLocalOrientation); putV(a, 101, c.link?.cablePlaneNormal);
      a[104] = c.radius?.radius ?? NaN; a[105] = c.friction?.mu ?? 0; a[106] = c.member?.physicalMass ?? NaN;
      const m = c.motor;
      a.set([m?.commandedAngle ?? 0, m?.deltaAngle ?? 0, m?.holdingTorque ?? 0,
        m?.numPolePairs ?? 50, m?.dampingCoeff ?? 0, m?.maxSpeedRad ?? 600, +Boolean(m?.closedLoop),
        m?.targetTorque ?? 0, m?.windageCoeff ?? NaN, m?.coulombFriction ?? NaN, m?.stictionTorque ?? NaN,
        m?.stictionSpeed ?? NaN, m?.coggingTorque ?? NaN, m?.coggingFreq ?? NaN], 107);
      putV(a, 121, c.encoder?.axis); a[124] = c.mass?.mass ?? 0;
      a[125] = m?.missedSteps ?? 0; a[126] = m?.currentMissedSteps ?? 0; a[127] = m?.missedStepEncoderOffset ?? NaN;
      a[133] = +Boolean(get(id, ecs.HybridKnotAngleComponent));
      a[135] = Object.keys(get(id, ecs.HybridKnotAngleComponent)?.pathAngles ?? {}).filter(k => k !== '__default__').length;
      a[128] = id;
      const machine = get(id, ecs.MachineTagComponent)?.id ?? null;
      if (!machines.has(machine)) machines.set(machine, machines.size);
      a[129] = machines.get(machine);
      a[131] = c.member ? (get(c.member.bodyEntity, ecs.RigidBodyComponent)?.members.indexOf(id) ?? -1) : 0;
      return c;
    });
    this.pathIds = world.query([CablePathComponent]);
    const paths = [];
    for (const id of this.pathIds) {
      const c = get(id, CablePathComponent), n = c.jointEntities.length;
      const data = new Float64Array(H + (n + 1) * L + n * J);
      const machine = get(id, ecs.MachineTagComponent)?.id ?? null;
      if (!machines.has(machine)) machines.set(machine, machines.size);
      data.set([n, id, c.spring_constant, c.compliance, c.damping, c.solverIterations, c.cableHalfWidth, c.totalRestLength, machines.get(machine)]);
      for (let li = 0; li <= n; li++) {
        const j = n ? get(c.jointEntities[li === n ? n - 1 : li], CableJointComponent) : null;
        const entity = j ? (li === n ? j.entityB : j.entityA) : null;
        const knot = get(entity, ecs.HybridKnotAngleComponent);
        const copyDefault = knot && !Number.isFinite(knot.pathAngles?.[id])
          && Number.isFinite(knot.pathAngles?.__default__)
          && Object.keys(knot.pathAngles).every(k => k === '__default__');
        data.set([kinds.indexOf(c.linkTypes[li]), +Boolean(c.cw[li]), c.stored[li],
          knot?.pathAngles?.[id] ?? (copyDefault ? knot.pathAngles.__default__ : NaN), copyDefault ? 2 : 0], H + li * L);
      }
      for (let i = 0; i < n; i++) {
        const jointId = c.jointEntities[i], j = get(jointId, CableJointComponent), o = H + (n + 1) * L + i * J;
        if (!this.index.has(j.entityA) || !this.index.has(j.entityB)) throw new Error(`Missing cable endpoint for joint ${jointId}`);
        data.set([jointId, this.index.get(j.entityA), this.index.get(j.entityB), j.restLength], o);
        putV(data, o + 4, j.attachmentPointA_world); putV(data, o + 7, j.attachmentPointB_world);
      }
      paths.push(data.length, ...data);
    }
    this.transport.create(bodies, new Float64Array(paths), this.pathIds.length);
    this.entityRegion = this.region(this.transport.entities(), () => this.transport.entities());
    this.optionsRegion = this.region(this.transport.options(), () => this.transport.options());
    this.bindingRegion = this.entityRegion;
    this.bindEntities(); this.syncPaths(world);
    this.topologyVersion = 0;
    this.revision = world.structureVersion; this.dirty = false;
  }
  bindEntities() {
    for (const [i, c] of this.entityObjects.entries()) {
      const o = i * E;
      for (const [key, field, offset] of [['position', 'pos', 0], ['orientation', 'quaternion', 3],
        ['prevPosition', 'pos', 7], ['prevOrientation', 'quaternion', 10], ['member', 'localPosition', 14],
        ['member', 'localOrientation', 17], ['velocity', 'vel', 69], ['angular', 'omega', 72],
        ['spool', 'axisLocal', 76], ['spool', 'referenceOrientation', 79], ['body', 'syncedPosition', 83],
        ['body', 'syncedOrientation', 86], ['link', 'prevCableAttachmentTimePos', 90],
        ['link', 'prevCableAttachmentTimeOrientation', 93], ['link', 'prevCableAttachmentTimeLocalOrientation', 97],
        ['link', 'cablePlaneNormal', 101], ['link', 'cablePlaneNormalLocal', 44], ['encoder', 'axis', 121]]) {
        if (c[key]) this.vector(c[key], field, o + offset);
      }
      for (const [key, field, offset] of [['encoder', 'angle', 66], ['radius', 'radius', 104], ['friction', 'mu', 105],
        ['member', 'physicalMass', 106], ['mass', 'mass', 124]]) if (c[key]) this.bind(c[key], field, o + offset, null,
          field === 'physicalMass' ? x => Number.isNaN(x) ? null : x : x => x, x => x ?? NaN);
      if (c.inertia) for (let r = 0; r < 3; r++) for (let col = 0; col < 3; col++) {
        this.bind(c.inertia.inertiaTensor[r], col, o + 25 + r * 3 + col);
        this.bind(c.inertia.invInertiaTensor[r], col, o + 34 + r * 3 + col);
      }
      if (c.motor) {
        const fields = ['commandedAngle', 'deltaAngle', 'holdingTorque', 'numPolePairs', 'dampingCoeff', 'maxSpeedRad',
          'closedLoop', 'targetTorque', 'windageCoeff', 'coulombFriction', 'stictionTorque', 'stictionSpeed', 'coggingTorque', 'coggingFreq'];
        fields.forEach((key, j) => this.bind(c.motor, key, o + 107 + j, null,
          key === 'closedLoop' ? Boolean : j >= 8 ? x => Number.isNaN(x) ? undefined : x : x => x,
          key === 'closedLoop' ? Number : j >= 8 ? x => x ?? NaN : x => x));
        this.bind(c.motor, 'torqueMode', o + 57, null, Boolean, Number);
        this.bind(c.motor, 'missedSteps', o + 125); this.bind(c.motor, 'currentMissedSteps', o + 126);
        this.bind(c.motor, 'missedStepEncoderOffset', o + 127, null, x => Number.isNaN(x) ? null : x, x => x ?? NaN);
      }
    }
  }
  syncPaths(world) {
    this.regions = [this.entityRegion, this.optionsRegion];
    this.pathShapes = [];
    for (const [p, id] of this.pathIds.entries()) {
      const group = `path:${id}`;
      this.unbind(group, false); this.refreshMemory();
      const region = this.region(this.transport.path(p), () => this.transport.path(p));
      this.bindingRegion = region;
      const data = region.data, o = 0, n = data[o];
      const path = world.getComponent(id, CablePathComponent), oldJoints = path.jointEntities;
      const newJoints = Array.from({ length: n }, (_, i) => data[o + H + (n + 1) * L + i * J]);
      for (const removed of oldJoints) if (!newJoints.includes(removed)) world.destroyEntity(removed);
      path.jointEntities = newJoints;
      path.linkTypes = Array.from({ length: n + 1 }, (_, i) => kinds[data[o + H + i * L]]);
      path.cw = Array.from({ length: n + 1 }, (_, i) => Boolean(data[o + H + i * L + 1]));
      path.stored = Array.from({ length: n + 1 }, (_, i) => data[o + H + i * L + 2]);
      this.pathShapes.push({ path, arrays: [path.jointEntities, path.linkTypes, path.cw, path.stored], count: n });
      for (const [i, key] of ['spring_constant', 'compliance', 'damping', 'solverIterations', 'cableHalfWidth', 'totalRestLength'].entries()) this.bind(path, key, o + i + 2, group);
      for (let li = 0; li <= n; li++) {
        this.bind(path.linkTypes, li, o + H + li * L, group, x => kinds[x], x => kinds.indexOf(x));
        this.bind(path.cw, li, o + H + li * L + 1, group, Boolean, Number);
        this.bind(path.stored, li, o + H + li * L + 2, group);
      }
      for (let i = 0; i < n; i++) {
        const jo = o + H + (n + 1) * L + i * J, jointId = newJoints[i];
        if (!world.entities.has(jointId)) {
          world.entities.set(jointId, new Set()); world.nextEntityId = Math.max(world.nextEntityId, jointId + 1);
          world.addComponent(jointId, new CableJointComponent(this.ids[data[jo + 1]], this.ids[data[jo + 2]],
            data[jo + 3], new ecs.PositionComponent(...data.slice(jo + 4, jo + 7)).pos,
            new ecs.PositionComponent(...data.slice(jo + 7, jo + 10)).pos));
          const machine = world.getComponent(id, ecs.MachineTagComponent);
          if (machine) world.addComponent(jointId, new ecs.MachineTagComponent(machine.id));
        }
        const joint = world.getComponent(jointId, CableJointComponent);
        for (const [side, key] of ['entityA', 'entityB'].entries()) this.bind(joint, key, jo + 1 + side, group, x => this.ids[x], x => {
          if (!this.index.has(x)) { this.invalidate(); joint[key] = x; return NaN; }
          return this.index.get(x);
        });
        this.bind(joint, 'restLength', jo + 3, group); this.bind(joint, 'constraintLambda', jo + 10, group);
        this.bind(joint, 'constraintForceMagnitude', jo + 11, group); this.bind(joint, 'transferredConstraintForceMagnitude', jo + 12, group);
        // Recreate detached vectors after a topology change invalidates offsets.
        joint.attachmentPointA_world = new ecs.PositionComponent(...data.slice(jo + 4, jo + 7)).pos;
        joint.attachmentPointB_world = new ecs.PositionComponent(...data.slice(jo + 7, jo + 10)).pos;
        joint.constraintForce = new ecs.PositionComponent(...data.slice(jo + 13, jo + 16)).pos;
        this.vector(joint, 'attachmentPointA_world', jo + 4, group); this.vector(joint, 'attachmentPointB_world', jo + 7, group);
        this.vector(joint, 'constraintForce', jo + 13, group);
      }
      for (const li of [0, n]) {
        if (!n || !Number.isFinite(data[o + H + li * L + 3]) || data[o + H + li * L + 4] === 2) continue;
        const joint = world.getComponent(newJoints[li === 0 ? 0 : n - 1], CableJointComponent);
        const entity = li === 0 ? joint.entityA : joint.entityB;
        let knot = world.getComponent(entity, ecs.HybridKnotAngleComponent);
        if (!knot) { knot = new ecs.HybridKnotAngleComponent(data[o + H + li * L + 3]); world.addComponent(entity, knot); }
        if (data[o + H + li * L + 4]) knot.angle = data[o + H + li * L + 3];
        data[o + H + li * L + 4] = 0;
        this.bind(knot.pathAngles, id, o + H + li * L + 3, group);
      }
    }
  }
  stepPhysics(world, dt) {
    let shapeChanged = false;
    for (const shape of this.pathShapes ?? []) {
      const { path, arrays, count } = shape;
      if (path.jointEntities !== arrays[0] || path.jointEntities.length !== count
          || path.linkTypes !== arrays[1] || path.linkTypes.length !== count + 1
          || path.cw !== arrays[2] || path.cw.length !== count + 1
          || path.stored !== arrays[3] || path.stored.length !== count + 1) { shapeChanged = true; break; }
    }
    if (this.world !== world || this.revision !== world.structureVersion || this.dirty || shapeChanged) this.prepare(world);
    this.refreshMemory();
    const data = this.optionsRegion.data, o = 0;
    const flag = (key, fallback) => typeof world.getResource(key) === 'boolean' ? world.getResource(key) : fallback;
    const finite = (key, fallback) => Number.isFinite(world.getResource(key)) ? world.getResource(key) : fallback;
    putV(data, o, world.getResource('gravity') ?? { x: 0, y: 0, z: 0 });
    data.set([+flag('enableLayering', true), +flag('layeringAttachmentUpdatePoints', true),
      +flag('layeringClampJointRestLength', true), +flag('layeringMergeJoints', false), +flag('layeringSplitJoints', false),
      +flag('layeringHybridLinkStates', true), world.getResource('grabbedBall') ?? -1, world.nextEntityId,
      finite('closedLoopStepperConstraintStiffness', 1e8), 0 /* reserved ABI slot; open-loop torque uses physical inertia */,
      finite('openLoopStepperHoldingTorqueScale', 1), +flag('closedLoopMotorsEnabled', false)], o + 3);
    putV(data, o + 16, world.getResource('defaultPlaneNormal') ?? { x: 0, y: 0, z: 1 });
    data[o + 15] = world.getResource('dt') ?? -1;
    const topology = this.transport.step(dt);
    this.observationVersion = (this.observationVersion ?? 0) + 1;
    this.refreshMemory();
    if (topology !== this.topologyVersion) {
      this.syncPaths(world); this.topologyVersion = topology; this.revision = world.structureVersion;
      world.nextEntityId = Math.max(world.nextEntityId, this.optionsRegion.data[o + 10]);
    }
    world.setResource('cableHybridTransitionStep', (world.getResource('cableHybridTransitionStep') ?? 0) + 1);
    // Maps are observations, assembled lazily when explicitly requested.
    for (const [key, slot, mask] of [['torqueModeCableLoadTorques', 59, 1], ['torqueModeCableLoadStiffnesses', 60, 2], ['torqueModeCableLoadDampings', 58, 4]]) {
      if (!Object.getOwnPropertyDescriptor(world.resources, key)?.get) {
        let version = -1, map;
        Object.defineProperty(world.resources, key, {
          configurable: true, enumerable: true, get: () => {
            if (version !== this.observationVersion) {
              this.refreshMemory(); map = new Map(this.ids.flatMap((id, i) => {
                const e = i * E, entities = this.entityRegion.data; return (entities[e + 68] & mask) ? [[id, entities[e + slot]]] : [];
              })); version = this.observationVersion;
            }
            return map;
          }, set: value => Object.defineProperty(world.resources, key, { configurable: true, enumerable: true, writable: true, value }),
        });
      }
    }
  }
}

export class RustPhysicsSystem {
  runInPause = false;
  constructor(backend) { this.backend = backend; }
  update(world, dt) { this.backend.stepPhysics(world, dt); }
}
