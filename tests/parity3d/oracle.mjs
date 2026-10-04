// Executable oracle: import production JS systems, never duplicate physics here.
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as ecs from '../../src/js/cable_joints_3d/ecs.js';
import * as commonSystems from '../../src/js/cable_joints_3d/commonSystems.js';
import { CableAttachmentCacheSystem } from '../../src/js/cable_joints_3d/cable_attachment_cache_system.js';
import { CableFrictionSystem } from '../../src/js/cable_joints_3d/cable_friction_system.js';
import { PBDResolveCableOverCorrections } from '../../src/js/cable_joints_3d/pbdResolveCableOverCorrections.js';
import * as rigid from '../../src/js/cable_joints_3d/rigid_bodies.js';
import * as spools from '../../hp-sim-3d/app/hangprinter_spools.js';
import { StepperMotorComponent, StepperMotorSystem } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { TorqueModeSystem } from '../../hp-sim-3d/app/torqueModeSystem.js';
import { MissedStepTrackingSystem, getMachineMotorDiagnostics, resetMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { ExtruderComponent, ExtruderSystem, estimateEffectorRotation } from '../../hp-sim-3d/app/hangprinter_extruder.js';
import { RemoteSpoolSystem } from '../../hp-sim-3d/app/remoteSpoolSystem.js';
import * as geometry from '../../src/js/cable_joints_3d/geometry3.js';
import * as cable from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { createCablePaths } from '../../src/js/cable_joints_3d/createCablePaths.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import { bakeCableSceneUsdaSource } from '../../src/js/usd/cable_scene_baker.js';
import { OpenText } from '../../src/js/usd/stage.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { registerSimulationSystems } from '../../hp-sim-3d/app/simulationSystems.js';

const contract = JSON.parse(fs.readFileSync(new URL('./contract.json', import.meta.url)));
const geometryContract = JSON.parse(fs.readFileSync(new URL('./geometry_contract.json', import.meta.url)));
const components = { ...ecs, ...spools, ...cable, StepperMotorComponent, ExtruderComponent };
const systems = { ...commonSystems, CableAttachmentCacheSystem, CableFrictionSystem, PBDResolveCableOverCorrections, StepperMotorSystem, TorqueModeSystem, MissedStepTrackingSystem, ExtruderSystem, RemoteSpoolSystem,
  CableAttachmentUpdateSystem: cable.CableAttachmentUpdateSystem,
  PBDCableConstraintSolver: cable.PBDCableConstraintSolver };
const vector = (value) => value == null ? null : new Vector3(...value);
const quaternion = (value) => value == null ? null : new Quaternion(...value);

export function runFixture(fixture) {
  fixture = structuredClone(fixture);
  const world = new ecs.World();
  const ids = Object.fromEntries(fixture.entities.map(e => [e.name, world.createEntity()]));
  for (const definition of fixture.scenes ?? []) {
    const source = definition.source ?? fs.readFileSync(new URL('../../' + definition.path, import.meta.url), 'utf8');
    const stage = OpenText(bakeCableSceneUsdaSource(source, definition.bakeOptions ?? {}).source);
    const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), definition.scenePrimPath, definition.options));
    if (!checked.valid) throw new Error(checked.warnings.join('\n'));
    applyEntityPlan(world, buildEntityPlan(checked, definition.options ?? {}));
  }
  if (fixture.scenes) {
    for (const id of world.entities.keys()) {
      const info = world.getComponent(id, ecs.SceneEntityInfoComponent);
      const machine = world.getComponent(id, ecs.MachineTagComponent)?.id;
      const name = info ? `${machine}::${info.name}` : `@${id}`;
      if (name in ids) throw new Error(`Duplicate scene entity name ${name}`);
      ids[name] = id;
    }
  }
  const names = Object.fromEntries(Object.entries(ids).map(([name, id]) => [id, name]));
  const decode = (value, kind) => {
    if (value == null) return null;
    if (kind === 'vector') return vector(value);
    if (kind === 'quaternion') return quaternion(value);
    if (kind === 'entity') return value == null ? null : ids[value];
    if (kind === 'entities') return value.map(name => ids[name]);
    if (kind === 'vectors') return value.map(point => vector(point));
    if (kind.endsWith('Map') && kind !== 'entityMap') return Object.fromEntries(Object.entries(value).map(
      ([key, item]) => [key, decode(item, kind.slice(0, -3))]));
    if (kind === 'entityMap') return Object.fromEntries(Object.entries(value).map(
      ([name, angle]) => [name === '__default__' ? name : ids[name], angle]));
    if (kind === 'parameter' && value === 'Infinity') return Infinity;
    return structuredClone(value);
  };
  function add(entity, name, args) {
    const Type = components[name];
    if (!Type || !contract[name]) throw new Error(`Unsupported component ${name}`);
    let component;
    if (name === 'RigidBodyComponent') {
      component = new Type(args[0].map(name => ids[name]));
    } else if (name === 'RigidBodyMemberComponent') {
      component = new Type(ids[args[0]], vector(args[1]), quaternion(args[2]), args[3]);
    } else if (name === 'DistanceConstraintComponent') {
      component = new Type(ids[args[0]], ids[args[1]], ...args.slice(2));
    } else if (name === 'SpoolStateComponent') {
      component = new Type(args[0], vector(args[1]), quaternion(args[2]));
    } else if (name === 'MomentOfInertiaComponent') {
      component = new Type(args[0], { axisLocal: vector(args[1]) ?? undefined });
    } else if (name === 'CableLinkComponent') {
      component = new Type(...args.slice(0, 3), quaternion(args[3]), vector(args[4]), vector(args[5]));
    } else if (name === 'CableJointComponent') {
      const values = [ids[args[0]], ids[args[1]], args[2], vector(args[3]), vector(args[4])];
      component = args[5] === 'local' ? Type.fromLocal(world, ...values) : Type.fromWorld(...values);
    } else if (name === 'CablePathComponent') {
      const values = args.slice(1);
      values[2] = decode(values[2], 'parameter');
      component = new Type(world, args[0].map(name => ids[name]), ...values);
    } else {
      component = new Type(...args);
    }
    world.addComponent(ids[entity], component);
  }
  function resources(values = {}, entityValues = {}, mapValues = {}) {
    for (const [key, value] of Object.entries(values)) {
      world.setResource(key, ['gravity', 'defaultPlaneNormal'].includes(key) ? vector(value)
        : key === 'grabbedBall' && value != null ? ids[value] : value);
    }
    for (const [key, definition] of Object.entries(entityValues)) {
      const entries = Object.entries(definition.values).map(([name, value]) => [ids[name], value]);
      world.setResource(key, definition.kind === 'object' ? Object.fromEntries(entries) : new Map(entries));
    }
    for (const [key, values] of Object.entries(mapValues)) world.setResource(key, new Map(Object.entries(values)));
  }
  function mutate(values = []) {
    for (const [entity, typeName, field, value] of values) {
      const definition = contract[typeName].find(([jsField]) => jsField === field);
      const component = world.getComponent(ids[entity], components[typeName]);
      const decoded = decode(value, definition[2]);
      if (['vector', 'quaternion'].includes(definition[2]) && component[field] != null && decoded != null) component[field].set(decoded);
      else component[field] = decoded;
    }
  }
  resources(fixture.resources, fixture.entityResources, fixture.mapResources);
  for (const entity of fixture.entities) {
    for (const [name, args] of Object.entries(entity.components)) add(entity.name, name, args);
  }
  for (const addition of fixture.addComponents ?? []) add(...addition);
  for (const { names: pathNames, args } of fixture.createPaths ?? []) {
    const values = args.slice(1);
    values[2] = decode(values[2], 'parameter');
    const created = createCablePaths(world, args[0].map(name => ids[name]), ...values);
    if (created.length !== pathNames.length) throw new Error('Unexpected number of split paths');
    created.forEach((id, i) => { ids[pathNames[i]] = id; names[id] = pathNames[i]; });
  }
  mutate(fixture.initialSet);
  for (const name of fixture.initializeRigidBodies ?? []) rigid.initializeRigidBodySyncState(world, ids[name]);
  for (const definition of fixture.systems) {
    const name = typeof definition === 'string' ? definition : definition.name;
    const args = typeof definition === 'string' ? [] : definition.args ?? [];
    if (!systems[name]) throw new Error(`Unsupported system ${name}`);
    world.registerSystem(new systems[name](...args));
  }
  if (fixture.pipeline) {
    if (fixture.systems.length) throw new Error('Pipeline fixtures must use production system registration');
    registerSimulationSystems(world);
    world.systems.find(system => system instanceof ExtruderSystem).update(world, 0);
  }
  const remote = world.systems.find(system => system instanceof RemoteSpoolSystem);
  if (fixture.initializeExtruder) world.systems.find(system => system instanceof ExtruderSystem).update(world, 0);
  const events = [];
  if (fixture.commands) remote.commands = fixture.commands;
  if (fixture.observeCommands) {
    remote.setCommandExecutedListener(value => events.push({ kind: 'command', value: structuredClone(value) }));
    remote.setExtrusionListener(value => events.push({ kind: 'extrusion', value: structuredClone(value) }));
  }
  function commandActions(actions = []) {
    for (const action of actions) {
      if (action.method === 'processCommand') remote._processCommand(world, action.command,
        { recordHistory: action.recordHistory ?? true, emitEvents: action.emitEvents ?? true });
      else if (action.method === 'setCommands') remote.commands = action.commands;
      else if (action.method === 'addCommand') remote.addCommand(action.command);
      else if (action.method === 'setPlaybackState') remote.setPlaybackState(action.state);
      else if (['clearCommandQueue', 'clearPlaybackState', 'resetAxisMapping'].includes(action.method)) remote[action.method]();
      else throw new Error(`Unsupported command action ${action.method}`);
    }
  }
  function encode(value, kind) {
    if (value == null) return null;
    if (kind === 'entity') return names[value];
    if (kind === 'entities') return value.map(id => names[id]);
    if (kind === 'booleans') return value.map(Boolean);
    if (kind === 'vectors') return value.map(point => encode(point, 'vector'));
    if (kind.endsWith('Map') && kind !== 'entityMap') return Object.fromEntries(Object.entries(value).map(
      ([key, item]) => [key, encode(item, kind.slice(0, -3))]));
    if (kind === 'entityMap') return Object.fromEntries(Object.entries(value).map(
      ([id, angle]) => [id === '__default__' ? id : names[id], angle]));
    if (kind === 'vector') return [value.x, value.y, value.z];
    if (kind === 'quaternion') return [value.x, value.y, value.z, value.w];
    if (kind === 'parameter' && value === Infinity) return 'Infinity';
    return structuredClone(value);
  }
  function snapshot(step) {
    const motorDiagnostics = fixture.motorDiagnostics?.map(machine => getMachineMotorDiagnostics(world, machine));
    const entities = {};
    for (const [name, id] of Object.entries(ids)) {
      entities[name] = {};
      for (const [typeName, fields] of Object.entries(contract)) {
        const component = world.getComponent(id, components[typeName]);
        if (component) entities[name][typeName] = Object.fromEntries(fields.map(
          ([jsField, , kind]) => {
            if (!(jsField in component) && kind !== 'optionalNumber') throw new Error(`Missing ${typeName}.${jsField}`);
            return [jsField, encode(component[jsField], kind)];
          }));
        if (component && typeName === 'CableJointComponent') {
          entities[name][typeName].geometricLength = component.attachmentPointA_world.distanceTo(component.attachmentPointB_world);
        }
      }
    }
    const queries = (fixture.queries ?? []).map(types => world.query(types.map(t => components[t])).map(id => names[id]));
    const attachments = (fixture.attachments ?? []).map(p => {
      const point = rigid.computeWorldAttachment(world, ids[p.entity], vector(p.localPoint));
      const endpoint = rigid.resolveRigidBodySolverEndpoint(world, ids[p.entity], ids[p.counterpart], point);
      return { worldPoint: encode(point, 'vector'), localPoint: encode(rigid.computeLocalAttachment(world, ids[p.entity], point), 'vector'),
        solverEntity: encode(endpoint.entityId, 'entity'), solverLocalPoint: encode(endpoint.localPoint, 'vector'),
        internalToBody: Boolean(endpoint.internalToBody) };
    });
    const state = { step, entities, queries, attachments };
    if (fixture.pipeline) state.systemOrder = world.systems.map(system => system.constructor.name);
    if (motorDiagnostics) state.motorDiagnostics = motorDiagnostics;
    if (fixture.commandState) state.commandState = {
      ...structuredClone(remote.getPlaybackState()), queueLength: remote.getQueueLength(),
      axisToEntity: Object.fromEntries(Object.entries(remote.axisToEntity).map(([axis, value]) =>
        [axis, encode(value, Array.isArray(value) ? 'entities' : 'entity')])),
    };
    if (fixture.observeCommands) state.commandEvents = structuredClone(events);
    if (fixture.effectorRotations) state.effectorRotations = fixture.effectorRotations.map(({ extruder, machine }) => {
      const component = world.getComponent(ids[extruder], ExtruderComponent);
      return { quaternion: encode(estimateEffectorRotation(component.centerSourceOffsets[machine],
        component.machineEffectorCenters[machine], component.centerSources[machine], world), 'quaternion') };
    });
    if (fixture.snapshotResources) state.resources = Object.fromEntries(fixture.snapshotResources.map(
      key => [key, ['gravity', 'defaultPlaneNormal'].includes(key)
        ? encode(world.getResource(key), 'vector') : world.getResource(key) ?? null]));
    if (fixture.snapshotMapResources) state.mapResources = Object.fromEntries(fixture.snapshotMapResources.map(
      key => [key, Object.fromEntries(world.getResource(key) ?? [])]));
    if (fixture.snapshotEntityMaps) state.entityMaps = Object.fromEntries(fixture.snapshotEntityMaps.map(key => {
      const value = world.getResource(key);
      const entries = value instanceof Map ? [...value] : Object.entries(value ?? {});
      return [key, value == null ? null : Object.fromEntries(entries.map(([id, number]) => [names[id], number]))];
    }));
    if (fixture.cableRotations) state.cableRotations = fixture.cableRotations.map(probe =>
      cable.cableStoredLengthAfterRotation(world, world.getComponent(ids[probe.path], cable.CablePathComponent),
        probe.index, ids[probe.entity], probe.delta));
    return state;
  }
  const snapshots = [snapshot(0)];
  for (const [index, step] of fixture.steps.entries()) {
    resources(step.resources, step.entityResources, step.mapResources);
    mutate(step.set);
    for (const [entity, type] of step.removeComponents ?? []) world.removeComponent(ids[entity], components[type]);
    for (const machine of step.resetMotorDiagnostics ?? []) resetMachineMotorDiagnostics(world, machine);
    commandActions(step.commandActions);
    world.update(step.dt);
    snapshots.push(snapshot(index + 1));
  }
  const result = { schema: 1, snapshots };
  if (fixture.usdBake) {
    const source = fixture.usdBake.source ?? fs.readFileSync(new URL('../../' + fixture.usdBake.path, import.meta.url), 'utf8');
    const baked = bakeCableSceneUsdaSource(source, fixture.usdBake.options ?? {});
    result.usdBake = baked.resolvedPaths.map(path => ({ ...path,
      jointResults: path.jointResults.map(joint => ({ ...joint,
        ...Object.fromEntries(['world0', 'world1', 'local0', 'local1'].map(key =>
          [key, encode(joint[key], 'vector')])),
      })),
    }));
  }
  if (fixture.geometry) {
    const plain = value => {
      if (value instanceof Vector3) return [value.x, value.y, value.z];
      if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value).map(([key, item]) => [key, plain(item)]));
      return value;
    };
    result.geometry = fixture.geometry.map(({ method, args }) => {
      const [, kinds] = geometryContract[method];
      return plain(geometry[method](...args.map((arg, i) => decode(arg, kinds[i]))));
    });
  }
  return result;
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const fixture = JSON.parse(fs.readFileSync(process.argv[2] ?? 0, 'utf8'));
  const result = runFixture(fixture);
  // JSON.stringify converts NaN/Infinity to null; reject them before that happens.
  JSON.stringify(result, (key, value) => {
    if (typeof value === 'number' && !Number.isFinite(value)) throw new Error(`Nonfinite state at ${key}`);
    return value;
  });
  process.stdout.write(`${JSON.stringify(result)}\n`);
}
