// Executable oracle: import production JS systems, never duplicate physics here.
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as ecs from '../../src/js/cable_joints_3d/ecs.js';
import * as systems from '../../src/js/cable_joints_3d/commonSystems.js';
import * as rigid from '../../src/js/cable_joints_3d/rigid_bodies.js';
import * as spools from '../../hp-sim-3d/app/hangprinter_spools.js';
import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';

const contract = JSON.parse(fs.readFileSync(new URL('./contract.json', import.meta.url)));
const components = { ...ecs, ...spools };
const vector = (value) => value == null ? null : new Vector3(...value);
const quaternion = (value) => value == null ? null : new Quaternion(...value);

export function runFixture(fixture) {
  const world = new ecs.World();
  const ids = Object.fromEntries(fixture.entities.map(e => [e.name, world.createEntity()]));
  const names = Object.fromEntries(Object.entries(ids).map(([name, id]) => [id, name]));
  const decode = (value, kind) => {
    if (kind === 'vector') return vector(value);
    if (kind === 'quaternion') return quaternion(value);
    if (kind === 'entity') return value == null ? null : ids[value];
    if (kind === 'entities') return value.map(name => ids[name]);
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
    } else {
      component = new Type(...args);
    }
    world.addComponent(ids[entity], component);
  }
  function resources(values = {}) {
    for (const [key, value] of Object.entries(values)) {
      world.setResource(key, ['gravity', 'defaultPlaneNormal'].includes(key) ? vector(value)
        : key === 'grabbedBall' && value != null ? ids[value] : value);
    }
  }
  resources(fixture.resources);
  for (const entity of fixture.entities) {
    for (const [name, args] of Object.entries(entity.components)) add(entity.name, name, args);
  }
  for (const addition of fixture.addComponents ?? []) add(...addition);
  for (const name of fixture.initializeRigidBodies ?? []) rigid.initializeRigidBodySyncState(world, ids[name]);
  for (const name of fixture.systems) {
    if (!systems[name]) throw new Error(`Unsupported system ${name}`);
    world.registerSystem(new systems[name]());
  }
  function encode(value, kind) {
    if (value == null) return null;
    if (kind === 'entity') return names[value];
    if (kind === 'entities') return value.map(id => names[id]);
    if (kind === 'vector') return [value.x, value.y, value.z];
    if (kind === 'quaternion') return [value.x, value.y, value.z, value.w];
    return structuredClone(value);
  }
  function snapshot(step) {
    const entities = {};
    for (const [name, id] of Object.entries(ids)) {
      entities[name] = {};
      for (const [typeName, fields] of Object.entries(contract)) {
        const component = world.getComponent(id, components[typeName]);
        if (component) entities[name][typeName] = Object.fromEntries(fields.map(
          ([jsField, , kind]) => {
            if (!(jsField in component)) throw new Error(`Missing ${typeName}.${jsField}`);
            return [jsField, encode(component[jsField], kind)];
          }));
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
    return { step, entities, queries, attachments };
  }
  const snapshots = [snapshot(0)];
  for (const [index, step] of fixture.steps.entries()) {
    resources(step.resources);
    for (const [entity, typeName, field, value] of step.set ?? []) {
      const definition = contract[typeName].find(([jsField]) => jsField === field);
      const component = world.getComponent(ids[entity], components[typeName]);
      const decoded = decode(value, definition[2]);
      if (['vector', 'quaternion'].includes(definition[2])) component[field].set(decoded);
      else component[field] = decoded;
    }
    world.update(step.dt);
    snapshots.push(snapshot(index + 1));
  }
  return { schema: 1, snapshots };
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
