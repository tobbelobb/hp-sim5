// Shared harness: full-world state serialization and numeric diffs.
// Every comparison in this session uses this serializer so "bit-identical"
// means the same thing in every script.
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { WasmPhysicsEngine } from '../../../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { NativePhysicsEngine } from '../../../../src/js/cable_joints_3d/native_physics_engine.js';
import { applyEntityPlan } from '../../../../hp-sim-3d/app/scene/entityPlanApplier.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan } from '../../../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { OpenText } from '../../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../../src/js/usd/cable_scene_baker.js';

export const repoURL = p => new URL('../../../../' + p, import.meta.url);
export const ROOT = new URL('../../../../', import.meta.url).pathname;

export const sceneText = name => fs.readFileSync(repoURL(`public/usd_scenes/${name}.usda`), 'utf8');

// A scene plan can be reused to rebuild into an already-used world, exactly as
// the app rebuilds a scene into a cleared world.
export function planFor(scene, { lineLayeringEnabled = true, scenePrimPath } = {}) {
  const baked = bakeCableSceneUsdaSource(scene, lineLayeringEnabled ? {} : { cablePathHalfWidthOverride: 0, deriveAll: true }).source;
  const sceneName = scene.match(/defaultPrim\s*=\s*"([^"]+)"/)?.[1];
  const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(OpenText(baked)), scenePrimPath ?? `/World/${sceneName}`));
  if (!checked.valid) throw new Error(checked.warnings.join('\n'));
  return buildEntityPlan(checked);
}

export function rebuild(world, plan) {
  world.clear();
  applyEntityPlan(world, plan);
}

export function loadWasmModule() {
  return new WebAssembly.Module(fs.readFileSync(repoURL('public/wasm/cable_solver.wasm')));
}

export function makeBackend(kind, module) {
  return kind === 'wasm' ? new WasmPhysicsEngine(module) : new NativePhysicsEngine();
}

export function makeWorld(scene, options = {}) {
  return createHeadlessWorld(scene, options);
}

// Fields that hold host objects rather than simulation state.
const RESOURCE_SKIP = new Set([
  'performanceMonitor', 'cableSolverBackend', 'physicsArtifact', 'renderSystem',
  'flightRecorder', 'debugRenderPoints', 'machineColors', 'researchClock',
]);

function scalar(value) {
  if (typeof value === 'number') return Number.isFinite(value) ? value : String(value);
  if (typeof value === 'boolean' || typeof value === 'string') return value;
  return undefined;
}

function flatten(value, path, out, depth = 0) {
  const primitive = scalar(value);
  if (primitive !== undefined) { out.push([path, primitive]); return; }
  if (value == null) { out.push([path, String(value)]); return; }
  if (depth > 4) return;
  if (Array.isArray(value)) {
    value.forEach((item, i) => flatten(item, `${path}[${i}]`, out, depth + 1));
    return;
  }
  if (value instanceof Map) {
    for (const key of [...value.keys()].sort((a, b) => String(a).localeCompare(String(b), undefined, { numeric: true }))) {
      flatten(value.get(key), `${path}.get(${key})`, out, depth + 1);
    }
    return;
  }
  if (value instanceof Set) {
    [...value].map(String).sort().forEach((item, i) => out.push([`${path}{${i}}`, item]));
    return;
  }
  if (typeof value === 'object') {
    for (const key of Object.keys(value).sort()) {
      if (typeof value[key] === 'function') continue;
      flatten(value[key], `${path}.${key}`, out, depth + 1);
    }
  }
}

// Full observable state: every component field of every entity, plus every
// primitive/scalar resource. Ordered deterministically so two worlds can be
// compared entry by entry.
export function state(world) {
  const out = [];
  const entities = [...world.entities.keys()].sort((a, b) => a - b);
  out.push(['world.nextEntityId', world.nextEntityId]);
  for (const id of entities) {
    const componentClasses = [...world.entities.get(id)]
      .map(Type => ({ Type, name: Type.name }))
      .sort((a, b) => a.name.localeCompare(b.name));
    for (const { Type, name } of componentClasses) {
      const component = world.getComponent(id, Type);
      if (!component) { out.push([`e${id}.${name}`, 'MISSING']); continue; }
      flatten(component, `e${id}.${name}`, out);
    }
  }
  for (const key of Object.keys(world.resources).sort()) {
    if (RESOURCE_SKIP.has(key)) continue;
    flatten(world.resources[key], `r.${key}`, out);
  }
  return out;
}

export function stateMap(list) {
  return new Map(list);
}

// Exact comparison. Returns null when identical, else the first difference.
export function firstExactDifference(a, b) {
  const max = Math.max(a.length, b.length);
  for (let i = 0; i < max; i++) {
    const [pa, va] = a[i] ?? ['<missing>', undefined];
    const [pb, vb] = b[i] ?? ['<missing>', undefined];
    if (pa !== pb) return { index: i, key: pa, left: va, right: vb, kind: 'key' };
    if (typeof va === 'number' && typeof vb === 'number') {
      if (va !== vb) return { index: i, key: pa, left: va, right: vb, kind: 'number', abs: Math.abs(va - vb) };
    } else if (va !== vb) return { index: i, key: pa, left: va, right: vb, kind: 'value' };
  }
  return null;
}

export function exactEqual(a, b) { return firstExactDifference(a, b) === null; }

// Numeric comparison over entries present in both states. Non-numeric and
// non-finite entries must match exactly (they are reported separately).
// Quaternion components are sign-sensitive here; callers that want sign-
// invariant orientation distances should compare positions/velocities.
export function numericDiff(a, b) {
  const left = stateMap(a), right = stateMap(b);
  let maxAbs = 0, maxKey = null, compared = 0, nonFinite = 0;
  const mismatchedKeys = [];
  for (const [key, va] of left) {
    if (!right.has(key)) { mismatchedKeys.push({ key, kind: 'missing-right', left: va }); continue; }
    const vb = right.get(key);
    if (typeof va === 'number' && typeof vb === 'number') {
      compared++;
      const d = Math.abs(va - vb);
      if (d > maxAbs) { maxAbs = d; maxKey = key; }
    } else if (va !== vb) mismatchedKeys.push({ key, kind: 'value', left: va, right: vb });
  }
  for (const key of right.keys()) if (!left.has(key)) mismatchedKeys.push({ key, kind: 'missing-left', right: right.get(key) });
  return { maxAbs, maxKey, compared, nonFinite, mismatchedKeys, equalKeys: compared + (left.size - compared) };
}


// Deterministic digest of the state, for cross-process determinism checks.
export function digest(list) {
  let h1 = 0x811c9dc5, h2 = 0x01000193;
  for (const [key, value] of list) {
    const text = `${key}=${typeof value === 'number' ? value.toExponential(17) : value};`;
    for (let i = 0; i < text.length; i++) {
      const c = text.charCodeAt(i);
      h1 = Math.imul(h1 ^ c, 0x01000193) >>> 0;
      h2 = Math.imul(h2 + c, 0x85ebca6b) >>> 0;
    }
  }
  return `${h1.toString(16).padStart(8, '0')}${h2.toString(16).padStart(8, '0')}`;
}
