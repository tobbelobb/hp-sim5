// Finding 2: rebuilding a used world must reproduce a newly constructed world.
//
// Reset in the app is "world.clear() then apply the scene plan again"
// (setupScene with append=false). This script performs that same rebuild on the
// production headless pipeline and compares the full entity state against a
// fresh world, both immediately after the rebuild and after 500 more steps.
// It also covers backend reuse across scenes and a layering reconfigure that
// re-bakes the plan (as refreshBakedStage does), for both hosts.
//
// Exit code 1 when any entity field differs.
import { state, firstExactDifference, sceneText, makeWorld, makeBackend, loadWasmModule, planFor, rebuild } from './lib.mjs';
import { setSolverIterations } from '../../../../hp-sim-3d/app/solver-iterations.js';
import { setLineLayeringFeatureFlags } from '../../../../hp-sim-3d/app/line-layering-flags.js';

const STEPS = 500;
const ENTITY_ONLY = list => list.filter(([key]) => !key.startsWith('r.'));
// Counters that intentionally survive a scene reload in the app.
const LIFECYCLE_RESOURCES = new Set(['r.sceneGeneration', 'r.cableHybridTransitionStep']);
const module = loadWasmModule();

function assertSameEntities(label, left, right) {
  const diff = firstExactDifference(ENTITY_ONLY(left), ENTITY_ONLY(right));
  if (diff) {
    console.error(`FAIL ${label}`);
    console.error(JSON.stringify(diff, null, 2));
    process.exit(1);
  }
  console.log(`ok   ${label} (${ENTITY_ONLY(left).length} entity fields exactly equal)`);
}

function resourceDiff(a, b) {
  const left = new Map(a.filter(([key]) => key.startsWith('r.') && !LIFECYCLE_RESOURCES.has(key)));
  const right = new Map(b.filter(([key]) => key.startsWith('r.') && !LIFECYCLE_RESOURCES.has(key)));
  const diffs = [];
  for (const [key, value] of left) {
    if (!right.has(key)) diffs.push({ key, kind: 'missing-in-rebuilt', value });
    else if (right.get(key) !== value) diffs.push({ key, fresh: value, rebuilt: right.get(key) });
  }
  for (const [key, value] of right) if (!left.has(key)) diffs.push({ key, kind: 'extra-in-rebuilt', value });
  return diffs;
}

const hp3 = sceneText('hp3');
const hp4 = sceneText('hp4');
const hp3Layered = planFor(hp3);
const hp3Unlayered = planFor(hp3, { lineLayeringEnabled: false });

function run(kind) {
  const make = (scene, options = {}) => makeWorld(scene, { ...options, cableSolverBackend: makeBackend(kind, module) });
  const step = (instance, n) => { for (let i = 0; i < n; i++) instance.world.update(instance.dt); };

  // ---- Reference trajectory: two fresh worlds ------------------------------
  const fresh = make(hp3);
  step(fresh, STEPS);
  const reference = state(fresh.world);
  const freshTwin = make(hp3);
  step(freshTwin, STEPS);
  assertSameEntities(`${kind}: two fresh HP3 worlds agree after 500 steps`, reference, state(freshTwin.world));

  // ---- Rebuild a used world and compare immediately ------------------------
  const freshUnstepped = make(hp3);
  const used = make(hp3);
  step(used, 137);
  rebuild(used.world, hp3Layered);
  assertSameEntities(`${kind}: rebuilt world equals a fresh unstepped world`,
    state(used.world), state(freshUnstepped.world));

  // ---- Rebuild then run the full trajectory --------------------------------
  step(used, STEPS);
  assertSameEntities(`${kind}: rebuilt world reproduces the fresh 500-step trajectory`, state(used.world), reference);
  const diffs = resourceDiff(reference, state(used.world));
  if (diffs.length) console.log(`info ${kind}: resource differences after rebuild: ${JSON.stringify(diffs)}`);

  // ---- Rebuild repeatedly: no monotone accumulation ------------------------
  for (let cycle = 2; cycle <= 3; cycle++) {
    step(used, 61 * cycle);
    rebuild(used.world, hp3Layered);
    step(used, STEPS);
    assertSameEntities(`${kind}: rebuild cycle ${cycle} reproduces the fresh trajectory`, state(used.world), reference);
  }

  // ---- Reusing one backend object across different scenes ------------------
  const old = make(hp4);
  step(old, 50);
  const reused = make(hp3, { cableSolverBackend: old.world.getResource('cableSolverBackend') });
  step(reused, STEPS);
  assertSameEntities(`${kind}: a backend reused after an HP4 world reproduces a fresh HP3 world`,
    state(reused.world), reference);

  // ---- Layering reconfigure with a re-baked plan ---------------------------
  const layeredFresh = make(hp3);
  step(layeredFresh, STEPS);
  const layeredReference = state(layeredFresh.world);
  // The app sets the flag first (featureFlagsController.setLineLayeringEnabledState)
  // and then rebuilds (onLineLayeringChanged -> rebuildScene -> refreshBakedStage).
  const converted = make(hp3, { lineLayeringEnabled: false });
  step(converted, 200);
  setLineLayeringFeatureFlags(converted.world, true);
  converted.world.getResource('cableSolverBackend')?.invalidate?.();
  rebuild(converted.world, hp3Layered);
  step(converted, STEPS);
  assertSameEntities(`${kind}: unlayered-then-rebaked-layered world reproduces a fresh layered world`,
    state(converted.world), layeredReference);

  const unlayeredFresh = make(hp3, { lineLayeringEnabled: false });
  step(unlayeredFresh, STEPS);
  const unlayeredReference = state(unlayeredFresh.world);
  const back = make(hp3);
  step(back, 200);
  setLineLayeringFeatureFlags(back.world, false);
  back.world.getResource('cableSolverBackend')?.invalidate?.();
  rebuild(back.world, hp3Unlayered);
  step(back, STEPS);
  assertSameEntities(`${kind}: layered-then-rebaked-unlayered world reproduces a fresh unlayered world`,
    state(back.world), unlayeredReference);

  // ---- Flag edits that are pure resources must be reversible --------------
  const edited = make(hp3);
  const control = make(hp3);
  setSolverIterations(edited.world, 8);
  setLineLayeringFeatureFlags(edited.world, false);
  setSolverIterations(edited.world, null);
  setLineLayeringFeatureFlags(edited.world, true);
  assertSameEntities(`${kind}: solver-iteration and layering flag cycling leaves initial state untouched`,
    state(edited.world), state(control.world));
  step(edited, 200);
  step(control, 200);
  assertSameEntities(`${kind}: solver-iteration and layering flag cycling leaves the trajectory untouched`,
    state(edited.world), state(control.world));

  for (const w of [fresh, freshTwin, freshUnstepped, used, old, reused, layeredFresh, converted,
    unlayeredFresh, back, edited, control]) w.world.getResource('cableSolverBackend')?.dispose?.();
}

const t0 = performance.now();
run('native');
run('wasm');
console.log(`PASS reset/rebuild (native and wasm): ${STEPS}-step trajectories, 3 rebuild cycles, shared backend, layering reconfigure, flag cycling (${(performance.now() - t0).toFixed(0)} ms)`);
