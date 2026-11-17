// Finding 1: two headless worlds in one process must be independent.
//
// The check is exact, not approximate: every component field of every entity
// plus every scalar resource is compared entry by entry. A world that was never
// stepped must be bit-identical to a freshly constructed world after another
// world in the same process has been stepped (including a topology change).
//
// Exit code 1 with a concrete first-difference message when cross-talk exists.
import { state, firstExactDifference, sceneText, makeWorld, loadWasmModule, makeBackend } from './lib.mjs';

const STEPS = 50;
const module = loadWasmModule();

function assertExact(label, left, right) {
  const diff = firstExactDifference(left, right);
  if (diff) {
    console.error(`FAIL ${label}`);
    console.error(JSON.stringify(diff, null, 2));
    process.exit(1);
  }
  console.log(`ok   ${label} (${left.length} entries exactly equal)`);
}

function run(kind) {
  const make = scene => makeWorld(scene, { cableSolverBackend: makeBackend(kind, module) });
  const step = (instance, n) => { for (let i = 0; i < n; i++) instance.world.update(instance.dt); };

  const hp3 = sceneText('hp3');
  const hp4 = sceneText('hp4');

  // ---- Case A: same scene text, three live worlds --------------------------
  const a = make(hp3);
  const b = make(hp3);
  const c = make(hp3);
  assertExact(`${kind}: A/B/C initial state identical`, state(a.world), state(b.world));
  assertExact(`${kind}: A/C initial state identical`, state(a.world), state(c.world));

  step(a, STEPS);
  assertExact(`${kind}: stepping A does not change untouched B`, state(b.world), state(c.world));

  step(b, STEPS);
  step(c, STEPS);
  assertExact(`${kind}: B and C remain identical after both step`, state(b.world), state(c.world));
  assertExact(`${kind}: A and B remain identical after the same steps`, state(a.world), state(b.world));

  // ---- Case A2: interleaved stepping of two live worlds --------------------
  const j = make(hp3);
  const k = make(hp3);
  for (let i = 0; i < STEPS; i++) { j.world.update(j.dt); k.world.update(k.dt); }
  const l = make(hp3);
  step(l, STEPS);
  assertExact(`${kind}: interleaved A/B stepping equals a solo run`, state(j.world), state(l.world));
  assertExact(`${kind}: interleaved worlds agree with each other`, state(k.world), state(l.world));

  // ---- Case B: different scene texts left alive together -------------------
  const d = make(hp4);
  const e = make(hp3);
  const f = make(hp3);
  step(d, STEPS);
  assertExact(`${kind}: stepping an HP4 world does not change HP3 E`, state(e.world), state(f.world));
  step(e, STEPS);
  step(f, STEPS);
  assertExact(`${kind}: HP3 E/F identical after HP4 ran in the same process`, state(e.world), state(f.world));

  // ---- Case C: disposing one backend leaves the other intact ---------------
  const g = make(hp3);
  const h = make(hp3);
  const i = make(hp3);
  step(g, STEPS);
  g.world.getResource('cableSolverBackend').dispose();
  step(h, STEPS);
  step(i, STEPS);
  assertExact(`${kind}: disposing G backend does not affect H`, state(h.world), state(i.world));

  for (const w of [a, b, c, d, e, f, g, h, i, j, k, l]) w.world.getResource('cableSolverBackend')?.dispose?.();
}

const t0 = performance.now();
run('native');
run('wasm');
console.log(`PASS independence (native and wasm): ${STEPS} steps; ${(performance.now() - t0).toFixed(0)} ms`);
