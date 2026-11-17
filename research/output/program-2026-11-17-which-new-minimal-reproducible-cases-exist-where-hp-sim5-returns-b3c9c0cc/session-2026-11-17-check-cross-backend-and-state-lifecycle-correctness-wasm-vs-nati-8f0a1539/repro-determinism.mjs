// Finding 3: identical input and steps must be bit-identical, in-process and
// across process restarts.
//
// Mode 1 (default): three fresh worlds per backend/machine in one process.
// Mode 2 (--digest KIND MACHINE STEPS): print one digest; the parent reruns this
// script in fresh child processes and compares.
//
// Exit code 1 on any digest or full-state mismatch.
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { state, digest, firstExactDifference, sceneText, makeWorld, loadWasmModule, makeBackend } from './lib.mjs';
import { EncoderComponent } from '../../../../src/js/cable_joints_3d/ecs.js';

const STEPS = 1500;

// Deterministic excitation: one Move command per 2 ms step, sinusoids on all
// four axes. No random source and no dependence on wall-clock time.
export function schedule(steps) {
  const commands = [];
  for (let i = 0; i < steps; i++) {
    commands.push({ type: 'Move',
      A: 0.9 * Math.sin(i / 90), B: 0.9 * Math.sin(i / 73 + 1),
      C: 0.9 * Math.sin(i / 111 + 2), D: 0.9 * Math.sin(i / 57 + 3) });
  }
  return commands;
}

export function runOnce(kind, machine, steps) {
  const module = loadWasmModule();
  const instance = makeWorld(sceneText(machine), {
    physicsSubsteps: 2, cableSolverBackend: makeBackend(kind, module) });
  instance.remote.commands = schedule(steps);
  for (let i = 0; i < steps; i++) instance.world.update(instance.dt);
  const snapshot = state(instance.world);
  const result = { hash: digest(snapshot), entries: snapshot.length,
    encoders: instance.world.query([EncoderComponent]).map(id => instance.world.getComponent(id, EncoderComponent).angle) };
  instance.world.getResource('cableSolverBackend').dispose();
  return { snapshot, result };
}

const args = process.argv.slice(2);
if (args[0] === '--digest') {
  const [, kind, machine, steps] = args;
  const { result } = runOnce(kind, machine, Number(steps));
  process.stdout.write(`${JSON.stringify(result)}\n`);
} else {
  const module = loadWasmModule();
  function repeat(kind, machine) {
    let reference = null;
    for (let run = 1; run <= 3; run++) {
      const t0 = performance.now();
      const instance = makeWorld(sceneText(machine), { physicsSubsteps: 2,
        cableSolverBackend: makeBackend(kind, module) });
      instance.remote.commands = schedule(STEPS);
      for (let i = 0; i < STEPS; i++) instance.world.update(instance.dt);
      const snapshot = state(instance.world);
      const hash = digest(snapshot);
      if (reference) {
        const diff = firstExactDifference(reference.snapshot, snapshot);
        if (diff || reference.hash !== hash) {
          console.error(`FAIL ${kind}/${machine} in-process run ${run} differs from run 1`);
          console.error(JSON.stringify(diff ?? { hash: [reference.hash, hash] }, null, 2));
          process.exit(1);
        }
      }
      reference ??= { snapshot, hash };
      console.log(`ok   ${kind}/${machine} in-process run ${run}: ${hash} (${snapshot.length} entries, ${(performance.now() - t0).toFixed(0)} ms)`);
      instance.world.getResource('cableSolverBackend').dispose();
    }
    return reference.hash;
  }

  const self = fileURLToPath(import.meta.url);
  function crossProcess(kind, machine) {
    const outputs = [1, 2].map(() => JSON.parse(execFileSync(process.execPath,
      [self, '--digest', kind, machine, String(STEPS)], { encoding: 'utf8' }).trim().split('\n').at(-1)));
    if (outputs[0].hash !== outputs[1].hash) {
      console.error(`FAIL ${kind}/${machine} differs across process restarts: ${outputs[0].hash} vs ${outputs[1].hash}`);
      process.exit(1);
    }
    console.log(`ok   ${kind}/${machine} cross-process runs agree: ${outputs[0].hash}`);
    return outputs[0].hash;
  }

  const t0 = performance.now();
  for (const kind of ['native', 'wasm']) for (const machine of ['hp3', 'hp4']) {
    const inProcess = repeat(kind, machine);
    const cross = crossProcess(kind, machine);
    if (inProcess !== cross) {
      console.error(`FAIL ${kind}/${machine} in-process and cross-process digests differ`);
      process.exit(1);
    }
  }
  console.log(`PASS determinism: ${STEPS} commanded steps, 3 in-process repeats + 2 process restarts per backend/machine (${(performance.now() - t0).toFixed(0)} ms)`);
}
