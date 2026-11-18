// Reproducer: the worker's "done" message is treated as "the print is finished",
// so the Quality History entry is built before the world has executed the
// commands the worker already handed over.
//
// Boundary under test (verbatim app wiring, drift-checked below):
//   workerController.js handleWorkerMessage(), 'done' branch:
//       remoteSystem.worker = null; onWorkerDone(worker);
//   appBootstrap.js:  onWorkerDone: () => controllers.commands?.finishJob?.(),
//   commandJobController.js finishJob():
//       setPrintActive(false);
//       recordFinalQualityAndResetJob();  // -> quality.runFinalQualityChecks(...)
//   qualityController.runFinalQualityChecks() -> monitor.runFinalCheck() for each
//       machine, then recordQualityHistoryEntry() stores metrics.score.
//
// The worker's 'done' means "no more commands will be produced", not "no more
// commands will be executed". RemoteSpoolSystem.update() consumes exactly one
// command per world step and the worker is only allowed to run highWaterMark
// commands ahead (320 at 1x, scaled by timeScale), so at 'done' the queue
// normally holds the full prefetch buffer. finishJob() then measures a print
// whose last commands - including the corners in them - have not happened yet.
//
// Expected: the Quality History entry describes the finished print: same sample
//           count, same corner count, same score as the monitor once the world
//           has executed every command the worker produced.
// Actual:   the entry is a snapshot of the executed prefix (68 % of the stream
//           below) and is completely insensitive to defects in the tail.
//
// Exit 0 = expected behaviour, exit 1 = defect reproduced.
//
// Run:  node repro_quality_recorded_before_drain.mjs
import { readFileSync } from 'node:fs';
import { World, MachineTagComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { RemoteSpoolSystem } from '../../../../hp-sim-3d/app/remoteSpoolSystem.js';
import { SpoolTagComponent, SpoolStateComponent } from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { ExtruderComponent } from '../../../../hp-sim-3d/app/hangprinter_extruder.js';
import { createQualityController } from '../../../../hp-sim-3d/app/qualityController.js';

const ROOT = new URL('../../../../', import.meta.url);

// --- drift checks on the two boundary functions being mirrored ---------------
const workerControllerSrc = readFileSync(new URL('hp-sim-3d/app/workerController.js', ROOT), 'utf8');
const jobControllerSrc = readFileSync(new URL('hp-sim-3d/app/commandJobController.js', ROOT), 'utf8');
const drift = [];
if (!/remoteSystem\.worker = null;\s*\n\s*onWorkerDone\(worker\);/.test(workerControllerSrc)) {
  drift.push('workerController.js done-branch shape changed');
}
if (!/asapState\.pendingFinalCheck = true;\s*\n\s*\} else \{\s*\n\s*recordFinalQualityAndResetJob\(\);/.test(jobControllerSrc)) {
  drift.push('commandJobController.finishJob() shape changed');
}

// --- minimal fake DOM (qualityController/QualityMonitor touch only these) ----
globalThis.HTMLElement = globalThis.HTMLElement || class HTMLElement {};
function fakeElement() {
  return Object.assign(new globalThis.HTMLElement(), {
    className: '', textContent: '', innerHTML: '', dataset: {}, parentElement: null, children: [],
    style: { setProperty() {}, removeProperty() {} },
    classList: {
      set: new Set(),
      add(c) { this.set.add(c); }, remove(c) { this.set.delete(c); }, contains(c) { return this.set.has(c); },
      toggle(c, force) { if (force === undefined) { this.set.has(c) ? this.set.delete(c) : this.set.add(c); } else if (force) this.set.add(c); else this.set.delete(c); },
    },
    appendChild(c) { this.children.push(c); c.parentElement = this; },
    removeChild(c) { const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); c.parentElement = null; },
    addEventListener() {}, removeEventListener() {}, setAttribute() {}, toggleAttribute() {},
    closest() { return null; },
  });
}

const SIM_DT = 0.002;               // public/usd_scenes/*.usda: timeCodesPerSecond = 500
const SIDE = 250;                   // segments per square side, 1 mm each
const SEGMENTS = 4 * SIDE;          // a closed square with 3 interior 90 deg corners
const EXTRUSION_PER_SEGMENT = 0.01; // mm of filament; same units as the reference
const HIGH_WATER_MARK = 320;        // RemoteSpoolSystem.baseHighWaterMark at timeScale 1
const TAIL_ERROR_M = 0.003;         // synthetic: 3 mm lateral error on the last side

// Square path: +X, +Y, -X, -Y. Corners at segments 250, 500, 750.
function buildReference() {
  const corners = [[0, 0], [0.25, 0], [0.25, 0.25], [0, 0.25], [0, 0]];
  const segments = [];
  let index = 0;
  for (let side = 0; side < 4; side += 1) {
    const [x0, y0] = corners[side];
    const [x1, y1] = corners[side + 1];
    for (let i = 0; i < SIDE; i += 1) {
      const t0 = i / SIDE;
      const t1 = (i + 1) / SIDE;
      segments.push({
        start: [x0 + (x1 - x0) * t0, y0 + (y1 - y0) * t0, 0],
        end: [x0 + (x1 - x0) * t1, y0 + (y1 - y0) * t1, 0],
        extrusion: EXTRUSION_PER_SEGMENT,
        index: index++,
      });
    }
  }
  return segments;
}

// Where the world's effector tip is when command `index` is consumed. Perfect on
// the first three sides; the last side drifts sideways by TAIL_ERROR_M.
function tipPosition(segments, index, tailErrorM) {
  const end = segments[index].end;
  const onTail = index >= 3 * SIDE;
  return { x: end[0], y: end[1] + (onTail ? tailErrorM : 0), z: end[2] };
}

function run({ tailErrorM }) {
  const segments = buildReference();
  const world = new World();
  world.setResource('timeScale', 1);
  world.setResource('pauseState', { paused: false });

  const spool = world.createEntity();
  world.addComponent(spool, new SpoolTagComponent());
  world.addComponent(spool, new SpoolStateComponent('A'));
  world.addComponent(spool, new MachineTagComponent('m1'));
  world.addComponent(spool, new StepperMotorComponent());

  const extruderEntity = world.createEntity();
  const extruder = new ExtruderComponent();
  extruder.machineTips = { m1: { x: 0, y: 0, z: 0 } };
  world.addComponent(extruderEntity, extruder);

  const remote = new RemoteSpoolSystem();
  world.registerSystem(remote);
  remote._ensureAxisMapping(world);

  const document = { createElement: () => fakeElement(), defaultView: null };
  const machines = [{ id: 'm1', sourceKey: null, name: 'repro machine' }];
  const quality = createQualityController({
    document, world, state: { qualityEnabled: false }, hudElement: fakeElement(),
    historyHud: fakeElement(), historyToggle: fakeElement(), historyList: fakeElement(),
    qualityToggle: null,                    // HUD off; metrics are still computed
    getMachines: () => machines,
    getMachineDisplayName: () => 'repro machine',
    getReferenceOverlayState: () => ({ segments: null }),
  });
  const monitor = quality.ensureQualityMonitorForMachine(machines[0]);
  monitor.setReferenceSegments(segments);   // app: reference path from inspection tools
  quality.attachToRemoteSystem(remote);     // app: machineSceneController.rebuildScene()

  const pending = segments.map(() => ({ type: 'Move', A: 0, E: EXTRUSION_PER_SEGMENT }));
  let executed = 0;
  for (let step = 0; step < SEGMENTS; step += 1) {
    // Fast-mode worker keeps the queue at its high water mark.
    while (pending.length > 0 && remote.getQueueLength() < HIGH_WATER_MARK) {
      remote.addCommand(pending.shift());
    }
    if (pending.length === 0) break;
    extruder.machineTips.m1 = tipPosition(segments, executed, tailErrorM);
    remote.update(world, SIM_DT);
    executed += 1;
  }

  // 'done' is posted right after the last command, before the queue drains.
  const queuedAtDone = remote.getQueueLength();
  remote.worker = null;                       // workerController 'done' branch
  quality.runFinalQualityChecks({ activeJobId: 1, lastRecordedJobId: null,
    currentJobDescriptor: { label: 'repro square' } });
  const record = quality.historyRecords.at(-1) ?? null;
  const metricsAtDone = monitor.getMetrics(); // what the history entry was built from
  const atDone = {
    executedCommands: executed,
    queuedCommands: queuedAtDone,
    samples: metricsAtDone?.samples ?? null,
    cornerCount: metricsAtDone?.cornerCount ?? null,
    progress: metricsAtDone?.progress ?? null,
    score: record?.score ?? null,
  };

  // The world keeps executing the commands it already owns.
  while (remote.getQueueLength() > 0) {
    extruder.machineTips.m1 = tipPosition(segments, executed, tailErrorM);
    remote.update(world, SIM_DT);
    executed += 1;
  }
  monitor.refreshHud(true);
  const finalMetrics = monitor.getMetrics();

  return {
    tailErrorMm: tailErrorM * 1000,
    atDone,
    atDoneSimTimeS: +(atDone.executedCommands * SIM_DT).toFixed(3),
    completed: {
      samples: finalMetrics.samples,
      cornerCount: finalMetrics.cornerCount,
      progress: finalMetrics.progress,
      score: finalMetrics.score,
    },
    commandsExecutedAfterDone: executed - atDone.executedCommands,
    scoreDelta: atDone.score == null ? null : +(finalMetrics.score - atDone.score).toFixed(3),
  };
}

const withTailError = run({ tailErrorM: TAIL_ERROR_M });
const withoutTailError = run({ tailErrorM: 0 });

const result = { sourceDrift: drift, withTailError, withoutTailError };
console.log(JSON.stringify(result, null, 2));

const failures = [...drift];
const near = (a, b, tol) => Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= tol;

for (const [label, run_] of [['tail error', withTailError], ['control', withoutTailError]]) {
  if (run_.atDone.samples !== run_.completed.samples) {
    failures.push(`${label}: recorded ${run_.atDone.samples} samples, the print has ${run_.completed.samples}`);
  }
  if (run_.atDone.cornerCount !== run_.completed.cornerCount) {
    failures.push(`${label}: recorded ${run_.atDone.cornerCount} corners, the print has ${run_.completed.cornerCount}`);
  }
  if (!near(run_.atDone.score, run_.completed.score, 0.5)) {
    failures.push(`${label}: recorded score ${run_.atDone.score} != completed-run score ${run_.completed.score}`);
  }
  if (!(run_.atDone.queuedCommands > 0)) failures.push(`${label}: nothing was queued at 'done'`);
}
if (withTailError.completed.score >= withoutTailError.completed.score - 1) {
  failures.push('completed-run score is insensitive to a 3 mm error on the last side');
}

if (failures.length) {
  console.error('FAIL (quality recorded before the world drains):\n - ' + failures.join('\n - '));
  process.exit(1);
}
console.error('PASS: the quality record describes the completed print');
