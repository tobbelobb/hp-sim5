// Reproducer: RemoteSpoolSystem's pacing mirrors survive the worker they describe.
//
// `fastModeActive` and `wasPaused` are the world's record of the last
// set_fast_mode / pause message it sent *to whichever worker was attached*. They
// are not reset when the worker changes (job end sets remote.worker = null in
// workerController's 'done' branch; the next job attaches a different worker via
// resetRemoteQueue()). The world then believes the new worker is already in a
// state it has never been told about, so the `set_fast_mode` / `resume` messages
// that should be sent are suppressed.
//
// Expected (the class's own invariant: "fastModeActive mirrors the last
// set_fast_mode sent to the current worker"): a fresh worker that is below the
// low water mark receives set_fast_mode(true), every time, on every job.
// Actual:   after a job that ended with the queue below the high water mark, the
//           next job's worker never receives set_fast_mode at all, so it paces
//           every command in real time (moveCommander's non-fast path) and the
//           world's queue never reaches the prefetch depth the design relies on.
//
// Also checks the two neighbouring boundary invariants that are *correct*
// (kept as negative controls): a superseded worker's late commands are dropped by
// the `remoteSystem.worker === worker` identity gate, and a paused world consumes
// no commands.
//
// Exit 0 = expected behaviour, exit 1 = defect reproduced.
//
// Run:  node repro_worker_replacement_state.mjs
import { readFileSync } from 'node:fs';
import { World, MachineTagComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { RemoteSpoolSystem } from '../../../../hp-sim-3d/app/remoteSpoolSystem.js';
import { SpoolTagComponent, SpoolStateComponent } from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';

const ROOT = new URL('../../../../', import.meta.url);
const remoteSrc = readFileSync(new URL('hp-sim-3d/app/remoteSpoolSystem.js', ROOT), 'utf8');
const drift = [];
if (!/if \(!this\.fastModeActive && queueSize < this\.lowWaterMark\)/.test(remoteSrc)) {
  drift.push('remoteSpoolSystem.js fast-mode enable condition changed');
}
if (!/} else if \(queueSize < this\.lowWaterMark && this\.wasPaused\)/.test(remoteSrc)) {
  drift.push('remoteSpoolSystem.js resume condition changed');
}
if (!/this\._worker = value \|\| null;/.test(remoteSrc)) {
  drift.push('remoteSpoolSystem.js worker setter changed');
}
if (!/remoteSystem\.worker === worker/.test(readFileSync(new URL('hp-sim-3d/app/workerController.js', ROOT), 'utf8'))) {
  drift.push('workerController.js late-message identity gate changed');
}

class StubWorker {
  constructor(label) { this.label = label; this.sent = []; }
  postMessage(message) { this.sent.push(message); }
  types() { return this.sent.map((message) => message.type); }
  count(type) { return this.sent.filter((message) => message.type === type).length; }
}

function makeWorld() {
  const world = new World();
  world.setResource('timeScale', 1);
  world.setResource('pauseState', { paused: false });
  const spool = world.createEntity();
  world.addComponent(spool, new SpoolTagComponent());
  world.addComponent(spool, new SpoolStateComponent('A'));
  world.addComponent(spool, new MachineTagComponent('m1'));
  world.addComponent(spool, new StepperMotorComponent());
  const remote = new RemoteSpoolSystem();
  world.registerSystem(remote);
  remote._ensureAxisMapping(world);
  return { world, remote };
}

const results = {};

// ---------------------------------------------------------------- job 1 + 2
{
  const { world, remote } = makeWorld();
  const workerA = new StubWorker('A');
  remote.worker = workerA;               // resetRemoteQueue(worker) in startSimulationWithWorker
  remote.update(world, 0.002);           // empty queue, well below lowWaterMark (160)
  const job1 = workerA.types();

  // The job finishes with the queue below the high water mark, which is the
  // normal steady state at 1x: the worker is only unpaused below 160 and the
  // queue can be anywhere in [160, 320] when the last command has been sent.
  for (let i = 0; i < 200; i += 1) remote.addCommand({ type: 'Move', A: 0 });
  remote.worker = null;                  // workerController 'done' branch
  const afterDone = { fastModeActive: remote.fastModeActive, wasPaused: remote.wasPaused,
    queued: remote.getQueueLength() };

  const workerB = new StubWorker('B');   // next job, same long-lived world
  remote.worker = workerB;
  remote.update(world, 0.002);           // queue (200) is above lowWaterMark: nothing expected yet
  // Drain the queue below the low water mark exactly as world steps would.
  while (remote.getQueueLength() >= remote.lowWaterMark) remote.update(world, 0.002);
  const job2 = workerB.types();

  results.job1 = { sent: job1, fastModeMessages: job1.filter((type) => type === 'set_fast_mode').length };
  results.betweenJobs = afterDone;
  results.job2 = { sent: job2, fastModeMessages: job2.filter((type) => type === 'set_fast_mode').length,
    queuedWhenBelowLowWaterMark: remote.getQueueLength() };
}

// ------------------------------------------------- control: mirror starts false
{
  const { world, remote } = makeWorld();
  const workerC = new StubWorker('C');
  remote.worker = workerC;
  // Same queue level job 2 was at when it should have been told: below the mark.
  remote.update(world, 0.002);
  results.controlFreshMirror = { sent: workerC.types(),
    fastModeMessages: workerC.types().filter((type) => type === 'set_fast_mode').length };
}

// ------------------------------------- negative control: late commands are gated
{
  const { world, remote } = makeWorld();
  const workerA = new StubWorker('A');
  remote.worker = workerA;
  remote.worker = null;                   // job A finished
  const workerB = new StubWorker('B');
  remote.worker = workerB;                // job B started
  // A message that the superseded worker A posts late.
  const gate = (worker, event) => {
    const remoteSystem = remote;
    if (remoteSystem && remoteSystem.worker === worker && event?.data?.action === 'gcode') {
      remoteSystem.addCommand(event.data.command);
    }
  };
  gate(workerA, { data: { action: 'gcode', command: { type: 'Move', A: 1 } } });
  const afterLateFromA = remote.getQueueLength();
  gate(workerB, { data: { action: 'gcode', command: { type: 'Move', A: 2 } } });
  results.lateSupersededWorker = { afterLateFromA, afterCurrentWorker: remote.getQueueLength() };
}

// -------------------------------------- negative control: a paused world holds
{
  const { world, remote } = makeWorld();
  world.getResource('pauseState').paused = true;
  const workerD = new StubWorker('D');
  remote.worker = workerD;
  remote.addCommand({ type: 'Move', A: 0 });
  remote.addCommand({ type: 'Move', A: 0 });
  // The pause gate lives in ecs.js World.update(): systems without runInPause are
  // skipped while pauseState.paused, and RemoteSpoolSystem sets no runInPause.
  for (let i = 0; i < 5; i += 1) world.update(0.002);
  const whilePaused = { queued: remote.getQueueLength(), executed: remote.history.length };
  world.getResource('pauseState').paused = false;
  world.update(0.002);
  results.pauseSemantics = { whilePaused,
    afterResume: { queued: remote.getQueueLength(), executed: remote.history.length } };
}

const result = { sourceDrift: drift, ...results };
console.log(JSON.stringify(result, null, 2));

const failures = [...drift];
if (!(results.job1.fastModeMessages >= 1)) failures.push('job 1 never received set_fast_mode(true)');
// The new worker is below the low water mark and has been told nothing, so the
// world must enable fast mode exactly as it does for a fresh RemoteSpoolSystem.
if (results.job2.fastModeMessages < 1) {
  failures.push('job 2 worker never received set_fast_mode(true) (mirror leaked from job 1)');
}
if (!(results.controlFreshMirror.fastModeMessages >= 1)) {
  failures.push('control: a fresh mirror did not enable fast mode');
}
if (results.lateSupersededWorker.afterLateFromA !== 0) failures.push('late command from a superseded worker was accepted');
if (results.lateSupersededWorker.afterCurrentWorker !== 1) failures.push('current worker command was not accepted');
if (results.pauseSemantics.whilePaused.executed !== 0) failures.push('a paused world executed commands');
if (results.pauseSemantics.afterResume.executed !== 1) failures.push('resume did not execute exactly one command');

if (failures.length) {
  console.error('FAIL (worker replacement state):\n - ' + failures.join('\n - '));
  process.exit(1);
}
console.error('PASS: pacing mirrors follow the worker');
