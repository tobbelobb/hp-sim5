// E6: ordering. (a) does the settings barrier hold back commands, or only
// encoder replies? (b) is set_speed_scale applied in arrival order?
import { createProbe, StubSocket, StubWorld, createStubRemote } from './harness.mjs';

const out = {};

// (a) settings barrier: machineType load is asynchronous.
{
  const world = new StubWorld();
  const remote = createStubRemote();
  let finishLoad;
  let loadedKey = 'hp4.usda';
  const machines = {
    getMachines: () => [{ sourceKey: loadedKey }],
    removeAllMachines: async () => { loadedKey = null; },
    addCatalogMachine: (sourceKey) => new Promise((resolve) => {
      finishLoad = () => { loadedKey = sourceKey; resolve({}); };
    }),
  };
  const probe = createProbe({
    world,
    machines,
    commands: { getRemoteSystem: () => remote, pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); return true; } },
  });
  probe.controller.connect();
  const socket = StubSocket.instances.at(-1);
  socket.open();
  remote.clearCommandQueue();

  socket.deliver({ type: 'simulation_settings', machineType: 'hangprinter_3', solverIterations: 3 });
  await new Promise(r => setTimeout(r, 0));
  const afterSettingsTick = { queueAfterSwapStarted: remote.queue.length, calls: socket.sent.length };
  socket.deliver({ type: 'command', command: { type: 'Move', B: 0.02 } });
  const commandLandedDuringSwap = remote.queue.length;
  socket.deliver({ type: 'encoder_request', requestId: 1, axes: [] });
  await new Promise(r => setTimeout(r, 0));
  const encoderAnsweredDuringSwap = socket.sent.some(p => p.requestId === 1);
  finishLoad();
  await new Promise(r => setTimeout(r, 0));
  out.settingsBarrier = {
    afterSettingsTick,
    commandLandedDuringSwap,
    encoderAnsweredDuringSwap,
    encoderAnsweredAfterLoad: socket.sent.some(p => p.requestId === 1),
    note: 'encoder replies wait for the machine swap; commands do not',
  };
}

// (b) set_speed_scale ordering against a command, faithful runner model.
{
  const model = () => {
    const state = { paused: true, scale: 1, resumes: 0, clockResets: 0 };
    return {
      state,
      runtime: {
        resume() { if (!state.paused) return; state.paused = false; state.resumes += 1; },
        setTimeScale(v) {
          if (Math.abs(v - state.scale) < 1e-6) return;
          state.scale = v; state.clockResets += 1;
        },
      },
    };
  };
  const run = (messages) => {
    const { state, runtime } = model();
    const remote = createStubRemote();
    const probe = createProbe({
      world: new StubWorld(), runtime,
      commands: {
        getRemoteSystem: () => remote,
        pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); runtime.resume(); return true; },
        applyTimeScaleChange: (v) => runtime.setTimeScale(v),
      },
    });
    probe.controller.connect();
    const s = StubSocket.instances.at(-1);
    s.open();
    // The connect-time Rebase goes through pushExternalCommands (printActive +
    // resume); record it, then start each scenario from the documented paused state.
    const resumedAtOpen = state.paused === false;
    remote.clearCommandQueue();
    state.paused = true;
    state.resumes = 0;
    for (const m of messages) s.deliver(m);
    return { resumedAtOpen, paused: state.paused, scale: state.scale, queued: remote.getQueueLength(),
      resumeCalls: state.resumes, clockResets: state.clockResets };
  };
  out.speedOrdering = {
    connect_only: run([]),
    speed_then_command: run([{ type: 'set_speed_scale', value: 25 },
      { type: 'command', command: { type: 'Move', B: 0.02 } }]),
    command_then_speed: run([{ type: 'command', command: { type: 'Move', B: 0.02 } },
      { type: 'set_speed_scale', value: 25 }]),
    speed_while_paused: run([{ type: 'set_speed_scale', value: 25 }]),
  };
}

console.log(JSON.stringify(out, null, 2));
