import { captureFlightRecorderSnapshot } from './flightRecorderSnapshot.js';

// This adapter operates the application's world and controllers, never a second simulator.
export function createResearchControls({ world, runtime, machines, commands, recorder, inspection, scenes = [], pageId = crypto.randomUUID() }) {
  const interventions = [];
  function context(selectedEntity = null) {
    const clock = world.getResource('researchClock') || { step: 0, time: 0 };
    return {
      backend: 'browser-js', page_id: pageId, scene_generation: world.getResource('sceneGeneration'),
      sim_step: clock.step, sim_time_s: clock.time, timeline: 'sim_step', selected_entity: selectedEntity,
      scenes: machines.getActiveSourceKeys(), paused: Boolean(world.getResource('pauseState')?.paused),
      recording: recorder.socket?.readyState === 1 && recorder.generation === world.getResource('sceneGeneration')
        ? { session: recorder.session, generation: recorder.generation, step: recorder.step, time: recorder.time } : null,
    };
  }
  function observe() {
    return { ...context(), ...captureFlightRecorderSnapshot(world), inspection: inspection.getState(),
      queue_length: commands.getRemoteSystem()?.getQueueLength() || 0 };
  }
  function log(action, args) {
    interventions.push({ action, args: structuredClone(args), context: context(), submitted_at: new Date().toISOString() });
  }
  async function execute(action, args = {}) {
    if (action === 'capabilities') return { backend: 'browser-js', page_id: pageId,
      actions: ['observe', 'pause', 'resume', 'step', 'reset', 'load_scene', 'commands', 'record', 'capture_context', 'interventions'],
      scenes, limits: { steps: 10000, commands: 10000 }, command_units: 'motor radians; geometry metres; torque Nm' };
    if (action === 'observe') return observe();
    if (action === 'interventions') return structuredClone(interventions);
    if (action === 'capture_context') {
      if (typeof args.message !== 'string' || !args.message.trim()) throw new Error('Supply a research message');
      const captured = { ...context(args.selected_entity ?? null), message: args.message,
        capture_id: crypto.randomUUID(), submitted_at: new Date().toISOString(), observation: observe() };
      interventions.push({ action, context: structuredClone(captured) });
      return captured;
    }
    const controls = runtime.getGameControls();
    if (!controls) throw new Error('Simulation is not ready');
    if (action === 'pause') controls.pause();
    else if (action === 'resume') controls.resume();
    else if (action === 'step') await controls.advanceFixedSteps(args.steps ?? 1);
    else if (action === 'reset') {
      const generation = world.getResource('sceneGeneration');
      commands.handleUserReset();
      if (world.getResource('sceneGeneration') === generation) throw new Error('Reset was blocked by the current application state');
    }
    else if (action === 'load_scene') {
      if (!scenes.includes(args.scene)) throw new Error('Select a scene from capabilities');
      const machine = await machines.addCatalogMachine(args.scene, { resetView: true });
      if (!machine) throw new Error('Scene could not be loaded');
      for (const previous of [...machines.getMachines()]) {
        if (previous.id !== machine.id) await machines.removeMachine(previous.id);
      }
      commands.handleUserReset();
    } else if (action === 'commands') {
      const remote = commands.getRemoteSystem();
      if (!remote) throw new Error('Scene has no remote motor controller');
      if (!world.getResource('pauseState')?.paused || remote.worker) throw new Error('Pause and finish the active worker before submitting commands');
      remote._ensureAxisMapping(world);
      const records = args.commands;
      if (!Array.isArray(records) || records.length + remote.getQueueLength() > 10000) throw new Error('Supply up to 10000 commands');
      const axes = new Set(Object.keys(remote.axisToEntity));
      for (const cmd of records) {
        if (!cmd || typeof cmd !== 'object' || Array.isArray(cmd)) throw new Error('Command must be an object');
        if (!Object.keys(cmd).length) continue;
        if (!['Move', 'Add to reference', 'SetTorqueMode', 'SetPositionMode'].includes(cmd.type)) throw new Error('Unknown command type');
        if (cmd.type.startsWith('Set')) {
          if (!axes.has(cmd.axis) || (cmd.type === 'SetTorqueMode' && !Number.isFinite(cmd.torqueNm))) throw new Error('Invalid motor mode');
        } else {
          for (const [axis, value] of Object.entries(cmd.axes || Object.fromEntries(Object.entries(cmd).filter(([key]) => key !== 'type')))) {
            if ((!axes.has(axis) && axis !== 'E') || !Number.isFinite(value)) throw new Error('Unknown axis or non-finite target');
          }
        }
      }
      for (const cmd of records) remote.addCommand(structuredClone(cmd));
    } else if (action === 'record') {
      if (args.enabled === false) recorder.disconnect(); else recorder.connect();
    } else throw new Error('Unknown browser action');
    log(action, args);
    return observe();
  }
  return { execute, context, recordHumanIntervention: (control) => log('human_control', { control }) };
}

export async function attachResearchControls({ api, document, window, url }) {
  const actions = ['capabilities', 'observe', 'pause', 'resume', 'step', 'reset', 'load_scene', 'commands', 'record', 'capture_context', 'interventions'];
  if (typeof document.modelContext?.registerTool === 'function') {
    for (const action of actions) {
      await document.modelContext.registerTool({
        name: `hp_sim5_${action}`, description: `Operate this exact browser-js page: ${action}. Native Python is a separate world.`,
        inputSchema: { type: 'object', properties: {
          steps: { type: 'integer', minimum: 1, maximum: 10000 }, scene: { type: 'string' },
          commands: { type: 'array', items: { type: 'object' }, maxItems: 10000 },
          enabled: { type: 'boolean' }, message: { type: 'string' }, selected_entity: { type: 'string' },
        }, additionalProperties: false },
        annotations: { readOnlyHint: ['capabilities', 'observe', 'interventions'].includes(action) },
        execute: (args) => api.execute(action, args),
      });
    }
  }
  const controlsRoot = document.getElementById('controls');
  controlsRoot?.addEventListener('click', (event) => {
    const button = event.target.closest?.('button, input');
    if (button?.id) api.recordHumanIntervention(button.id);
  }, { capture: true });
  controlsRoot?.addEventListener('change', (event) => {
    if (event.target.id) api.recordHumanIntervention(event.target.id);
  }, { capture: true });
  if (!url) return;
  const socket = new window.WebSocket(url);
  let queue = Promise.resolve();
  socket.addEventListener('message', (event) => {
    queue = queue.then(async () => {
      const request = JSON.parse(event.data);
      try {
        const result = await api.execute(request.action, request.args);
        socket.send(JSON.stringify({ id: request.id, result }));
      } catch (error) {
        socket.send(JSON.stringify({ id: request.id, error: error.message }));
      }
    });
  });
  socket.addEventListener('open', () => socket.send(JSON.stringify({ type: 'ready', ...api.context() })));
  window.addEventListener('pagehide', () => socket.close(), { once: true });
  // Capture the exact page/time at submission; subsequent navigation cannot alter it.
  const host = document.getElementById('controls');
  const panel = document.createElement('div');
  const label = document.createElement('span');
  label.textContent = `Research backend: browser-js (${api.context().page_id.slice(0, 8)}) `;
  const note = document.createElement('input');
  note.placeholder = 'Research note about this state';
  note.setAttribute('aria-label', 'Research note');
  const entity = document.createElement('input');
  entity.placeholder = 'Entity or measurement (optional)';
  entity.setAttribute('aria-label', 'Research entity');
  const button = document.createElement('button');
  button.textContent = 'Capture for research';
  button.addEventListener('click', async () => {
    try {
      if (socket.readyState !== 1) throw new Error('Research supervisor is disconnected');
      const captured = await api.execute('capture_context', { message: note.value, selected_entity: entity.value || null });
      socket.send(JSON.stringify({ type: 'context', result: captured }));
      label.textContent = `Captured browser-js step ${captured.sim_step} (${captured.sim_time_s.toFixed(3)} s) `;
    } catch (error) { label.textContent = error.message; }
  });
  panel.append(label, note, entity, button);
  host?.append(panel);
}
