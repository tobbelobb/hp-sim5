import { dumpWorldState } from '../../src/js/cable_joints/debugUtils.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import { InputSystem } from './hangprinter_input.js';

const ASAP_RENDER_STRIDE = 10;
const ASAP_SLICE_BUDGET_MS = 28;
const ASAP_MIN_STEPS_PER_SLICE = 64;
const ASAP_INPUT_CHECK_MASK = 0x3f;
const ASAP_PAUSED_DELAY_MS = 12;
const PAUSED_POLL_DELAY_MS = 100;
const SPEED_UPDATE_PERIOD = 10;
const SIMULATION_PLAYBACK_RESOURCE = 'simulationPlayback';

export function runGame(world, internalSetupScene, options = {}) {
  const pauseBtn = document.getElementById('pauseBtn');
  const resetBtn = document.getElementById('resetBtn');
  const stepBtn = document.getElementById('stepBtn');
  const dumpBtn = document.getElementById('dumpBtn');
  const dtEl = document.getElementById('dt');
  const speedEl = document.getElementById('speed');

  const {
    initialTimeScale = 1.0,
    onTimeScaleChange,
  } = options;

  function sanitizeTimeScale(value, fallback) {
    if (!Number.isFinite(value) || value <= 0) {
      return fallback;
    }
    return value;
  }

  function sanitizeRenderOverride(value) {
    if (value == null) {
      return null;
    }
    if (!Number.isFinite(value)) {
      return Number.POSITIVE_INFINITY;
    }
    return Math.max(1, Math.floor(value));
  }

  const getPauseState = () => world.getResource('pauseState');
  const getRenderSystem = () => world.getResource('renderSystem') || null;
  const measurePerfBlock = (name, fn) => {
    const perf = world.getResource('performanceMonitor');
    if (perf?.enabled && typeof perf.measureBlock === 'function') {
      return perf.measureBlock(name, fn);
    }
    return fn();
  };

  const updatePauseButtonLabel = () => {
    const pauseState = getPauseState();
    if (!pauseBtn || !pauseState) {
      return;
    }
    if (!pauseState.paused) {
      pauseBtn.textContent = 'Pause';
    } else {
      pauseBtn.textContent = hasStarted ? 'Resume' : 'Start';
    }
  };

  const updatePlaybackResource = () => {
    world.setResource(SIMULATION_PLAYBACK_RESOURCE, {
      mode: playbackMode,
      renderEveryNth: renderStride,
    });
  };

  const recomputeRenderStride = () => {
    const base = renderStrideBase;
    const next = renderStrideOverride != null ? renderStrideOverride : base;
    const normalized = Number.isFinite(next) ? Math.max(1, Math.floor(next)) : Number.POSITIVE_INFINITY;
    if (renderStride !== normalized) {
      renderStride = normalized;
      renderCounter = 0;
    }
    updatePlaybackResource();
  };

  const stopLoops = () => {
    const renderSystem = getRenderSystem();
    if (renderSystem && typeof renderSystem.clearAnimationLoop === 'function') {
      renderSystem.clearAnimationLoop();
    }
    if (rafHandle != null) {
      cancelAnimationFrame(rafHandle);
      rafHandle = null;
    }
    if (asapTimer != null) {
      clearTimeout(asapTimer);
      asapTimer = null;
    }
    if (idleTimer != null) {
      clearTimeout(idleTimer);
      idleTimer = null;
    }
    loopToken += 1;
  };

  const shouldRunContinuously = () => {
    const pauseState = getPauseState();
    return !pauseState || !pauseState.paused || doStep;
  };

  const renderOnce = () => {
    const renderSystem = getRenderSystem();
    if (!renderSystem || renderSystem.drawingSuspended) {
      return;
    }

    if (typeof renderSystem.requestRender === 'function') {
      measurePerfBlock('RenderSystem3D.requestRender', () => {
        renderSystem.requestRender(world);
      });
      return;
    }

    measurePerfBlock('RenderSystem3D.update', () => {
      renderSystem.update?.(world, 0);
    });
  };

  const scheduleIdleCheck = () => {
    if (idleTimer != null) {
      return;
    }
    idleTimer = setTimeout(() => {
      idleTimer = null;
      updatePauseButtonLabel();
      if (shouldRunContinuously()) {
        startActiveLoop();
        return;
      }
      scheduleIdleCheck();
    }, PAUSED_POLL_DELAY_MS);
  };

  const enterIdleMode = () => {
    renderOnce();
    scheduleIdleCheck();
  };

  const setPlaybackMode = (mode) => {
    const normalized = mode === 'asap' ? 'asap' : 'linear';
    if (normalized === playbackMode) {
      return false;
    }
    playbackMode = normalized;
    renderStrideBase = playbackMode === 'asap' ? ASAP_RENDER_STRIDE : 1;
    lastTime = 0;
    accumulator = 0;
    renderCounter = 0;
    recomputeRenderStride();
    stopLoops();
    updatePlaybackResource();
    return true;
  };

  const maybeRender = () => {
    const renderSystem = getRenderSystem();
    if (!renderSystem || renderSystem.drawingSuspended) {
      return;
    }
    if (renderStride === Number.POSITIVE_INFINITY) {
      return;
    }
    renderCounter += 1;
    if (renderStride > 1 && renderCounter < renderStride) {
      return;
    }
    renderCounter = 0;
    measurePerfBlock('RenderSystem3D.update', () => {
      renderSystem.update?.(world, 0);
    });
  };

  const updateSpeedMeter = () => {
    if (!speedEl) {
      return;
    }
    if (!hasStarted || startTime <= 0) {
      speedEl.textContent = 'N/A';
      return;
    }
    const elapsed = (performance.now() - startTime) / 1000;
    if (elapsed > 0) {
      const avgSpeed = totalSim / elapsed;
      speedEl.textContent = `${avgSpeed.toFixed(2)}x`;
    }
  };

  let cachedRemoteSystem = null;
  const getRemoteSystem = () => {
    if (cachedRemoteSystem && world.systems.includes(cachedRemoteSystem)) {
      return cachedRemoteSystem;
    }
    cachedRemoteSystem = world.systems.find((system) => system instanceof RemoteSpoolSystem) || null;
    return cachedRemoteSystem;
  };

  const advancePhysics = (dt) => {
    const generation = world.getResource('sceneGeneration');
    let clock = world.getResource('researchClock');
    if (!clock || clock.generation !== generation) clock = { generation, step: 0, time: 0 };
    world.update(dt);
    clock.step += 1;
    clock.time += dt;
    world.setResource('researchClock', clock);
  };

  const advanceFixedSteps = async (steps) => {
    if (!Number.isInteger(steps) || steps < 1 || steps > 10000) throw new Error('steps must be in [1, 10000]');
    if (!getPauseState()?.paused) throw new Error('Pause before bounded stepping');
    if (getRemoteSystem()?.worker) throw new Error('Finish the active worker before bounded stepping');
    stopLoops();
    doStep = false;
    let executed = 0;
    try {
      while (executed < steps) {
        const recorder = world.getResource('flightRecorder');
        if (recorder?.readyForStep() === false) {
          const deadline = performance.now() + 5000;
          while (recorder.readyForStep() === false) {
            if (performance.now() > deadline) throw new Error('Flight recorder did not acknowledge samples');
            await new Promise((resolve) => setTimeout(resolve, 5));
          }
        }
        const pauseState = getPauseState();
        if (!pauseState?.paused || getRemoteSystem()?.worker) throw new Error(`Human intervention stopped bounded stepping after ${executed} steps`);
        pauseState.paused = false;
        try { advancePhysics(world.getResource('dt')); } finally { pauseState.paused = true; }
        executed += 1;
      }
      return { steps_executed: executed };
    } finally {
      stopLoops();
      if (getPauseState()) getPauseState().paused = true;
      enterIdleMode();
    }
  };

  const remoteQueueReady = () => {
    if (world.getResource('flightRecorder')?.readyForStep() === false) {
      return false;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return true;
    }
    const queueLength = typeof remoteSystem.getQueueLength === 'function'
      ? remoteSystem.getQueueLength()
      : (Array.isArray(remoteSystem.commands)
          ? Math.max(0, remoteSystem.commands.length - (remoteSystem.commandHead || 0))
          : 0);
    if (queueLength > 0) {
      return true;
    }
    return playbackMode !== 'asap' || !remoteSystem.worker;
  };

  const runLinearFrame = (currentTime) => {
    updatePauseButtonLabel();
    const dt = world.getResource('dt');
    if (dtEl && dtEl.textContent === 'N/A') {
      dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
    }
    const pauseState = getPauseState();
    if (!pauseState) {
      maybeRender();
      return;
    }
    if (lastTime === 0) {
      lastTime = currentTime;
    }
    const speedScale = targetTimeScale;
    const frameSec = speedScale * (currentTime - lastTime) / 1000;
    let simTimeProcessed = 0;

    if (frameSec >= dt) {
      lastTime = currentTime;
      const maxSteps = 500;
      const maxAccum = dt * maxSteps;
      accumulator = Math.min(accumulator + frameSec, maxAccum);
      while (accumulator >= dt) {
        if (!remoteQueueReady()) {
          break;
        }
        if (!pauseState.paused || doStep) {
          if (doStep) {
            pauseState.paused = false;
          }
          advancePhysics(dt);
          simTimeProcessed += dt;
          if (doStep) {
            pauseState.paused = true;
            doStep = false;
            break;
          }
        }
        if (pauseState.paused) {
          accumulator = 0;
          break;
        }
        accumulator -= dt;
      }
    }

    totalSim += simTimeProcessed;
    frameCounter += 1;
    if (frameCounter % SPEED_UPDATE_PERIOD === 0) {
      updateSpeedMeter();
    }
    maybeRender();
  };

  const runAsapBatch = () => {
    updatePauseButtonLabel();
    const dt = world.getResource('dt');
    if (dtEl && dtEl.textContent === 'N/A') {
      dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
    }
    const pauseState = getPauseState();
    if (!pauseState) {
      maybeRender();
      return ASAP_PAUSED_DELAY_MS;
    }

    let stepsRun = 0;
    const sliceDeadline = performance.now() + ASAP_SLICE_BUDGET_MS;
    const checkInput = typeof navigator !== 'undefined'
      && navigator !== null
      && navigator.scheduling
      && typeof navigator.scheduling.isInputPending === 'function'
      ? () => navigator.scheduling.isInputPending()
      : null;

    while (true) {
      if (!remoteQueueReady()) {
        return ASAP_PAUSED_DELAY_MS;
      }
      if (pauseState.paused && !doStep) {
        break;
      }
      if (doStep) {
        pauseState.paused = false;
      }
      advancePhysics(dt);
      totalSim += dt;
      stepsRun += 1;
      if (doStep) {
        pauseState.paused = true;
        doStep = false;
        break;
      }
      if (pauseState.paused) {
        break;
      }
      if (stepsRun >= ASAP_MIN_STEPS_PER_SLICE) {
        if (performance.now() >= sliceDeadline) {
          break;
        }
        if (checkInput && (stepsRun & ASAP_INPUT_CHECK_MASK) === 0 && checkInput()) {
          break;
        }
      }
    }

    frameCounter += 1;
    if (frameCounter % SPEED_UPDATE_PERIOD === 0) {
      updateSpeedMeter();
    }
    maybeRender();

    if (pauseState.paused && !doStep) {
      return ASAP_PAUSED_DELAY_MS;
    }
    return 0;
  };

  const startLinearLoop = (token) => {
    const step = (time) => {
      if (token !== loopToken) {
        return;
      }
      runLinearFrame(time);
      if (!shouldRunContinuously()) {
        stopLoops();
        enterIdleMode();
        return;
      }
      const renderSystem = getRenderSystem();
      if (!renderSystem || typeof renderSystem.setAnimationLoop === 'function') {
        return;
      }
      rafHandle = requestAnimationFrame(step);
    };

    const renderSystem = getRenderSystem();
    if (renderSystem && typeof renderSystem.setAnimationLoop === 'function') {
      renderSystem.setAnimationLoop(step);
      return;
    }
    rafHandle = requestAnimationFrame(step);
  };

  const startAsapLoop = (token) => {
    const iterate = () => {
      if (token !== loopToken) {
        return;
      }
      const delay = runAsapBatch();
      if (token !== loopToken) {
        return;
      }
      if (!shouldRunContinuously()) {
        stopLoops();
        enterIdleMode();
        return;
      }
      asapTimer = setTimeout(iterate, delay);
    };
    iterate();
  };

  const startActiveLoop = () => {
    if (!shouldRunContinuously()) {
      enterIdleMode();
      return;
    }
    const currentToken = loopToken;
    if (playbackMode === 'asap') {
      startAsapLoop(currentToken);
    } else {
      startLinearLoop(currentToken);
    }
  };

  const resetGame = ({ autoPause = true } = {}) => {
    internalSetupScene();
    world.setResource('researchClock', { generation: world.getResource('sceneGeneration'), step: 0, time: 0 });
    for (const system of world.systems) {
      if (system instanceof InputSystem && typeof system.reset === 'function') {
        system.reset();
      }
    }
    lastTime = 0;
    accumulator = 0;
    frameCounter = 0;
    renderCounter = 0;
    if (speedEl) {
      speedEl.textContent = 'N/A';
    }
    totalSim = 0;
    const pauseState = getPauseState();
    if (autoPause) {
      startTime = 0;
      hasStarted = false;
      if (pauseState) {
        pauseState.paused = true;
      }
    } else {
      startTime = performance.now();
      hasStarted = true;
      if (pauseState) {
        pauseState.paused = false;
      }
      lastTime = performance.now();
    }
    doStep = false;
    updatePauseButtonLabel();
    stopLoops();
    if (shouldRunContinuously()) {
      startActiveLoop();
    } else {
      enterIdleMode();
    }
  };

  const setRenderEveryNth = (value) => {
    const sanitized = sanitizeRenderOverride(value);
    if (renderStrideOverride === sanitized) {
      return;
    }
    renderStrideOverride = sanitized;
    renderCounter = 0;
    recomputeRenderStride();
  };

  const setTimeScale = (scale) => {
    const clamped = sanitizeTimeScale(scale, targetTimeScale);
    const desiredMode = 'linear';
    const modeChanged = setPlaybackMode(desiredMode);
    if (Math.abs(clamped - targetTimeScale) < 1e-6 && !modeChanged) {
      return;
    }
    targetTimeScale = clamped;
    world.setResource('timeScale', targetTimeScale);
    lastTime = 0;
    accumulator = 0;
    frameCounter = 0;
    renderCounter = 0;
    totalSim = 0;
    if (speedEl) {
      speedEl.textContent = 'N/A';
    }
    const pauseState = getPauseState();
    startTime = pauseState && !pauseState.paused ? performance.now() : 0;
    if (typeof onTimeScaleChange === 'function') {
      onTimeScaleChange(targetTimeScale);
    }
    if (modeChanged) {
      startActiveLoop();
    }
  };

  const getTimeScale = () => targetTimeScale;

  const resumeSimulation = () => {
    const pauseState = getPauseState();
    if (!pauseState || !pauseState.paused) {
      return;
    }
    pauseState.paused = false;
    if (!hasStarted) {
      startTime = performance.now();
      totalSim = 0;
      hasStarted = true;
    }
    lastTime = performance.now();
    updatePauseButtonLabel();
    stopLoops();
    startActiveLoop();
  };

  const pauseSimulation = () => {
    const pauseState = getPauseState();
    if (!pauseState || pauseState.paused) {
      return;
    }
    pauseState.paused = true;
    updatePauseButtonLabel();
    stopLoops();
    enterIdleMode();
  };

  if (pauseBtn) {
    pauseBtn.addEventListener('click', (event) => {
      event.preventDefault();
      const pauseState = getPauseState();
      if (!pauseState) {
        return;
      }
      if (pauseState.paused) {
        resumeSimulation();
        return;
      }
      pauseSimulation();
    });
  }

  if (resetBtn) {
    resetBtn.addEventListener('click', (event) => {
      event.preventDefault();
      resetGame({ autoPause: true });
    });
  }

  if (stepBtn) {
    stepBtn.addEventListener('click', (event) => {
      event.preventDefault();
      const pauseState = getPauseState();
      if (pauseState && pauseState.paused) {
        doStep = true;
        stopLoops();
        startActiveLoop();
      }
    });
  }

  if (dumpBtn) {
    dumpBtn.addEventListener('click', (event) => {
      event.preventDefault();
      console.log(dumpWorldState(world));
    });
  }

  let targetTimeScale = sanitizeTimeScale(initialTimeScale, 1.0);
  world.setResource('timeScale', targetTimeScale);

  let lastTime = 0;
  let accumulator = 0.0;
  let doStep = false;
  let frameCounter = 0;
  let renderCounter = 0;
  let startTime = 0;
  let totalSim = 0;
  let hasStarted = false;
  let playbackMode = 'linear';
  let renderStrideBase = 1;
  let renderStrideOverride = null;
  let renderStride = 1;
  let rafHandle = null;
  let asapTimer = null;
  let idleTimer = null;
  let loopToken = 0;

  recomputeRenderStride();
  setPlaybackMode('linear');
  resetGame({ autoPause: false });
  if (typeof onTimeScaleChange === 'function') {
    onTimeScaleChange(targetTimeScale);
  }

  return {
    reset: resetGame,
    setTimeScale,
    getTimeScale,
    setRenderEveryNth,
    resume: resumeSimulation,
    pause: pauseSimulation,
    advanceFixedSteps,
  };
}
