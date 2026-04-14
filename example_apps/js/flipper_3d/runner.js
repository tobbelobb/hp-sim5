import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';
import { InputSystem, InputReplaySystem } from './flipper_common_3d.js';

export function runGame(world, setupScene, sceneData) {
  const pauseBtn = document.getElementById('pauseBtn');
  const resetBtn = document.getElementById('resetBtn');
  const stepBtn = document.getElementById('stepBtn');
  const dumpBtn = document.getElementById('dumpBtn');
  const dtEl = document.getElementById('dt');
  const speedEl = document.getElementById('speed');

  let lastTime = 0;
  let accumulator = 0.0;
  let doStep = false;
  let stepWhileKeyHeld = false;
  let speedSamples = [];
  const numSpeedSamples = 60;
  let frameCounter = 0;
  const debugEnabled = Boolean(window._flipper3dDebug);
  const manualLoopMode = Boolean(window._flipperDisableAutoLoop);

  function getRenderSystem() {
    return world.systems.find((system) => system instanceof RenderSystem3D);
  }

  function renderCurrentState() {
    const renderSystem = getRenderSystem();
    if (renderSystem) {
      renderSystem.update(world, 0);
      if (sceneData?.bumperFxOverlay) {
        sceneData.bumperFxOverlay.update(world, renderSystem);
      }
    }
  }

  function stepPausedFrame() {
    const pauseState = world.getResource('pauseState');
    const dt = world.getResource('dt');
    if (!pauseState || !pauseState.paused) {
      return;
    }
    pauseState.paused = false;
    world.update(dt);
    pauseState.paused = true;
    doStep = false;
    renderCurrentState();
  }

  const requestSingleStep = () => {
    const pauseState = world.getResource('pauseState');
    if (pauseState && pauseState.paused) {
      doStep = true;
      if (manualLoopMode) {
        stepPausedFrame();
      }
    }
  };

  function gameLoop(currentTime) {
    const dt = world.getResource('dt');
    if (dtEl && dtEl.textContent === 'N/A') {
      dtEl.textContent = `${(dt * 1000).toFixed(2)}ms`;
    }

    const pauseState = world.getResource('pauseState');

    if (lastTime === 0) {
      lastTime = currentTime;
    }

    const speedScale = window._flipperSpeedScale ?? 1.0;
    const frameSec = speedScale * (currentTime - lastTime) / 1000;
    let simTimeProcessed = 0;

    if (frameSec >= dt) {
      lastTime = currentTime;
      const maxSteps = window._flipperMaxSubSteps ?? 500;
      const maxAccum = dt * maxSteps;
      accumulator = Math.min(accumulator + frameSec, maxAccum);

      while (accumulator >= dt) {
        const shouldStepWhilePaused = doStep || stepWhileKeyHeld;
        if (!pauseState.paused || shouldStepWhilePaused) {
          const steppingWhilePaused = pauseState.paused && shouldStepWhilePaused;
          if (steppingWhilePaused) pauseState.paused = false;
          world.update(dt);
          simTimeProcessed += dt;
          if (steppingWhilePaused) pauseState.paused = true;
          if (doStep) doStep = false;
        }

        if (pauseState.paused && !stepWhileKeyHeld) {
          accumulator = 0;
          break;
        }

        accumulator -= dt;
      }
    }

    if (frameSec > 1e-6 && simTimeProcessed > 0) {
      const currentSpeed = simTimeProcessed / frameSec;
      speedSamples.push(currentSpeed);
      if (speedSamples.length > numSpeedSamples) {
        speedSamples.shift();
      }
    }

    frameCounter += 1;
    if (frameCounter % 10 === 0 && speedEl && speedSamples.length > 0) {
      const avgSpeed = speedSamples.reduce((a, b) => a + b, 0) / speedSamples.length;
      speedEl.textContent = `${avgSpeed.toFixed(2)}x`;
    }

    renderCurrentState();
  }

  function resetLoopTiming() {
    lastTime = 0;
    accumulator = 0;
    speedSamples = [];
    frameCounter = 0;
    if (speedEl) speedEl.textContent = 'N/A';
  }

  pauseBtn.addEventListener('click', (event) => {
    event.preventDefault();
    const pauseState = world.getResource('pauseState');
    if (!pauseState) {
      return;
    }

    pauseState.paused = !pauseState.paused;
    pauseBtn.textContent = pauseState.paused ? 'Resume' : 'Pause';
    if (debugEnabled) {
      console.debug('[flipper3d] pause toggle', { paused: pauseState.paused });
    }

    if (!pauseState.paused) {
      lastTime = performance.now();
    }
  });

  resetBtn.addEventListener('click', (event) => {
    event.preventDefault();
    setupScene(sceneData);

    for (const system of world.systems) {
      if (system instanceof InputSystem || system instanceof InputReplaySystem) {
        if (typeof system.reset === 'function') {
          system.reset();
        }
      }
    }

    resetLoopTiming();

    const pauseState = world.getResource('pauseState');
    if (pauseState) {
      pauseState.paused = true;
    }

    pauseBtn.textContent = 'Start';
    doStep = false;
    stepWhileKeyHeld = false;
    if (manualLoopMode) {
      renderCurrentState();
    }
    if (debugEnabled) {
      console.debug('[flipper3d] reset', { paused: true });
    }
  });

  stepBtn.addEventListener('click', (event) => {
    event.preventDefault();
    requestSingleStep();
    if (debugEnabled) {
      console.debug('[flipper3d] step requested');
    }
  });

  document.addEventListener('keydown', (event) => {
    const targetTag = event.target && event.target.tagName;
    const isEditableTarget = targetTag === 'INPUT' || targetTag === 'TEXTAREA' || targetTag === 'SELECT' || event.target?.isContentEditable;
    if (isEditableTarget || event.repeat) {
      return;
    }
    const key = typeof event.key === 'string' ? event.key.toLowerCase() : '';
    if (key === 's' || key === 't') {
      event.preventDefault();
      stepWhileKeyHeld = true;
      requestSingleStep();
    }
  });

  document.addEventListener('keyup', (event) => {
    const targetTag = event.target && event.target.tagName;
    const isEditableTarget = targetTag === 'INPUT' || targetTag === 'TEXTAREA' || targetTag === 'SELECT' || event.target?.isContentEditable;
    if (isEditableTarget) {
      return;
    }
    const key = typeof event.key === 'string' ? event.key.toLowerCase() : '';
    if (key === 's' || key === 't') {
      stepWhileKeyHeld = false;
    }
  });

  dumpBtn.addEventListener('click', (event) => {
    event.preventDefault();
    if (typeof window.dumpFlipper3DState === 'function') {
      window.dumpFlipper3DState(world);
      return;
    }
    console.log(world);
  });

  setupScene(sceneData);

  const pauseState = world.getResource('pauseState');
  pauseBtn.textContent = pauseState.paused ? 'Start' : 'Pause';

  if (manualLoopMode) {
    renderCurrentState();
    return;
  }

  const renderSystem = getRenderSystem();
  if (renderSystem && typeof renderSystem.setAnimationLoop === 'function') {
    renderSystem.setAnimationLoop(gameLoop);
    return;
  }

  const rafLoop = (time) => {
    gameLoop(time);
    requestAnimationFrame(rafLoop);
  };
  requestAnimationFrame(rafLoop);
}
