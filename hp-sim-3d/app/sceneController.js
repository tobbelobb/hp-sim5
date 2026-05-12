import { runGame } from './runner.js';

export function createSceneChangeQueue({ onError = console.error } = {}) {
  let sceneChangeQueue = Promise.resolve();

  function enqueueSceneChange(task) {
    const run = sceneChangeQueue.then(() => task());
    sceneChangeQueue = run.catch((error) => {
      onError('hp-sim-3d: scene change task failed', error);
    });
    return run;
  }

  return { enqueueSceneChange };
}

export function createSimulationRuntime({ world, state, runGameImpl = runGame } = {}) {
  let timeScaleChangeListener = null;

  function handleTimeScaleChange(scale) {
    const safeScale = Number.isFinite(scale) && scale > 0 ? scale : 1.0;
    state.currentTimeScale = safeScale;
    timeScaleChangeListener?.(safeScale);
  }

  function start(sceneInitializer, { onTimeScaleChange = null } = {}) {
    if (state.gameControls) {
      return state.gameControls;
    }
    timeScaleChangeListener = onTimeScaleChange;
    state.gameControls = runGameImpl(world, sceneInitializer, {
      initialTimeScale: state.currentTimeScale,
      onTimeScaleChange: handleTimeScaleChange,
    });
    return state.gameControls;
  }

  function reset(options = {}) {
    if (state.gameControls && typeof state.gameControls.reset === 'function') {
      state.gameControls.reset(options);
      return true;
    }
    return false;
  }

  function resume() {
    if (state.gameControls && typeof state.gameControls.resume === 'function') {
      state.gameControls.resume();
      return;
    }
    const pauseState = world.getResource('pauseState');
    if (pauseState && pauseState.paused) {
      pauseState.paused = false;
    }
  }

  function setTimeScale(scale) {
    const safeScale = Number.isFinite(scale) && scale > 0 ? scale : 1.0;
    if (state.gameControls && typeof state.gameControls.setTimeScale === 'function') {
      state.gameControls.setTimeScale(safeScale);
      return typeof state.gameControls.getTimeScale === 'function'
        ? state.gameControls.getTimeScale()
        : safeScale;
    }
    handleTimeScaleChange(safeScale);
    return safeScale;
  }

  function getTimeScale() {
    if (state.gameControls && typeof state.gameControls.getTimeScale === 'function') {
      return state.gameControls.getTimeScale();
    }
    return state.currentTimeScale;
  }

  function setStageReady(ready) {
    state.stageReady = Boolean(ready);
  }

  return {
    start,
    reset,
    resume,
    setTimeScale,
    getTimeScale,
    setStageReady,
    getGameControls: () => state.gameControls,
    isStageReady: () => Boolean(state.stageReady),
    setRenderEveryNth(value) {
      state.gameControls?.setRenderEveryNth?.(value);
    },
  };
}
