export const DEFAULT_PRESET_KEY = 'hangprinterLogo';
export const DEFAULT_VIEW_SCALE = 0.6;

export function createAppState({ qualityEnabled = false, showConstraintForces = false, lineLayeringEnabled = true } = {}) {
  return {
    machines: [],
    machineIdCounter: 0,
    simDtSec: null,
    stageReady: false,
    currentPresetKey: DEFAULT_PRESET_KEY,
    gameControls: null,
    view: {
      scale: DEFAULT_VIEW_SCALE,
      offsetX: 0,
      offsetY: 0,
      offsetZ: 0,
    },
    navigationCursorActive: false,
    panModeActive: false,
    printActive: false,
    fullscreenActive: false,
    currentTimeScale: 1.0,
    speedStatusArmed: false,
    qualityEnabled,
    showConstraintForces,
    lineLayeringEnabled,
    closedLoopMotorsEnabled: false,
    jobSequenceCounter: 0,
    activeJobId: null,
    lastRecordedJobId: null,
    currentJobDescriptor: null,
  };
}

