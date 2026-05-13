import { detectFileFormat, FileFormat, isKlipperFormat, isRrfFormat } from '../../integrations/shared/fileFormatUtils.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../integrations/rrf/rrfFirmwareModel.js';
import { DEFAULT_PRESET_KEY } from './appState.js';
import {
  PRESET_GCODE_MAP,
  describeSelectedPresetFile,
  getPresetActionLabel,
  resolvePresetCommand,
} from './commandPresetResolver.js';
import { RRF_CONFIG_VARIANTS_BY_USDA_KEY, buildPublicProjectUrl } from './machineCatalog.js';
import { RemoteSpoolSystem } from './remoteSpoolSystem.js';
import { KLIPPER_UPLOAD_PIPELINE, shouldUseRawKlipperUploadPipeline } from './workerController.js';

function getJobLabel(descriptor) {
  if (!descriptor) {
    return 'Unknown Job';
  }
  if (typeof descriptor.label === 'string' && descriptor.label.trim().length > 0) {
    return descriptor.label;
  }
  if (typeof descriptor.name === 'string' && descriptor.name.trim().length > 0) {
    return descriptor.name;
  }
  if (typeof descriptor.key === 'string' && descriptor.key.trim().length > 0) {
    return descriptor.key;
  }
  return 'Program';
}

export function createCommandJobController({
  window,
  world,
  state,
  dom,
  machines,
  inspectionTools,
  workers,
  quality,
  runtime,
  view,
  uploadPresetConfig,
} = {}) {
  const asapState = {
    active: false,
    finishingPromise: null,
    previousTimeScale: null,
    previousQualityEnabled: null,
    previousQualityToggleDisabled: null,
    previousQualityToggleChecked: null,
    previousPauseState: null,
    prevPauseDisabled: null,
    prevResetDisabled: null,
    renderSuspended: false,
    pendingFinalCheck: false,
  };

  function getRemoteSystem() {
    return world.systems.find((sys) => sys instanceof RemoteSpoolSystem) || null;
  }

  function showPrintStatus(message) {
    if (!dom.printStatusEl) {
      return;
    }
    dom.printStatusEl.textContent = message;
    dom.printStatusEl.classList.remove('sim-hidden');
  }

  function hidePrintStatus() {
    if (!dom.printStatusEl) {
      return;
    }
    dom.printStatusEl.textContent = '';
    dom.printStatusEl.classList.add('sim-hidden');
  }

  function showAsapStatus(message = 'Finishing print ASAP...') {
    if (!dom.asapStatusEl) {
      return;
    }
    dom.asapStatusEl.textContent = message;
    dom.asapStatusEl.classList.remove('sim-hidden');
  }

  function hideAsapStatus() {
    if (!dom.asapStatusEl) {
      return;
    }
    dom.asapStatusEl.textContent = '';
    dom.asapStatusEl.classList.add('sim-hidden');
  }

  function updateFinishAsapButtonState() {
    view?.setButtonDisabled?.(dom.finishAsapBtn, !state.printActive || asapState.active);
  }

  function setPrintActive(active) {
    state.printActive = Boolean(active);
    view?.updatePrintingState?.();
    if (!state.printActive) {
      hidePrintStatus();
      hideAsapStatus();
    }
    updateFinishAsapButtonState();
  }

  function isPrintActive() {
    return Boolean(state.printActive);
  }

  function beginJob(descriptor = null) {
    state.jobSequenceCounter += 1;
    state.activeJobId = state.jobSequenceCounter;
    state.currentJobDescriptor = descriptor ? { ...descriptor } : null;
  }

  function resetJobTracking() {
    state.activeJobId = null;
    state.currentJobDescriptor = null;
  }

  function recordFinalQualityAndResetJob() {
    const recorded = quality?.runFinalQualityChecks?.({
      activeJobId: state.activeJobId,
      lastRecordedJobId: state.lastRecordedJobId,
      currentJobDescriptor: state.currentJobDescriptor,
    });
    if (state.activeJobId != null) {
      state.lastRecordedJobId = state.activeJobId;
    }
    resetJobTracking();
    return recorded;
  }

  function finishJob() {
    setPrintActive(false);
    if (asapState.active) {
      asapState.pendingFinalCheck = true;
    } else {
      recordFinalQualityAndResetJob();
    }
  }

  function stopAndClearWorkers() {
    workers.stopAndClearWorkers();
  }

  function handleWorkerError() {
    setPrintActive(false);
  }

  function maybeResumeFromPause() {
    runtime.resume();
  }

  function setReferenceVisibilityForReset() {
    inspectionTools?.setVisibility?.(false);
  }

  function handleUserReset() {
    if (!runtime.getGameControls() || machines.getMachines().length === 0) {
      return;
    }
    if (asapState.active) {
      console.warn('hp-sim-3d: Reset ignored while Finish ASAP is active.');
      return;
    }
    setPrintActive(false);
    stopAndClearWorkers();
    resetJobTracking();
    runtime.reset({ autoPause: true });
    runtime.setTimeScale(1.0);
    view?.resetViewStateDefaults?.();
    view?.setNavigationCursorActive?.(false);
    view?.resetCanvasTapTracking?.();
    view?.setPanMode?.(false);
    view?.reapplyViewState?.({ clearExtrusions: true });
    quality?.resetQualityMonitors?.({ keepReference: true });
    inspectionTools?.setVisibility?.(false);
    state.currentPresetKey = DEFAULT_PRESET_KEY;
  }

  function ensureReadyForNewJob() {
    if (state.printActive) {
      handleUserReset();
    }
  }

  function startSimulationWithWorker(worker, jobDescriptor = null) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return false;
    }
    workers.stopInactiveWorkers(worker);
    workers.resetRemoteQueue(worker);
    if (worker) {
      try {
        worker.postMessage({ type: 'resume' });
      } catch (err) {
        console.warn('hp-sim-3d: unable to resume worker before starting job.', err);
      }
      worker.postMessage({ type: 'set_speed_scale', value: state.currentTimeScale });
    }
    if (runtime.getGameControls()) {
      runtime.reset({ autoPause: false });
      view?.reapplyViewState?.({ clearExtrusions: true });
    }
    quality?.resetQualityMonitors?.({ keepReference: true });
    setPrintActive(true);
    hidePrintStatus();
    resetJobTracking();
    beginJob(jobDescriptor);
    return true;
  }

  function resolveActiveRrfConfigPath() {
    for (const sourceKey of machines.getActiveSourceKeys()) {
      const variants = RRF_CONFIG_VARIANTS_BY_USDA_KEY[sourceKey];
      if (!variants) {
        continue;
      }
      return state.lineLayeringEnabled && variants.lineLayered ? variants.lineLayered : variants.default;
    }
    return null;
  }

  async function configureRrfWorkerForActiveMachine(worker) {
    if (!worker || typeof worker.postMessage !== 'function') {
      return;
    }
    const configPath = resolveActiveRrfConfigPath();
    if (!configPath) {
      return;
    }
    try {
      const response = await fetch(buildPublicProjectUrl(configPath));
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      const driverToAxis = parseRrfMotorAxisMapFromConfigText(await response.text());
      if (driverToAxis.size === 0) {
        return;
      }
      worker.postMessage({
        type: 'set_driver_to_axis',
        driverToAxis: Array.from(driverToAxis.entries()),
      });
    } catch (error) {
      console.warn(`hp-sim-3d: unable to load RRF motor map from ${configPath}.`, error);
    }
  }

  async function runPreset(presetKey) {
    ensureReadyForNewJob();
    if (!runtime.isStageReady()) {
      return;
    }
    const preset = resolvePresetCommand(presetKey, state.lineLayeringEnabled, machines.getActiveSourceKeys());
    if (!preset?.url) {
      console.warn('hp-sim-3d: unknown preset', presetKey);
      return;
    }
    console.info(`hp-sim-3d: ${getPresetActionLabel(presetKey)} selected file`, describeSelectedPresetFile(preset.url));
    const referencePresetKey = preset.referencePresetKey || presetKey;
    inspectionTools?.loadForPreset?.(referencePresetKey, { setActive: true }).catch((error) => {
      console.warn('hp-sim-3d: failed to prepare reference path for preset', referencePresetKey, error);
    });
    const format = preset.format || detectFileFormat(preset.url);
    let worker = null;
    if (format === FileFormat.GCODE) {
      worker = workers.ensureMoveWorker();
    } else if (isKlipperFormat(format)) {
      worker = workers.ensureKlipperMcuCommandPlayerWorker();
    } else if (isRrfFormat(format)) {
      worker = workers.ensureRrfWorker();
    } else {
      console.warn('hp-sim-3d: unsupported preset format', format);
      return;
    }
    state.currentPresetKey = referencePresetKey;
    const jobDescriptor = {
      type: 'preset',
      key: presetKey,
      label: PRESET_GCODE_MAP[referencePresetKey]?.label || presetKey,
    };
    if (!startSimulationWithWorker(worker, jobDescriptor)) {
      return;
    }
    if (state.simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: state.simDtSec });
    }
    if (isRrfFormat(format)) {
      await configureRrfWorkerForActiveMachine(worker);
    }
    worker.postMessage({ type: 'filename_fetch', filename: preset.url });
  }

  async function queueCommandFile(file, detectedFormat = null) {
    if (!file) {
      return;
    }
    const format = detectedFormat ?? detectFileFormat(file.name);
    if (format === FileFormat.USD_STAGE) {
      ensureReadyForNewJob();
      await machines.addUploadedMachine(file);
      return;
    }
    if (!runtime.isStageReady()) {
      console.warn('hp-sim-3d: unable to queue print job because no scene is loaded.');
      return;
    }
    ensureReadyForNewJob();
    let worker = null;
    if (format === FileFormat.GCODE) {
      worker = workers.ensureMoveWorker();
    } else if (isKlipperFormat(format)) {
      worker = shouldUseRawKlipperUploadPipeline(KLIPPER_UPLOAD_PIPELINE)
        ? workers.createKlipperRawUploadBridge()
        : workers.ensureKlipperMcuCommandPlayerWorker();
    } else if (isRrfFormat(format)) {
      worker = workers.ensureRrfWorker();
    } else {
      console.warn('hp-sim-3d: unsupported upload format', file?.name || 'unknown');
      return;
    }
    const jobDescriptor = {
      type: 'upload',
      name: file?.name || 'Uploaded File',
      label: file?.name || 'Uploaded File',
      format,
    };
    if (!startSimulationWithWorker(worker, jobDescriptor)) {
      return;
    }
    if (state.simDtSec != null) {
      worker.postMessage({ type: 'set_dt', dt: state.simDtSec });
    }
    if (isRrfFormat(format)) {
      await configureRrfWorkerForActiveMachine(worker);
    }
    worker.postMessage({ type: 'filename_upload', filename: file, format });
  }

  function getPresetKeyForUploadFile(file) {
    if (!file?.name) {
      return null;
    }
    const name = file.name.trim();
    const extension = name.includes('.') ? name.slice(name.lastIndexOf('.')).toLowerCase() : '';
    if (!uploadPresetConfig.extensionSet.has(extension)) {
      return null;
    }
    const normalized = name.toLowerCase();
    for (const entry of uploadPresetConfig.presets) {
      if (entry?.substring && entry?.presetKey && normalized.includes(entry.substring.toLowerCase())) {
        return entry.presetKey;
      }
    }
    return null;
  }

  async function handleUpload(file) {
    if (!file) {
      return;
    }
    const matchedPresetKey = getPresetKeyForUploadFile(file);
    if (matchedPresetKey) {
      const descriptor = PRESET_GCODE_MAP[matchedPresetKey];
      const label = descriptor?.label || matchedPresetKey;
      console.log(`hp-sim-3d: upload "${file.name}" matched preset "${matchedPresetKey}" (${label}).`);
      void inspectionTools?.loadForPreset?.(matchedPresetKey, { setActive: true });
    }
    const detectedFormat = detectFileFormat(file.name);
    if (detectedFormat === FileFormat.GCODE) {
      const wantsSimulation = window.confirm('Do you want to simulate a print of this G-code?\nPress OK to simulate, or Cancel to draw the toolpath only.');
      if (!wantsSimulation) {
        ensureReadyForNewJob();
      }
      await inspectionTools?.loadFromFile?.(file, { setActive: true, makeVisible: !wantsSimulation });
      if (wantsSimulation) {
        await queueCommandFile(file, detectedFormat);
      }
      return;
    }
    await queueCommandFile(file, detectedFormat);
  }

  function applyTimeScaleChange(scale, { show = false } = {}) {
    state.speedStatusArmed = show || state.speedStatusArmed;
    const applied = runtime.setTimeScale(scale);
    workers.applyTimeScale(applied);
    if (state.speedStatusArmed) {
      view?.showSpeedStatus?.(applied);
    }
    return applied;
  }

  function handleRuntimeTimeScaleChange(scale) {
    workers.applyTimeScale(scale);
    if (state.speedStatusArmed) {
      view?.showSpeedStatus?.(scale);
    }
  }

  function waitForAnimationFrames(count = 1) {
    const target = Math.max(0, Math.floor(count));
    if (target <= 0) {
      return Promise.resolve();
    }
    return new Promise((resolve) => {
      let remaining = target;
      const step = () => {
        remaining -= 1;
        if (remaining <= 0) {
          resolve();
          return;
        }
        requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    });
  }

  async function runAsapFastForward(remoteSystem) {
    if (!remoteSystem) {
      return;
    }
    const checkIntervalMs = 25;
    const maxIdleChecks = 10;
    const maxDurationMs = 10 * 60 * 1000;
    const startTime = performance.now();
    let idleChecks = 0;
    const queueLengthAccessor = typeof remoteSystem.getQueueLength === 'function'
      ? () => remoteSystem.getQueueLength()
      : () => (Array.isArray(remoteSystem.commands) ? remoteSystem.commands.length : 0);
    while (asapState.active) {
      const queueLen = queueLengthAccessor();
      const workerActive = Boolean(remoteSystem.worker);
      if (!state.printActive && queueLen === 0) {
        idleChecks += 1;
        if (idleChecks >= maxIdleChecks) {
          break;
        }
      } else {
        idleChecks = 0;
      }
      if (!workerActive && queueLen === 0) {
        break;
      }
      if (performance.now() - startTime > maxDurationMs) {
        console.warn('hp-sim-3d: Finish ASAP exceeded expected duration; continuing to finalize.');
        break;
      }
      await new Promise((resolve) => setTimeout(resolve, checkIntervalMs));
    }
    await waitForAnimationFrames(8);
  }

  async function finalizeAsapMode() {
    const remoteSystem = getRemoteSystem();
    const renderSystem = world.getResource('renderSystem');
    runtime.setRenderEveryNth(null);
    remoteSystem?.worker?.postMessage?.({ type: 'set_asap_mode', enable: false });
    if (asapState.renderSuspended) {
      renderSystem?.setDrawingSuspended?.(false);
      renderSystem?.update?.(world, 0);
    }
    const pauseState = world.getResource('pauseState');
    if (pauseState != null && typeof asapState.previousPauseState === 'boolean') {
      pauseState.paused = asapState.previousPauseState;
    }
    if (asapState.prevPauseDisabled !== null) {
      view?.setButtonDisabled?.(dom.pauseBtn, asapState.prevPauseDisabled);
    }
    if (asapState.prevResetDisabled !== null) {
      view?.setButtonDisabled?.(dom.resetBtn, asapState.prevResetDisabled);
    }
    if (dom.qualityToggle) {
      dom.qualityToggle.disabled = Boolean(asapState.previousQualityToggleDisabled);
      dom.qualityToggle.toggleAttribute('aria-disabled', dom.qualityToggle.disabled);
      if (asapState.previousQualityToggleChecked != null) {
        dom.qualityToggle.checked = asapState.previousQualityToggleChecked;
      }
    }
    if (asapState.previousQualityEnabled !== null) {
      quality?.setQualityEnabledState?.(asapState.previousQualityEnabled);
      if (asapState.previousQualityEnabled) {
        quality?.refreshAllQualityMonitors?.(true);
      } else {
        quality?.updateQualityHudVisibility?.();
      }
    }
    if (asapState.previousTimeScale != null) {
      state.speedStatusArmed = false;
      applyTimeScaleChange(asapState.previousTimeScale);
    }
    hideAsapStatus();
    asapState.active = false;
    asapState.finishingPromise = null;
    asapState.previousTimeScale = null;
    asapState.previousQualityEnabled = null;
    asapState.previousQualityToggleDisabled = null;
    asapState.previousQualityToggleChecked = null;
    asapState.previousPauseState = null;
    asapState.prevPauseDisabled = null;
    asapState.prevResetDisabled = null;
    asapState.renderSuspended = false;
    const shouldRunFinalCheck = asapState.pendingFinalCheck;
    asapState.pendingFinalCheck = false;
    if (shouldRunFinalCheck) {
      recordFinalQualityAndResetJob();
    }
    updateFinishAsapButtonState();
  }

  async function triggerFinishAsap() {
    if (asapState.active) {
      return asapState.finishingPromise;
    }
    if (!state.printActive) {
      showAsapStatus('Finish ASAP is available only while printing.');
      setTimeout(() => {
        if (!asapState.active) {
          hideAsapStatus();
        }
      }, 1500);
      return null;
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      console.warn('hp-sim-3d: Finish ASAP requested but remote system is unavailable.');
      return null;
    }
    asapState.active = true;
    asapState.pendingFinalCheck = false;
    asapState.previousTimeScale = runtime.getTimeScale();
    asapState.previousQualityEnabled = quality?.enabled ?? false;
    asapState.previousQualityToggleDisabled = dom.qualityToggle ? dom.qualityToggle.disabled : null;
    asapState.previousQualityToggleChecked = dom.qualityToggle ? dom.qualityToggle.checked : null;
    const pauseState = world.getResource('pauseState');
    asapState.previousPauseState = pauseState ? pauseState.paused : null;
    asapState.prevPauseDisabled = dom.pauseBtn ? dom.pauseBtn.disabled : null;
    asapState.prevResetDisabled = dom.resetBtn ? dom.resetBtn.disabled : null;
    if (pauseState) {
      pauseState.paused = false;
    }
    view?.setButtonDisabled?.(dom.pauseBtn, true);
    view?.setButtonDisabled?.(dom.resetBtn, true);
    showAsapStatus();
    quality?.setQualityEnabledState?.(false);
    if (dom.qualityToggle) {
      dom.qualityToggle.checked = false;
      dom.qualityToggle.disabled = true;
      dom.qualityToggle.setAttribute('aria-disabled', 'true');
    }
    const renderSystem = world.getResource('renderSystem');
    if (renderSystem?.setDrawingSuspended) {
      renderSystem.setDrawingSuspended(true);
      asapState.renderSuspended = true;
    }
    const asapScale = Math.min(512, Math.max(256, Math.max(1, asapState.previousTimeScale || 1) * 8));
    state.speedStatusArmed = false;
    applyTimeScaleChange(asapScale);
    runtime.setRenderEveryNth(Number.POSITIVE_INFINITY);
    remoteSystem.worker?.postMessage?.({ type: 'set_asap_mode', enable: true });
    updateFinishAsapButtonState();
    asapState.finishingPromise = (async () => {
      try {
        await runAsapFastForward(remoteSystem);
      } catch (error) {
        console.error('hp-sim-3d: Finish ASAP failed.', error);
      } finally {
        await finalizeAsapMode();
      }
    })();
    return asapState.finishingPromise;
  }

  function ensureExternalKlipperApiBridge() {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return null;
    }
    const bridge = workers.getKlipperRawUploadBridge() || workers.createKlipperRawUploadBridge();
    if (remoteSystem.worker !== bridge) {
      remoteSystem.worker = bridge;
    }
    setPrintActive(true);
    maybeResumeFromPause();
    return bridge;
  }

  function pushExternalCommands(commands) {
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem || !Array.isArray(commands) || commands.length === 0) {
      return false;
    }
    for (const command of commands) {
      remoteSystem.addCommand(command);
    }
    setPrintActive(true);
    maybeResumeFromPause();
    return true;
  }

  function bindUi() {
    dom.printLogoBtn?.addEventListener('click', () => runPreset('hangprinterLogo'));
    dom.printSquareBtn?.addEventListener('click', () => runPreset('straightMoves'));
    if (dom.uploadBtn && dom.gcodeInput) {
      dom.uploadBtn.addEventListener('click', () => dom.gcodeInput.click());
      dom.gcodeInput.addEventListener('change', (event) => {
        const file = event.target.files?.[0];
        if (!file) {
          return;
        }
        handleUpload(file).catch((error) => {
          console.error('hp-sim-3d: upload handling failed.', error);
        });
        dom.gcodeInput.value = '';
      });
    }
    dom.resetBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      event.stopImmediatePropagation();
      handleUserReset();
    }, { capture: true });
    dom.finishAsapBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      void triggerFinishAsap();
    });
    dom.speedSlowerBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      if (!runtime.isStageReady() || !runtime.getGameControls()?.setTimeScale) {
        return;
      }
      state.speedStatusArmed = true;
      applyTimeScaleChange(runtime.getTimeScale() / 2, { show: true });
    });
    dom.speedFasterBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      if (!runtime.isStageReady() || !runtime.getGameControls()?.setTimeScale) {
        return;
      }
      state.speedStatusArmed = true;
      applyTimeScaleChange(runtime.getTimeScale() * 2, { show: true });
    });
    setPrintActive(false);
    updateFinishAsapButtonState();
    quality?.updateQualityHistoryUI?.();
    dom.qualityHistoryToggleBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      quality?.toggleQualityHistory?.();
    });
  }

  return {
    bindUi,
    runPreset,
    handleUpload,
    queueCommandFile,
    stopAndClearWorkers,
    beginJob,
    finishJob,
    resetJobTracking,
    setPrintActive,
    isPrintActive,
    handleUserReset,
    hidePrintStatus,
    showPrintStatus,
    showAsapStatus,
    hideAsapStatus,
    updateFinishAsapButtonState,
    handleWorkerError,
    handleRuntimeTimeScaleChange,
    applyTimeScaleChange,
    getRemoteSystem,
    ensureExternalKlipperApiBridge,
    pushExternalCommands,
    setReferenceVisibilityForReset,
    getJobLabel: () => getJobLabel(state.currentJobDescriptor),
  };
}
