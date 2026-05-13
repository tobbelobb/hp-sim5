import { OpenText as UsdOpenText, getAttribute } from '../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../src/js/usd/cable_scene_baker.js';
import { setupScene } from './setupScene.js';

async function fetchUsdaText(url) {
  const response = await fetch(url, { cache: 'no-cache' });
  if (!response.ok) {
    throw new Error(`Failed to fetch USDA from ${url}: ${response.status}`);
  }
  const text = await response.text();
  if (text.trim().startsWith('<')) {
    throw new Error(`Fetch for ${url} returned HTML, not a USDA file.`);
  }
  return text;
}

function colorArrayToHex(colorArr) {
  if (!Array.isArray(colorArr) || colorArr.length < 3) {
    return null;
  }
  const numeric = colorArr.map((value) => Number(value));
  if (!numeric.every((num) => Number.isFinite(num))) {
    return null;
  }
  const maxComponent = Math.max(...numeric.map((num) => Math.abs(num)));
  const scale = maxComponent > 1 ? 255 : 1;
  const clamp = (num) => {
    if (scale === 255) {
      return Math.min(Math.max(num, 0), 255);
    }
    return Math.min(Math.max(num, 0), 1);
  };
  const toHex = (num) => {
    const clamped = clamp(num);
    const scaled = scale === 255 ? clamped : clamped * 255;
    return Math.round(scaled).toString(16).padStart(2, '0');
  };
  return `#${toHex(colorArr[0])}${toHex(colorArr[1])}${toHex(colorArr[2])}`;
}

function hexToRgb(hex) {
  if (typeof hex !== 'string') {
    return null;
  }
  const match = hex.trim().toLowerCase().match(/^#?([0-9a-f]{6})$/);
  if (!match) {
    return null;
  }
  const value = parseInt(match[1], 16);
  return {
    r: (value >> 16) & 0xff,
    g: (value >> 8) & 0xff,
    b: value & 0xff,
  };
}

function rgbToHex({ r, g, b }) {
  const clampChannel = (channel) => Math.min(255, Math.max(0, Math.round(channel)));
  const toHex = (channel) => clampChannel(channel).toString(16).padStart(2, '0');
  return `#${toHex(r)}${toHex(g)}${toHex(b)}`;
}

function mixColors(baseHex, tintHex, amount = 0.5) {
  const base = hexToRgb(baseHex);
  const tint = hexToRgb(tintHex);
  if (!base || !tint) {
    return baseHex;
  }
  const mixFactor = Math.min(Math.max(Number.isFinite(amount) ? amount : 0.5, 0), 1);
  return rgbToHex({
    r: base.r * (1 - mixFactor) + tint.r * mixFactor,
    g: base.g * (1 - mixFactor) + tint.g * mixFactor,
    b: base.b * (1 - mixFactor) + tint.b * mixFactor,
  });
}

function createTintPalette(tintHex) {
  if (!tintHex) {
    return null;
  }
  return {
    spool: mixColors('#a0a0a0', tintHex, 0.65),
    anchor: mixColors('#aaaaaa', tintHex, 0.55),
    pinhole: mixColors('#cccccc', tintHex, 0.5),
    cable: mixColors('#ffff00', tintHex, 0.6),
    distanceConstraint: mixColors('#00ff00', tintHex, 0.55),
    rigidGroup: mixColors('#55ff88', tintHex, 0.55),
  };
}

function getParentPath(path) {
  if (typeof path !== 'string') {
    return null;
  }
  const trimmed = path.trim();
  if (!trimmed || trimmed === '/') {
    return null;
  }
  const separatorIndex = trimmed.lastIndexOf('/');
  return separatorIndex <= 0 ? '/' : trimmed.slice(0, separatorIndex);
}

function findScenePrimPath(stage) {
  if (!stage) {
    return '/World/SlideprinterScene';
  }
  const defaultPrimName = stage?.ast?.descriptor?.defaultPrim;
  if (typeof defaultPrimName === 'string' && defaultPrimName.length > 0) {
    for (const candidate of [`/${defaultPrimName}`, `/World/${defaultPrimName}`]) {
      if (stage.GetPrimAtPath(candidate)) {
        return candidate;
      }
    }
  }
  try {
    for (const [path, prim] of stage.Traverse()) {
      if (!path || !prim || prim.type !== 'definition') {
        continue;
      }
      const tags = getAttribute(prim, 'ecs:tags');
      if ((Array.isArray(tags) && tags.includes('Spool')) || prim.defType === 'CableJoint') {
        const parentPath = getParentPath(path);
        if (parentPath && stage.GetPrimAtPath(parentPath)) {
          return parentPath;
        }
      }
    }
  } catch (error) {
    console.warn('hp-sim-3d: failed to derive scene root, falling back to default.', error);
  }
  return '/World/SlideprinterScene';
}

function extractMachineColors(stage, scenePrimPath) {
  const result = { tintColor: null, extrusionColor: null };
  const prim = scenePrimPath ? stage?.GetPrimAtPath(scenePrimPath) : null;
  if (!prim) {
    return result;
  }
  const tintValue = getAttribute(prim, 'machine:tintColor') ?? getAttribute(prim, 'machine:tint');
  const extrusionValue = getAttribute(prim, 'machine:extrusionColor');
  result.tintColor = tintValue ? colorArrayToHex(tintValue) : null;
  result.extrusionColor = extrusionValue ? colorArrayToHex(extrusionValue) : null;
  return result;
}

function extractTimeCodesPerSecond(stage) {
  const assignments = stage?.ast?.descriptor?.assignments;
  if (!Array.isArray(assignments)) {
    return null;
  }
  const match = assignments.find((entry) => entry?.type === 'assignment' && entry?.identifier === 'timeCodesPerSecond');
  return match?.value ?? null;
}

export function createMachineSceneController({
  document,
  world,
  state,
  dom,
  catalog,
  defaultSourceKey,
  sceneQueue,
  featureFlags,
  quality,
  runtime,
  view,
  getCommands = () => null,
  getExternal = () => null,
} = {}) {
  const presetOptionInputs = new Map();
  const presetOptionLabels = new Map();
  const presetOptionColorChips = new Map();

  function getMachines() {
    return state.machines;
  }

  function getActiveSourceKeys() {
    return state.machines.map((machine) => machine.sourceKey).filter(Boolean);
  }

  function getMachineById(machineOrId) {
    if (machineOrId && typeof machineOrId === 'object') {
      return machineOrId;
    }
    return state.machines.find((machine) => machine.id === machineOrId) || null;
  }

  function getMachineDisplayName(machineOrId) {
    const machine = getMachineById(machineOrId);
    if (!machine) {
      return 'Machine';
    }
    if (machine.name) {
      return machine.name;
    }
    if (machine.sourceKey && catalog.has(machine.sourceKey)) {
      return catalog.get(machine.sourceKey)?.label || machine.id || 'Machine';
    }
    return machine.id || 'Machine';
  }

  function bakeUsdaSourceForLineLayering(sourceText, enabled) {
    const options = enabled ? {} : { cablePathHalfWidthOverride: 0.0, deriveAll: true };
    return bakeCableSceneUsdaSource(sourceText, options).source;
  }

  function openBakedUsdaStage(sourceText, enabled = featureFlags.lineLayeringEnabled) {
    return UsdOpenText(bakeUsdaSourceForLineLayering(sourceText, enabled));
  }

  function refreshBakedStage(machine) {
    if (!machine?.sourceText) {
      return true;
    }
    try {
      machine.stage = openBakedUsdaStage(machine.sourceText, featureFlags.lineLayeringEnabled);
      machine.scenePrimPath = findScenePrimPath(machine.stage);
      return true;
    } catch (error) {
      console.error(`hp-sim-3d: unable to bake USDA source for ${machine.name || machine.id}.`, error);
      return false;
    }
  }

  function refreshBakedStages() {
    return state.machines.every((machine) => refreshBakedStage(machine));
  }

  function loadCatalogUsdaSource(sourceKey) {
    if (!sourceKey || !catalog.has(sourceKey)) {
      return Promise.resolve(null);
    }
    const entry = catalog.get(sourceKey);
    if (typeof entry.sourceText === 'string') {
      return Promise.resolve(entry.sourceText);
    }
    if (entry.sourcePromise) {
      return entry.sourcePromise;
    }
    entry.sourcePromise = fetchUsdaText(entry.url)
      .then((sourceText) => {
        entry.sourceText = sourceText;
        return sourceText;
      })
      .finally(() => {
        entry.sourcePromise = null;
      });
    return entry.sourcePromise;
  }

  function createColorChip() {
    const chip = document.createElement('span');
    chip.className = 'sim-machines-option-color';
    chip.setAttribute('aria-hidden', 'true');
    return chip;
  }

  function applyColorChipTint(chip, tintHex) {
    if (!chip) {
      return;
    }
    if (typeof tintHex === 'string' && tintHex.trim().length > 0) {
      chip.style.backgroundColor = tintHex;
      chip.classList.remove('sim-machines-option-color--empty');
    } else {
      chip.style.removeProperty('background-color');
      chip.classList.add('sim-machines-option-color--empty');
    }
  }

  function applyPresetMachineTint(sourceKey, tintHex) {
    if (sourceKey && catalog.has(sourceKey)) {
      catalog.get(sourceKey).tintColor = tintHex ?? null;
    }
    for (const machine of state.machines) {
      if (machine.sourceKey !== sourceKey) {
        continue;
      }
      machine.tintColor = tintHex ?? null;
      quality?.ensureQualityMonitorForMachine?.(machine);
    }
    applyColorChipTint(presetOptionColorChips.get(sourceKey), tintHex);
    const label = presetOptionLabels.get(sourceKey);
    if (label) {
      if (tintHex) {
        label.dataset.tintColor = tintHex;
      } else {
        label.removeAttribute('data-tint-color');
      }
    }
  }

  function loadPresetTintColor(sourceKey) {
    if (!sourceKey || !catalog.has(sourceKey)) {
      return Promise.resolve(null);
    }
    const entry = catalog.get(sourceKey);
    if (entry.tintColorLoaded) {
      applyPresetMachineTint(sourceKey, entry.tintColor);
      return Promise.resolve(entry.tintColor);
    }
    if (entry.tintColorPromise) {
      return entry.tintColorPromise;
    }
    entry.tintColorPromise = (async () => {
      try {
        const sourceText = await loadCatalogUsdaSource(sourceKey);
        const stage = openBakedUsdaStage(sourceText, true);
        const scenePrimPath = findScenePrimPath(stage);
        entry.tintColor = extractMachineColors(stage, scenePrimPath).tintColor || null;
      } catch (error) {
        console.warn(`hp-sim-3d: unable to read tint color for USDA preset ${sourceKey}.`, error);
        entry.tintColor = null;
      } finally {
        entry.tintColorLoaded = true;
        entry.tintColorPromise = null;
        applyPresetMachineTint(sourceKey, entry.tintColor);
      }
      return entry.tintColor;
    })();
    return entry.tintColorPromise;
  }

  function registerMachine(stage, { name = null, sourceKey = null, sourceUrl = null, sourceText = null } = {}) {
    if (!stage) {
      return null;
    }
    if (sourceKey && state.machines.some((machine) => machine.sourceKey === sourceKey)) {
      return state.machines.find((machine) => machine.sourceKey === sourceKey) || null;
    }
    const machineId = `machine-${state.machineIdCounter++}`;
    const scenePrimPath = findScenePrimPath(stage);
    const { tintColor, extrusionColor } = extractMachineColors(stage, scenePrimPath);
    const machine = {
      id: machineId,
      stage,
      palette: tintColor ? createTintPalette(tintColor) : null,
      tintColor,
      extrusionColor,
      name: name || null,
      scenePrimPath,
      sourceKey: sourceKey || null,
      sourceUrl: sourceUrl || null,
      sourceText: typeof sourceText === 'string' ? sourceText : null,
    };
    if (sourceKey && catalog.has(sourceKey)) {
      const entry = catalog.get(sourceKey);
      entry.tintColor = tintColor || null;
      entry.tintColorLoaded = true;
      entry.tintColorPromise = null;
      applyPresetMachineTint(sourceKey, tintColor || null);
    }
    state.machines.push(machine);
    quality?.ensureQualityMonitorForMachine?.(machine);
    updateMachineMenuUI();
    return machine;
  }

  function rebuildScene() {
    if (!dom.canvas || state.machines.length === 0) {
      return;
    }
    let isFirst = true;
    for (const machine of state.machines) {
      if (!refreshBakedStage(machine)) {
        continue;
      }
      setupScene(world, machine.stage, dom.canvas, {
        remote: false,
        append: !isFirst,
        palette: machine.palette || null,
        scenePrimPath: machine.scenePrimPath,
        namespace: machine.id,
        tintColor: machine.tintColor || null,
        extrusionColor: machine.extrusionColor || null,
      });
      isFirst = false;
    }
    featureFlags.setLineLayeringEnabledState(featureFlags.lineLayeringEnabled, { fromToggle: true });
    const remoteSystem = getCommands()?.getRemoteSystem?.();
    quality?.attachToRemoteSystem?.(remoteSystem);
    getExternal()?.flushQueue?.();
  }

  async function resetAfterMachineListChange({ resetView = false } = {}) {
    if (state.machines.length === 0) {
      quality?.clearQualityMonitors?.();
      quality?.updateQualityHudVisibility?.();
      view?.resetForEmptyScene?.();
      runtime.setStageReady(false);
      view?.setSceneControlsEnabled?.(false);
      getCommands()?.hidePrintStatus?.();
      getCommands()?.stopAndClearWorkers?.();
      getCommands()?.setPrintActive?.(false);
      return;
    }

    if (resetView) {
      view?.resetViewStateDefaults?.();
    }
    rebuildScene();
    view?.syncCanvasDimensions?.();
    runtime.setStageReady(true);
    view?.setSceneControlsEnabled?.(true);
    if (runtime.getGameControls()) {
      getCommands()?.handleUserReset?.();
    } else {
      getCommands()?.stopAndClearWorkers?.();
      getCommands()?.resetJobTracking?.();
      getCommands()?.setPrintActive?.(false);
      view?.reapplyViewState?.({ clearExtrusions: true });
      quality?.resetQualityMonitors?.({ keepReference: true });
      getCommands()?.setReferenceVisibilityForReset?.();
    }
  }

  function updatePresetMachineSelections() {
    if (presetOptionInputs.size === 0) {
      buildPresetMachineOptions();
    }
    for (const [key, checkbox] of presetOptionInputs.entries()) {
      checkbox.checked = state.machines.some((machine) => machine.sourceKey === key);
    }
  }

  function updateCustomMachineList() {
    if (!dom.customMachinesSection || !dom.customMachinesList) {
      return;
    }
    dom.customMachinesList.innerHTML = '';
    const uploads = state.machines.filter((machine) => !machine.sourceKey);
    dom.customMachinesSection.classList.toggle('sim-hidden', uploads.length === 0);
    for (const machine of uploads) {
      const item = document.createElement('li');
      item.className = 'sim-machines-custom-item';
      const info = document.createElement('div');
      info.className = 'sim-machines-custom-info';
      const colorChip = createColorChip();
      applyColorChipTint(colorChip, machine.tintColor);
      const name = document.createElement('span');
      name.className = 'sim-machines-custom-name';
      name.textContent = machine.name || 'Uploaded scene';
      name.title = machine.sourceUrl || machine.name || machine.id;
      info.appendChild(colorChip);
      info.appendChild(name);
      item.appendChild(info);
      const removeBtn = document.createElement('button');
      removeBtn.type = 'button';
      removeBtn.className = 'sim-tertiary sim-machines-remove';
      removeBtn.textContent = 'Remove';
      removeBtn.dataset.machineId = machine.id;
      item.appendChild(removeBtn);
      dom.customMachinesList.appendChild(item);
    }
  }

  function updateRemoveAllButtonState() {
    if (dom.machinesRemoveAllBtn) {
      const disabled = state.machines.length === 0;
      dom.machinesRemoveAllBtn.disabled = disabled;
      dom.machinesRemoveAllBtn.toggleAttribute('aria-disabled', disabled);
    }
  }

  function updateMachineMenuUI() {
    updatePresetMachineSelections();
    updateCustomMachineList();
    updateRemoveAllButtonState();
    if (dom.machinesToggle) {
      const count = state.machines.length;
      dom.machinesToggle.setAttribute(
        'aria-label',
        count === 0 ? 'Machines (no scenes loaded)' : `Machines (${count} scenes loaded)`
      );
    }
  }

  function buildPresetMachineOptions() {
    if (!dom.presetMachinesList) {
      return;
    }
    dom.presetMachinesList.innerHTML = '';
    presetOptionInputs.clear();
    presetOptionLabels.clear();
    presetOptionColorChips.clear();
    for (const [key, entry] of catalog.entries()) {
      const item = document.createElement('li');
      item.className = 'sim-machines-option';
      const label = document.createElement('label');
      label.className = 'sim-machines-option-label';
      label.dataset.sourceKey = key;
      const checkbox = document.createElement('input');
      checkbox.type = 'checkbox';
      checkbox.className = 'sim-machines-checkbox';
      checkbox.dataset.sourceKey = key;
      checkbox.value = key;
      const colorChip = createColorChip();
      applyColorChipTint(colorChip, entry.tintColor);
      const text = document.createElement('span');
      text.className = 'sim-machines-option-text';
      text.textContent = entry.label;
      label.appendChild(checkbox);
      label.appendChild(colorChip);
      label.appendChild(text);
      item.appendChild(label);
      dom.presetMachinesList.appendChild(item);
      presetOptionInputs.set(key, checkbox);
      presetOptionLabels.set(key, label);
      presetOptionColorChips.set(key, colorChip);
      if (!entry.tintColorLoaded) {
        loadPresetTintColor(key).catch((error) => {
          console.warn(`hp-sim-3d: tint color preload failed for ${key}.`, error);
        });
      }
    }
  }

  async function addUploadedMachine(file) {
    if (!file) {
      return null;
    }
    const label = file.name || 'uploaded.usda';
    let sourceText = null;
    try {
      sourceText = await file.text();
    } catch (error) {
      console.error(`hp-sim-3d: failed to read USDA file ${label}:`, error);
      return null;
    }
    let stage = null;
    try {
      stage = openBakedUsdaStage(sourceText, featureFlags.lineLayeringEnabled);
    } catch (error) {
      console.error(`hp-sim-3d: unable to bake/parse USDA file ${label}:`, error);
      return null;
    }
    return sceneQueue.enqueueSceneChange(async () => {
      const machine = registerMachine(stage, { name: label, sourceText });
      if (!machine) {
        return null;
      }
      const timeCodesPerSecond = extractTimeCodesPerSecond(stage);
      if (timeCodesPerSecond) {
        const uploadedDt = 1.0 / timeCodesPerSecond;
        if (state.simDtSec == null) {
          state.simDtSec = uploadedDt;
        } else if (Math.abs(uploadedDt - state.simDtSec) > 1e-6) {
          console.warn(
            `hp-sim-3d: USDA file ${label} uses timeCodesPerSecond=${timeCodesPerSecond}, which differs from the active simulation. Using existing dt=${state.simDtSec.toFixed(6)}s.`
          );
        }
      }
      await resetAfterMachineListChange({ resetView: state.machines.length === 1 });
      return machine;
    });
  }

  async function addCatalogMachine(sourceKey, { resetView = false } = {}) {
    if (!sourceKey || !catalog.has(sourceKey)) {
      return null;
    }
    const existing = state.machines.find((machine) => machine.sourceKey === sourceKey);
    if (existing) {
      return existing;
    }
    const entry = catalog.get(sourceKey);
    let sourceText = null;
    let stage = null;
    try {
      sourceText = await loadCatalogUsdaSource(sourceKey);
      stage = openBakedUsdaStage(sourceText, featureFlags.lineLayeringEnabled);
    } catch (error) {
      console.error(`hp-sim-3d: unable to bake/load USDA preset ${sourceKey}.`, error);
      return null;
    }
    return sceneQueue.enqueueSceneChange(async () => {
      const machine = registerMachine(stage, {
        name: entry.label,
        sourceKey,
        sourceUrl: entry.url,
        sourceText,
      });
      if (!machine) {
        return null;
      }
      const timeCodesPerSecond = extractTimeCodesPerSecond(stage);
      if (timeCodesPerSecond) {
        const presetDt = 1.0 / timeCodesPerSecond;
        if (state.simDtSec == null) {
          state.simDtSec = presetDt;
        } else if (Math.abs(presetDt - state.simDtSec) > 1e-6) {
          console.warn(
            `hp-sim-3d: USDA preset ${sourceKey} uses timeCodesPerSecond=${timeCodesPerSecond}, which differs from the active simulation. Using existing dt=${state.simDtSec.toFixed(6)}s.`
          );
        }
      }
      await resetAfterMachineListChange({ resetView: resetView || state.machines.length === 1 });
      return machine;
    });
  }

  function loadDefaultMachine() {
    if (!catalog.has(defaultSourceKey)) {
      throw new Error(`hp-sim-3d: default USDA '${defaultSourceKey}' is not available.`);
    }
    return addCatalogMachine(defaultSourceKey, { resetView: true });
  }

  function removeMachineById(machineId) {
    return sceneQueue.enqueueSceneChange(async () => {
      const index = state.machines.findIndex((machine) => machine.id === machineId);
      if (index === -1) {
        return;
      }
      const [removedMachine] = state.machines.splice(index, 1);
      quality?.removeQualityMonitor?.(removedMachine.id);
      updateMachineMenuUI();
      await resetAfterMachineListChange({ resetView: state.machines.length === 0 });
    });
  }

  function removeAllMachines() {
    return sceneQueue.enqueueSceneChange(async () => {
      if (state.machines.length === 0) {
        return;
      }
      state.machines.splice(0, state.machines.length);
      quality?.clearQualityMonitors?.();
      updateMachineMenuUI();
      await resetAfterMachineListChange({ resetView: true });
    });
  }

  function bindUi() {
    buildPresetMachineOptions();
    updateMachineMenuUI();
    dom.presetMachinesList?.addEventListener('change', (event) => {
      const target = event.target;
      if (!(target instanceof HTMLInputElement) || target.type !== 'checkbox') {
        return;
      }
      const sourceKey = target.dataset?.sourceKey;
      if (!sourceKey) {
        return;
      }
      if (target.checked) {
        target.disabled = true;
        addCatalogMachine(sourceKey, { resetView: state.machines.length === 0 })
          .catch((error) => {
            console.error('hp-sim-3d: failed to add USDA preset from catalog.', error);
            target.checked = false;
          })
          .finally(() => {
            target.disabled = false;
            updateMachineMenuUI();
          });
        return;
      }
      const machine = state.machines.find((entry) => entry.sourceKey === sourceKey);
      if (machine) {
        void removeMachineById(machine.id);
      } else {
        updateMachineMenuUI();
      }
    });
    dom.customMachinesList?.addEventListener('click', (event) => {
      const button = event.target?.closest?.('button[data-machine-id]');
      const machineId = button?.dataset?.machineId;
      if (machineId) {
        event.preventDefault();
        void removeMachineById(machineId);
      }
    });
    dom.machinesRemoveAllBtn?.addEventListener('click', (event) => {
      event.preventDefault();
      void removeAllMachines();
    });
  }

  return {
    bindUi,
    loadDefaultMachine,
    addCatalogMachine,
    addUploadedMachine,
    removeMachine: removeMachineById,
    removeAllMachines,
    getMachines,
    getActiveSourceKeys,
    getMachineDisplayName,
    getSimDtSec: () => state.simDtSec,
    rebuildScene,
    refreshBakedStages,
  };
}
