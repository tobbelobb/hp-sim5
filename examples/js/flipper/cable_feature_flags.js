export const CABLE_FEATURE_FLAG_DEFS = Object.freeze([
  { key: 'enableLayering', label: 'enableLayering', defaultValue: true },
  { key: 'layeringCableBaseRadius', label: 'cable/baseRadius', defaultValue: true },
  { key: 'layeringCableStoredLayerRadius', label: 'cable/storedLayerRadius', defaultValue: true },
  { key: 'layeringFrictionEffectiveRadius', label: 'cable/frictionRadius', defaultValue: true },
  { key: 'layeringCollisionOverlayRadius', label: 'collision/overlayRadius', defaultValue: true },
  { key: 'layeringCollisionOverlayRamp', label: 'collision/overlayRamp', defaultValue: true },
  { key: 'layeringCollisionCircleSectors', label: 'collision/circleSectors', defaultValue: true },
  { key: 'layeringCollisionSectorSolvers', label: 'collision/sectorSolvers', defaultValue: true },
  { key: 'layeringCollisionPinchShare', label: 'collision/pinchShare', defaultValue: true },
  { key: 'layeringVelocityContactOffset', label: 'contact/velocityOffset', defaultValue: true },
  { key: 'layeringObstacleRawHitFilter', label: 'obstacle/rawHitFilter', defaultValue: true },
  { key: 'layeringAttachmentUpdatePoints', label: 'cable/updateAttachment', defaultValue: true },
  { key: 'layeringMergeJoints', label: 'cable/mergeJoints', defaultValue: true },
  { key: 'layeringSplitJoints', label: 'cable/splitJoints', defaultValue: true },
  { key: 'layeringSplitQualityGuard', label: 'cable/splitQualityGuard', defaultValue: false },
  { key: 'layeringHybridLinkStates', label: 'cable/hybridLinkStates', defaultValue: true },
  { key: 'layeringClampJointRestLength', label: 'cable/clampRestLength', defaultValue: true },
  { key: 'layeringRenderWraps', label: 'render/layeredWraps', defaultValue: true },
  { key: 'layeringRenderCollisionOverlays', label: 'render/collisionOverlays', defaultValue: false },
  { key: 'renderBumperHitFx', label: 'render/bumperHitFx', defaultValue: true },
  { key: 'scoreBounceSoundEnabled', label: 'audio/scoreBounce', defaultValue: true }
]);

export const CABLE_FEATURE_DEFAULTS = Object.freeze(
  CABLE_FEATURE_FLAG_DEFS.reduce((acc, def) => {
    acc[def.key] = def.defaultValue;
    return acc;
  }, {})
);

function _toCanonicalFlagKey(key) {
  if (key === 'layering') {
    return 'enableLayering';
  }
  return key;
}

function _readWorldBool(world, key, fallback) {
  const value = world?.getResource?.(key);
  return typeof value === 'boolean' ? value : fallback;
}

function _collectPatch(partial = {}) {
  const patch = {};
  if (!partial || typeof partial !== 'object') {
    return patch;
  }
  for (const [rawKey, value] of Object.entries(partial)) {
    const key = _toCanonicalFlagKey(rawKey);
    if (!Object.prototype.hasOwnProperty.call(CABLE_FEATURE_DEFAULTS, key)) {
      continue;
    }
    if (typeof value === 'boolean') {
      patch[key] = value;
    }
  }
  return patch;
}

export function getCableFeatureFlags(world) {
  const flags = {};
  for (const def of CABLE_FEATURE_FLAG_DEFS) {
    flags[def.key] = _readWorldBool(world, def.key, def.defaultValue);
  }
  flags.layering = flags.enableLayering;
  return flags;
}

export function ensureCableFeatureFlags(world) {
  const flags = getCableFeatureFlags(world);
  for (const def of CABLE_FEATURE_FLAG_DEFS) {
    world.setResource(def.key, flags[def.key]);
  }
  return getCableFeatureFlags(world);
}

export function setCableFeatureFlags(world, partial = {}) {
  const next = getCableFeatureFlags(world);
  const patch = _collectPatch(partial);
  for (const [key, value] of Object.entries(patch)) {
    next[key] = value;
  }
  for (const def of CABLE_FEATURE_FLAG_DEFS) {
    world.setResource(def.key, next[def.key]);
  }
  return getCableFeatureFlags(world);
}

export function bindCableFeatureFlagCheckboxes(world, container) {
  if (!container) {
    return {
      inputs: new Map(),
      syncFromWorld: () => {},
      detach: () => {}
    };
  }

  const activeListeners = [];
  const inputs = new Map();

  container.innerHTML = '';
  container.style.display = 'flex';
  container.style.flexWrap = 'wrap';
  container.style.gap = '4px 10px';
  container.style.justifyContent = 'center';
  container.style.marginTop = '6px';

  ensureCableFeatureFlags(world);

  for (const def of CABLE_FEATURE_FLAG_DEFS) {
    const label = document.createElement('label');
    label.htmlFor = `cableFlag_${def.key}`;
    label.style.whiteSpace = 'nowrap';
    label.style.fontSize = '0.78rem';

    const input = document.createElement('input');
    input.id = `cableFlag_${def.key}`;
    input.type = 'checkbox';
    input.checked = _readWorldBool(world, def.key, def.defaultValue);
    input.style.marginRight = '4px';

    const onChange = () => {
      setCableFeatureFlags(world, { [def.key]: input.checked });
    };
    input.addEventListener('change', onChange);
    activeListeners.push(() => input.removeEventListener('change', onChange));

    label.appendChild(input);
    label.appendChild(document.createTextNode(def.label));
    container.appendChild(label);
    inputs.set(def.key, input);
  }

  const syncFromWorld = () => {
    const flags = getCableFeatureFlags(world);
    for (const def of CABLE_FEATURE_FLAG_DEFS) {
      const input = inputs.get(def.key);
      if (input) {
        input.checked = flags[def.key];
      }
    }
  };

  return {
    inputs,
    syncFromWorld,
    detach: () => {
      for (const removeListener of activeListeners) {
        removeListener();
      }
    }
  };
}
