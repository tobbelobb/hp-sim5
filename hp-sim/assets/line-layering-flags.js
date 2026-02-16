export const LINE_LAYERING_FEATURE_KEYS = Object.freeze([
  'enableLayering',
  'layeringFrictionEffectiveRadius',
  'layeringAttachmentUpdatePoints',
  'layeringMergeJoints',
  'layeringSplitJoints',
  'layeringHybridLinkStates',
  'layeringRenderWraps',
]);

export function setLineLayeringFeatureFlags(world, enabled) {
  if (world === null || world === undefined || typeof world.setResource !== 'function') {
    return;
  }
  const next = enabled === true;
  for (const key of LINE_LAYERING_FEATURE_KEYS) {
    world.setResource(key, next);
  }
}
