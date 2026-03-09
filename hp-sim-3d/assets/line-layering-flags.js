export const LINE_LAYERING_TOGGLE_KEYS = Object.freeze([
  'enableLayering',
  'layeringFrictionEffectiveRadius',
  'layeringRenderWraps',
]);

export function setLineLayeringFeatureFlags(world, enabled) {
  if (world === null || world === undefined || typeof world.setResource !== 'function') {
    return;
  }
  const next = enabled === true;
  for (const key of LINE_LAYERING_TOGGLE_KEYS) {
    world.setResource(key, next);
  }
}
