export const MACHINE_TYPE_ALIASES = Object.freeze({
  hp3: 'hangprinter_3', hp4: 'hangprinter_4', v3: 'hangprinter_3', v4: 'hangprinter_4',
  four_high_anchors: 'skycam', cube_corners: 'cubecorners',
});

export const RRF_SIM_CONFIGS = Object.freeze(Object.fromEntries(
  Object.entries({ hangprinter_3: 'hp3', hangprinter_4: 'hp4', skycam: 'skycam', cubecorners: 'cubecorners', slideprinter: 'slideprinter' })
    .map(([type, name]) => [type, Object.freeze({ default: `sys/config_${name}.g`, lineLayer: `sys/config_${name}_w_line_layers.g` })])
));

export function normalizeMachineType(machineType, fallback = 'slideprinter') {
  const raw = typeof machineType === 'string' && machineType.trim().length > 0
    ? machineType.trim().toLowerCase()
    : fallback;
  return MACHINE_TYPE_ALIASES[raw] || raw;
}

export function resolveRrfSimulatorConfig(machineType, { preferLineLayerConfig = false } = {}) {
  const normalized = normalizeMachineType(machineType);
  const configSet = RRF_SIM_CONFIGS[normalized];
  if (!configSet) throw new Error(`No default RRF simulator config for ${normalized}`);
  if (preferLineLayerConfig && typeof configSet.lineLayer === 'string' && configSet.lineLayer.length > 0) {
    return configSet.lineLayer;
  }
  return configSet.default;
}
