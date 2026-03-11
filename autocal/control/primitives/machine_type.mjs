export const MACHINE_TYPE_ALIASES = Object.freeze({
  hp3: 'hangprinter_4',
  hp4: 'hangprinter_4',
  hangprinter_3: 'hangprinter_4',
});

export const RRF_SIM_CONFIGS = Object.freeze({
  slideprinter: Object.freeze({
    default: 'sys/config_slideprinter.g',
    lineLayer: 'sys/config_slideprinter_w_line_layers.g',
  }),
  hangprinter_4: Object.freeze({
    default: 'sys/config_hp3.g',
    lineLayer: 'sys/config_hp3_w_line_layers.g',
  }),
});

export function normalizeMachineType(machineType, fallback = 'slideprinter') {
  const raw = typeof machineType === 'string' && machineType.trim().length > 0
    ? machineType.trim().toLowerCase()
    : fallback;
  return MACHINE_TYPE_ALIASES[raw] || raw;
}

export function resolveRrfSimulatorConfig(machineType, { preferLineLayerConfig = false } = {}) {
  const normalized = normalizeMachineType(machineType);
  const configSet = RRF_SIM_CONFIGS[normalized] || RRF_SIM_CONFIGS.slideprinter;
  if (preferLineLayerConfig && typeof configSet.lineLayer === 'string' && configSet.lineLayer.length > 0) {
    return configSet.lineLayer;
  }
  return configSet.default;
}
