import path from 'node:path';
import { findMachine, machineForScene } from '../../machines/catalog.js';

export function normalizeMachineType(value) {
  switch (String(value || '').trim()) {
    case 'hp3':
    case 'hangprinter3':
    case 'hangprinter_3':
    case 'hangprinter-v3':
    case 'hangprinter_v3':
      return 'hp3';
    case 'hp4':
    case 'hangprinter4':
    case 'hangprinter_4':
    case 'hangprinter-v4':
    case 'hangprinter_v4':
      return 'hp4';
    case 'hp5':
    case 'hangprinter5':
    case 'hangprinter_5':
    case 'hangprinter-v5':
    case 'hangprinter_v5':
      return 'hp5';
    case 'hp5_pulley_reduced':
    case 'hangprinter_5_pulley_reduced':
      return 'hp5_pulley_reduced';
    case 'slideprinter':
    case 'skycam':
    case 'cubecorners':
      return String(value).trim();
    default:
      throw new Error(`unsupported machine type: ${value}`);
  }
}

export function isBuildupFlag(value) {
  return value === '--buildup' || value === '--line-layers';
}

export function isNoBuildupFlag(value) {
  return value === '--no-buildup'
    || value === '--no_buildup'
    || value === 'no_buildup'
    || value === '--no-line-layers';
}

export function normalizeRrfConfigName(value) {
  const text = String(value || '').trim();
  const match = /^(?:sys\/)?config_([A-Za-z0-9_-]+?)(?:_w_line_layers)?\.g$/u.exec(text);
  if (!match) {
    return null;
  }
  const machine = normalizeMachineType(match[1]);
  return {
    machine,
    buildup: text.endsWith('_w_line_layers.g'),
  };
}

export function resolveKlipperConfigFromMachine(machineType, { buildup } = {}) {
  const machine = findMachine(normalizeMachineType(machineType));
  const selected = buildup === undefined
    ? machineForScene(machine.scene, true) ?? machine
    : machineForScene(machine.scene, buildup);
  if (!selected) throw new Error(`${machine.name} has no variant with buildup=${buildup}`);
  return `./${selected.klipper_config}`;
}

export function resolveKlipperConfigFromRrfConfig(value) {
  const parsed = normalizeRrfConfigName(value);
  if (!parsed) {
    return null;
  }
  return resolveKlipperConfigFromMachine(parsed.machine, { buildup: parsed.buildup });
}

export function resolveKlipperConfigSelector(value) {
  const fromRrf = resolveKlipperConfigFromRrfConfig(value);
  if (fromRrf) {
    return fromRrf;
  }
  return value;
}

export function resolveKlipperConfigPath(value, cwd = process.cwd()) {
  const selected = resolveKlipperConfigSelector(value);
  return path.isAbsolute(selected) ? selected : path.resolve(cwd, selected);
}
