import fs from 'node:fs';
import path from 'node:path';
import { mapStepperNameToAxis } from './klipperMotionCore.js';

function stripComment(line) {
  const commentIndex = line.search(/[#;]/);
  if (commentIndex < 0) {
    return line;
  }
  return line.slice(0, commentIndex);
}

function parseMotorAddress(rawValue) {
  if (rawValue == null) {
    return null;
  }
  const text = String(rawValue).trim();
  if (!text) {
    return null;
  }
  const [canPart, driverPart] = text.split('.', 2);
  const canAddress = Number.parseInt(canPart, 10);
  if (!Number.isFinite(canAddress)) {
    return null;
  }
  let driver = 0;
  if (driverPart !== undefined && driverPart.trim().length > 0) {
    const parsedDriver = Number.parseInt(driverPart, 10);
    if (Number.isFinite(parsedDriver)) {
      driver = parsedDriver;
    }
  }
  return { canAddress, driver };
}

function motorAddressKey(address) {
  if (!address) {
    return null;
  }
  return `${address.canAddress}.${address.driver ?? 0}`;
}

function buildEmptyMotorCatalog() {
  return {
    stepperDescriptors: [],
    stepperByName: new Map(),
    stepperByAddress: new Map(),
    hasConfiguredAddresses: false,
    printerOptions: new Map(),
  };
}

function parseOptionalNumber(rawValue) {
  if (rawValue == null) {
    return null;
  }
  const numeric = Number.parseFloat(String(rawValue).trim());
  return Number.isFinite(numeric) ? numeric : null;
}

function parseOptionalInteger(rawValue) {
  if (rawValue == null) {
    return null;
  }
  const numeric = Number.parseInt(String(rawValue).trim(), 10);
  return Number.isFinite(numeric) ? numeric : null;
}

function parseNumberList(rawValue) {
  if (rawValue == null) {
    return [];
  }
  return String(rawValue)
    .split(/[,:]/)
    .map((part) => Number.parseFloat(part.trim()))
    .filter((value) => Number.isFinite(value));
}

function axisToIndex(axis) {
  if (typeof axis !== 'string' || axis.length === 0) {
    return null;
  }
  const upper = axis[0].toUpperCase();
  const code = upper.charCodeAt(0);
  if (code < 65 || code > 90) {
    return null;
  }
  return code - 65;
}

function parseConfigFile(configPath, catalog, visited = new Set()) {
  const resolvedPath = path.resolve(configPath);
  if (visited.has(resolvedPath)) {
    return;
  }
  visited.add(resolvedPath);

  let source;
  try {
    source = fs.readFileSync(resolvedPath, 'utf8');
  } catch (_error) {
    return;
  }

  const baseDir = path.dirname(resolvedPath);
  let currentSection = null;
  let currentOptions = new Map();

  const flushSection = () => {
    if (!currentSection) {
      return;
    }
    if (currentSection.toLowerCase() === 'printer') {
      catalog.printerOptions = new Map(currentOptions);
      return;
    }
    const axis = mapStepperNameToAxis(currentSection);
    if (!axis) {
      return;
    }
    const motorAddress = parseMotorAddress(currentOptions.get('m569_address'));
    const addressKey = motorAddressKey(motorAddress);
    const descriptor = {
      stepperName: currentSection,
      axis,
      motorAddress,
      addressKey,
      canAddress: motorAddress?.canAddress ?? null,
      driver: motorAddress?.driver ?? null,
      hasMotorAddress: Boolean(motorAddress),
      rotationDistanceMm: parseOptionalNumber(currentOptions.get('rotation_distance')),
      microsteps: parseOptionalInteger(currentOptions.get('microsteps')),
      mechanicalAdvantage: 1,
    };
    catalog.stepperDescriptors.push(descriptor);
    catalog.stepperByName.set(descriptor.stepperName, descriptor);
    if (descriptor.hasMotorAddress) {
      catalog.stepperByAddress.set(addressKey, descriptor);
      catalog.hasConfiguredAddresses = true;
    }
  };

  for (const rawLine of source.split(/\r?\n/)) {
    const line = stripComment(rawLine).trim();
    if (!line) {
      continue;
    }
    const sectionMatch = line.match(/^\[(.+)\]$/);
    if (sectionMatch) {
      flushSection();
      currentSection = sectionMatch[1].trim();
      currentOptions = new Map();

      const includeMatch = currentSection.match(/^include\s+(.+)$/i);
      if (includeMatch) {
        currentSection = null;
        const includeSpec = includeMatch[1].trim();
        if (includeSpec.length > 0) {
          parseConfigFile(path.resolve(baseDir, includeSpec), catalog, visited);
        }
      }
      continue;
    }
    if (!currentSection) {
      continue;
    }
    const optionMatch = line.match(/^([A-Za-z0-9_]+)\s*:\s*(.*)$/);
    if (optionMatch) {
      currentOptions.set(optionMatch[1].toLowerCase(), optionMatch[2].trim());
    }
  }

  flushSection();
  visited.delete(resolvedPath);
}

function applyMechanicalAdvantageFromPrinterConfig(catalog) {
  if (!catalog || !(catalog.printerOptions instanceof Map)) {
    return;
  }
  const rawMechanicalAdvantage = catalog.printerOptions.get('winch_mechanical_advantage');
  const values = parseNumberList(rawMechanicalAdvantage);
  if (values.length === 0) {
    return;
  }
  for (const descriptor of catalog.stepperDescriptors) {
    const axisIndex = axisToIndex(descriptor?.axis);
    if (!Number.isFinite(axisIndex)) {
      continue;
    }
    const value = values[axisIndex];
    if (Number.isFinite(value) && value > 0) {
      descriptor.mechanicalAdvantage = value;
    }
  }
}

export function computeMmPerDegreeFromDescriptor(descriptor) {
  const rotationDistanceMm = Number(descriptor?.rotationDistanceMm);
  if (!Number.isFinite(rotationDistanceMm) || rotationDistanceMm === 0) {
    return null;
  }
  const mechanicalAdvantage = Number(descriptor?.mechanicalAdvantage);
  const ma = Number.isFinite(mechanicalAdvantage) && mechanicalAdvantage > 0
    ? mechanicalAdvantage
    : 1;
  return rotationDistanceMm / (360 * ma);
}

export function buildMotorCatalogFromConfig(configPath) {
  const catalog = buildEmptyMotorCatalog();
  if (typeof configPath !== 'string' || configPath.trim().length === 0) {
    return catalog;
  }
  parseConfigFile(configPath, catalog);
  applyMechanicalAdvantageFromPrinterConfig(catalog);
  return catalog;
}

export function resolveMotorToken(token, catalog) {
  const text = String(token || '').trim();
  if (!catalog || !Array.isArray(catalog.stepperDescriptors)) {
    return { error: `${text} did not match any stepper name or m569_address.` };
  }

  const byName = catalog.stepperByName instanceof Map ? catalog.stepperByName : new Map();
  const byAddress = catalog.stepperByAddress instanceof Map ? catalog.stepperByAddress : new Map();

  const nameMatch = byName.get(text) || null;
  const numericMatch = (() => {
    const motorAddress = parseMotorAddress(text);
    if (!motorAddress) {
      return null;
    }
    return byAddress.get(motorAddressKey(motorAddress)) || null;
  })();

  if (numericMatch) {
    return {
      resolved: numericMatch,
      token: text,
    };
  }

  if (nameMatch) {
    if (catalog.hasConfiguredAddresses && !nameMatch.hasMotorAddress) {
      return { error: `m569_address not configured for ${nameMatch.stepperName}` };
    }
    return {
      resolved: nameMatch,
      token: text,
    };
  }

  return { error: `${text} did not match any stepper name or m569_address.` };
}
