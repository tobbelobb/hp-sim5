import fs from 'node:fs';
import path from 'node:path';
import { mapStepperNameToAxis } from './klipperMotionCore.js';

export const DEFAULT_M569_FIRST_ADDRESS = 40;
export const DEFAULT_M569_AXIS_ORDER = ['A', 'B', 'C', 'D', 'E', 'I', 'J', 'K', 'L', 'O'];

function stripComment(line) {
  const commentIndex = line.search(/[#;]/);
  if (commentIndex < 0) {
    return line;
  }
  return line.slice(0, commentIndex);
}

export function parseMotorAddressDescriptor(rawValue, fallbackCanAddress = DEFAULT_M569_FIRST_ADDRESS) {
  if (rawValue == null) {
    return { canAddress: fallbackCanAddress, driver: 0 };
  }

  const text = String(rawValue).trim();
  if (!text) {
    return { canAddress: fallbackCanAddress, driver: 0 };
  }

  const [canPart, driverPart] = text.split('.', 2);
  const canAddress = Number.parseInt(canPart, 10);
  if (!Number.isFinite(canAddress)) {
    return { canAddress: fallbackCanAddress, driver: 0 };
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

export function buildDefaultDriverToAxisMap({
  axisOrder = DEFAULT_M569_AXIS_ORDER,
  firstAddress = DEFAULT_M569_FIRST_ADDRESS,
} = {}) {
  const map = new Map();
  axisOrder.forEach((axis, index) => {
    map.set(firstAddress + index, axis);
  });
  return map;
}

export function buildDriverToAxisMapFromConfig(configPath, {
  axisOrder = DEFAULT_M569_AXIS_ORDER,
  firstAddress = DEFAULT_M569_FIRST_ADDRESS,
} = {}) {
  const merged = buildDefaultDriverToAxisMap({ axisOrder, firstAddress });
  if (typeof configPath !== 'string' || configPath.trim().length === 0) {
    return merged;
  }

  const visited = new Set();
  const parsed = [];

  const parseFile = (filePath) => {
    const resolvedPath = path.resolve(filePath);
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
      const axis = mapStepperNameToAxis(currentSection);
      if (!axis) {
        return;
      }
      const rawAddress = currentOptions.get('m569_address');
      const descriptor = parseMotorAddressDescriptor(
        rawAddress,
        firstAddress + parsed.length,
      );
      parsed.push({
        section: currentSection,
        axis,
        descriptor,
      });
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
            parseFile(path.resolve(baseDir, includeSpec));
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
  };

  parseFile(configPath);

  for (const entry of parsed) {
    merged.set(entry.descriptor.canAddress, entry.axis);
  }
  return merged;
}
