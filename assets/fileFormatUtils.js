export const FileFormat = {
  GCODE: 'gcode',
  MCU_TEXT: 'mcu-text',
  MCU_SERIAL: 'mcu-serial',
};

const EXTENSION_MAP = new Map([
  ['.gcode', FileFormat.GCODE],
  ['.gc', FileFormat.GCODE],
  ['.txt', FileFormat.MCU_TEXT],
  ['.log', FileFormat.MCU_TEXT],
  ['.serial', FileFormat.MCU_SERIAL],
]);

export function detectFileFormat(name) {
  if (typeof name !== 'string') {
    return null;
  }
  const trimmed = name.trim();
  if (!trimmed) {
    return null;
  }
  const lower = trimmed.toLowerCase();
  for (const [ext, format] of EXTENSION_MAP) {
    if (lower.endsWith(ext)) {
      return format;
    }
  }
  return null;
}

export function isMcuFormat(format) {
  return format === FileFormat.MCU_TEXT || format === FileFormat.MCU_SERIAL;
}
