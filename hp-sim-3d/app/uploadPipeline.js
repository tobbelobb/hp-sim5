export const DEFAULT_UPLOAD_PRESET_MATCHES = Object.freeze([
  { substring: 'Hangprinter_logo6', presetKey: 'hangprinterLogo' },
  { substring: 'draw_squares_bigger', presetKey: 'straightMovesBigger' },
  { substring: 'draw_squares', presetKey: 'straightMoves' },
]);

export const DEFAULT_UPLOAD_PRESET_EXTENSIONS = Object.freeze(['.txt', '.serial', '.csv', '.can']);

export function parseUploadPresetMappings(value) {
  if (typeof value !== 'string' || value.trim().length === 0) {
    return DEFAULT_UPLOAD_PRESET_MATCHES;
  }
  const entries = [];
  const normalized = value
    .split(',')
    .map((segment) => segment.trim())
    .filter((segment) => segment.length > 0);
  for (const segment of normalized) {
    const [name, key] = segment.split('=').map((entry) => entry.trim());
    if (name && key) {
      entries.push({ substring: name, presetKey: key });
    }
  }
  return entries.length > 0 ? entries : DEFAULT_UPLOAD_PRESET_MATCHES;
}

export function parseUploadPresetExtensions(value) {
  if (typeof value !== 'string' || value.trim().length === 0) {
    return DEFAULT_UPLOAD_PRESET_EXTENSIONS;
  }
  const entries = value
    .split(',')
    .map((segment) => segment.trim().toLowerCase())
    .filter((segment) => segment.length > 0)
    .map((segment) => (segment.startsWith('.') ? segment : `.${segment}`));
  return entries.length > 0 ? entries : DEFAULT_UPLOAD_PRESET_EXTENSIONS;
}

export function buildUploadPresetConfig(inputElement) {
  const dataset = inputElement?.dataset;
  const presets = parseUploadPresetMappings(dataset?.referencePresets);
  const extensions = parseUploadPresetExtensions(dataset?.referenceExtensions);
  return {
    presets,
    extensionSet: new Set(extensions),
  };
}

