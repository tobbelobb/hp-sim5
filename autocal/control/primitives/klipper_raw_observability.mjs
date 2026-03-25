function summarizeNumericMap(command, excludedKeys = new Set()) {
  const axes = {};
  for (const [key, value] of Object.entries(command || {})) {
    if (excludedKeys.has(key)) {
      continue;
    }
    const numericValue = Number(value);
    if (Number.isFinite(numericValue)) {
      axes[key] = numericValue;
    }
  }
  return axes;
}

export function summarizeKlipperCommand(command) {
  if (!command || typeof command !== 'object') {
    return null;
  }

  const summary = {
    type: typeof command.type === 'string' && command.type.trim() ? command.type.trim() : 'unknown',
  };

  const at = Number(command.at);
  if (Number.isFinite(at)) {
    summary.at = at;
  }

  const span = Number(command.span);
  if (Number.isFinite(span)) {
    summary.span = span;
  }

  const axes = summarizeNumericMap(command, new Set(['type', 'at', 'span']));
  if (Object.keys(axes).length > 0) {
    summary.axes = axes;
  }

  return summary;
}

export function summarizeKlipperParsedLines(lines) {
  const normalizedLines = Array.isArray(lines)
    ? lines.filter((line) => typeof line === 'string' && line.length > 0)
    : [];
  if (normalizedLines.length === 0) {
    return { lineCount: 0, firstLine: null, lastLine: null };
  }
  return {
    lineCount: normalizedLines.length,
    firstLine: normalizedLines[0],
    lastLine: normalizedLines[normalizedLines.length - 1],
  };
}

export function summarizeKlipperBridgeMessage(msg) {
  if (!msg || typeof msg !== 'object') {
    return null;
  }

  if (msg.action === 'klipper_parsed') {
    return {
      action: 'klipper_parsed',
      ...summarizeKlipperParsedLines(msg.lines),
    };
  }

  if (msg.action === 'klipper_clock') {
    const summary = { action: 'klipper_clock' };
    for (const key of ['mcu_clock', 'host_time', 'request_host_time', 'round_trip', 'clock_hz']) {
      const numericValue = Number(msg[key]);
      if (Number.isFinite(numericValue)) {
        summary[key] = numericValue;
      }
    }
    return summary;
  }

  if (msg.action === 'klipper_serial') {
    const payload = typeof msg.data === 'string'
      ? msg.data
      : typeof msg.chunk === 'string'
        ? msg.chunk
        : typeof msg.payload === 'string'
          ? msg.payload
          : null;
    return {
      action: 'klipper_serial',
      payloadLength: payload ? payload.length : 0,
    };
  }

  return {
    action: typeof msg.action === 'string' && msg.action.length > 0 ? msg.action : 'unknown',
  };
}
