const DEFAULT_DT = 1 / 500;
const DEFAULT_SPEED_SCALE = 1;

function ensureWorkerShims() {
  if (!globalThis.self) {
    globalThis.self = {};
  }
  if (typeof globalThis.self.addEventListener !== 'function') {
    globalThis.self.addEventListener = () => {};
  }
  if (typeof globalThis.postMessage !== 'function') {
    globalThis.postMessage = () => {};
  }
}

function encodeParsedLines(lines) {
  const encoder = new TextEncoder();
  const payload = [];
  for (const line of lines) {
    if (typeof line !== 'string') {
      continue;
    }
    const trimmed = line.trim();
    if (!trimmed) {
      continue;
    }
    payload.push(encoder.encode(`${trimmed}\n`));
  }
  return payload;
}

export async function createKlipperMotionRelay(options = {}) {
  ensureWorkerShims();

  const { KlipperCommander } = await import('../../../examples/js/slideprinter/klipperCommander.js');
  const onCommand = typeof options.onCommand === 'function' ? options.onCommand : null;
  const dt = Number.isFinite(options.dt) && options.dt > 0 ? options.dt : DEFAULT_DT;
  const speedScale = Number.isFinite(options.speedScale) && options.speedScale > 0
    ? options.speedScale
    : DEFAULT_SPEED_SCALE;
  const asapMode = Boolean(options.asapMode);

  const commander = new KlipperCommander();
  commander.setDt(dt);
  commander.setSpeedScale(speedScale);
  commander.setAsapMode(asapMode);
  commander.sendCommand = async (command) => {
    if (!command) {
      return;
    }
    if (onCommand) {
      await onCommand(command);
    }
  };

  let controller = null;
  const stream = new ReadableStream({
    start(streamController) {
      controller = streamController;
    },
  });

  const runPromise = commander.run(stream);

  const feedParsedLines = (lines) => {
    if (!controller || controller.desiredSize === null) {
      return;
    }
    const chunks = Array.isArray(lines) ? encodeParsedLines(lines) : [];
    for (const chunk of chunks) {
      controller.enqueue(chunk);
    }
  };

  const setSpeedScale = (value) => {
    commander.setSpeedScale(value);
  };

  const setAsapMode = (value) => {
    commander.setAsapMode(value);
  };

  const close = () => {
    if (!controller) {
      return;
    }
    try {
      controller.close();
    } catch (_err) {
      // Ignore repeated close attempts.
    }
    controller = null;
  };

  return {
    commander,
    feedParsedLines,
    setSpeedScale,
    setAsapMode,
    close,
    runPromise,
  };
}

