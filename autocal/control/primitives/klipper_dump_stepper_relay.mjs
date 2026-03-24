const STEP_PIN_AXIS_MAP = {
  'gpiochip1/gpio0': 'A',
  'gpiochip1/gpio3': 'B',
  'gpiochip1/gpio6': 'C',
  'gpiochip1/gpio9': 'E',
};

const DEFAULT_AXIS_ORDER = ['A', 'B', 'C', 'E'];
const DEFAULT_CLOCK_HZ = 50_000_000;
const DEFAULT_BUCKET_DURATION_MS = 2;
const DEFAULT_SPEED_SCALE = 1;
const DEFAULT_LIVE_LEAD_MS = 0;
const POSITION_EPSILON = 1e-12;

function parseKvPairs(text) {
  const out = {};
  for (const part of text.trim().split(/\s+/)) {
    const match = part.match(/^([a-zA-Z_]+)=(.+)$/);
    if (!match) {
      continue;
    }
    const [, key, rawVal] = match;
    const num = Number(rawVal);
    out[key] = Number.isNaN(num) ? rawVal : num;
  }
  return out;
}

function stepperNameToAxis(stepperName) {
  if (typeof stepperName !== 'string' || !stepperName) {
    return null;
  }
  const normalized = stepperName.trim().toLowerCase();
  if (normalized === 'extruder' || normalized.startsWith('extruder')) {
    return 'E';
  }
  const match = normalized.match(/^stepper_([a-z])$/);
  if (match) {
    const suffix = match[1];
    if (suffix === 'a') return 'A';
    if (suffix === 'b') return 'B';
    if (suffix === 'c') return 'C';
    if (suffix === 'd') return 'E';
  }
  const suffix = normalized.at(-1);
  if (suffix === 'a' || suffix === 'b' || suffix === 'c') {
    return suffix.toUpperCase();
  }
  if (suffix === 'd') {
    return 'E';
  }
  return null;
}

function normalizeBatchData(batch) {
  if (!batch || !Array.isArray(batch.data)) {
    return [];
  }
  return batch.data
    .map((entry) => {
      if (!Array.isArray(entry) || entry.length < 3) {
        return null;
      }
      const interval = Number(entry[0]);
      const count = Number(entry[1]);
      const add = Number(entry[2]);
      if (!Number.isFinite(interval) || !Number.isFinite(count) || !Number.isFinite(add)) {
        return null;
      }
      return {
        interval: Math.max(1, Math.trunc(interval)),
        count: Math.max(0, Math.trunc(count)),
        add: Math.trunc(add),
      };
    })
    .filter(Boolean);
}

function getClockHz(clockState) {
  return Number.isFinite(clockState.clockHz) && clockState.clockHz > 0
    ? clockState.clockHz
    : DEFAULT_CLOCK_HZ;
}

function getPrintTimeBase(printTimeBase) {
  if (!printTimeBase) {
    return null;
  }
  const sampleWallMs = Number(printTimeBase.sampleWallMs);
  const samplePrintTime = Number(printTimeBase.samplePrintTime);
  if (!Number.isFinite(sampleWallMs) || !Number.isFinite(samplePrintTime)) {
    return null;
  }
  return {
    sampleWallMs,
    samplePrintTime,
  };
}

function mapPrintTimeToWallMs(printTime, printTimeBase, {
  speedScale = DEFAULT_SPEED_SCALE,
  nowMs = performance.now(),
} = {}) {
  const base = getPrintTimeBase(printTimeBase);
  const scale = Number.isFinite(speedScale) && speedScale > 0 ? speedScale : DEFAULT_SPEED_SCALE;
  if (!base || !Number.isFinite(printTime)) {
    return nowMs;
  }
  const mapped = base.sampleWallMs + ((printTime - base.samplePrintTime) * 1000) / scale;
  return Number.isFinite(mapped) ? mapped : nowMs;
}

function bucketAtMs(atMs, bucketDurationMs) {
  const duration = Number.isFinite(bucketDurationMs) && bucketDurationMs > 0
    ? bucketDurationMs
    : DEFAULT_BUCKET_DURATION_MS;
  const bucket = Math.floor(Math.max(0, atMs) / duration) * duration;
  return Number.isFinite(bucket) ? bucket : 0;
}

export function expandStepperDumpToEvents({
  axis,
  batch,
  dirSign = 1,
  currentPosition = 0,
  clockHz = DEFAULT_CLOCK_HZ,
  printTimeBase = null,
  speedScale = DEFAULT_SPEED_SCALE,
  nowMs = performance.now(),
} = {}) {
  const data = normalizeBatchData(batch);
  const stepDistance = Number(batch?.step_distance);
  const firstStepTime = Number(batch?.first_step_time ?? batch?.first_time);

  if (!axis || data.length === 0 || !Number.isFinite(stepDistance) || stepDistance <= 0 || !Number.isFinite(firstStepTime)) {
    return [];
  }

  const hz = getClockHz({ clockHz });
  const events = [];
  let position = Number.isFinite(currentPosition) ? currentPosition : 0;
  let stepPrintTime = firstStepTime - (data[0]?.interval || 1) / hz;

  for (const segment of data) {
    let interval = segment.interval;
    for (let i = 0; i < segment.count; i += 1) {
      interval = Math.max(1, interval);
      stepPrintTime += interval / hz;
      position += dirSign * stepDistance;
      const command = { type: 'Move' };
      if (axis === 'E') {
        command.E = dirSign * stepDistance;
      } else {
        command[axis] = position;
      }
      const atMs = mapPrintTimeToWallMs(stepPrintTime, printTimeBase, {
        speedScale,
        nowMs,
      });
      events.push({
        axis,
        atMs,
        command,
        position,
        stepPrintTime,
      });
      interval = Math.max(1, interval + segment.add);
    }
  }

  return events;
}

export function createKlipperDumpStepperRelay(options = {}) {
  const onCommand = typeof options.onCommand === 'function' ? options.onCommand : null;
  const nowFn = typeof options.nowFn === 'function' ? options.nowFn : () => performance.now();
  const setTimeoutImpl = typeof options.setTimeoutImpl === 'function' ? options.setTimeoutImpl : setTimeout;
  const clearTimeoutImpl = typeof options.clearTimeoutImpl === 'function' ? options.clearTimeoutImpl : clearTimeout;
  const bucketDurationMs = Number.isFinite(options.bucketDurationMs) && options.bucketDurationMs > 0
    ? options.bucketDurationMs
    : DEFAULT_BUCKET_DURATION_MS;
  const leadMs = Number.isFinite(options.leadMs) && options.leadMs >= 0
    ? options.leadMs
    : DEFAULT_LIVE_LEAD_MS;

  const clockState = {
    clockHz: DEFAULT_CLOCK_HZ,
  };
  const printTimeBase = {
    sampleWallMs: null,
    samplePrintTime: null,
  };
  const axisByOid = new Map();
  const stepperToAxis = new Map();
  const axisStates = new Map();
  const bucketMap = new Map();
  const bucketOrder = [];
  let timer = null;
  let timerTargetMs = null;
  let speedScale = Number.isFinite(options.speedScale) && options.speedScale > 0
    ? options.speedScale
    : DEFAULT_SPEED_SCALE;
  let asapMode = Boolean(options.asapMode);

  function ensureAxisState(axis) {
    let state = axisStates.get(axis);
    if (!state) {
      state = {
        position: 0,
        rawPosition: null,
        dir: 1,
        stepDistance: null,
        initialized: false,
      };
      axisStates.set(axis, state);
    }
    return state;
  }

  function resolveAxisFromStepperName(stepperName) {
    if (stepperToAxis.has(stepperName)) {
      return stepperToAxis.get(stepperName);
    }
    const guessed = stepperNameToAxis(stepperName);
    if (guessed) {
      stepperToAxis.set(stepperName, guessed);
      ensureAxisState(guessed);
      return guessed;
    }
    return null;
  }

  function resolveAxisFromOid(oid) {
    return axisByOid.get(Number(oid)) || null;
  }

  function insertBucketKey(key) {
    let lo = 0;
    let hi = bucketOrder.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (bucketOrder[mid] < key) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    bucketOrder.splice(lo, 0, key);
  }

  function ensureBucket(atMs) {
    const key = bucketAtMs(atMs, bucketDurationMs);
    let bucket = bucketMap.get(key);
    if (!bucket) {
      bucket = {
        atMs: key,
        add: null,
        move: null,
      };
      bucketMap.set(key, bucket);
      insertBucketKey(key);
    }
    return bucket;
  }

  function rescheduleTimerIfNeeded() {
    if (timer == null || bucketOrder.length === 0) {
      return;
    }
    const nextTarget = Math.max(0, bucketOrder[0] - leadMs);
    if (timerTargetMs === null || nextTarget + 1e-6 < timerTargetMs) {
      clearTimeoutImpl(timer);
      timer = null;
      timerTargetMs = null;
      armTimer();
    }
  }

  function scheduleAddReference(axis, delta, atMs) {
    if (!Number.isFinite(delta) || Math.abs(delta) <= POSITION_EPSILON) {
      return;
    }
    const bucket = ensureBucket(atMs);
    if (!bucket.add) {
      bucket.add = {};
    }
    bucket.add[axis] = (bucket.add[axis] || 0) + delta;
    rescheduleTimerIfNeeded();
    armTimer();
  }

  function scheduleMove(axis, value, deltaE, atMs) {
    const bucket = ensureBucket(atMs);
    if (!bucket.move) {
      bucket.move = { type: 'Move' };
    }
    if (axis === 'E') {
      bucket.move.E = (bucket.move.E || 0) + deltaE;
    } else {
      bucket.move[axis] = value;
    }
    rescheduleTimerIfNeeded();
    armTimer();
  }

  function emitBucket(bucket) {
    if (!bucket) {
      return;
    }
    if (bucket.add && onCommand) {
      const addCmd = { type: 'Add to reference', ...bucket.add };
      onCommand(addCmd);
    }
    if (bucket.move && onCommand) {
      const moveCmd = { ...bucket.move };
      onCommand(moveCmd);
    }
  }

  function flushDueCommands(nowMs = nowFn(), force = false) {
    if (timer != null) {
      clearTimeoutImpl(timer);
      timer = null;
      timerTargetMs = null;
    }
    const threshold = force ? Infinity : nowMs + (asapMode ? 0 : leadMs);
    while (bucketOrder.length > 0 && bucketOrder[0] <= threshold) {
      const key = bucketOrder.shift();
      const bucket = bucketMap.get(key);
      bucketMap.delete(key);
      emitBucket(bucket);
    }
    armTimer();
  }

  function armTimer() {
    if (timer != null || bucketOrder.length === 0) {
      return;
    }
    const nextTarget = Math.max(0, bucketOrder[0] - (asapMode ? 0 : leadMs));
    timerTargetMs = nextTarget;
    const delay = Math.max(0, nextTarget - nowFn());
    timer = setTimeoutImpl(() => {
      timer = null;
      timerTargetMs = null;
      flushDueCommands(nowFn(), false);
    }, delay);
  }

  function handleParsedLines(lines) {
    if (!Array.isArray(lines)) {
      return;
    }
    for (const line of lines) {
      if (typeof line !== 'string') {
        continue;
      }
      const trimmed = line.trim();
      if (!trimmed) {
        continue;
      }
      if (trimmed.startsWith('config_stepper')) {
        const kv = parseKvPairs(trimmed.slice('config_stepper'.length));
        const oid = Number(kv.oid);
        if (!Number.isFinite(oid)) {
          continue;
        }
        let axis = STEP_PIN_AXIS_MAP[kv.step_pin];
        if (!axis || Array.from(axisByOid.values()).includes(axis)) {
          axis = DEFAULT_AXIS_ORDER.find((candidate) => !Array.from(axisByOid.values()).includes(candidate)) || null;
        }
        if (axis) {
          axisByOid.set(oid, axis);
          ensureAxisState(axis);
        }
        continue;
      }
      if (trimmed.startsWith('set_next_step_dir')) {
        const kv = parseKvPairs(trimmed.slice('set_next_step_dir'.length));
        const axis = resolveAxisFromOid(kv.oid);
        if (!axis) {
          continue;
        }
        const state = ensureAxisState(axis);
        state.dir = Number(kv.dir) === 0 ? -1 : 1;
        continue;
      }
      if (trimmed.startsWith('set_position')) {
        const kv = parseKvPairs(trimmed.slice('set_position'.length));
        const axis = resolveAxisFromOid(kv.oid);
        if (!axis) {
          continue;
        }
        const rawPos = Number(kv.pos);
        if (!Number.isFinite(rawPos)) {
          continue;
        }
        const state = ensureAxisState(axis);
        state.rawPosition = rawPos;
        if (Number.isFinite(state.stepDistance) && state.stepDistance > 0) {
          const nextPosition = rawPos * state.stepDistance;
          const delta = nextPosition - state.position;
          state.position = nextPosition;
          state.initialized = true;
          scheduleAddReference(axis, delta, nowFn());
        }
      }
    }
  }

  function handleClockMessage(msg = {}) {
    if (Number.isFinite(msg.clock_hz) && msg.clock_hz > 0) {
      clockState.clockHz = msg.clock_hz;
    }
  }

  function handleMotionReportStatus(payload = {}) {
    const status = payload.status || payload?.result?.status || null;
    const toolhead = status?.toolhead || null;
    const samplePrintTime = Number.isFinite(toolhead?.estimated_print_time)
      ? Number(toolhead.estimated_print_time)
      : Number.isFinite(toolhead?.print_time)
        ? Number(toolhead.print_time)
        : null;
    if (Number.isFinite(samplePrintTime)) {
      printTimeBase.sampleWallMs = nowFn();
      printTimeBase.samplePrintTime = samplePrintTime;
    }
  }

  function feedStepperDump(stepperName, batch = {}) {
    const axis = resolveAxisFromStepperName(stepperName);
    if (!axis) {
      return;
    }
    const state = ensureAxisState(axis);
    const batchStepDistance = Number(batch.step_distance);
    if (Number.isFinite(batchStepDistance) && batchStepDistance > 0) {
      state.stepDistance = batchStepDistance;
    }
    const startPosition = Number(batch.start_position);
    if (Number.isFinite(startPosition)) {
      const delta = startPosition - state.position;
      state.position = startPosition;
      state.initialized = true;
      if (Math.abs(delta) > POSITION_EPSILON) {
        scheduleAddReference(axis, delta, nowFn());
      }
    } else if (!state.initialized && Number.isFinite(state.rawPosition) && Number.isFinite(state.stepDistance)) {
      state.position = state.rawPosition * state.stepDistance;
      state.initialized = true;
    }

    const events = expandStepperDumpToEvents({
      axis,
      batch,
      dirSign: state.dir || 1,
      currentPosition: state.position,
      clockHz: clockState.clockHz,
      printTimeBase,
      speedScale,
      nowMs: nowFn(),
    });

    for (const event of events) {
      state.position = event.position;
      scheduleMove(axis, event.command[axis] ?? null, event.command.E ?? 0, event.atMs);
    }

    flushDueCommands(nowFn(), false);
  }

  function setSpeedScale(value) {
    if (!Number.isFinite(value) || value <= 0) {
      speedScale = DEFAULT_SPEED_SCALE;
      return;
    }
    speedScale = value;
    flushDueCommands(nowFn(), false);
  }

  function setAsapMode(enable) {
    asapMode = Boolean(enable);
    flushDueCommands(nowFn(), false);
  }

  function close() {
    if (timer != null) {
      clearTimeoutImpl(timer);
      timer = null;
      timerTargetMs = null;
    }
    bucketMap.clear();
    bucketOrder.length = 0;
  }

  return {
    handleParsedLines,
    handleClockMessage,
    handleMotionReportStatus,
    feedStepperDump,
    setSpeedScale,
    setAsapMode,
    flushDueCommands,
    close,
  };
}
