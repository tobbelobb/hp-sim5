import { performance } from 'node:perf_hooks';
import { WebSocketServer } from 'ws';
import { MCU_CLOCK_HZ_KLIPPER_HOST } from './klipperFirmwareModel.js';

const MAX_PENDING_WS_PAYLOADS = 5000;
const DEFAULT_MOTION_IDLE_MS = 650;
const DEFAULT_MOTION_END_TIME_TOLERANCE_S = 0.010;

function splitTerminalLines(response) {
  return String(response)
    .split(/\r?\n/)
    .map((line) => line.trimEnd())
    .filter((line) => line.length > 0);
}

function buildWsHelpers({
  wsPort,
  quiet = false,
  onClientChange = null,
  onBroadcast = null,
} = {}) {
  const wss = wsPort ? new WebSocketServer({ port: wsPort }) : null;
  const pendingWsPayloads = [];
  let waitingForClientNoticePrinted = false;

  const getReadyWsClients = () => (wss
    ? Array.from(wss.clients).filter((client) => client.readyState === 1)
    : []);

  const hasReadyWsClients = () => getReadyWsClients().length > 0;

  const enqueuePendingPayload = (payload) => {
    if (!wss || !payload) {
      return;
    }
    pendingWsPayloads.push(payload);
    const overflow = pendingWsPayloads.length - MAX_PENDING_WS_PAYLOADS;
    if (overflow > 0) {
      pendingWsPayloads.splice(0, overflow);
    }
    if (!waitingForClientNoticePrinted && !quiet) {
      console.log('Waiting for hp-sim WebSocket client to connect before streaming...');
      waitingForClientNoticePrinted = true;
    }
  };

  const broadcast = (payload) => {
    if (!payload) {
      return;
    }
    if (typeof onBroadcast === 'function') {
      onBroadcast(payload);
    }
    if (!wss) {
      return;
    }
    const readyClients = getReadyWsClients();
    if (readyClients.length === 0) {
      enqueuePendingPayload(payload);
      return;
    }
    const data = JSON.stringify(payload);
    readyClients.forEach((client) => {
      client.send(data);
    });
  };

  const flushPendingPayloads = () => {
    if (!hasReadyWsClients() || pendingWsPayloads.length === 0) {
      return;
    }
    const batch = pendingWsPayloads.splice(0, pendingWsPayloads.length);
    if (!quiet) {
      console.log(`Streaming ${batch.length} queued message${batch.length === 1 ? '' : 's'} to hp-sim client.`);
    }
    batch.forEach((payload) => broadcast(payload));
    waitingForClientNoticePrinted = false;
  };

  if (wss) {
    wss.on('connection', (socket) => {
      flushPendingPayloads();
      if (onClientChange) {
        onClientChange(true);
      }
      socket.on('close', () => {
        if (onClientChange) {
          onClientChange(hasReadyWsClients());
        }
      });
    });
  }

  const waitForHpSimConnection = (timeoutMs = 0) => {
    if (!wss || hasReadyWsClients()) {
      return Promise.resolve();
    }
    let settled = false;
    const resolveOnce = (resolve) => {
      if (settled) {
        return;
      }
      settled = true;
      resolve();
    };

    return new Promise((resolve) => {
      const cleanup = () => {
        wss.off('connection', onConnection);
      };

      const onConnection = () => {
        cleanup();
        resolveOnce(resolve);
      };

      wss.on('connection', onConnection);

      const timer = timeoutMs > 0
        ? setTimeout(() => {
          cleanup();
          resolveOnce(resolve);
        }, timeoutMs)
        : null;

      process.nextTick(() => {
        if (settled) {
          return;
        }
        if (hasReadyWsClients()) {
          if (timer) {
            clearTimeout(timer);
          }
          cleanup();
          resolveOnce(resolve);
        }
      });
    });
  };

  const close = () => {
    if (wss) {
      wss.close();
    }
    pendingWsPayloads.splice(0, pendingWsPayloads.length);
  };

  return {
    wss,
    hasReadyWsClients,
    getReadyWsClients,
    broadcast,
    flushPendingPayloads,
    waitForHpSimConnection,
    close,
  };
}

function createSettler(session, motionIdleMs, finalize) {
  return new Promise((resolve) => {
    let timer = null;
    const finish = () => {
      if (timer) {
        clearTimeout(timer);
        timer = null;
      }
      finalize();
      resolve();
    };
    const arm = () => {
      if (timer) {
        clearTimeout(timer);
      }
      timer = setTimeout(finish, motionIdleMs);
    };
    session.motionSettler = {
      arm,
      finish,
    };
    arm();
  });
}

function normalizeMotionTime(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : null;
}

function getStepperBatchLastTime(batch = {}) {
  return normalizeMotionTime(batch.last_time ?? batch.last_step_time);
}

function getTrapqBatchEndTime(batch = {}) {
  const data = Array.isArray(batch.data) ? batch.data : [];
  let maxEndTime = null;
  for (const entry of data) {
    if (!Array.isArray(entry)) {
      continue;
    }
    const startTime = normalizeMotionTime(entry[0]);
    const duration = normalizeMotionTime(entry[1]) ?? 0;
    if (!Number.isFinite(startTime)) {
      continue;
    }
    const endTime = startTime + duration;
    maxEndTime = Number.isFinite(maxEndTime)
      ? Math.max(maxEndTime, endTime)
      : endTime;
  }
  return maxEndTime;
}

export function buildKlippyBridgeWsHint(wsPort, { host = 'localhost' } = {}) {
  return `?gcode_ws=ws://${host}:${wsPort}`;
}

export function createKlipperTerminalBridge({
  client,
  klippyState = null,
  wsPort = 8790,
  quiet = false,
  debugLog = null,
  debugTrapq = false,
  onClientChange = null,
  onBroadcast = null,
  motionIdleMs = DEFAULT_MOTION_IDLE_MS,
  motionEndTimeToleranceS = DEFAULT_MOTION_END_TIME_TOLERANCE_S,
  now = () => performance.now(),
  clockHz = MCU_CLOCK_HZ_KLIPPER_HOST,
} = {}) {
  if (!client) {
    throw new Error('createKlipperTerminalBridge() requires a KlippyApiClient instance.');
  }

  const helpers = buildWsHelpers({
    wsPort,
    quiet,
    onClientChange,
    onBroadcast,
  });

  const stepperSubscriptionIds = new Map();
  const trapqSubscriptionIds = new Map();
  let activeSession = null;
  const logDebug = typeof debugLog === 'function' ? debugLog : null;

  const emitDebug = (event, payload = {}) => {
    if (!logDebug) {
      return;
    }
    logDebug(event, payload);
  };

  const hasStepperCoverageForPlannedMotion = (session) => {
    if (!session) {
      return false;
    }
    const plannedEndTime = session.plannedMotionEndTime;
    const maxStepperLastTime = session.maxStepperLastTime;
    if (!Number.isFinite(plannedEndTime) || !Number.isFinite(maxStepperLastTime)) {
      return false;
    }
    return maxStepperLastTime >= (plannedEndTime - motionEndTimeToleranceS);
  };

  const ensureMotionSessionStarted = (session) => {
    if (!session || session.motionStreamStarted) {
      return;
    }
    session.motionStreamStarted = true;
    helpers.broadcast({
      type: 'klipper_api_session_start',
      gcode: session.gcode,
      clock_hz: clockHz,
    });
  };

  const handleStepperBatch = (stepperName, batch = {}) => {
    emitDebug('api-response', {
      method: 'motion_report/dump_stepper',
      channel: 'async',
      name: stepperName,
      params: batch,
    });
    const session = activeSession;
    if (!session) {
      return;
    }
    ensureMotionSessionStarted(session);
    helpers.broadcast({
      type: 'klipper_api_stepper_batch',
      gcode: session.gcode,
      batch: {
        name: stepperName,
        first_clock: batch.first_clock,
        first_time: batch.first_time ?? batch.first_step_time ?? null,
        last_clock: batch.last_clock,
        last_time: batch.last_time ?? batch.last_step_time ?? null,
        start_mcu_position: batch.start_mcu_position,
        data: Array.isArray(batch.data) ? batch.data : [],
      },
    });
    const lastTime = getStepperBatchLastTime(batch);
    if (Number.isFinite(lastTime)) {
      session.maxStepperLastTime = Number.isFinite(session.maxStepperLastTime)
        ? Math.max(session.maxStepperLastTime, lastTime)
        : lastTime;
    }
    session.lastMotionAt = now();
    if (session.motionSettler) {
      if (hasStepperCoverageForPlannedMotion(session)) {
        session.motionSettler.finish();
        return;
      }
      session.motionSettler.arm();
    }
  };

  const handleTrapqBatch = (trapqName, batch = {}) => {
    emitDebug('api-response', {
      method: 'motion_report/dump_trapq',
      channel: 'async',
      name: trapqName,
      params: batch,
    });
    const session = activeSession;
    if (!session || trapqName !== 'toolhead') {
      return;
    }
    ensureMotionSessionStarted(session);
    helpers.broadcast({
      type: 'klipper_api_trapq_batch',
      gcode: session.gcode,
      batch: {
        name: trapqName,
        data: Array.isArray(batch.data) ? batch.data : [],
      },
    });
    const trapqEndTime = getTrapqBatchEndTime(batch);
    if (Number.isFinite(trapqEndTime)) {
      session.plannedMotionEndTime = Number.isFinite(session.plannedMotionEndTime)
        ? Math.max(session.plannedMotionEndTime, trapqEndTime)
        : trapqEndTime;
    }
    if (session.motionSettler && hasStepperCoverageForPlannedMotion(session)) {
      session.motionSettler.finish();
    }
  };

  const syncMotionSources = async (stepperNames = [], trapqNames = []) => {
    const normalized = Array.from(new Set(
      (Array.isArray(stepperNames) ? stepperNames : [])
        .filter((name) => typeof name === 'string' && name.trim().length > 0)
        .map((name) => name.trim()),
    )).sort();
    const nextNames = new Set(normalized);
    const normalizedTrapq = Array.from(new Set(
      (Array.isArray(trapqNames) ? trapqNames : [])
        .filter((name) => typeof name === 'string' && name.trim().length > 0)
        .map((name) => name.trim()),
    )).sort();

    for (const [stepperName, subscriptionId] of stepperSubscriptionIds.entries()) {
      if (nextNames.has(stepperName)) {
        continue;
      }
      client.unsubscribe(subscriptionId);
      stepperSubscriptionIds.delete(stepperName);
    }

    for (const stepperName of normalized) {
      if (stepperSubscriptionIds.has(stepperName)) {
        continue;
      }
      const subscriptionId = `sub:motion:stepper:${stepperName}`;
      await client.subscribe(
        'motion_report/dump_stepper',
        { name: stepperName },
        (params) => handleStepperBatch(stepperName, params),
        {
          id: subscriptionId,
          onResponse: (result) => {
            emitDebug('api-response', {
              method: 'motion_report/dump_stepper',
              channel: 'initial',
              name: stepperName,
              result,
            });
          },
        },
      );
      stepperSubscriptionIds.set(stepperName, subscriptionId);
    }

    const subscribedTrapq = debugTrapq
      ? normalizedTrapq
      : normalizedTrapq.filter((name) => name === 'toolhead');
    const subscribedTrapqNames = new Set(subscribedTrapq);

    if (subscribedTrapq.length === 0) {
      for (const [trapqName, subscriptionId] of trapqSubscriptionIds.entries()) {
        client.unsubscribe(subscriptionId);
        trapqSubscriptionIds.delete(trapqName);
      }
      return;
    }

    for (const [trapqName, subscriptionId] of trapqSubscriptionIds.entries()) {
      if (subscribedTrapqNames.has(trapqName)) {
        continue;
      }
      client.unsubscribe(subscriptionId);
      trapqSubscriptionIds.delete(trapqName);
    }

    for (const trapqName of subscribedTrapq) {
      if (trapqSubscriptionIds.has(trapqName)) {
        continue;
      }
      const subscriptionId = `sub:motion:trapq:${trapqName}`;
      await client.subscribe(
        'motion_report/dump_trapq',
        { name: trapqName },
        (params) => handleTrapqBatch(trapqName, params),
        {
          id: subscriptionId,
          onResponse: (result) => {
            emitDebug('api-response', {
              method: 'motion_report/dump_trapq',
              channel: 'initial',
              name: trapqName,
              result,
            });
          },
        },
      );
      trapqSubscriptionIds.set(trapqName, subscriptionId);
    }
  };

  const handleGcodeOutput = (response) => {
    const session = activeSession;
    if (!session) {
      return;
    }
    for (const line of splitTerminalLines(response)) {
      session.replyLines.push(line);
    }
  };

  const beginCommandSession = (gcode) => {
    activeSession = {
      gcode,
      replyLines: [],
      sawMotion: false,
      lastMotionAt: null,
      maxStepperLastTime: null,
      plannedMotionEndTime: null,
      motionStreamStarted: false,
      motionSettler: null,
    };
    return activeSession;
  };

  const finalizeCommandSession = async (session) => {
    if (!session || activeSession !== session) {
      return {
        reply: 'ok',
        replyLines: [],
        hadMotion: false,
        printedLiveOutput: false,
      };
    }

    if (!hasStepperCoverageForPlannedMotion(session)
      && (session.motionStreamStarted || stepperSubscriptionIds.size > 0)) {
      await createSettler(session, motionIdleMs, () => {
        // Give Klipper's batched motion_report stream one quiet window to deliver
        // any tail queue_step batch that belongs to the just-completed command.
      });
    }

    const replyLines = [...session.replyLines];
    const reply = replyLines.length > 0 ? replyLines.join('\n') : 'ok';
    if (session.motionStreamStarted) {
      helpers.broadcast({
        type: 'klipper_api_session_end',
        gcode: session.gcode,
      });
    }
    helpers.broadcast({ type: 'reply', gcode: session.gcode, reply });
    activeSession = null;
    return {
      reply,
      replyLines,
      hadMotion: session.motionStreamStarted,
      printedLiveOutput: replyLines.length > 0,
    };
  };

  const abortCommandSession = () => {
    activeSession = null;
  };

  const runGcodeCommand = async (gcode, execute) => {
    const trimmed = gcode?.trim?.();
    if (!trimmed) {
      return null;
    }
    const session = beginCommandSession(trimmed);
    try {
      await execute();
      return await finalizeCommandSession(session);
    } catch (error) {
      abortCommandSession();
      throw error;
    }
  };

  if (klippyState) {
    klippyState.on('motion-sources-changed', ({ steppers, trapq }) => {
      syncMotionSources(steppers, trapq).catch((error) => {
        if (!quiet) {
          console.error(`Failed to subscribe to Klipper motion streams: ${error.message}`);
        }
      });
    });
    const initialMotionSources = klippyState.getSnapshot?.()?.motionSources || {};
    const initialSteppers = initialMotionSources.steppers;
    const initialTrapq = initialMotionSources.trapq;
    if ((Array.isArray(initialSteppers) && initialSteppers.length > 0)
      || (debugTrapq && Array.isArray(initialTrapq) && initialTrapq.length > 0)) {
      syncMotionSources(initialSteppers, initialTrapq).catch((error) => {
        if (!quiet) {
          console.error(`Failed to subscribe to initial Klipper motion streams: ${error.message}`);
        }
      });
    }
  }

  const close = () => {
    abortCommandSession();
    for (const subscriptionId of stepperSubscriptionIds.values()) {
      client.unsubscribe(subscriptionId);
    }
    stepperSubscriptionIds.clear();
    for (const subscriptionId of trapqSubscriptionIds.values()) {
      client.unsubscribe(subscriptionId);
    }
    trapqSubscriptionIds.clear();
    helpers.close();
  };

  return {
    ...helpers,
    syncMotionSources,
    handleGcodeOutput,
    runGcodeCommand,
    close,
  };
}
