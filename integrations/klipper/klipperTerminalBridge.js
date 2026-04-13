import { performance } from 'node:perf_hooks';
import fs from 'node:fs';
import path from 'node:path';
import { WebSocketServer } from 'ws';
import {
  MCU_CLOCK_HZ_KLIPPER_HOST,
} from './klipperFirmwareModel.js';
import {
  buildMotorCatalogFromConfig,
  resolveMotorToken,
} from './klipperMotorAddressConfig.js';

const MAX_PENDING_WS_PAYLOADS = 5000;
const DEFAULT_MOTION_IDLE_MS = 650;
const DEFAULT_MOTION_END_TIME_TOLERANCE_S = 0.010;
const DEFAULT_ENCODER_TIMEOUT_MS = 2000;

function splitTerminalLines(response) {
  return String(response)
    .split(/\r?\n/)
    .map((line) => line.trimEnd())
    .filter((line) => line.length > 0);
}

function parseMotorTokens(gcode) {
  if (typeof gcode !== 'string' || !/^M569\.3\b/i.test(gcode.trim())) {
    return { tokens: [], setReference: false };
  }
  const pMatch = gcode.match(/P([A-Za-z0-9_:\.]+)/i);
  const tokens = pMatch ? pMatch[1].split(':').map((value) => value.trim()).filter(Boolean) : [];
  const setReference = /\bS(?:\s|$|-?[0-9])/i.test(gcode);
  return { tokens, setReference };
}

function isEncoderQuery(gcode) {
  return typeof gcode === 'string' && /^M569\.3\b/i.test(gcode.trim());
}

function parseForceModeCommand(gcode) {
  if (typeof gcode !== 'string' || !/^M569\.4\b/i.test(gcode.trim())) {
    return { tokens: [] };
  }
  const pMatch = gcode.match(/P([A-Za-z0-9_:\.]+)/i);
  const tokens = pMatch ? pMatch[1].split(':').map((value) => value.trim()).filter(Boolean) : [];
  return { tokens };
}

function parseForceModeReplyTokens(replyText) {
  return String(replyText || '')
    .split(',')
    .map((token) => token.trim())
    .filter((token) => token.length > 0);
}

const GCODE_AXIS_TO_STEPPER_AXIS = {
  X: 'A',
  Y: 'B',
  Z: 'C',
  U: 'D',
  V: 'E',
  W: 'F',
};

function normalizeStepperAxisForGcode(gcodeAxis) {
  const upper = String(gcodeAxis || '').trim().toUpperCase();
  if (!upper) {
    return null;
  }
  return GCODE_AXIS_TO_STEPPER_AXIS[upper] || upper;
}

function parseRelativeForceMoveGcode(gcode) {
  const text = String(gcode || '').trim();
  if (!/^G0?1\b/i.test(text) || !/\bH2\b/i.test(text)) {
    return null;
  }
  const feedMatch = text.match(/\bF(-?[0-9]+(?:\.[0-9]+)?)/i);
  const feedMmPerMin = feedMatch ? Number.parseFloat(feedMatch[1]) : null;
  const velocityMmPerS = Number.isFinite(feedMmPerMin) && feedMmPerMin > 0
    ? feedMmPerMin / 60
    : 1;

  const axisMoves = [];
  const axisPattern = /\b([A-Za-z])(-?[0-9]+(?:\.[0-9]+)?)/g;
  let match = axisPattern.exec(text);
  while (match) {
    const axis = String(match[1]).toUpperCase();
    if (axis !== 'G' && axis !== 'H' && axis !== 'F') {
      const distanceMm = Number.parseFloat(match[2]);
      if (Number.isFinite(distanceMm)) {
        axisMoves.push({ axis, distanceMm });
      }
    }
    match = axisPattern.exec(text);
  }

  if (axisMoves.length === 0) {
    return null;
  }

  return {
    axisMoves,
    velocityMmPerS,
  };
}

function buildDriverLookupCatalog({
  configPath = null,
  driverToAxis = null,
} = {}) {
  if (driverToAxis && typeof driverToAxis === 'object' && driverToAxis.stepperDescriptors) {
    return driverToAxis;
  }
  if (typeof configPath !== 'string' || configPath.trim().length === 0) {
    return buildMotorCatalogFromConfig(null);
  }
  const resolvedConfigPath = path.resolve(configPath);
  return fs.existsSync(resolvedConfigPath)
    ? buildMotorCatalogFromConfig(resolvedConfigPath)
    : buildMotorCatalogFromConfig(null);
}

function resolveMotorTokens(tokens, catalog) {
  const resolved = [];
  for (const token of tokens) {
    const entry = resolveMotorToken(token, catalog);
    if (entry.error) {
      return { error: entry.error };
    }
    resolved.push(entry.resolved);
  }
  return { resolved };
}

function formatEncoderReply(values) {
  if (!Array.isArray(values) || values.length === 0) {
    return '[ ]';
  }
  const formatValue = (value) => {
    if (!Number.isFinite(value)) {
      return 'nan';
    }
    return value.toFixed(2);
  };
  return `[${values.map((value) => formatValue(value)).join(', ')}, ]`;
}

function buildWsHelpers({
  wsPort,
  quiet = false,
  onClientChange = null,
  onBroadcast = null,
} = {}) {
  const wss = wsPort ? new WebSocketServer({ port: wsPort }) : null;
  const pendingWsPayloads = [];
  const pendingEncoderRequests = new Map();
  let encoderRequestSeq = 1;
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

  const handleIncomingWsMessage = (data) => {
    if (!data) {
      return;
    }
    let payload = null;
    try {
      payload = typeof data === 'string' ? JSON.parse(data) : JSON.parse(data.toString());
    } catch (_error) {
      return;
    }
    if (payload?.type === 'encoder_response' && payload.requestId != null) {
      const pending = pendingEncoderRequests.get(payload.requestId);
      if (!pending) {
        return;
      }
      pendingEncoderRequests.delete(payload.requestId);
      pending.resolve(payload);
    }
  };

  const sendEncoderRequest = (axes, timeoutMs = DEFAULT_ENCODER_TIMEOUT_MS) => {
    const readyClients = getReadyWsClients();
    if (readyClients.length === 0) {
      throw new Error('Message not received');
    }
    const requestId = encoderRequestSeq++;
    const payload = { type: 'encoder_request', requestId, axes };
    const data = JSON.stringify(payload);
    readyClients.forEach((client) => {
      try {
        client.send(data);
      } catch (_error) {
        // Ignore send errors. The timeout below handles missing responses.
      }
    });
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        pendingEncoderRequests.delete(requestId);
        reject(new Error('Message not received'));
      }, Math.max(1, timeoutMs));
      pendingEncoderRequests.set(requestId, {
        resolve: (value) => {
          clearTimeout(timeout);
          pendingEncoderRequests.delete(requestId);
          resolve(value);
        },
        reject: (error) => {
          clearTimeout(timeout);
          pendingEncoderRequests.delete(requestId);
          reject(error);
        },
      });
    });
  };

  if (wss) {
    wss.on('connection', (socket) => {
      flushPendingPayloads();
      if (onClientChange) {
        onClientChange(true);
      }
      socket.on('message', (data) => handleIncomingWsMessage(data));
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
    for (const pending of pendingEncoderRequests.values()) {
      pending.reject(new Error('encoder request bridge closed'));
    }
    pendingEncoderRequests.clear();
    pendingWsPayloads.splice(0, pendingWsPayloads.length);
  };

  return {
    wss,
    hasReadyWsClients,
    getReadyWsClients,
    broadcast,
    flushPendingPayloads,
    sendEncoderRequest,
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
  configPath = null,
  driverToAxis = null,
  encoderTimeoutMs = DEFAULT_ENCODER_TIMEOUT_MS,
  encoderResolver = null,
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

  const resolvedDriverCatalog = buildDriverLookupCatalog({
    configPath,
    driverToAxis,
  });

  const resolveStepperByAxis = (() => {
    const byAxis = new Map();
    const descriptors = Array.isArray(resolvedDriverCatalog?.stepperDescriptors)
      ? [...resolvedDriverCatalog.stepperDescriptors]
      : [];
    descriptors.sort((left, right) => String(left?.stepperName || '').localeCompare(String(right?.stepperName || '')));
    for (const descriptor of descriptors) {
      const axis = String(descriptor?.axis || '').toUpperCase();
      if (!axis || byAxis.has(axis)) {
        continue;
      }
      byAxis.set(axis, descriptor);
    }
    return (axis) => byAxis.get(String(axis || '').toUpperCase()) || null;
  })();

  const rewriteGcodeLine = (gcode) => {
    const parsed = parseRelativeForceMoveGcode(gcode);
    if (!parsed) {
      return gcode;
    }
    const forceMoveLines = [];
    for (const move of parsed.axisMoves) {
      const stepperAxis = normalizeStepperAxisForGcode(move.axis);
      const descriptor = resolveStepperByAxis(stepperAxis);
      if (!descriptor?.stepperName) {
        return gcode;
      }
      forceMoveLines.push(
        `FORCE_MOVE STEPPER=${descriptor.stepperName} DISTANCE=${move.distanceMm} VELOCITY=${parsed.velocityMmPerS} ACCEL=500`,
      );
    }
    if (forceMoveLines.length === 0) {
      return gcode;
    }
    return forceMoveLines.join('\n');
  };

  const stepperSubscriptionIds = new Map();
  const trapqSubscriptionIds = new Map();
  const encoderReferences = new Map();
  let activeSession = null;
  const logDebug = typeof debugLog === 'function' ? debugLog : null;
  const resolveEncoderAngles = typeof encoderResolver === 'function'
    ? encoderResolver
    : helpers.wss
      ? async ({ axes, timeoutMs }) => {
        const response = await helpers.sendEncoderRequest(
          axes,
          Number.isFinite(timeoutMs)
            ? Math.max(1, Math.min(timeoutMs, 5000))
            : encoderTimeoutMs,
        );
        if (Array.isArray(response?.anglesDeg)) {
          return response.anglesDeg;
        }
        if (Array.isArray(response?.angles)) {
          return response.angles;
        }
        return [];
      }
      : null;

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
    const lines = splitTerminalLines(response);
    const session = activeSession;
    if (!session) {
      return lines;
    }
    for (const line of lines) {
      session.replyLines.push(line);
    }
    if (session.deferLiveOutput) {
      return [];
    }
    if (lines.length > 0) {
      session.printedLiveOutput = true;
    }
    return lines;
  };

  const maybeOverrideEncoderReply = async (session) => {
    if (!session?.isEncoderQuery || typeof resolveEncoderAngles !== 'function') {
      return null;
    }
    const { tokens, setReference } = parseMotorTokens(session.gcode);
    if (tokens.length === 0) {
      return null;
    }

    const resolvedTokens = resolveMotorTokens(tokens, resolvedDriverCatalog);
    if (resolvedTokens.error) {
      return resolvedTokens.error;
    }
    const axesForQuery = resolvedTokens.resolved
      .map((descriptor) => descriptor?.axis)
      .filter(Boolean);
    if (axesForQuery.length === 0) {
      return null;
    }

    let anglesDeg = null;
    try {
      const resolverResult = await resolveEncoderAngles({
        axes: axesForQuery,
        timeoutMs: encoderTimeoutMs,
        gcode: session.gcode,
      });
      if (Array.isArray(resolverResult)) {
        anglesDeg = resolverResult;
      } else if (resolverResult && Array.isArray(resolverResult.anglesDeg)) {
        anglesDeg = resolverResult.anglesDeg;
      }
    } catch (_error) {
      return null;
    }
    if (!anglesDeg || anglesDeg.length === 0) {
      return null;
    }

    const values = [];
    let encoderIdx = 0;
    for (const descriptor of resolvedTokens.resolved) {
      const rawValue = anglesDeg[encoderIdx++];
      const referenceKey = descriptor?.addressKey || descriptor?.stepperName || null;
      if (setReference && referenceKey && Number.isFinite(rawValue)) {
        encoderReferences.set(referenceKey, rawValue);
      }
      const reference = referenceKey && encoderReferences.has(referenceKey)
        ? encoderReferences.get(referenceKey)
        : 0;
      const relative = Number.isFinite(rawValue) ? rawValue - reference : null;
      values.push(Number.isFinite(relative) ? relative : null);
    }
    return formatEncoderReply(values);
  };

  const maybeBroadcastForceModeReply = (session) => {
    const { tokens: descriptorTokens } = parseForceModeCommand(session?.gcode);
    if (!descriptorTokens.length) {
      return;
    }
    const resolved = resolveMotorTokens(descriptorTokens, resolvedDriverCatalog);
    if (resolved.error) {
      return;
    }
    const replyTokens = parseForceModeReplyTokens(session.replyLines.join('\n'));
    if (replyTokens.length !== resolved.resolved.length) {
      return;
    }
    const commands = [];
    for (let index = 0; index < resolved.resolved.length; index += 1) {
      const descriptor = resolved.resolved[index];
      const axis = descriptor?.axis;
      const token = replyTokens[index];
      if (!axis || !token) {
        return;
      }
      if (/^pos_mode$/i.test(token)) {
        commands.push({
          type: 'SetPositionMode',
          axis,
          driver: descriptor.addressKey || descriptor.stepperName,
          torqueNm: 0,
          timestamp: Date.now(),
        });
        continue;
      }
      const torqueMatch = token.match(/^(-?\d+(?:\.\d+)?)\s*Nm$/i);
      if (!torqueMatch) {
        return;
      }
      commands.push({
        type: 'SetTorqueMode',
        axis,
        driver: descriptor.addressKey || descriptor.stepperName,
        torqueNm: Number.parseFloat(torqueMatch[1]),
        timestamp: Date.now(),
      });
    }
    for (const command of commands) {
      helpers.broadcast({
        type: 'command',
        command,
        gcode: session.gcode,
      });
    }
  };

  const beginCommandSession = (gcode) => {
    const encoderQuery = isEncoderQuery(gcode);
    activeSession = {
      gcode,
      deferLiveOutput: encoderQuery,
      isEncoderQuery: encoderQuery,
      printedLiveOutput: false,
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
      printedLiveOutput: session.printedLiveOutput,
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
      const encoderReply = await maybeOverrideEncoderReply(session);
      if (typeof encoderReply === 'string' && encoderReply.trim().length > 0) {
        session.replyLines = [encoderReply];
      }
      maybeBroadcastForceModeReply(session);
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
    rewriteGcodeLine,
    runGcodeCommand,
    close,
  };
}
