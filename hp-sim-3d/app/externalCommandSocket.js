import { EncoderComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { setClosedLoopMotorFeatureFlags } from './closed-loop-flags.js';
import { setLineLayeringFeatureFlags } from './line-layering-flags.js';

export function normalizeWsUrl(raw) {
  if (!raw || typeof raw !== 'string') {
    return null;
  }
  const trimmed = raw.trim();
  if (!trimmed) {
    return null;
  }
  if (trimmed.startsWith('ws://') || trimmed.startsWith('wss://')) {
    return trimmed;
  }
  const cleaned = trimmed.replace(/^\/+/, '');
  return `ws://${cleaned}`;
}

export function createExternalCommandSocket({
  url,
  WebSocketCtor = globalThis.WebSocket,
  onPayload,
  onOpen,
  logger = console,
  reconnectDelayMs = 5000,
  logEveryAttempts = 10,
} = {}) {
  let socket = null;
  let connecting = false;
  let reconnectTimer = null;
  let attemptCount = 0;

  function clearReconnectTimer() {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
  }

  function scheduleReconnect(reason = '') {
    if (!url || !WebSocketCtor || socket || connecting || reconnectTimer) {
      return;
    }
    const attempts = Math.max(1, attemptCount);
    if (attempts % logEveryAttempts === 0) {
      const suffix = reason ? ` (${reason})` : '';
      logger.log(
        `hp-sim-3d: tried external G-code stream ${attempts} times${suffix}. Continuing to retry every ${Math.round(reconnectDelayMs)}ms`
      );
    }
    reconnectTimer = setTimeout(() => {
      reconnectTimer = null;
      connect();
    }, reconnectDelayMs);
  }

  function send(payload) {
    if (!socket || typeof socket.send !== 'function') {
      return false;
    }
    if (WebSocketCtor && socket.readyState !== WebSocketCtor.OPEN) {
      return false;
    }
    socket.send(typeof payload === 'string' ? payload : JSON.stringify(payload));
    return true;
  }

  function connect() {
    if (!url || socket || connecting || !WebSocketCtor) {
      return;
    }
    clearReconnectTimer();
    connecting = true;
    attemptCount += 1;
    try {
      socket = new WebSocketCtor(url);
    } catch (_err) {
      socket = null;
      connecting = false;
      scheduleReconnect('failed to open');
      return;
    }
    socket.addEventListener('open', () => {
      connecting = false;
      const attemptsBeforeConnect = attemptCount;
      attemptCount = 0;
      if (attemptsBeforeConnect > 1) {
        logger.log(`hp-sim-3d: external G-code stream connected after ${attemptsBeforeConnect} attempts:`, url);
      } else {
        logger.log('hp-sim-3d: external G-code stream connected:', url);
      }
      onOpen?.();
    });
    socket.addEventListener('message', (event) => {
      try {
        const payload = typeof event.data === 'string' ? JSON.parse(event.data) : event.data;
        onPayload?.(payload);
      } catch (err) {
        logger.warn('hp-sim-3d: failed to process external G-code payload.', err);
      }
    });
    socket.addEventListener('close', () => {
      socket = null;
      connecting = false;
      scheduleReconnect('connection closed');
    });
    socket.addEventListener('error', () => {
      try {
        socket?.close();
      } catch (_err) {
        // Ignore socket close errors.
      }
    });
  }

  return {
    connect,
    send,
    get socket() {
      return socket;
    },
    get connecting() {
      return connecting;
    },
  };
}

export function createExternalCommandController({
  world,
  url,
  commands,
  runtime,
  inspectionTools,
  featureFlags,
  WebSocketCtor = globalThis.WebSocket,
  logger = console,
  queueLimit = 5000,
} = {}) {
  const externalCommandQueue = [];
  let socketController = null;
  let recordingReady = null;
  let recordingError = null;

  function getRemoteSystem() {
    return commands?.getRemoteSystem?.() || null;
  }

  function pushCommands(batch) {
    if (!Array.isArray(batch) || batch.length === 0) {
      return;
    }
    if (!commands?.pushExternalCommands?.(batch)) {
      const overflow = externalCommandQueue.length + batch.length - queueLimit;
      if (overflow > 0) {
        externalCommandQueue.splice(0, overflow);
      }
      externalCommandQueue.push(...batch);
    }
  }

  function flushQueue() {
    if (externalCommandQueue.length === 0) {
      return;
    }
    const batch = externalCommandQueue.splice(0, externalCommandQueue.length);
    pushCommands(batch);
  }

  function resolveEncoderAngles(axes = []) {
    if (!Array.isArray(axes) || axes.length === 0) {
      return [];
    }
    const remoteSystem = getRemoteSystem();
    if (!remoteSystem) {
      return [];
    }
    if (typeof remoteSystem._ensureAxisMapping === 'function') {
      try {
        remoteSystem._ensureAxisMapping(world);
      } catch (_err) {
        // Mapping can be unavailable until a scene is built.
      }
    }
    return axes.map((axis) => {
      const mapping = remoteSystem.axisToEntity ? remoteSystem.axisToEntity[axis] : null;
      const entityIds = Array.isArray(mapping) ? mapping : (mapping != null ? [mapping] : []);
      for (const entityId of entityIds) {
        const encoder = world.getComponent(entityId, EncoderComponent);
        if (encoder && Number.isFinite(encoder.angle)) {
          return encoder.angle * (180 / Math.PI);
        }
      }
      return null;
    });
  }

  function respondToEncoderRequest(requestId, axes) {
    const payload = {
      type: 'encoder_response',
      requestId,
      axes,
      anglesDeg: resolveEncoderAngles(axes),
      simulationSettings: {
        closedLoopMotorsEnabled: world.getResource('closedLoopMotorsEnabled') === true,
        lineLayeringEnabled: world.getResource('enableLayering') === true,
        recording: world.getResource('flightRecorder')?.socket?.readyState === 1,
        recordingError,
      },
      simulationClock: {
        time_ms: (world.getResource('researchClock')?.time ?? 0) * 1000,
        generation: world.getResource('sceneGeneration') || 0,
        wall_time_ms: Date.now(), source: world.getResource('researchClock')?.source || 'browser.researchClock',
      },
    };
    if (axes.length) world.getResource('flightRecorder')?.recordEvent('encoder_response_sent', payload);
    socketController?.send?.(payload);
  }

  function handlePayload(payload) {
    if (!payload) {
      return;
    }
    if (payload.type !== 'encoder_request' || payload.axes?.length) {
      world.getResource('flightRecorder')?.recordEvent('external_payload_received', payload);
    }
    if (payload.type === 'encoder_request') {
      if (payload.requestId != null && Array.isArray(payload.axes)) {
        if (recordingReady) recordingReady.then(() => respondToEncoderRequest(payload.requestId, payload.axes));
        else respondToEncoderRequest(payload.requestId, payload.axes);
      }
      return;
    }
    if (payload.type === 'simulation_settings') {
      if (typeof payload.closedLoopMotorsEnabled === 'boolean') {
        if (featureFlags) featureFlags.setClosedLoopMotorsEnabledState(payload.closedLoopMotorsEnabled);
        else setClosedLoopMotorFeatureFlags(world, payload.closedLoopMotorsEnabled);
      }
      if (typeof payload.lineLayeringEnabled === 'boolean') {
        if (featureFlags) {
          const changed = featureFlags.lineLayeringEnabled !== payload.lineLayeringEnabled;
          featureFlags.setLineLayeringEnabledState(payload.lineLayeringEnabled);
          // Layering changes the baked initial cable lengths, as with the UI toggle.
          if (changed) commands?.handleUserReset?.();
        } else setLineLayeringFeatureFlags(world, payload.lineLayeringEnabled);
      }
      if (payload.recordingUrl) {
        const recorder = world.getResource('flightRecorder');
        recordingReady = Promise.resolve().then(() => {
          if (!recorder) throw new Error('This simulator has no browser flight recorder');
          recorder.extendedReference = true;
          return recorder.ensureConnected(payload.recordingUrl);
        }).then(() => { recordingError = null; }).catch(error => {
          recordingError = error.message;
          logger.warn('hp-sim-3d: recording startup failed:', recordingError);
        });
      }
      world.getResource('flightRecorder')?.recordEvent('simulation_settings_applied', payload);
      return;
    }
    if (payload.type === 'reset') {
      commands?.handleUserReset?.();
      return;
    }
    if (payload.type === 'set_speed_scale' && Number.isFinite(payload.value) && payload.value > 0) {
      commands?.applyTimeScaleChange?.(payload.value, { show: true });
      return;
    }
    if (payload.type === 'position_trace_mode') {
      const renderSystem = world.getResource('renderSystem');
      renderSystem?.setPositionTraceEnabled?.(Boolean(payload.enabled));
      renderSystem?.update?.(world, 0);
      inspectionTools?.updatePositionTraceToggleUI?.();
      return;
    }
    if (payload.type === 'klipper_api_session_start') {
      commands?.ensureExternalKlipperApiBridge?.()?.postMessage({
        type: 'api_motion_start',
        clockHz: payload.clock_hz,
      });
      return;
    }
    if (payload.type === 'klipper_api_stepper_batch' && payload.batch) {
      commands?.ensureExternalKlipperApiBridge?.()?.postMessage({
        type: 'api_motion_batch',
        batch: payload.batch,
      });
      return;
    }
    if (payload.type === 'klipper_api_trapq_batch' && payload.batch) {
      commands?.ensureExternalKlipperApiBridge?.()?.postMessage({
        type: 'api_motion_trapq_batch',
        batch: payload.batch,
      });
      return;
    }
    if (payload.type === 'klipper_api_session_end') {
      commands?.ensureExternalKlipperApiBridge?.()?.postMessage({ type: 'api_motion_finish' });
      return;
    }
    const batch = [];
    if (payload.type === 'command' && payload.command) {
      batch.push(payload.command);
    }
    if (Array.isArray(payload.commands)) {
      for (const command of payload.commands) {
        if (command) {
          batch.push(command);
        }
      }
    }
    pushCommands(batch);
    runtime?.resume?.();
    if (typeof payload.reply === 'string' && payload.reply.trim().length > 0) {
      logger.info('hp-sim-3d: gcode reply', payload.reply.trim());
    }
  }

  function connect() {
    if (!url) {
      return;
    }
    if (!socketController) {
      socketController = createExternalCommandSocket({
        url,
        WebSocketCtor,
        logger,
        onPayload: handlePayload,
        onOpen: flushQueue,
      });
    }
    socketController.connect();
  }

  return {
    connect,
    flushQueue,
    handlePayload,
  };
}
