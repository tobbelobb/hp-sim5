import { EventEmitter } from 'node:events';

export const REQUIRED_RUNTIME_OBJECTS = [
  'webhooks',
  'toolhead',
  'gcode_move',
  'motion_report',
];

export const OPTIONAL_RUNTIME_OBJECTS = [
  'print_stats',
  'virtual_sdcard',
];

const DEFAULT_OBJECT_SUBSCRIPTION_ID = 'sub:objects:runtime';
const DEFAULT_GCODE_OUTPUT_SUBSCRIPTION_ID = 'sub:gcode:output';

function isPlainObject(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function cloneValue(value) {
  if (Array.isArray(value)) {
    return value.map((entry) => cloneValue(entry));
  }
  if (isPlainObject(value)) {
    return Object.fromEntries(
      Object.entries(value).map(([key, entry]) => [key, cloneValue(entry)]),
    );
  }
  return value;
}

function deepMerge(base, patch) {
  if (!isPlainObject(base) || !isPlainObject(patch)) {
    return cloneValue(patch);
  }
  const next = { ...base };
  for (const [key, value] of Object.entries(patch)) {
    next[key] = deepMerge(base[key], value);
  }
  return next;
}

function normalizeStringList(value) {
  return Array.from(new Set(
    (Array.isArray(value) ? value : [])
      .filter((entry) => typeof entry === 'string' && entry.trim().length > 0)
      .map((entry) => entry.trim()),
  )).sort();
}

function haveSameStrings(left, right) {
  if (left.length !== right.length) {
    return false;
  }
  for (let i = 0; i < left.length; i += 1) {
    if (left[i] !== right[i]) {
      return false;
    }
  }
  return true;
}

function buildObjectKey(objects) {
  return JSON.stringify(Object.keys(objects).sort());
}

function summarizeSnapshot(runtime) {
  const printerState = runtime.status.webhooks?.state || runtime.info?.state || null;
  const printerStateMessage = runtime.status.webhooks?.state_message
    || runtime.info?.state_message
    || null;
  return {
    connected: runtime.connected,
    primed: runtime.primed,
    info: cloneValue(runtime.info),
    availableObjects: [...runtime.availableObjects],
    subscribedObjects: [...runtime.subscribedObjects],
    status: cloneValue(runtime.status),
    eventtime: runtime.eventtime,
    motionSources: {
      steppers: [...runtime.motionSources.steppers],
      trapq: [...runtime.motionSources.trapq],
    },
    printerState,
    printerStateMessage,
  };
}

function captureComparableState(snapshot) {
  return {
    connected: snapshot.connected,
    printerState: snapshot.printerState,
    printerStateMessage: snapshot.printerStateMessage,
    availableObjects: snapshot.availableObjects,
    motionSteppers: snapshot.motionSources.steppers,
    motionTrapq: snapshot.motionSources.trapq,
  };
}

export function buildRuntimeObjectSubscription(availableObjects = []) {
  const available = new Set(Array.isArray(availableObjects) ? availableObjects : []);
  const objects = {};
  const includeRequired = available.size === 0;

  for (const objectName of REQUIRED_RUNTIME_OBJECTS) {
    if (includeRequired || available.has(objectName)) {
      objects[objectName] = null;
    }
  }
  for (const objectName of OPTIONAL_RUNTIME_OBJECTS) {
    if (available.has(objectName)) {
      objects[objectName] = null;
    }
  }
  return objects;
}

export class KlippyRuntimeState extends EventEmitter {
  constructor({
    client,
    clientInfo = null,
    objectSubscriptionId = DEFAULT_OBJECT_SUBSCRIPTION_ID,
    gcodeOutputSubscriptionId = DEFAULT_GCODE_OUTPUT_SUBSCRIPTION_ID,
  } = {}) {
    super();
    if (!client) {
      throw new Error('KlippyRuntimeState requires a KlippyApiClient instance.');
    }

    this.client = client;
    this.clientInfo = clientInfo;
    this.objectSubscriptionId = objectSubscriptionId;
    this.gcodeOutputSubscriptionId = gcodeOutputSubscriptionId;

    this.connected = false;
    this.primed = false;
    this.info = null;
    this.availableObjects = [];
    this.subscribedObjects = [];
    this.objectSubscriptionKey = '';
    this.gcodeOutputSubscribed = false;
    this.eventtime = null;
    this.status = {};
    this.objectEventtimes = new Map();
    this.motionSources = {
      steppers: [],
      trapq: [],
    };
    this.primePromise = null;

    this.client.on('connected', () => {
      this.connected = true;
      this._emitChangeEvents();
    });

    this.client.on('disconnected', () => {
      this.connected = false;
      this.primed = false;
      this._emitChangeEvents();
    });
  }

  getSnapshot() {
    return summarizeSnapshot(this);
  }

  getPrinterState() {
    return this.status.webhooks?.state || this.info?.state || null;
  }

  getPrinterStateMessage() {
    return this.status.webhooks?.state_message || this.info?.state_message || null;
  }

  async prime() {
    if (this.primePromise) {
      return this.primePromise;
    }
    const nextPrime = (async () => {
      await this.client.waitForConnection();
      const info = await this.client.request('info', this.clientInfo ? {
        client_info: this.clientInfo,
      } : {});
      this._applyInfo(info);
      const objectsList = await this.client.request('objects/list', {});
      const availableObjects = normalizeStringList(objectsList?.objects);
      this._setAvailableObjects(availableObjects);
      await this._ensureSubscriptions(availableObjects);
      this.primed = true;
      this._emitChangeEvents();
      return {
        info,
        objectsList: {
          objects: [...availableObjects],
        },
        snapshot: this.getSnapshot(),
      };
    })();
    this.primePromise = nextPrime;
    try {
      return await nextPrime;
    } finally {
      if (this.primePromise === nextPrime) {
        this.primePromise = null;
      }
    }
  }

  async refreshInfo({ includeClientInfo = false } = {}) {
    const info = await this.client.request('info', includeClientInfo && this.clientInfo ? {
      client_info: this.clientInfo,
    } : {});
    this._applyInfo(info);
    return info;
  }

  async waitForReady(timeoutMs = 15_000) {
    const deadline = Date.now() + Math.max(1, timeoutMs);
    while (Date.now() < deadline) {
      await this.prime();
      const state = this.getPrinterState();
      if (state === 'ready') {
        return this.getSnapshot();
      }
      if (state === 'shutdown' || state === 'error') {
        throw new Error(this.getPrinterStateMessage() || `Klippy is in ${state} state.`);
      }
      await new Promise((resolve) => setTimeout(resolve, 200));
      await this.refreshInfo();
    }
    const details = this.getPrinterStateMessage() || this.getPrinterState() || 'unknown state';
    throw new Error(`Klippy did not become ready within ${timeoutMs}ms (${details}).`);
  }

  _applyInfo(info) {
    this.info = cloneValue(info || {});
    this._emitChangeEvents();
  }

  _setAvailableObjects(objects) {
    const nextObjects = normalizeStringList(objects);
    if (haveSameStrings(this.availableObjects, nextObjects)) {
      return;
    }
    this.availableObjects = nextObjects;
    this._emitChangeEvents();
  }

  async _ensureSubscriptions(availableObjects) {
    const objectSubscription = buildRuntimeObjectSubscription(availableObjects);
    const nextObjectKey = buildObjectKey(objectSubscription);

    if (nextObjectKey !== this.objectSubscriptionKey) {
      await this.client.subscribe(
        'objects/subscribe',
        { objects: objectSubscription },
        (params) => this._handleStatusPayload(params),
        {
          id: this.objectSubscriptionId,
          onResponse: (result) => this._handleStatusPayload(result),
        },
      );
      this.objectSubscriptionKey = nextObjectKey;
      this.subscribedObjects = Object.keys(objectSubscription).sort();
      this._emitChangeEvents();
    }

    if (!this.gcodeOutputSubscribed) {
      await this.client.subscribe(
        'gcode/subscribe_output',
        {},
        (params) => this._handleGcodeOutput(params),
        { id: this.gcodeOutputSubscriptionId },
      );
      this.gcodeOutputSubscribed = true;
    }
  }

  _handleStatusPayload(payload = {}) {
    const payloadEventtime = Number.isFinite(payload?.eventtime) ? payload.eventtime : null;
    const nextStatus = payload?.status;
    if (isPlainObject(nextStatus)) {
      for (const [objectName, value] of Object.entries(nextStatus)) {
        const lastEventtime = this.objectEventtimes.get(objectName);
        if (payloadEventtime !== null
          && Number.isFinite(lastEventtime)
          && payloadEventtime < lastEventtime) {
          continue;
        }
        this.status[objectName] = deepMerge(this.status[objectName], value);
        if (payloadEventtime !== null) {
          this.objectEventtimes.set(objectName, payloadEventtime);
        }
      }
    }
    if (payloadEventtime !== null && (!Number.isFinite(this.eventtime) || payloadEventtime > this.eventtime)) {
      this.eventtime = payloadEventtime;
    }
    this._refreshMotionSources();
    this._emitChangeEvents();
  }

  _handleGcodeOutput(params = {}) {
    if (typeof params.response !== 'string' || params.response.length === 0) {
      return;
    }
    this.emit('gcode-output', {
      response: params.response,
      snapshot: this.getSnapshot(),
    });
  }

  _refreshMotionSources() {
    const motionReport = this.status.motion_report || {};
    const nextSteppers = normalizeStringList(motionReport.steppers);
    const nextTrapq = normalizeStringList(motionReport.trapq);
    if (haveSameStrings(this.motionSources.steppers, nextSteppers)
      && haveSameStrings(this.motionSources.trapq, nextTrapq)) {
      return;
    }
    this.motionSources = {
      steppers: nextSteppers,
      trapq: nextTrapq,
    };
  }

  _emitChangeEvents() {
    const snapshot = this.getSnapshot();
    const previous = this._lastComparableState || captureComparableState(snapshot);
    const nextComparable = captureComparableState(snapshot);
    this._lastComparableState = nextComparable;

    this.emit('update', snapshot);

    if (previous.connected !== nextComparable.connected) {
      this.emit('connection-changed', {
        connected: nextComparable.connected,
        snapshot,
      });
    }
    if (previous.printerState !== nextComparable.printerState
      || previous.printerStateMessage !== nextComparable.printerStateMessage) {
      this.emit('printer-state-changed', {
        previousState: previous.printerState,
        previousStateMessage: previous.printerStateMessage,
        state: nextComparable.printerState,
        stateMessage: nextComparable.printerStateMessage,
        snapshot,
      });
    }
    if (!haveSameStrings(previous.availableObjects, nextComparable.availableObjects)) {
      this.emit('available-objects-changed', {
        availableObjects: [...nextComparable.availableObjects],
        snapshot,
      });
    }
    if (!haveSameStrings(previous.motionSteppers, nextComparable.motionSteppers)
      || !haveSameStrings(previous.motionTrapq, nextComparable.motionTrapq)) {
      this.emit('motion-sources-changed', {
        steppers: [...nextComparable.motionSteppers],
        trapq: [...nextComparable.motionTrapq],
        snapshot,
      });
    }
  }
}
