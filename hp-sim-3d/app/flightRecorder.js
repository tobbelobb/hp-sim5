import { captureFlightRecorderSnapshot } from './flightRecorderSnapshot.js';

const MAX_PENDING_STEPS = 32;
const EVENT_BATCH_SIZE = 256;
const EVENT_BATCH_BYTES = 1024 * 1024;
const MAX_PENDING_EVENTS = 65536;
const MAX_PENDING_EVENT_BYTES = 32 * 1024 * 1024;

export class FlightRecorder {
  constructor({ world, button, url = 'ws://127.0.0.1:9877', WebSocketClass = globalThis.WebSocket, source = 'browser' }) {
    this.world = world;
    this.button = button;
    this.url = url;
    this.WebSocketClass = WebSocketClass;
    this.source = source;
    this.socket = null;
    this.pending = 0;
    this.pendingEvents = 0;
    this.pendingEventBytes = 0;
    this.eventBatch = [];
    this.eventBatchBytes = 0;
    this.eventBatches = [];
    this.eventTimer = null;
    this.eventEncoder = new TextEncoder();
    this.ackSequence = 0;
    this.error = null;
    this.generation = null;
    this.step = 0;
    this.time = 0;
    this.sampleStride = 1;
    this.geometryDetail = 'full';
    this.lastSentStep = null;
    this.acknowledged = false;
    button?.addEventListener('click', () => this.socket ? this.disconnect() : this.connect());
  }

  setStatus(label, active = Boolean(this.socket)) {
    if (!this.button) return;
    this.button.textContent = label;
    this.button.setAttribute('aria-pressed', String(active));
  }

  connect() {
    if (this.socket) return;
    const socket = new this.WebSocketClass(this.url);
    this.socket = socket;
    this.generation = null;
    this.session = globalThis.crypto.randomUUID();
    this.pending = 0;
    this.pendingEvents = 0;
    this.pendingEventBytes = 0;
    this.eventBatch = [];
    this.eventBatchBytes = 0;
    this.eventBatches = [];
    this.error = null;
    this.acknowledged = false;
    this.setStatus('Rerun: connecting');
    socket.addEventListener('open', () => {
      if (this.socket !== socket) return;
      this.setStatus('Rerun: recording');
      this.update(this.world, 0);
    });
    socket.addEventListener('message', (event) => {
      if (this.socket !== socket) return;
      const message = JSON.parse(event.data);
      if (message.type === 'recording_config') {
        this.sampleStride = message.sample_stride;
        this.geometryDetail = message.geometry_detail;
      }
      if (message.type === 'ack') {
        this.ackSequence += 1;
        this.acknowledged = true;
        this.pending = Math.max(0, this.pending - 1);
      }
      if (message.type === 'event_ack') {
        const batch = this.eventBatches[0];
        if (!batch || message.count !== batch.count) {
          this.fail(new Error('Invalid flight recorder event acknowledgement'));
          return;
        }
        this.eventBatches.shift();
        this.ackSequence += 1;
        this.pendingEvents -= batch.count;
        this.pendingEventBytes -= batch.bytes;
      }
    });
    socket.addEventListener('close', (event) => {
      if (this.socket !== socket) return;
      this.error = event.reason ? `Flight recorder disconnected: ${event.reason}` : 'Flight recorder disconnected';
      clearTimeout(this.eventTimer);
      this.eventTimer = null;
      this.socket = null;
      this.pending = 0;
      this.setStatus('Rerun: disconnected', false);
    });
    socket.addEventListener('error', () => {
      if (this.socket === socket) {
        this.error = 'Flight recorder connection error';
        this.setStatus('Rerun: connection error');
      }
    });
  }

  disconnect() {
    this.flushEvents();
    clearTimeout(this.eventTimer);
    this.eventTimer = null;
    this.socket?.close();
    this.socket = null;
    this.pending = 0;
    this.error = null;
    this.setStatus('Rerun', false);
  }

  fail(error) {
    this.error = error.message;
    clearTimeout(this.eventTimer);
    this.eventTimer = null;
    this.socket?.close();
    this.socket = null;
    this.setStatus('Rerun: failed', false);
  }

  async ensureConnected(url, timeoutMs = 5000) {
    if (this.socket && this.url !== url) throw new Error(`Already recording to ${this.url}`);
    this.url = url;
    this.connect();
    const socket = this.socket;
    if (socket.readyState === 1 && this.acknowledged) return;
    try {
      await new Promise((resolve, reject) => {
        const timer = setTimeout(() => finish(new Error('Rerun connection timed out')), timeoutMs);
        const acknowledged = event => {
          if (JSON.parse(event.data).type === 'ack') finish();
        };
        const failed = () => finish(new Error('Rerun connection failed'));
        function finish(error) {
          clearTimeout(timer);
          socket.removeEventListener('message', acknowledged);
          socket.removeEventListener('error', failed);
          socket.removeEventListener('close', failed);
          if (error) reject(error); else resolve();
        }
        socket.addEventListener('message', acknowledged);
        socket.addEventListener('error', failed);
        socket.addEventListener('close', failed);
      });
    } catch (error) {
      if (this.socket === socket) this.disconnect();
      throw error;
    }
  }

  // Backpressure applies to sampled physics, never to G-code dispatch.
  readyForStep() {
    return !this.error && (!this.socket || (this.socket.readyState === 1 && this.pending < MAX_PENDING_STEPS));
  }

  async drain(timeoutMs = 10000, { all = true, wallTimeoutMs = 120000 } = {}) {
    this.flushEvents();
    const socket = this.socket;
    let sequence = this.ackSequence;
    let deadline = performance.now() + timeoutMs;
    const wallDeadline = performance.now() + wallTimeoutMs;
    while (true) {
      if (!socket || this.socket !== socket || socket.readyState !== 1) throw new Error(this.error || 'Flight recorder disconnected');
      if (all ? !this.pending && !this.pendingEvents : this.readyForStep()) return;
      if (sequence !== this.ackSequence) {
        sequence = this.ackSequence;
        deadline = performance.now() + timeoutMs;
      }
      if (performance.now() > deadline || performance.now() > wallDeadline) throw new Error('Flight recorder did not acknowledge samples');
      await new Promise(resolve => setTimeout(resolve, 5));
    }
  }

  recordEvent(kind, payload) {
    if (this.error) throw new Error(this.error);
    if (!this.socket || this.socket.readyState !== 1 || !this.extendedReference) return;
    const clock = this.world.getResource('researchClock');
    const data = JSON.stringify({ version: 1, type: 'autocal_event', source: this.source,
      wall_time_ms: Date.now(), wall_time_source: `${this.source}.Date.now`,
      monotonic_ms: performance.now(), kind, payload, sim_time_s: clock?.time ?? null,
      sim_time_source: clock ? `${this.source}.researchClock` : 'unavailable',
      session: this.session, generation: this.world.getResource('sceneGeneration') || 0,
      recorder_sim_time_s: this.time });
    const bytes = this.eventEncoder.encode(data).byteLength;
    if (this.pendingEvents >= MAX_PENDING_EVENTS || this.pendingEventBytes + bytes > MAX_PENDING_EVENT_BYTES) {
      const error = new Error('Flight recorder event queue exceeded its limit; capture is incomplete');
      this.fail(error);
      throw error;
    }
    if (this.eventBatchBytes + bytes > EVENT_BATCH_BYTES) this.flushEvents();
    this.eventBatch.push(data);
    this.eventBatchBytes += bytes;
    this.pendingEvents += 1;
    this.pendingEventBytes += bytes;
    if (this.eventBatch.length >= EVENT_BATCH_SIZE || this.eventBatchBytes >= EVENT_BATCH_BYTES) this.flushEvents();
    else if (!this.eventTimer) {
      this.eventTimer = setTimeout(() => this.flushEvents(), 20);
      this.eventTimer.unref?.();
    }
  }

  flushEvents() {
    clearTimeout(this.eventTimer);
    this.eventTimer = null;
    if (!this.eventBatch.length || this.socket?.readyState !== 1) return;
    try {
      this.socket.send(`{"version":1,"type":"autocal_event_batch","events":[${this.eventBatch.join(',')}]}`);
      this.eventBatches.push({ count: this.eventBatch.length, bytes: this.eventBatchBytes });
      this.eventBatch = [];
      this.eventBatchBytes = 0;
    } catch (error) {
      this.fail(error);
      throw error;
    }
  }

  update(world, dt) {
    if (!this.socket || this.socket.readyState !== 1) return;
    const generation = world.getResource('sceneGeneration') || 0;
    if (generation !== this.generation) {
      this.generation = generation;
      this.step = 0;
      this.time = 0;
      this.lastSentStep = null;
      this.recordEvent('scene_context', this.contextProvider?.() || {});
    }
    if (dt > 0) {
      this.step += 1;
      this.time += dt;
    }
    if (this.lastSentStep !== null && this.step - this.lastSentStep < this.sampleStride) return;
    this.lastSentStep = this.step;
    // Preserve event/sample order, including scene context before step zero.
    this.flushEvents();
    this.socket.send(JSON.stringify({
      source: this.source,
      sample_stride: this.sampleStride, geometry_detail: this.geometryDetail,
      research_clock: world.getResource('researchClock') || null,
      wall_time_ms: Date.now(), speed_scale: world.getResource('timeScale') || 1,
      version: 1, session: this.session, generation, step: this.step, time: this.time, dt,
      ...captureFlightRecorderSnapshot(world, { geometryDetail: this.geometryDetail }),
    }));
    this.pending += 1;
  }
}
