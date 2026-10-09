import { captureFlightRecorderSnapshot } from './flightRecorderSnapshot.js';

const MAX_PENDING_STEPS = 32;

export class FlightRecorder {
  constructor({ world, button, url = 'ws://127.0.0.1:9877', WebSocketClass = globalThis.WebSocket }) {
    this.world = world;
    this.button = button;
    this.url = url;
    this.WebSocketClass = WebSocketClass;
    this.socket = null;
    this.pending = 0;
    this.generation = null;
    this.step = 0;
    this.time = 0;
    this.sampleStride = 1;
    this.geometryDetail = 'full';
    this.lastSentStep = null;
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
      if (message.type === 'ack') this.pending = Math.max(0, this.pending - 1);
    });
    socket.addEventListener('close', () => {
      if (this.socket !== socket) return;
      this.socket = null;
      this.pending = 0;
      this.setStatus('Rerun: disconnected', false);
    });
    socket.addEventListener('error', () => {
      if (this.socket === socket) this.setStatus('Rerun: connection error');
    });
  }

  disconnect() {
    this.socket?.close();
    this.socket = null;
    this.pending = 0;
    this.setStatus('Rerun', false);
  }

  // Backpressure applies to sampled physics, never to G-code dispatch.
  readyForStep() {
    return !this.socket || (this.socket.readyState === 1 && this.pending < MAX_PENDING_STEPS);
  }

  recordEvent(kind, payload) {
    if (!this.socket || this.socket.readyState !== 1 || !this.extendedReference) return;
    const clock = this.world.getResource('researchClock');
    this.socket.send(JSON.stringify({ version: 1, type: 'autocal_event', source: 'browser',
      wall_time_ms: Date.now(), wall_time_source: 'browser.Date.now',
      monotonic_ms: performance.now(), kind, payload, sim_time_s: clock?.time ?? null,
      sim_time_source: clock ? 'browser.researchClock' : 'unavailable',
      session: this.session, generation: this.world.getResource('sceneGeneration') || 0,
      recorder_sim_time_s: this.time }));
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
    this.socket.send(JSON.stringify({
      sample_stride: this.sampleStride, geometry_detail: this.geometryDetail,
      research_clock: world.getResource('researchClock') || null,
      wall_time_ms: Date.now(), speed_scale: world.getResource('timeScale') || 1,
      version: 1, session: this.session, generation, step: this.step, time: this.time, dt,
      ...captureFlightRecorderSnapshot(world, { geometryDetail: this.geometryDetail }),
    }));
    this.pending += 1;
  }
}
