// Negative control: the browser FlightRecorder clock follows physics steps and
// does not advance while paused (the runner stops calling update), and a scene
// generation change restarts the sample clock. No browser or server is used.
import { World } from '../../../../src/js/cable_joints_3d/ecs.js';
import { FlightRecorder } from '../../../../hp-sim-3d/app/flightRecorder.js';

class FakeSocket {
  constructor() { this.readyState = 0; this.listeners = new Map(); this.samples = []; }
  addEventListener(type, listener) { this.listeners.set(type, listener); }
  emit(type, message) { this.listeners.get(type)?.({ data: JSON.stringify(message) }); }
  send(message) { this.samples.push(JSON.parse(message)); }
  close() { this.readyState = 3; }
}

const world = new World();
world.setResource('sceneGeneration', 1);
const recorder = new FlightRecorder({ world, WebSocketClass: FakeSocket, source: 'headless' });
recorder.connect();
const socket = recorder.socket;
socket.readyState = 1;
socket.emit('open');
const clock = () => ({ step: recorder.step, time: recorder.time });
const seen = [clock()];
for (let step = 0; step < 3; step++) { recorder.update(world, 0.002); seen.push(clock()); }
const afterRun = seen.at(-1);
// Paused: the runner does not call world.update, so the recorder is not ticked.
const afterPause = clock();
// Resume: the next completed step continues the clock.
recorder.update(world, 0.002);
const afterResume = clock();
// Scene reset: generation changes; the recorder restarts at step 1 / time dt.
world.setResource('sceneGeneration', 2);
recorder.update(world, 0.002);
const afterReset = clock();
const result = { seen, afterPause, afterResume, afterReset, samples: socket.samples.length };
console.log(JSON.stringify(result, null, 2));

const failures = [];
if (afterRun.step !== 3 || Math.abs(afterRun.time - 0.006) > 1e-12) failures.push('run clock');
if (afterPause.step !== 3 || Math.abs(afterPause.time - 0.006) > 1e-12) failures.push('pause advanced the clock');
if (afterResume.step !== 4 || Math.abs(afterResume.time - 0.008) > 1e-12) failures.push('resume did not continue the clock');
if (afterReset.step !== 1 || Math.abs(afterReset.time - 0.002) > 1e-12) failures.push('generation change did not restart the clock');
if (socket.samples.filter(sample => sample.step !== undefined).map(sample => sample.step).join(',') !== '0,1,2,3,4,1') {
  failures.push('sample steps do not match the clock');
}
recorder.disconnect();
if (failures.length) {
  console.error('FAIL (pause/resume clock): ' + failures.join('; '));
  process.exit(1);
}
