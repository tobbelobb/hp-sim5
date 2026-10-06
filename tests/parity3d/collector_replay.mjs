// Replay a native collector's real firmware schedule through production browser physics and endpoint.
import fs from 'node:fs';
import { World } from '../../src/js/cable_joints_3d/ecs.js';
import { OpenText } from '../../src/js/usd/stage.js';
import { parseStage, readMachineSceneSpec, validateMachineSceneSpec, buildEntityPlan, applyEntityPlan } from '../../hp-sim-3d/app/scene/machineScenePipeline.js';
import { registerSimulationSystems } from '../../hp-sim-3d/app/simulationSystems.js';
import { RemoteSpoolSystem } from '../../hp-sim-3d/app/remoteSpoolSystem.js';
import { createExternalCommandController } from '../../hp-sim-3d/app/externalCommandSocket.js';

const [scenePath, eventsPath, outputPath] = process.argv.slice(2);
const world = new World();
const stage = OpenText(fs.readFileSync(scenePath, 'utf8'));
const checked = validateMachineSceneSpec(readMachineSceneSpec(parseStage(stage), '/World/HangprinterScene'));
if (!checked.valid) throw new Error(checked.warnings.join('\n'));
applyEntityPlan(world, buildEntityPlan(checked));
registerSimulationSystems(world);
const remote = world.systems.find(system => system instanceof RemoteSpoolSystem);
let response;
class Socket {
  static OPEN = 1;
  readyState = 1;
  addEventListener() {}
  send(value) { response = JSON.parse(value); }
}
const controller = createExternalCommandController({
  world, url: 'ws://replay', WebSocketCtor: Socket, logger: { info() {}, log() {} },
  commands: { getRemoteSystem: () => remote, pushExternalCommands(batch) { batch.forEach(command => remote.addCommand(command)); return true; } },
  runtime: { resume() {} },
});
controller.connect();
let step = 0;
const samples = [];
for (const line of fs.readFileSync(eventsPath, 'utf8').trim().split('\n')) {
  const event = JSON.parse(line);
  while (step < event.step) { world.update(.002); step += 1; }
  if (event.type === 'bridge_payload' && event.payload.type !== 'encoder_request') {
    controller.handlePayload(event.payload);
  }
  if (event.type === 'encoder_response' && event.axes.length) {
    controller.handlePayload({ type: 'encoder_request', requestId: event.request_id, axes: event.axes });
    samples.push({ step, axes: event.axes, native: event.angles_deg, browser: response.anglesDeg,
      max_abs_error_deg: Math.max(...event.angles_deg.map((value, i) => Math.abs(value - response.anglesDeg[i]))) });
  }
}
const maxError = Math.max(...samples.map(s => s.max_abs_error_deg));
const result = { steps: step, encoder_reads: samples.length, max_abs_error_deg: maxError,
  tolerance_deg: .01, passed: samples.length > 0 && Number.isFinite(maxError) && maxError <= .01, samples };
fs.writeFileSync(outputPath, JSON.stringify(result));
console.log(JSON.stringify({ steps: step, encoder_reads: samples.length, max_abs_error_deg: maxError, passed: result.passed }));
if (!result.passed) process.exitCode = 1;
