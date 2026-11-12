// Headless Print Logo replay (native engine) with the production QualityMonitor.
// usage: node logo_qc.mjs <label> <hp3|hp4> [closed=1] [seconds=all]
// env: SCENE=<usda override>  CAN=<can override>  CONFIG=<rrf config override>
// Writes <label>.json: QC metrics, extrusion samples [x,y,z,len,step] and 50 ms subsampled state.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { SpoolStateComponent } from '../../../../hp-sim-3d/app/hangprinter_spools.js';
import { getMachineMotorDiagnostics } from '../../../../hp-sim-3d/app/motor-diagnostics.js';
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
import { EncoderComponent, PositionComponent, OrientationComponent, RigidBodyComponent, SceneEntityInfoComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
import { parseGcodeText } from './gcode_parse_copy.mjs';
const root = new URL('../../../../', import.meta.url);
const [label, machine, closedText = '1', secondsText] = process.argv.slice(2);
const can = process.env.CAN ?? `public/RRF_CAN_commands/Hangprinter_logo6_${machine}_no_buildup.can`;
const config = process.env.CONFIG ?? `RRF/run/vsd/sys/config_${machine}.g`;
const scenePath = process.env.SCENE ?? `public/usd_scenes/${machine}_rigid_body.usda`;
const maxSteps = secondsText ? Math.round(Number(secondsText) / .002) : Infinity;
globalThis.self = { addEventListener() {} };
globalThis.postMessage = m => { if (m.type === 'error') throw new Error(m.message); };
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const player = new RrfCanPlayer();
player.setDt(.002); player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(new URL(config, root), 'utf8')));
const commands = [];
player.sendCommand = async c => { if (commands.length < maxSteps) commands.push(c); };
await player.run(Readable.toWeb(fs.createReadStream(new URL(can, root))), FileFormat.RRF_CAN_BINARY);
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(new URL(scenePath, root), 'utf8'),
  { lineLayeringEnabled: false, closedLoopMotorsEnabled: closedText === '1', physicsSubsteps: 1 });
const motors = world.query([StepperMotorComponent]).map(id => ({ id, axis: world.getComponent(id, SpoolStateComponent).axis }))
  .sort((a, b) => a.axis.localeCompare(b.axis));
const body = world.query([RigidBodyComponent])[0];
const parsed = parseGcodeText(fs.readFileSync(new URL('public/gcode/Hangprinter_logo6.gcode', root), 'utf8'));
const monitor = new QualityMonitor();
monitor.setReferenceSegments(parsed.segments.map(s => ({ start: s.start.map(v => v * .001), end: s.end.map(v => v * .001), extrusion: s.extrusion })));
monitor.setEnabled(false);
monitor.setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world));
let step = 0;
const ext = [];
remote.setExtrusionListener(e => { ext.push([e.pos[0], e.pos[1], e.pos[2], e.length, step]); monitor.recordExtrusion({ pos: e.pos, length: e.length }); });
remote.commands = commands.slice();
const paths = world.query([CablePathComponent]);
const pathNames = paths.map(pid => world.getComponent(pid, SceneEntityInfoComponent)?.name ?? String(pid));
const sub = { t: [], pos: [], quat: [], phase: [], cmd: [], tension: [], pathNames };
const t0 = performance.now();
for (step = 1; step <= commands.length; step++) {
  world.update(dt);
  if (step % 25 === 0) {
    const p = world.getComponent(body, PositionComponent).pos;
    const o = world.getComponent(body, OrientationComponent);
    const q = o.quaternion;
    sub.t.push(step * dt); sub.pos.push([p.x, p.y, p.z]); sub.quat.push([q.x, q.y, q.z, q.w]);
    sub.phase.push(motors.map(m => world.getComponent(m.id, EncoderComponent).angle - world.getComponent(m.id, StepperMotorComponent).commandedAngle));
    sub.cmd.push(motors.map(m => world.getComponent(m.id, StepperMotorComponent).commandedAngle));
    sub.tension.push(paths.map(pid => { const js = world.getComponent(pid, CablePathComponent).jointEntities; return world.getComponent(js[js.length - 1], CableJointComponent).constraintForceMagnitude; }));
  }
  if (world.getResource('errorState')?.hasError) throw new Error(`error at ${step}`);
}
const wall = (performance.now() - t0) / 1000;
monitor.setEnabled(true);
monitor.runFinalCheck();
const metrics = monitor.getMetrics();
fs.writeFileSync(new URL(`runs/${label}.json`, import.meta.url), JSON.stringify({ label, machine, can, config, scene: scenePath, closed: closedText === '1', steps: commands.length, wall, axes: motors.map(m => m.axis), metrics, ext, sub }));
const m = metrics;
console.log(JSON.stringify({ label, steps: commands.length, wall: +wall.toFixed(1), metrics: m }));
world.getResource('cableSolverBackend')?.dispose?.();
