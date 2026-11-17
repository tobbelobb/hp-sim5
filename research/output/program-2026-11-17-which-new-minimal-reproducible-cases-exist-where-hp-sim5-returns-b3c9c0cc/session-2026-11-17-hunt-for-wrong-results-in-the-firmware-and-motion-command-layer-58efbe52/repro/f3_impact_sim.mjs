#!/usr/bin/env node
// Follow-up F3, end-to-end physical impact: drive the shipped slideprinter
// scene through the production physics path twice -- once with the motor
// angles implied by the firmware's exact step stream, once with the angles the
// Klipper MCU player actually delivers -- and compare the effector pose.
//
//   node repro/f3_impact_sim.mjs [stream.txt]
//
// Angles are the exact/player cumulative steps of the stream times
// STEP_ANGLE_RAD, exactly what KlipperMcuCommandPlayer posts as
// {type:'Move', A, B, C} to RemoteSpoolSystem.  Physics: the checked-in Wasm
// cable solver on public/usd_scenes/slideprinter.usda, zero gravity (the
// machine is planar: anchors and spools all at z = 0), which is what the
// benchmark tests use to isolate motion.
//
// Step totals come from repro/f3_player_totals.mjs.

import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

globalThis.postMessage = () => {};
globalThis.self = { addEventListener() {}, onmessage: null, postMessage: globalThis.postMessage, console };

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');
const { createHeadlessWorld } = await import(path.join(ROOT, 'hp-sim-3d/app/headlessWorld.js'));
const { WasmPhysicsEngine } = await import(path.join(ROOT, 'src/js/cable_joints_3d/wasm_physics_engine.js'));
const { PositionComponent, OrientationComponent, RigidBodyComponent } = await import(
  path.join(ROOT, 'src/js/cable_joints_3d/ecs.js'));
const { STEP_ANGLE_RAD } = await import(
  path.join(ROOT, 'integrations/klipper/klipperFirmwareModel.js'));
const Vector3 = (await import(path.join(ROOT, 'src/js/cable_joints_3d/vector3.js'))).default;

const streamPath = process.argv[2]
  ? path.resolve(process.argv[2])
  : path.join(ROOT, 'public/mcu_commands/slideprinter/draw_squares_bigger_no_buildup.txt');

const totalsRaw = execFileSync('node',
  [path.join(__dirname, 'f3_player_totals.mjs'), streamPath], { encoding: 'utf8' });
const totals = JSON.parse(totalsRaw.split('\n').filter((l) => l.startsWith('{')).pop());

const scene = fs.readFileSync(path.join(ROOT, 'public/usd_scenes/slideprinter.usda'), 'utf8');
const wasm = new WebAssembly.Module(fs.readFileSync(path.join(ROOT, 'public/wasm/cable_solver.wasm')));
const dt = 1 / 500;              // scene timeCodesPerSecond = 500 (0.002 s, the RRF timestep)

function poseFor(stepsByAxis) {
  const backend = new WasmPhysicsEngine(wasm);
  const { world, remote } = createHeadlessWorld(scene, {
    lineLayeringEnabled: false,
    cableSolverBackend: backend,
  });
  world.setResource('gravity', new Vector3());
  const body = world.query([RigidBodyComponent])[0];
  const angles = {
    A: stepsByAxis.A * STEP_ANGLE_RAD,
    B: stepsByAxis.B * STEP_ANGLE_RAD,
    C: stepsByAxis.C * STEP_ANGLE_RAD,
  };
  // Settle from the authored pose to the commanded pose, then average the last
  // window to remove solver jitter.
  const settleSteps = 4000;
  const avgSteps = 400;
  let sx = 0, sy = 0, sz = 0, sqx = 0, sqy = 0, sqz = 0, sqw = 0;
  for (let i = 0; i < settleSteps + avgSteps; i++) {
    remote.commands = [{ type: 'Move', ...angles }];
    world.update(dt);
    if (i >= settleSteps) {
      const p = world.getComponent(body, PositionComponent).pos;
      const q = world.getComponent(body, OrientationComponent).quaternion;
      sx += p.x; sy += p.y; sz += p.z;
      sqx += q.x; sqy += q.y; sqz += q.z; sqw += q.w;
    }
  }
  const pose = {
    x: sx / avgSteps, y: sy / avgSteps, z: sz / avgSteps,
    q: [sqx / avgSteps, sqy / avgSteps, sqz / avgSteps, sqw / avgSteps],
  };
  backend.dispose();
  return pose;
}

const exactPose = poseFor(totals.exact_steps);
const playerPose = poseFor(totals.player_steps);
const d = [playerPose.x - exactPose.x, playerPose.y - exactPose.y, playerPose.z - exactPose.z];

const round = (v, n = 6) => +(+v).toFixed(n);
console.log(JSON.stringify({
  stream: path.relative(ROOT, streamPath),
  exact_steps: totals.exact_steps,
  player_steps: Object.fromEntries(Object.entries(totals.player_steps).map(([k, v]) => [k, round(v, 4)])),
  exact_effector_m: { x: round(exactPose.x), y: round(exactPose.y), z: round(exactPose.z) },
  player_effector_m: { x: round(playerPose.x), y: round(playerPose.y), z: round(playerPose.z) },
  effector_delta_mm: d.map((v) => round(v * 1000, 4)),
  effector_delta_norm_mm: round(Math.hypot(...d) * 1000, 4),
  effector_delta_direction: {
    along_x: d[0] > 0 ? '+X' : '-X',
    along_y: d[1] > 0 ? '+Y' : '-Y',
  },
}, null, 2));

const lost = Math.hypot(
  (totals.deficit_steps.B || 0) * STEP_ANGLE_RAD,
  (totals.deficit_steps.C || 0) * STEP_ANGLE_RAD);
if (lost < 1e-6) {
  console.error('\nUNEXPECTED: no F3 step loss in this stream');
  process.exit(2);
}
console.error(`\nF3 END-TO-END: effector moves ${(Math.hypot(...d) * 1000).toFixed(4)} mm `
  + `(dx=${(d[0] * 1000).toFixed(4)}, dy=${(d[1] * 1000).toFixed(4)}, dz=${(d[2] * 1000).toFixed(4)} mm) `
  + 'purely because two motors lost their first queue_step batch');
process.exit(1);
