// C2 under both shipped motor modes (open loop is the hp4 default).
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { ExtruderComponent } from '../../../../hp-sim-3d/app/hangprinter_extruder.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
const config = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');
async function emit(g, wdt) { const c = new MoveCommander(); c.configure(config); c.setAsapMode(true); c.dt = wdt ?? 0.002; const q=[]; c.sendCommand=async x=>q.push(x);
  await c.run(new ReadableStream({start(ctl){ctl.enqueue(new TextEncoder().encode(g));ctl.close();}})); return q; }
async function run(label, gcode, closed) {
  const { world, remote, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, requireRrfTimestep: false });
  const axes=['A','B','C','D'];
  const st=(a)=>world.getComponent(remote.axisToEntity[a][0], StepperMotorComponent);
  const ext=()=>world.getComponent([...world.query([ExtruderComponent])][0], ExtruderComponent);
  const tip=()=>{const p=ext()?.tipPos||ext()?.centerPos;return p?[p.x,p.y,p.z]:null;};
  if (closed) for (const a of axes) st(a).closedLoop = true;
  for(let i=0;i<300;i++) world.update(dt);
  const p0=tip(); const cmds=await emit(gcode, dt);
  let maxD=0; for(const c of cmds){remote.addCommand(c);world.update(dt);const p=tip();if(p)maxD=Math.max(maxD,Math.hypot(p[0]-p0[0],p[1]-p0[1],p[2]-p0[2]));}
  for(let i=0;i<3000;i++) world.update(dt);
  const p1=tip();
  console.log(label.padEnd(34), 'peak', (maxD*1000).toFixed(3).padStart(7), 'mm  final X', (p1[0]*1000).toFixed(3).padStart(8), 'mm  missed A/B/C',
    axes.slice(0,3).map(a=>st(a).missedSteps).join('/'));
}
for (const closed of [false, true]) {
  const tag = closed ? ' [closed loop]' : ' [open loop, default]';
  await run('G1 X5 (equivalent)'+tag, 'G1 X5 Y0 Z0 F600\n', closed);
  await run('G92 X5 + G1 X10'+tag, 'G92 X5\nG1 X10 Y0 Z0 F600\n', closed);
  await run('G92 X5 only'+tag, 'G92 X5\n', closed);
}
