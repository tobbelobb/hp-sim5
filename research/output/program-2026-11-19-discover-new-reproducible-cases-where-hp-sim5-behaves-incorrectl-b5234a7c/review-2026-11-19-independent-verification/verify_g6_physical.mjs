// C1 physical side: does the mover end where it started after tighten.gcode?
import fs from 'node:fs';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { ExtruderComponent } from '../../../../hp-sim-3d/app/hangprinter_extruder.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
const config = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const scene = fs.readFileSync(new URL('public/usd_scenes/hp4.usda', ROOT), 'utf8');
const { world, remote, dt } = createHeadlessWorld(scene, { lineLayeringEnabled: false, requireRrfTimestep: false });
const axes=['A','B','C','D'];
const st=(a)=>world.getComponent(remote.axisToEntity[a][0], StepperMotorComponent);
const ext=()=>world.getComponent([...world.query([ExtruderComponent])][0], ExtruderComponent);
const tip=()=>{const p=ext()?.tipPos||ext()?.centerPos;return p?[p.x,p.y,p.z]:null;};
const c = new MoveCommander(); c.configure(config); c.setAsapMode(true); c.dt = dt;
const cmds=[]; c.sendCommand=async x=>cmds.push(x);
await c.run(new ReadableStream({start(ctl){ctl.enqueue(new TextEncoder().encode('G6 A-42 B-42 C-42 F5000\n'));ctl.close();}}));
for(let i=0;i<300;i++) world.update(dt);
const p0=tip();
let peak=0; for(const cmd of cmds){remote.addCommand(cmd);world.update(dt);const p=tip();if(p&&p0)peak=Math.max(peak,Math.hypot(p[0]-p0[0],p[1]-p0[1],p[2]-p0[2]));}
console.log('peak |tip-p0| during G6:',(peak*1000).toFixed(2),'mm');
for(let i=0;i<3000;i++) world.update(dt);
const p1=tip();
console.log('cmds',cmds.length,'tip start(mm)',p0.map(v=>(v*1000).toFixed(3)),'end(mm)',p1.map(v=>(v*1000).toFixed(3)),
  'net |d| mm',(Math.hypot(p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2])*1000).toFixed(3),
  'missed A/B/C',axes.slice(0,3).map(a=>st(a).missedSteps).join('/'));
