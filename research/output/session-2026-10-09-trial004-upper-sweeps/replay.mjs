import fs from 'node:fs';
import {createHeadlessWorld} from '../../../hp-sim-3d/app/headlessWorld.js';
import {EncoderComponent} from '../../../src/js/cable_joints_3d/ecs.js';
import {StepperMotorComponent} from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import {captureFlightRecorderSnapshot} from '../../../hp-sim-3d/app/flightRecorderSnapshot.js';
const {world,remote,dt}=createHeadlessWorld(fs.readFileSync(new URL('./scene.usda',import.meta.url),'utf8'));
world.setResource('enableLayering',true);world.setResource('layeringFrictionEffectiveRadius',true);
const commands=JSON.parse(fs.readFileSync(new URL('./commands.json',import.meta.url),'utf8'));
const axes=['A','B','C','D'];let index=0,step=0,lastProgress=performance.now();
const rows=[];
while(step*dt<3200){
 while(index<commands.length && commands[index].time<=step*dt+1e-7)remote.addCommand(commands[index++].command);
 world.update(dt);step++;
 if(step%100===0 && step*dt>=2760){
  const snapshot=captureFlightRecorderSnapshot(world,{geometryDetail:'compact'});
  rows.push({sim_s:step*dt,motors:axes.map(axis=>{
   const id=remote.axisToEntity[axis][0], m=world.getComponent(id,StepperMotorComponent);
   return {axis,encoder_deg:world.getComponent(id,EncoderComponent).angle*180/Math.PI,target_deg:(m.commandedAngle-m.deltaAngle)*180/Math.PI,torque_mode:m.torqueMode,torque_nm:m.targetTorque,missed_steps:m.currentMissedSteps,max_missed_steps:m.missedSteps};
  }),effector:snapshot.frames.find(f=>f.kind==='effector').position,cables:snapshot.cables.map(c=>({name:c.name,lengths:c.lengths,force_n:Math.max(...c.segments.map(s=>s.force_n))}))});
 }
 if(performance.now()-lastProgress>20000){console.log('replay simulation seconds',step*dt);lastProgress=performance.now();}
}
fs.writeFileSync(new URL('./replay-results.json',import.meta.url),JSON.stringify(rows));
for(const target of [2771,2796,2828,2850,2918,2968,3017,3097,3163]){
 const row=rows.reduce((a,b)=>Math.abs(b.sim_s-target)<Math.abs(a.sim_s-target)?b:a);
 console.log(JSON.stringify(row));
}
