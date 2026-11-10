import fs from 'node:fs';
import {createHeadlessWorld} from '../../../hp-sim-3d/app/headlessWorld.js';
import {EncoderComponent} from '../../../src/js/cable_joints_3d/ecs.js';
import {measureMaxTravelMm} from '../../../autocal/control/behaviors/sweep_data_collection.mjs';
const {world,remote,dt}=createHeadlessWorld(fs.readFileSync(new URL('./scene.usda',import.meta.url),'utf8'));
world.setResource('enableLayering',true);
world.setResource('layeringFrictionEffectiveRadius',true);
const commands=JSON.parse(fs.readFileSync(new URL('./commands.json',import.meta.url),'utf8')).filter(c=>c.time<1566.262-1e-6);
let step=0, index=0;
const axes=['A','B','C','D'];
const angles=()=>axes.map(a=>world.getComponent(remote.axisToEntity[a][0],EncoderComponent).angle*180/Math.PI);
function tick(){
 while(index<commands.length && commands[index].time<=step*dt+1e-7) remote.addCommand(commands[index++].command);
 world.update(dt);step++;
}
while(step*dt<1566.262-1e-7) tick();
const start=step*dt, startAngles=angles();
const polls=JSON.parse(fs.readFileSync(new URL('./poll-times.json',import.meta.url),'utf8'));
const issued=[], readings=[];
let pollIndex=0, rampIndex=0;
const send=async line=>{
 issued.push({sim_s:step*dt,line});
 if(line.startsWith('M569.4')){
  const forces=line.match(/ T(.*)/)[1].split(':').map(Number);
  for(const [i,a] of axes.entries())remote.addCommand(forces[i]===0?{type:'SetPositionMode',axis:a}:{type:'SetTorqueMode',axis:a,torqueNm:Number((-.03*forces[i]).toPrecision(6))});
 }
 if(line.startsWith('M569.3'))readings.push({sim_s:step*dt,angles:angles()});
 return {reply:angles().map(v=>v.toFixed(2)).join(' ')};
};
send.simulationClock={now:()=>step*dt*1000,sleep:async ms=>{
 let target;
 if(ms===100 && rampIndex<2)target=[1566.442,1566.722][rampIndex++];
 else{
  while(pollIndex<polls.length && polls[pollIndex]<=step*dt+1e-7)pollIndex++;
  target=pollIndex<polls.length?polls[pollIndex]:step*dt+ms/1000;
 }
 while(step*dt<target-1e-7)tick();
}};
const result={start_s:start,start_angles:startAngles};
try{result.max_travel_mm=await measureMaxTravelMm(send,{
 motorIds:['40.0','41.0','42.0','43.0'],mmPerDeg:axes.map(()=>Math.PI/6),
 forceLow:.011363575731199994,forceMax:12.884911335493843,
 pairAnchors:[1,2],forbiddenForceAnchors:[3],speedup:25,
});}catch(e){result.error=e.message;}
result.end_s=step*dt;result.duration_s=step*dt-start;result.end_angles=angles();
result.last_command=issued.at(-1);
// Apply the queued final hold and prove the production motor state changed.
tick();
result.commands=issued;result.readings=readings;
console.log(JSON.stringify({...result,readings:undefined,commands:undefined},null,2));
fs.writeFileSync(new URL('./measurement-probe-results.json',import.meta.url),JSON.stringify(result,null,2));
