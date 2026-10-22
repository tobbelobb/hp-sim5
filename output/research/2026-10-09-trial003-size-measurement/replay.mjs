import fs from 'node:fs';
import {createHeadlessWorld} from '../../../hp-sim-3d/app/headlessWorld.js';
import {EncoderComponent} from '../../../src/js/cable_joints_3d/ecs.js';
import {waitForStableEncoders} from '../../../autocal/control/primitives/uncalibrated_actions.mjs';
const {world,remote,dt}=createHeadlessWorld(fs.readFileSync(new URL('./scene.usda',import.meta.url),'utf8'));
world.setResource('enableLayering',true);
world.setResource('layeringFrictionEffectiveRadius',true);
const commands=JSON.parse(fs.readFileSync(new URL('./commands.json',import.meta.url),'utf8'));
let step=0, index=0;
const axes=['A','B','C','D'];
const angles=()=>axes.map(a=>world.getComponent(remote.axisToEntity[a][0],EncoderComponent).angle*180/Math.PI);
function tick(){
 while(index<commands.length && commands[index].time<=step*dt+1e-7) remote.addCommand(commands[index++].command);
 world.update(dt);step++;
}
while(step*dt<1566.722-1e-7) tick();
console.log('replayed',step*dt,angles());
let start=step*dt;
const polls=JSON.parse(fs.readFileSync(new URL('./poll-times.json',import.meta.url),'utf8'));
let pollIndex=0;
const send=async()=>({reply:angles().map(v=>v.toFixed(2)).join(' ')});
send.simulationClock={now:()=>step*dt*1000,sleep:async ms=>{
 while(pollIndex<polls.length && polls[pollIndex]<=step*dt+1e-7)pollIndex++;
 const target=pollIndex<polls.length?polls[pollIndex]:step*dt+ms/1000;
 while(step*dt<target-1e-7)tick();
}};
const result={start_s:start,start_angles:angles()};
try{result.settled=await waitForStableEncoders(send,['40.0','41.0','42.0','43.0'],{timeoutMs:120000});}
catch(e){result.error=e.message;}
result.end_s=step*dt;result.end_angles=angles();
console.log(JSON.stringify(result,null,2));
fs.writeFileSync(new URL('./replay-results.json',import.meta.url),JSON.stringify(result,null,2));
