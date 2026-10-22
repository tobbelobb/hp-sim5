import fs from 'node:fs';
import {createHeadlessWorld} from '../../../../hp-sim-3d/app/headlessWorld.js';
import {EncoderComponent} from '../../../../src/js/cable_joints_3d/ecs.js';
import {measureMaxTravelMm} from '../../../../autocal/control/behaviors/sweep_data_collection.mjs';
const {world,remote,dt}=createHeadlessWorld(fs.readFileSync('public/usd_scenes/hp4_rigid_body.usda','utf8'),{lineLayeringEnabled:false,closedLoopMotorsEnabled:true,physicsSubsteps:1});
let t=0;
function advance(ms){for(let i=0;i<Math.ceil(ms/1000/dt);i++){world.update(dt);t+=dt*1000;}}
const send=async line=>{
 if(line.startsWith('M569.4')){
  const forces=line.match(/ T(.*)/)[1].split(':').map(Number);
  forces.forEach((force,i)=>remote.addCommand(force===0?{type:'SetPositionMode',axis:'ABCD'[i]}:{type:'SetTorqueMode',axis:'ABCD'[i],torqueNm:-force*.015}));
  advance(10);
 }
 const angles=[...'ABCD'].map(axis=>world.getComponent(remote.axisToEntity[axis][0],EncoderComponent).angle*180/Math.PI);
 return {reply:`[${angles.join(',')}]`};
};
send.simulationClock={now:()=>t,sleep:async ms=>advance(ms)};
const start=performance.now();
const maxTravelMm=await measureMaxTravelMm(send,{motorIds:['40.0','41.0','42.0','43.0'],mmPerDeg:[Math.PI/12,Math.PI/12,Math.PI/12,Math.PI/12],forceLow:.0222726,forceMax:3.354,pairAnchors:[1,2],forbiddenForceAnchors:[3],speedup:1,settleOptions:{onProgress:()=>{}}});
const result={physicsSubsteps:world.getResource('physicsSubsteps'),simulated_s:t/1000,wall_s:(performance.now()-start)/1000,maxTravelMm};
fs.writeFileSync(new URL('size-tune-result.json',import.meta.url),JSON.stringify(result,null,2));console.log(result);
