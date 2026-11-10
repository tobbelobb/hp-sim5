import {pathToFileURL} from 'node:url';
const {runForceTrial}=await import(pathToFileURL(process.argv[2]));
let time=0, modes=[0,0,0], angles=[0,0,0];
const send=async line=>{
 if(line.startsWith('M569.4'))modes=line.split(' T')[1].split(':').map(Number);
 if(line.startsWith('G1 H2'))for(const match of line.matchAll(/([XYZ])(-?[\d.]+)/g))angles[['X','Y','Z'].indexOf(match[1])]+=Number(match[2]);
 return {reply:angles.join(' ')};
};
send.simulationClock={now:()=>time,sleep:async ms=>{if(modes[0]>.02){angles[0]+=ms/500;angles[1]+=ms/1000;}time+=ms;}};
try{
const result=await runForceTrial(send,{motorIds:['40.0','41.0','42.0'],activeAnchor:0,fixedAnchor:2,restAnchors:[1],idleForce:.011363575731199994,testForce:.6236330361282555,speedup:25,sampleWindowMs:1000,stallTimeoutMs:30000,waitForStall:true,axes:['X','Y','Z'],mmPerDeg:[1,1,1],feed:1000});
console.log(JSON.stringify({ok:true,result,angles}));
}catch(error){console.log(JSON.stringify({ok:false,error:error.message,angles}));process.exitCode=1;}
