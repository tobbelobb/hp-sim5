import fs from 'node:fs';
import {collectSweepData} from '../../../../autocal/control/behaviors/sweep_data_collection.mjs';
import {createHeadlessBridge} from '../../../../autocal/control/primitives/headless_bridge.mjs';
import {sendHpSimSettings,sendHpSimSpeedScale} from '../../../../autocal/control/primitives/encoder_utils.mjs';
const [url,output,machine,rpcArg='1']=process.argv.slice(2);
const bridge=await createHeadlessBridge(url);
await sendHpSimSettings(bridge,{closedLoopMotorsEnabled:true},{quiet:true});
await sendHpSimSpeedScale(bridge,25,{quiet:true});
let phase='setup',probing=false;const phases=[];
const log=console.log;
console.log=(...args)=>{const text=args.join(' ');if(text.startsWith('; phase ')){
 const match=text.match(/^; phase ([^:]+): (.*)/);if(match){phase=match[1];probing=match[2]==='start'&&['size-probe','size-probe-return','sweep-endpoint-probe'].includes(phase);phases.push({phase,message:match[2],physicsMs:bridge.simulationClock.now()});}}
 log(...args);
};
const rpcS=Number(rpcArg);
const send=async(line,...args)=>{
 if(probing&&rpcS>0)await bridge.simulationClock.sleep(rpcS*1000);
 return bridge.sendGcodeLine(line,...args);
};
send.simulationClock=bridge.simulationClock;
const started=performance.now();let outcome='unknown',point;const points=[];
try {
 await collectSweepData(send,{machineType:machine==='hp3'?'hangprinter_3':'hangprinter_4',motorIds:['40.0','41.0','42.0','43.0'],speedup:25,
 args:{sweepConfigFile:new URL('./one-sweep.txt',import.meta.url).pathname,sweepPoints:2,noiseSamples:8,noiseMinSamples:4,noiseSampleIntervalMs:40,feed:1400},
 onPoint:p=>{point=p;points.push(p);if(process.env.SMOKE_FIRST_POINT!=='0')throw new Error('SMOKE_STOP_AFTER_FIRST_POINT');}});
 outcome='complete';
}catch(error){outcome=error.message;log('; smoke boundary:',outcome);}
probing=false;
phases.push({phase:'final-held-observation',physicsMs:bridge.simulationClock.now()});
await send('M569.4 P40.0:41.0:42.0:43.0 T0:0:0:0');
await bridge.simulationClock.sleep(15000);
fs.writeFileSync(output,JSON.stringify({outcome,machine,rpcPhysicsS:rpcS,phases,point,points,wallS:(performance.now()-started)/1000,physicsMs:bridge.simulationClock.now()},null,2));
