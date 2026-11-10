import fs from 'node:fs';
import {waitForStableEncoders} from '../../../../autocal/control/primitives/uncalibrated_actions.mjs';
const results=[];
for(const n of [1,4]){
 const {rows}=JSON.parse(fs.readFileSync(new URL(`pull-${n}.json`,import.meta.url)));
 for(const timeoutMs of [120000,1200000]){
  let t=10;
  const send=async()=>({reply:`[${[0,5,10,15].map(i=>rows[Math.min(t,rows.length-1)].enc[i]).join(',')}]`});
  try{
   await waitForStableEncoders(send,[40,41,42,43],1,{timeoutMs,pollIntervalMs:1000,sleepFn:async ms=>{t+=ms/1000;if(t>=rows.length)throw Error('trace exhausted');},nowFn:()=>t*1000,wallNowFn:()=>0,onProgress:()=>{}});
   results.push({substeps:n,timeoutMs,outcome:'settled',elapsed_s:t-10});
  }catch(error){results.push({substeps:n,timeoutMs,outcome:error.message,elapsed_s:t-10});}
 }
}
console.log(results);fs.writeFileSync(new URL('settling-results.json',import.meta.url),JSON.stringify(results,null,2));
