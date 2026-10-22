import { readFileSync, writeFileSync } from 'node:fs';
import { waitForStableEncoders as baseline } from './E03-baseline-actions.mjs';
import { waitForStableEncoders as candidate } from './E03-candidate-actions.mjs';
const source=new URL('./native/71d9d6d8506f48a4ae7984d848ad44b1/5c3fe5661f19484cb54361b1250ce5cc/events.jsonl',import.meta.url);
const events=readFileSync(source,'utf8').trim().split('\n').map(JSON.parse);
let group=[];const groups=[];
function flush(){if(group.length>=4)groups.push(group);group=[];}
for(const r of events){
 if(r.type==='bridge_payload' && r.payload.type==='command'){flush();continue;}
 if(r.type!=='encoder_response'||r.angles_deg.length!==4)continue;
 // The bridge returns duplicate identical encoder responses at each poll.
 if(group.length && r.sim_time_s===group.at(-1).sim_time_s && JSON.stringify(r.angles_deg)===JSON.stringify(group.at(-1).angles_deg))continue;
 if(group.length && Math.abs((r.sim_time_s-group.at(-1).sim_time_s)-.5)>.02)flush();
 group.push(r);
}
flush();
const results=[];
for(const [id,g] of groups.entries()){
 const trials={};
 for(const [variant,wait] of [['baseline',baseline],['candidate',candidate]]){
  let index=0,now=0;
  const send=async()=>{if(index>=g.length)throw Error('recording exhausted');const r=g[index++];now=(r.sim_time_s-g[0].sim_time_s)*1000;return {reply:r.angles_deg.join(' ')};};
  try{const r=await wait(send,['40.0','41.0','42.0','43.0'],1,{sleepFn:async()=>{},nowFn:()=>now,timeoutMs:30000});
   const end=g.at(-1).angles_deg;
   trials[variant]={...r,reads:index,driftToRecordedEndDeg:r.anglesDeg.map((a,i)=>a-end[i])};
  }catch(e){trials[variant]={error:e.message,reads:index};}
 }
 results.push({episode:id,start_s:g[0].sim_time_s,end_s:g.at(-1).sim_time_s,recorded_read_count:g.length,...trials});
}
const usable=results.filter(r=>!r.baseline.error&&!r.candidate.error);
const maxDrift=usable.map(r=>Math.max(...r.candidate.driftToRecordedEndDeg.map(Math.abs)));
const sorted=maxDrift.toSorted((a,b)=>a-b);
const summary={source:source.pathname,extraction:'Consecutive four-axis encoder responses separated by 0.5 +/- 0.02 simulation seconds; split on motor commands and sampling intervals; not an exact collector call-site trace.',episodes:results.length,completed_pairs:usable.length,censored_pairs:results.length-usable.length,earlier_returns:usable.filter(r=>r.candidate.elapsedMs<r.baseline.elapsedMs).length,total_recorded_baseline_ms:usable.reduce((a,r)=>a+r.baseline.elapsedMs,0),total_candidate_ms:usable.reduce((a,r)=>a+r.candidate.elapsedMs,0),max_drift_to_later_recorded_end_deg:Math.max(...maxDrift),p95_drift_deg:sorted[Math.floor(.95*(sorted.length-1))],median_drift_deg:sorted[Math.floor(.5*(sorted.length-1))],pairs:results};
writeFileSync(new URL('./E06-recorded-settling.json',import.meta.url),JSON.stringify(summary,null,2));
const {pairs,...small}=summary;console.log(JSON.stringify(small,null,2));
