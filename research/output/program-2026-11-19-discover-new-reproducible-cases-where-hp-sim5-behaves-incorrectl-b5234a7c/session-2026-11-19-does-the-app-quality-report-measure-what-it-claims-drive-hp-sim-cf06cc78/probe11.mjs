import { polylineSegments, samplesFromPoints, MM, QualityMonitor, feed } from './harness.mjs';
const mm=(v)=>v*MM; let seed=987654321;
const rand=()=>{seed=(seed*1103515245+12345)%2147483648;return seed/2147483648;};
const S=mm(40);
const sq=polylineSegments([[0,0,0],[S,0,0],[S,S,0],[0,S,0],[0,0,0]]);
console.log('segs', sq.map(s=>s.length));
const pts=[];
for(const seg of sq){const n=Math.round(seg.length/0.00025);
 for(let i=0;i<n;i+=1){const f=i/n; pts.push([seg.start[0]+f*(seg.end[0]-seg.start[0])+(rand()-0.5)*0.0001, seg.start[1]+f*(seg.end[1]-seg.start[1])+(rand()-0.5)*0.0001, (rand()-0.5)*0.0001]);}}
console.log('pts', pts.length);
const samples=samplesFromPoints(pts,sq);
console.log('first 5', samples.slice(0,5).map(s=>[s.pos.map(v=>v*1000), s.E]));
console.log('at 160', samples.slice(158,163).map(s=>[s.pos.map(v=>v*1000), s.E]));
const m=new QualityMonitor(); m.setReferenceSegments(sq);
console.log('corners', m.corners.length, m.corners.map(c=>[c.arcLength*1000, c.vertex.map(v=>v*1000)]));
feed(m,samples); const x=m.getMetrics? null:null; m.runFinalCheck();
console.log('metrics', {progress:m.getMetrics().progress, samples:m.getMetrics().samples, contourMax:m.getMetrics().contourMax});
