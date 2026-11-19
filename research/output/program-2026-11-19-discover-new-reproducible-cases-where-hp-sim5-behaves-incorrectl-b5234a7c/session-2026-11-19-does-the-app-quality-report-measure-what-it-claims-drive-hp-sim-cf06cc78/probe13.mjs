import { polylineSegments, arcCoordinate, MM } from './harness.mjs';
const mm=(v)=>v*MM; let seed=987654321;
const rand=()=>{seed=(seed*1103515245+12345)%2147483648;return seed/2147483648;};
const S=mm(40);
const sq=polylineSegments([[0,0,0],[S,0,0],[S,S,0],[0,S,0],[0,0,0]]);
const pts=[];
for(const seg of sq){const n=Math.round(seg.length/0.00025);
 for(let i=0;i<n;i+=1){const f=i/n; pts.push([seg.start[0]+f*(seg.end[0]-seg.start[0])+(rand()-0.5)*0.0001, seg.start[1]+f*(seg.end[1]-seg.start[1])+(rand()-0.5)*0.0001, (rand()-0.5)*0.0001]);}}
let cursor=0; const arcs=[];
for(const p of pts.slice(0,5)){const r=arcCoordinate(sq,p,cursor); console.log(p.map(v=>v*1000),'->arc',r.arc*1000,'dist',r.dist*1000,'prev',cursor*1000); cursor=r.arc; arcs.push(r.arc);}
console.log(arcs);
