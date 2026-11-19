import { polylineSegments, arcCoordinate, MM } from './harness.mjs';
const mm=(v)=>v*MM;
const S=mm(40);
const sq=polylineSegments([[0,0,0],[S,0,0],[S,S,0],[0,S,0],[0,0,0]]);
const pts=[[0,0,0],[mm(0.25),0,0],[mm(0.5),0,0],[mm(39.75),0,0],[mm(40),mm(0.25),0],[mm(40),mm(39.75),0],[mm(39.75),mm(40),0],[0,mm(40),0],[0,mm(39.75),0]];
let cursor=0;
for(const p of pts){const r=arcCoordinate(sq,p,cursor); console.log(p.map(v=>v*1000), 'arc(mm)', r.arc*1000, 'dist(mm)', r.dist*1000, 'cursor was', cursor*1000); cursor=r.arc;}
