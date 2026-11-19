import { QualityMonitor, polylineSegments, feed } from './harness.mjs';
const MM = 0.001; const mm = (v) => v * MM;
const segsL = polylineSegments([[0, 0, 0], [mm(50), 0, 0], [mm(50), mm(50), 0]]);
const pts = [];
for (let x = 0; x <= 49.5; x += 0.5) pts.push([mm(x), 0, 0]);
pts.push([mm(49.8), 0, 0]);
pts.push([mm(50), mm(0.2), 0]);
for (let y = 0.7; y <= 49.5; y += 0.5) pts.push([mm(50), mm(y), 0]);
pts.push([mm(50), mm(50), 0]);
const totalE = segsL.reduce((a, s) => a + s.extrusion, 0);
const eStep = totalE / pts.length;
const m = new QualityMonitor();
m.setReferenceSegments(segsL);
let i = -1;
const origOff = m._updateOffsetEstimate.bind(m);
m._updateOffsetEstimate = () => {
  const o = origOff();
  if (i >= 99 && i <= 103) {
    console.log('i', i, 'normal', m.offsetNormal.map(v=>v.toExponential(3)), 'right', m.offsetRight.map(v=>v.toExponential(3)), 'offset(m)', Array.from(o));
  }
  return o;
};
const origRec = m._recordNormalizedExtrusion.bind(m);
m._recordNormalizedExtrusion = (x,y,z,l,ev) => { i += 1; return origRec(x,y,z,l,ev); };
feed(m, pts.map((p) => ({ pos: p, E: eStep })));
