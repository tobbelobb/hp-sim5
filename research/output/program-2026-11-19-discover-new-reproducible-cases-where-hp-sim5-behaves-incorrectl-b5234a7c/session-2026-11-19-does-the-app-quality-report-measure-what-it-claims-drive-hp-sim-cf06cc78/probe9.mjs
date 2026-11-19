// Probe 9: histogram/quantile accuracy against exact order statistics.
// Raw (uncorrected) contour distances are used so the true multiset is known.
import {
  QualityMonitor, polylineSegments, nearestRank, rms, MM,
} from './harness.mjs';

const mm = (v) => v * MM;

// 400 mm straight reference along +X; sample i at x = i*step, y = dist[i].
// E per sample = step so the E cursor stays on the sample position.
function feedDistances(distances, { step = 0.0005 } = {}) {
  const L = step * distances.length;
  const segs = polylineSegments([[0, 0, 0], [L, 0, 0]]);
  const m = new QualityMonitor();
  m.setReferenceSegments(segs);
  for (let i = 0; i < distances.length; i += 1) {
    m.recordExtrusion({ pos: [i * step, distances[i], 0], length: step * 1000 });
  }
  m.runFinalCheck();
  return m.getMetrics();
}

function check(name, distances) {
  const x = feedDistances(distances);
  const exact95 = nearestRank(distances, 0.95);
  const exactRms = rms(distances);
  const exactMax = Math.max(...distances);
  const rel95 = exact95 > 0 ? (x.contourP95 - exact95) / exact95 : 0;
  console.log(name.padEnd(46),
    `exact95=${(exact95 * 1000).toFixed(6)}mm reported=${(x.contourP95 * 1000).toFixed(6)}mm rel=${(rel95 * 100).toFixed(3)}%`,
    `rmsRel=${exactRms > 0 ? ((x.contourRms - exactRms) / exactRms * 100).toFixed(4) : 0}%`,
    `maxRel=${exactMax > 0 ? ((x.contourMax - exactMax) / exactMax * 100).toFixed(6) : 0}%`);
  return { exact95, reported: x.contourP95, rel95 };
}

// 1. exact lattice: distances (k+1)*0.01 mm, k=0..99
check('uniform 0.01..1.00 mm (N=100)', Array.from({ length: 100 }, (_, k) => (k + 1) * 0.01 * MM));

// 2. bimodal +-d, zero mean
check('bimodal +-0.2mm (N=1000)', Array.from({ length: 1000 }, (_, k) => (k % 2 ? 0.2 : -0.2) * MM));

// 3. log-uniform random
let seed = 12345;
const rand = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
const logUni = Array.from({ length: 2000 }, () => Math.exp(Math.log(1e-7) + rand() * (Math.log(10) - Math.log(1e-7))) * (rand() < 0.5 ? 1 : -1));
const neg = logUni.map(Math.abs);
const r3 = check('|log-uniform 1e-7..10 m| (N=2000)', neg);

// 4. worst case over many random sets, N small and large
let worstRel = -Infinity; let worstCase = null;
for (let t = 0; t < 60; t += 1) {
  const N = 5 + Math.floor(rand() * 400);
  const vals = Array.from({ length: N }, () => Math.abs(Math.exp(Math.log(1e-9) + rand() * (Math.log(5) - Math.log(1e-9)))));
  const x = feedDistances(vals);
  const e95 = nearestRank(vals, 0.95);
  const rel = e95 > 0 ? (x.contourP95 - e95) / e95 : 0;
  if (rel > worstRel) { worstRel = rel; worstCase = { N, e95, reported: x.contourP95 }; }
  if (rel < -1e-12) console.log('UNDER-REPORT!', N, e95, x.contourP95);
}
console.log('worst over-report of p95 over 60 random sets:', (worstRel * 100).toFixed(3), '%', worstCase);

// 5. all-zero distances
{
  const x = feedDistances(Array.from({ length: 500 }, () => 0));
  console.log('all zero: p95', x.contourP95, 'rms', x.contourRms, 'max', x.contourMax, 'score', x.score);
}

// 6. single non-zero sample at D with N-1 zeros: expected nearest-rank p95 = 0
{
  const D = 0.5 * MM;
  const x = feedDistances(Array.from({ length: 100 }, (_, k) => (k === 50 ? D : 0)));
  console.log('1 outlier 0.5mm / 99 zero: exact p95=0, reported', x.contourP95, 'm =', x.contourP95 * 1000, 'mm; max', x.contourMax * 1000, 'mm');
}

// 7. distances above the histogram's top bin edge (~10.18 m)
{
  const x = feedDistances(Array.from({ length: 100 }, () => 20));
  console.log('all 20 m: exact p95=20 m, reported', x.contourP95, 'm; max', x.contourMax);
}
