// repro_percent_within_is_time_share.mjs
//
// The HUD line `Path within 0.50 mm` (:934) is fed `withinTolerance` =
// withinToleranceCount / sampleCount (:822,674-676). Extrusion samples are
// emitted once per executed command / simulation step (:359-390), so the
// percentage is a share of *printing time*, not of path length: slow moves
// produce many samples per mm. (One 2 ms sim step per sample in the HP4 scene,
// public/usd_scenes/hp4.usda:7; the percentages do not depend on the tick.)
//
// Input: a 100 mm straight line printed in two halves, the slow one on the path
// and the fast one 1 mm off the path. Path-length share within 0.5 mm = 10.0 %.
// Exit 1 = the printed percentage is not the path share it claims to be.
// Run: timeout 30 node repro_percent_within_is_time_share.mjs
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';

const MM = 0.001;
const ref = [{ start: [0, 0, 0], end: [100 * MM, 0, 0], extrusion: 100 }];

function stubHud() {
  const classes = new Set(['sim-hidden']);
  return {
    innerHTML: '', textContent: '', style: { setProperty() {} },
    classList: { add: (...c) => c.forEach((x) => classes.add(x)), remove: (...c) => c.forEach((x) => classes.delete(x)),
      contains: (c) => classes.has(c), toggle: (c, on) => (on ? classes.add(c) : classes.delete(c)) },
    addEventListener() {}, removeEventListener() {},
  };
}

const hud = stubHud();
const m = new QualityMonitor({ hudElement: hud });
m.setReferenceSegments(ref);

// Slow half: 10 mm at 5 mm/s, one sample per 2 ms sim step -> 0.01 mm per sample.
// Fast half: 90 mm at 50 mm/s, one sample per 2 ms sim step -> 0.1 mm per sample.
// Constant line width: E per sample is proportional to the step length.
for (let i = 0; i < 1000; i += 1) m.recordExtrusion({ pos: [i * 0.01 * MM, 0, 0], length: 0.01 });
for (let i = 0; i < 900; i += 1) m.recordExtrusion({ pos: [(10 + i * 0.1) * MM, 1 * MM, 0], length: 0.1 });
m.runFinalCheck();
m.refreshHud(true);

const x = m.getMetrics();
const field = (label) => {
  const r = hud.innerHTML.match(new RegExp(`${label.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}</span><span>([^<]*)</span>`));
  return r ? r[1] : '(not rendered)';
};
const reportedPct = x.withinTolerance * 100;
const pathSharePct = 10.0;
console.log('reference: 100 mm straight line');
console.log('print: first 10 mm at 5 mm/s on the path, last 90 mm at 50 mm/s 1 mm off the path');
console.log(`samples: 1000 on-path (slow), 900 off-path (fast) -> ${x.samples} total`);
console.log(`HUD "Path within 0.50 mm"        : ${field('Path within 0.50 mm')}`);
console.log(`HUD "Path within 0.25 mm"        : ${field('Path within 0.25 mm')}`);
console.log(`HUD "Path deviation RMS / 95% / max": ${field('Path deviation RMS / 95% / max')}`);
console.log('note: the reported RMS (' + (x.contourRms * 1000).toFixed(3) +
  ' mm) is the correct sample RMS for the metric actually computed; a path-length-weighted RMS would be ' +
  (Math.sqrt(0.9) * 1).toFixed(3) + ' mm, but that is not the number this HUD line claims to be');
console.log(`HUD "Quality"                    : ${(hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, '--'])[1]}`);
console.log(`\nreported within-0.50 mm: ${reportedPct.toFixed(1)} % (share of samples / of printing time)`);
console.log(`path-length share within 0.50 mm: ${pathSharePct.toFixed(1)} %`);
console.log(`disagreement: ${(reportedPct - pathSharePct).toFixed(1)} percentage points`);

if (Math.abs(reportedPct - pathSharePct) > 1) {
  console.log('\nRESULT: MISLEADING. The label "Path within 0.50 mm" reports a time share, not a path share.');
  process.exit(1);
}
console.log('\nRESULT: the percentage matches the path-length share');
