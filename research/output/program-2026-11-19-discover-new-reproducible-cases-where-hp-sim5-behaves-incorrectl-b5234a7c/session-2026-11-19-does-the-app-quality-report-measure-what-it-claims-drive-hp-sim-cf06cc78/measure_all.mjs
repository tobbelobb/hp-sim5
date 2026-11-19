// measure_all.mjs -- regenerates measurements.json with the numbers quoted in
// report.md. Pure JS, one process, a few seconds, < 100 MB RSS.
import { writeFileSync } from 'node:fs';
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';
import { parseGcodeText } from '../../../../hp-sim-3d/app/inspectionToolsController.js';
import {
  polylineSegments, nearestRank, rms, pointToPolylineDistance, vertexMinDistance,
  samplesFromPoints, MM,
} from './harness.mjs';

const out = { generatedBy: 'measure_all.mjs', gitHead: '56a66f77d3bfa58e52621d06f6273d5a1f3501f5', node: process.version, claims: [], findings: {}, negatives: {} };

function stubHud() {
  const c = new Set(['sim-hidden']);
  return { innerHTML: '', textContent: '', style: { setProperty() {} },
    classList: { add: (...n) => n.forEach((x) => c.add(x)), remove: (...n) => n.forEach((x) => c.delete(x)),
      contains: (x) => c.has(x), toggle: (x, on) => (on ? c.add(x) : c.delete(x)) },
    addEventListener() {}, removeEventListener() {} };
}
const seg = (a, b) => ({ start: a, end: b, extrusion: Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) });

// The app's sampling convention (tests/js/hp-sim-3d/qualityMonitor.test.js and
// RemoteSpoolSystem): the tip sits at the start of the step whose E it carries.
function print(segments, offset = () => [0, 0, 0], step = 0.0001, hud = null) {
  const m = new QualityMonitor(hud ? { hudElement: hud } : {});
  m.setReferenceSegments(segments);
  for (const s of segments) {
    const d = s.end.map((v, i) => v - s.start[i]);
    const length = Math.hypot(...d);
    if (length === 0) { m.recordExtrusion({ pos: s.start, length: s.extrusion }); continue; }
    const count = Math.max(1, Math.round(length / step));
    for (let i = 0; i < count; i += 1) {
      const p = s.start.map((v, k) => v + d[k] * (i / count));
      const o = offset(p);
      m.recordExtrusion({ pos: p.map((v, k) => v + o[k]), length: s.extrusion / count });
    }
  }
  m.runFinalCheck();
  if (hud) m.refreshHud(true);
  return m;
}

// ---------------------------------------------------------------- claims
out.claims = [
  { id: 'histogram', quote: 'Distances binned on a log scale (2 % wide bins from 0.1 um to 10 m), so any percentile is read off to within one bin, in O(1) memory and time per sample.', at: 'hp-sim-3d/app/quality-monitor.js:58-59' },
  { id: 'corner', quote: 'per sharp corner (after removing the mean offset): deviation of the attained path from the corner point, the largest deviation after it, and the distance travelled until the path stays within tolerance', at: 'hp-sim-3d/app/quality-monitor.js:128-131' },
  { id: 'corner-shape', quote: 'Corners are judged on shape too: rounding and overshoot, not the offset.', at: 'hp-sim-3d/app/quality-monitor.js:681' },
  { id: 'score', quote: 'Each term deducts weight * (1 - exp(-value / scale)) points; weights sum to 100.', at: 'hp-sim-3d/app/quality-monitor.js:137' },
  { id: 'score-shift', quote: 'a rigidly shifted but otherwise perfect print still scores high', at: 'hp-sim-3d/app/quality-monitor.js:132-135' },
  { id: 'path', quote: 'path deviation: distance to the intended path near that point (contour error, as ISO 9283 path accuracy), split into horizontal and vertical', at: 'hp-sim-3d/app/quality-monitor.js:126-127' },
  { id: 'hud-within', quote: 'metric(`Path within ${formatLengthMm(m.tolerance)} mm`, formatPercent(m.withinTolerance))', at: 'hp-sim-3d/app/quality-monitor.js:934' },
  { id: 'hud-overshoot', quote: "metric('Corner overshoot avg / max', pair(m.cornerOvershootAvg, m.cornerOvershootMax))", at: 'hp-sim-3d/app/quality-monitor.js:940' },
];

// ------------------------------------------------- F1: corner overshoot
{
  const L = [seg([0, 0, 0], [0.05, 0, 0]), seg([0.05, 0, 0], [0.05, 0.05, 0])];
  const rows = [];
  for (const [label, off] of [['along +X (first move)', [0.001, 0, 0]], ['across +Y', [0, 0.001, 0]], ['across +Z', [0, 0, 0.001]], ['45 deg', [0.0007071067811865476, 0.0007071067811865476, 0]]]) {
    const hud = stubHud();
    const m = print(L, () => off, 0.0001, hud);
    const x = m.getMetrics();
    rows.push({
      case: label,
      offsetMm: off.map((v) => v * 1000),
      offsetProjectionOnFirstMoveMm: Math.abs(off[0]) * 1000,
      expectedCornerOvershootMaxMm: 0,
      reportedCornerOvershootMaxMm: x.cornerOvershootMax * 1000,
      expectedSettleMaxMm: 0,
      reportedSettleMaxMm: x.settleDistanceMax * 1000,
      cornerDeviationMaxMm: x.cornerDeviationMax * 1000,
      shapeP95Mm: x.shapeP95 * 1000,
      meanOffsetMm: x.meanOffset.map((v) => v * 1000),
      score: x.score,
      hudOvershoot: (hud.innerHTML.match(/Corner overshoot avg \/ max<\/span><span>([^<]*)</) || [, ''])[1],
      hudScore: (hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, ''])[1],
    });
  }
  // tiled square / real-corner count
  out.findings.cornerOvershootReportsOffset = {
    path: 'L, 50 mm +X then 50 mm +Y, 0.1 mm samples, print exactly on the path',
    rows,
    scoreDifferenceSameOffset: rows[1].score - rows[0].score,
    mechanism: 'offset estimate is taken before the current sample joins the normal equations (:646 vs :659-667), so the first sample of the first new direction is compared without that direction\'s offset and its shape distance (= offset projection) is recorded as overshoot (:772-773) / ringing (:775-777).',
    genuineOvershootUnderReport: null,
  };

  // How often the same print shows the artefact: sweep the number of sim steps
  // per 50 mm leg (seen by the report as E-chunk count).
  const sweep = [];
  for (const c of [100, 137, 200, 250, 300, 333, 400, 500, 640, 800, 1000, 1024, 1500, 1600, 2000, 2499, 2500, 2501]) {
    const mm2 = new QualityMonitor();
    mm2.setReferenceSegments([seg([0, 0, 0], [0.05, 0, 0]), seg([0.05, 0, 0], [0.05, 0.05, 0])]);
    for (const s of [seg([0, 0, 0], [0.05, 0, 0]), seg([0.05, 0, 0], [0.05, 0.05, 0])]) {
      const d = s.end.map((v, i) => v - s.start[i]);
      for (let i = 0; i < c; i += 1) {
        const p = s.start.map((v, k) => v + d[k] * (i / c));
        mm2.recordExtrusion({ pos: [p[0] + 0.001, p[1], p[2]], length: s.extrusion / c });
      }
    }
    mm2.runFinalCheck();
    const mx = mm2.getMetrics();
    sweep.push({ stepsPerLeg: c, overshootMaxMm: mx.cornerOvershootMax * 1000, settleMaxMm: mx.settleDistanceMax * 1000, score: mx.score });
  }
  out.findings.cornerOvershootReportsOffset.triggerSweep = {
    offsetMm: 1.0,
    thresholdMm: 0.5,
    triggered: sweep.filter((r) => r.overshootMaxMm > 0.5).length,
    of: sweep.length,
    rows: sweep,
    note: 'fires when the E accumulator at the corner is a float epsilon past the vertex: exact binary E steps do not fire (1024, 1600, 2000, 2500) while the neighbouring non-exact counts do (2499, 2501) -- a pure floating-point coin flip',
  };

  // Genuine bulge: 0.5 mm overshoot at a corner; exact max distance to the
  // reference path is 0.5 mm, final mean offset 0.0097 mm.
  const pts = [];
  for (let x = 0; x <= 49.5; x += 0.5) pts.push([x * MM, 0, 0]);
  pts.push([50 * MM, 0, 0], [50.25 * MM, 0.25 * MM, 0], [50.5 * MM, 0.5 * MM, 0], [50.25 * MM, 0.75 * MM, 0], [50 * MM, 1 * MM, 0]);
  for (let y = 1.5; y <= 50; y += 0.5) pts.push([50 * MM, y * MM, 0]);
  const ref = [seg([0, 0, 0], [50 * MM, 0, 0]), seg([50 * MM, 0, 0], [50 * MM, 50 * MM, 0])];
  const m = new QualityMonitor();
  m.setReferenceSegments(ref);
  for (const s of samplesFromPoints(pts, ref, { ePerMeter: 1 })) m.recordExtrusion({ pos: s.pos, length: s.E });
  m.runFinalCheck();
  const bx = m.getMetrics();
  out.findings.cornerOvershootReportsOffset.genuineOvershootUnderReport = {
    path: 'L with a 0.5 mm overshoot bulge just after the corner (5 samples)',
    exactMaxDistanceToReferencePathMm: 0.5,
    finalMeanOffsetMm: Math.hypot(...bx.meanOffset) * 1000,
    expectedOvershootAfterRemovingMeanOffsetMm: 0.5 - Math.hypot(...bx.meanOffset) * 1000,
    reportedOvershootMaxMm: bx.cornerOvershootMax * 1000,
    note: 'the transient bulge is partly absorbed by the running offset estimate',
  };
}

// ------------------------------------------------- F2: within-tolerance share
{
  const ref = [seg([0, 0, 0], [100 * MM, 0, 0])];
  const hud = stubHud();
  const m = new QualityMonitor({ hudElement: hud });
  m.setReferenceSegments(ref);
  for (let i = 0; i < 1000; i += 1) m.recordExtrusion({ pos: [i * 0.01 * MM, 0, 0], length: 0.01 });
  for (let i = 0; i < 900; i += 1) m.recordExtrusion({ pos: [(10 + i * 0.1) * MM, 1 * MM, 0], length: 0.1 });
  m.runFinalCheck(); m.refreshHud(true);
  const x = m.getMetrics();
  out.findings.withinToleranceIsTimeShare = {
    path: '100 mm line: 10 mm at F300 (5 mm/s) on path, 90 mm at F3000 (50 mm/s) 1 mm off path',
    samplesOnPath: 1000, samplesOffPath: 900, totalSamples: x.samples,
    sampleRmsMm: x.contourRms * 1000,
    pathLengthWeightedRmsMm: Math.sqrt(0.9),
    sampleRmsNote: '0.688 mm is the correct sample RMS for the metric computed; the 0.949 mm path-length-weighted RMS is a different quantity and is NOT claimed by the code. Only the within-tolerance share is the wrong-metric issue.',
    reportedWithin0_5mmPercent: x.withinTolerance * 100,
    reportedWithin0_25mmPercent: x.withinFineTolerance * 100,
    pathLengthShareWithin0_5mmPercent: 10.0,
    disagreementPercentagePoints: x.withinTolerance * 100 - 10.0,
    hudLine: (hud.innerHTML.match(/Path within 0\.50 mm<\/span><span>([^<]*)</) || [, ''])[1],
    hudScore: (hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, ''])[1],
    note: 'extrusion samples are emitted once per executed command / sim step (2 ms per step in the HP4 scene, public/usd_scenes/hp4.usda:7), so sample density is proportional to time, not to path length; the percentages do not depend on the tick',
  };
}

// ------------------- F3: reference describes more path than was executed
{
  // Reachability: the reference parser accepts `G0 ... E...` as motion while the
  // command layer ignores G0, so the reference is 150 mm / 15 mm E while the
  // machine executes 50 mm / 5 mm E. (The G0 gap itself is a separate prior issue;
  // here it only supplies the reference/executed mismatch.)
  const gcode = ['G21', 'G90', 'M82', 'G92 E0', 'G1 X50 Y0 E5', 'G0 X50 Y50 E10', 'G0 X0 Y50 E15', ''].join('\n');
  const parsed = parseGcodeText(gcode);
  const ref = parsed.segments.map((s) => ({ start: s.start.map((v) => v * 0.001), end: s.end.map((v) => v * 0.001), extrusion: s.extrusion }));
  const hud = stubHud();
  const m = new QualityMonitor({ hudElement: hud });
  m.setReferenceSegments(ref);
  const ex = ref[0];
  const n = Math.round(Math.hypot(ex.end[0] - ex.start[0], ex.end[1] - ex.start[1]) / 0.00001);
  for (let i = 0; i < n; i += 1) {
    const f = i / n;
    m.recordExtrusion({ pos: [ex.start[0] + f * (ex.end[0] - ex.start[0]), ex.start[1] + f * (ex.end[1] - ex.start[1]), 0], length: ex.extrusion / n });
  }
  m.runFinalCheck(); m.refreshHud(true);
  const x = m.getMetrics();
  out.findings.quality100OnReferenceOverrun = {
    path: 'G-code with G1 X50 E5 then two G0 lines carrying E; reference 150 mm / 15 mm E, executed 50 mm / 5 mm E',
    referenceLengthMm: parsed.segments.reduce((a, s) => a + s.length, 0),
    referenceExtrusionMm: parsed.segments.reduce((a, s) => a + s.extrusion, 0),
    executedLengthMm: parsed.segments[0].length,
    executedExtrusionMm: parsed.segments[0].extrusion,
    progress: x.progress, cornersAssessed: x.cornerCount, cornersInPath: ref.length - 1,
    score: x.score,
    hudQuality: (hud.innerHTML.match(/Quality <strong>([^<]*)<\/strong>/) || [, ''])[1],
    hudProgress: (hud.innerHTML.match(/Progress<\/span><span>([^<]*)</) || [, ''])[1],
    hudDeductions: (hud.innerHTML.match(/Score deductions<\/span><span>([^<]*)</) || [, ''])[1],
    note: 'the card also shows Progress 33.3 %, so this is classified misleading, not wrong; a plain abort of a print whose reference and executed paths agree does not produce this card',
  };
}

// ---------------------------------------------------------- negatives
// N1 histogram: p95 vs exact nearest-rank order statistic
{
  let seed = 12345;
  const rand = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
  const cases = [];
  for (let t = 0; t < 60; t += 1) {
    const N = 5 + Math.floor(rand() * 400);
    const vals = Array.from({ length: N }, () => Math.abs(Math.exp(Math.log(1e-9) + rand() * (Math.log(5) - Math.log(1e-9)))));
    const L = 0.0005 * N;
    const m = new QualityMonitor();
    m.setReferenceSegments([seg([0, 0, 0], [L, 0, 0])]);
    for (let i = 0; i < N; i += 1) m.recordExtrusion({ pos: [i * 0.0005, vals[i], 0], length: 0.5 });
    m.runFinalCheck();
    const x = m.getMetrics();
    const e95 = nearestRank(vals, 0.95);
    const eRms = rms(vals);
    const eMax = Math.max(...vals);
    cases.push({
      N, relP95: (x.contourP95 - e95) / e95, absP95Mm: (x.contourP95 - e95) * 1000,
      relRms: (x.contourRms - eRms) / eRms, relMax: (x.contourMax - eMax) / eMax,
    });
  }
  const worst = cases.reduce((a, c) => (c.relP95 > a.relP95 ? c : a), cases[0]);
  out.negatives.histogram = {
    cases: cases.length,
    worstP95OverReportRelative: worst.relP95,
    worstP95OverReportAbsMm: worst.absP95Mm,
    worstCaseN: worst.N,
    worstRmsRelativeError: cases.reduce((a, c) => Math.max(a, Math.abs(c.relRms)), 0),
    worstMaxRelativeError: cases.reduce((a, c) => Math.max(a, Math.abs(c.relMax)), 0),
    anyUnderReport: cases.some((c) => c.relP95 < -1e-12),
    verdict: 'p95 is the upper edge of the 2 %-wide log bin holding the nearest-rank order statistic: never below exact, at most 1.91 % above; rms and max are exact.',
  };
}
// N2 histogram range limits
{
  const allSame = (d) => {
    const m = new QualityMonitor();
    m.setReferenceSegments([seg([0, 0, 0], [0.1, 0, 0])]);
    for (let i = 0; i < 100; i += 1) m.recordExtrusion({ pos: [i * 0.001, d, 0], length: 1 });
    m.runFinalCheck();
    return m.getMetrics();
  };
  const z = allSame(0);
  const big = allSame(20);
  const small = allSame(5e-8);
  out.negatives.histogramEdges = {
    allZero: { p95: z.contourP95, rms: z.contourRms, max: z.contourMax, score: z.score },
    aboveTopBin_20m: { exactP95: 20, reportedP95: big.contourP95, reportedMax: big.contourMax },
    belowBottomBin_0_05um: { exactP95: 5e-8, reportedP95: small.contourP95 },
  };
}
// N3 rms / max / h+v exactness on a noisy square
{
  const S = 0.05;
  const sq = [seg([0, 0, 0], [S, 0, 0]), seg([S, 0, 0], [S, S, 0]), seg([S, S, 0], [0, S, 0]), seg([0, S, 0], [0, 0, 0])];
  let seed = 987654321;
  const rand = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
  const pts = [];
  for (const s of sq) {
    const n = Math.round(s.extrusion / 0.00025);
    for (let i = 0; i < n; i += 1) {
      const f = i / n;
      pts.push([s.start[0] + f * (s.end[0] - s.start[0]) + (rand() - 0.5) * 0.0001,
        s.start[1] + f * (s.end[1] - s.start[1]) + (rand() - 0.5) * 0.0001, (rand() - 0.5) * 0.0001]);
    }
  }
  const exact = pts.map((p) => pointToPolylineDistance(p, sq));
  const m = new QualityMonitor();
  m.setReferenceSegments(sq);
  for (const s of samplesFromPoints(pts, sq, { ePerMeter: 1 })) m.recordExtrusion({ pos: s.pos, length: s.E });
  m.runFinalCheck();
  const x = m.getMetrics();
  out.negatives.rmsMax = {
    samples: x.samples,
    contourRmsExact: rms(exact), contourRmsReported: x.contourRms, rel: (x.contourRms - rms(exact)) / rms(exact),
    contourMaxExact: Math.max(...exact), contourMaxReported: x.contourMax,
    p95Exact: nearestRank(exact, 0.95), p95Reported: x.contourP95,
    cornerCountReported: x.cornerCount, cornersInClosedSquare: 4, note: 'the seam corner at the path start/end is not assessed by design',
  };
}
// N4 corner deviation closed form (0.2 mm bevel -> 0.2/sqrt(2))
{
  const ref = [seg([0, 0, 0], [50 * MM, 0, 0]), seg([50 * MM, 0, 0], [50 * MM, 50 * MM, 0])];
  const pts = [];
  for (let x = 0; x <= 49.5; x += 0.5) pts.push([x * MM, 0, 0]);
  pts.push([49.8 * MM, 0, 0], [50 * MM, 0.2 * MM, 0]);
  for (let y = 0.7; y <= 50; y += 0.5) pts.push([50 * MM, y * MM, 0]);
  const m = new QualityMonitor();
  m.setReferenceSegments(ref);
  for (const s of samplesFromPoints(pts, ref, { ePerMeter: 1 })) m.recordExtrusion({ pos: s.pos, length: s.E });
  m.runFinalCheck();
  const x = m.getMetrics();
  out.negatives.cornerDeviationClosedForm = {
    input: 'L with a 0.2 mm bevel at the corner, 0.5 mm samples',
    exactVertexToAttainedPolylineMm: vertexMinDistance(pts, [50 * MM, 0, 0]) * 1000,
    reportedCornerDeviationMaxMm: x.cornerDeviationMax * 1000,
    absErrorMm: (x.cornerDeviationMax - vertexMinDistance(pts, [50 * MM, 0, 0])) * 1000,
  };
}
// N5 corner threshold
{
  const rows = [];
  for (const deg of [30, 44, 44.9, 45, 45.1, 60, 90]) {
    const th = deg * Math.PI / 180;
    const b = [40 * MM, 0, 0];
    const c = [b[0] + 40 * MM * Math.cos(th), b[1] + 40 * MM * Math.sin(th), 0];
    const x = print([seg([0, 0, 0], b), seg(b, c)]).getMetrics();
    rows.push({ turnDeg: deg, cornerCount: x.cornerCount });
  }
  out.negatives.cornerThreshold = { thresholdDeg: 45, rows };
}
// N6 lag is (mostly) position error, not path deviation
{
  const ref = [seg([0, 0, 0], [200 * MM, 0, 0])];
  const rows = [];
  for (const lagmm of [0, 2, 4.9, 5.5, 10]) {
    const x = print(ref, () => [-lagmm * MM, 0, 0]).getMetrics();
    rows.push({ lagMm: lagmm, contourP95Mm: x.contourP95 * 1000, contourMaxMm: x.contourMax * 1000, meanLagMm: x.meanLag * 1000, withinTolerance: x.withinTolerance });
  }
  out.negatives.lag = {
    rows,
    note: 'the attained point is on the line for a pure lag, so contour rms/p95 stay ~0; the reported max is the lag of the first samples that fall before the path start (a real distance to the nominal path, not a window artefact)',
  };
}
// N7 end-to-end parse + convert + monitor: units and 3-D path
{
  const gcode = 'G21\nG90\nM82\nG92 E0\nG1 Z0.2\nG1 X0 Y0 F3000\nG1 X50 Y0 E5\nG1 X50 Y50 E10\nG1 E9\nG0 X0 Y0\nG1 X0 Y0 Z0.4 E10\nG1 X40 Y0 E14\n';
  const parsed = parseGcodeText(gcode);
  const sim = parsed.segments.map((s) => ({
    start: s.start.map((v) => v * 0.001), end: s.end.map((v) => v * 0.001), extrusion: s.extrusion, length: s.length * 0.001,
  }));
  const m = new QualityMonitor();
  m.setReferenceSegments(sim);
  const n = 100;
  for (const s of sim) {
    const d = s.end.map((v, i) => v - s.start[i]);
    for (let i = 0; i < n; i += 1) {
      m.recordExtrusion({ pos: s.start.map((v, k) => v + d[k] * i / n + (k === 1 ? 0.0002 : 0)), length: s.extrusion / n });
    }
  }
  m.runFinalCheck();
  const x = m.getMetrics();
  out.negatives.endToEnd = {
    gcodeSummary: 'travel, retract, Z-only prime with E, 90 deg corner after a travel',
    parsedSegments: parsed.segments.map((s) => ({ start: s.start.map((v) => +v.toFixed(3)), end: s.end.map((v) => +v.toFixed(3)), E: +s.extrusion.toFixed(6), len: +s.length.toFixed(3) })),
    parsedBoundsMm: parsed.bounds,
    expectedOffsetMm: 0.2,
    reportedContourMaxMm: x.contourMax * 1000, reportedContourP95Mm: x.contourP95 * 1000,
    reportedMeanOffsetMm: x.meanOffset.map((v) => v * 1000),
    reportedVerticalRmsMm: x.contourVerticalRms * 1000,
    reportedProgress: x.progress,
    note: 'the +Y offset is along the (Y) second segment, so 1 of 4 segments correctly reports zero contour deviation; rms is therefore 0.2*sqrt(3/4) = 0.1732 mm',
  };
}
// N8 score formula reproduces exactly from the reported metrics
{
  const L = [seg([0, 0, 0], [0.05, 0, 0]), seg([0.05, 0, 0], [0.05, 0.05, 0])];
  const x = print(L, () => [0.0003, 0.0002, 0]).getMetrics();
  const terms = [
    [45, 0.001, x.shapeP95], [20, 0.0006, x.shapeRms],
    [10, 0.001, x.cornerCount > 0 ? Math.max(x.cornerDeviationAvg, x.cornerOvershootAvg) : 0],
    [5, 0.005, x.cornerCount > 0 ? x.settleDistanceAvg : 0],
    [10, 0.005, Math.hypot(...x.meanOffset)], [10, 20, 0],
  ];
  const recomputed = 100 - terms.reduce((a, [w, s, v]) => a + w * (1 - Math.exp(-v / s)), 0);
  out.negatives.scoreFormula = { reported: x.score, recomputedFromReportedMetrics: recomputed, absoluteDifference: Math.abs(x.score - recomputed), deductions: x.scoreDeductions };
}

writeFileSync(new URL('./measurements.json', import.meta.url), `${JSON.stringify(out, null, 2)}\n`);
console.log('wrote measurements.json');
console.log('F1 overshoot along/across:', out.findings.cornerOvershootReportsOffset.rows.map((r) => `${r.reportedCornerOvershootMaxMm.toFixed(6)}mm`).join(' / '));
console.log('F2 within 0.5:', out.findings.withinToleranceIsTimeShare.reportedWithin0_5mmPercent.toFixed(1), 'vs path', out.findings.withinToleranceIsTimeShare.pathLengthShareWithin0_5mmPercent);
console.log('F3 score/progress:', out.findings.quality100OnReferenceOverrun.score, out.findings.quality100OnReferenceOverrun.progress);
console.log('N histogram worst p95 over-report:', (out.negatives.histogram.worstP95OverReportRelative * 100).toFixed(3), '%');
console.log('N rms rel err:', out.negatives.rmsMax.rel);
console.log('N corner deviation:', out.negatives.cornerDeviationClosedForm.reportedCornerDeviationMaxMm, 'exact', out.negatives.cornerDeviationClosedForm.exactVertexToAttainedPolylineMm);
console.log('N end-to-end contourMax:', out.negatives.endToEnd.reportedContourMaxMm);
