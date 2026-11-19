// Shared harness: drives the shipped QualityMonitor through its public API only.
// Reference segments are given in SIMULATION units (meters, as
// inspectionToolsController.convertParsedSegmentsToSimulation produces) with
// extrusion in mm (as parseGcodeText produces and as remoteSpoolSystem emits
// per command step). Nothing about the statistics is reimplemented here.
import { QualityMonitor } from '../../../../hp-sim-3d/app/quality-monitor.js';

export { QualityMonitor };

export const MM = 0.001; // mm -> m

// Build reference segments from a polyline in meters. E per segment is
// proportional to its 3-D length (1 mm filament per mm of travel by default),
// which is what a parse of a normal G-code file gives.
export function polylineSegments(points, { ePerMeter = 1000 } = {}) {
  const segments = [];
  for (let i = 0; i + 1 < points.length; i += 1) {
    const a = points[i];
    const b = points[i + 1];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    if (len === 0) continue;
    segments.push({ start: [...a], end: [...b], extrusion: len * ePerMeter, length: len });
  }
  return segments;
}

// Feed one sample per reference "step". `samples` is a list of
// {pos:[x,y,z], E} with E in mm.
export function feed(monitor, samples) {
  for (const s of samples) {
    monitor.recordExtrusion({ pos: s.pos, length: s.E });
  }
}

// Sample a polyline with equal E chunks. `place` decides where inside its own
// E-chunk the reported tip position sits: 0 = chunk start (matches the
// monitor's convention), 1 = chunk end (matches the simulator, whose command
// emits the tip after the step).
export function samplePolyline(segments, samplesPerSegment, { place = 1, offset = () => [0, 0, 0] } = {}) {
  const out = [];
  for (const seg of segments) {
    const n = typeof samplesPerSegment === 'function' ? samplesPerSegment(seg) : samplesPerSegment;
    const eStep = seg.extrusion / n;
    for (let i = 0; i < n; i += 1) {
      const f = (i + place) / n;
      const p = [
        seg.start[0] + f * (seg.end[0] - seg.start[0]),
        seg.start[1] + f * (seg.end[1] - seg.start[1]),
        seg.start[2] + f * (seg.end[2] - seg.start[2]),
      ];
      const d = offset(p, f, seg, i);
      out.push({ pos: [p[0] + d[0], p[1] + d[1], p[2] + d[2]], E: eStep });
    }
  }
  return out;
}

export function metricsOf(monitor) {
  monitor.runFinalCheck();
  return monitor.getMetrics();
}

// Exact distance from a point to a polyline (meters). Independent reference.
export function pointToPolylineDistance(p, segments) {
  let best = Infinity;
  for (const s of segments) {
    const [ax, ay, az] = s.start;
    const bx = s.end[0] - ax;
    const by = s.end[1] - ay;
    const bz = s.end[2] - az;
    const ll = bx * bx + by * by + bz * bz;
    let t = 0;
    if (ll > 0) {
      t = ((p[0] - ax) * bx + (p[1] - ay) * by + (p[2] - az) * bz) / ll;
      t = Math.min(Math.max(t, 0), 1);
    }
    const d = Math.hypot(p[0] - (ax + t * bx), p[1] - (ay + t * by), p[2] - (az + t * bz));
    if (d < best) best = d;
  }
  return best;
}

// Exact nearest-rank percentile of a list of distances (the definition the
// module uses: rank = p*N, first order statistic whose cumulative count >= rank).
export function nearestRank(values, p) {
  const sorted = [...values].sort((a, b) => a - b);
  const rank = p * sorted.length;
  let cumulative = 0;
  for (const v of sorted) {
    cumulative += 1;
    if (cumulative >= rank) return v;
  }
  return sorted[sorted.length - 1];
}

export function rms(values) {
  return Math.sqrt(values.reduce((a, v) => a + v * v, 0) / values.length);
}

// Arc-length coordinate of the nearest point of the reference polyline, found by
// a monotone forward search (the same matching assumption the monitor makes:
// successive samples advance along the path).
export function arcCoordinate(segments, p, startArc = 0) {
  let best = Infinity;
  let bestArc = startArc;
  let acc = 0;
  for (const s of segments) {
    const [ax, ay, az] = s.start;
    const bx = s.end[0] - ax;
    const by = s.end[1] - ay;
    const bz = s.end[2] - az;
    const len = Math.hypot(bx, by, bz);
    const ll = len * len;
    let t = ll > 0 ? ((p[0] - ax) * bx + (p[1] - ay) * by + (p[2] - az) * bz) / ll : 0;
    t = Math.min(Math.max(t, 0), 1);
    const d = Math.hypot(p[0] - (ax + t * bx), p[1] - (ay + t * by), p[2] - (az + t * bz));
    const arc = acc + t * len;
    const inWindow = arc >= startArc - 1e-6 && arc <= startArc + 0.02;
    if (d < best - 1e-15 && inWindow) { best = d; bestArc = arc; }
    acc += len;
  }
  return { arc: bestArc, dist: best };
}

// Samples whose E chunk starts at the sample position (the simulator's
// convention: the command carrying E for the next step is applied while the tip
// is still at the step's start). E per sample = arc length to the next sample.
export function samplesFromPoints(points, segments, { ePerMeter = 1000 } = {}) {
  const arcs = [];
  let cursor = 0;
  for (const p of points) {
    const r = arcCoordinate(segments, p, cursor);
    cursor = r.arc;
    arcs.push(r.arc);
  }
  const out = [];
  for (let i = 0; i < points.length; i += 1) {
    const nextArc = i + 1 < points.length ? arcs[i + 1] : cursor;
    const E = Math.max(1e-9, (nextArc - arcs[i]) * ePerMeter);
    out.push({ pos: points[i], E });
  }
  return out;
}

export function vertexMinDistance(points, v) {
  let best = Infinity;
  for (let i = 0; i + 1 < points.length; i += 1) {
    const a = points[i];
    const b = points[i + 1];
    const bx = b[0] - a[0], by = b[1] - a[1], bz = b[2] - a[2];
    const ll = bx * bx + by * by + bz * bz;
    let t = ll > 0 ? ((v[0] - a[0]) * bx + (v[1] - a[1]) * by + (v[2] - a[2]) * bz) / ll : 0;
    t = Math.max(0, Math.min(1, t));
    const d = Math.hypot(v[0] - (a[0] + t * bx), v[1] - (a[1] + t * by), v[2] - (a[2] + t * bz));
    if (d < best) best = d;
  }
  return best;
}
