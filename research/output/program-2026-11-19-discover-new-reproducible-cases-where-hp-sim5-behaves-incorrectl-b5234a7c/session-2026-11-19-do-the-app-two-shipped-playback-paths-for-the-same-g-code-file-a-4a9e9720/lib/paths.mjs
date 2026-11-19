// Streaming accumulators so the big CAN plans stay small in memory.
import {
  parseMachine, linesFromAngles, idealLines, positionFromLines,
  gcodePolyline, nearestOnPolyline,
} from './rrfModel.mjs';

class Grow {
  constructor(width, cap = 4096) {
    this.width = width;
    this.data = new Float64Array(cap * width);
    this.n = 0;
  }

  push(values) {
    if ((this.n + 1) * this.width > this.data.length) {
      const next = new Float64Array(this.data.length * 2);
      next.set(this.data);
      this.data = next;
    }
    this.data.set(values, this.n * this.width);
    this.n += 1;
  }

  row(i) {
    return this.data.subarray(i * this.width, (i + 1) * this.width);
  }
}

/**
 * Fold the world-facing command stream into per-move samples.
 * 'Move' commands are absolute motor angles; 'Add to reference' shifts them.
 */
export function createAccumulator(machine) {
  const na = machine.axes.length;
  const angles = new Grow(na);
  const lines = new Grow(na);
  const pos = new Grow(3);
  const residual = [];
  const offsets = new Float64Array(na);
  const absLine = new Float64Array(na);
  const absAngle = new Float64Array(na);
  const angleOf = (c) => {
    const out = new Float64Array(na);
    for (let k = 0; k < na; k += 1) {
      const v = Number.isFinite(c[machine.axes[k]]) ? c[machine.axes[k]] : 0;
      out[k] = v + offsets[k];
    }
    return out;
  };
  let guess = [0, 0, 0];
  let prevAngle = null;
  let prevLine = null;
  let pathTravel = 0;
  let n = 0;

  return {
    push(command) {
      if (command.type === 'Add to reference') {
        for (let k = 0; k < na; k += 1) {
          if (Number.isFinite(command[machine.axes[k]])) offsets[k] += command[machine.axes[k]];
        }
        return;
      }
      if (command.type !== 'Move') return;
      const a = angleOf(command);
      const angObj = {};
      for (let k = 0; k < na; k += 1) angObj[machine.axes[k]] = a[k];
      const l = linesFromAngles(machine, angObj);
      if (prevAngle) for (let k = 0; k < na; k += 1) absAngle[k] += Math.abs(a[k] - prevAngle[k]);
      if (prevLine) for (let k = 0; k < na; k += 1) absLine[k] += Math.abs(l[k] - prevLine[k]);
      const r = positionFromLines(machine, l, guess);
      guess = r.p;
      if (n > 0) {
        pathTravel += Math.hypot(
          guess[0] - pos.row(n - 1)[0], guess[1] - pos.row(n - 1)[1], guess[2] - pos.row(n - 1)[2]);
      }
      angles.push(a);
      lines.push(l);
      pos.push(guess);
      residual.push(r.residualRms);
      prevAngle = a;
      prevLine = l;
      n += 1;
    },
    finish() {
      const netAngle = new Float64Array(na);
      if (n > 0) for (let k = 0; k < na; k += 1) netAngle[k] = angles.row(n - 1)[k] - angles.row(0)[k];
      return {
        n, axes: machine.axes, angles, lines, pos,
        residual: Float64Array.from(residual),
        absLine, absAngle, netAngle, pathTravel,
      };
    },
  };
}

export function accumulate(machine, commands) {
  const acc = createAccumulator(machine);
  for (const c of commands) acc.push(c);
  return acc.finish();
}

export function pathStats(acc, polyline) {
  let maxD = 0, sum2 = 0, worst = null, worstSeg = -1;
  const perSeg = new Map();
  for (let i = 0; i < acc.n; i += 1) {
    const p = Array.from(acc.pos.row(i));
    const near = nearestOnPolyline(p, polyline);
    if (near.d > maxD) { maxD = near.d; worst = p; worstSeg = near.seg; }
    sum2 += near.d * near.d;
    const cur = perSeg.get(near.seg) || { max: 0, sum2: 0, n: 0 };
    cur.max = Math.max(cur.max, near.d); cur.sum2 += near.d * near.d; cur.n += 1;
    perSeg.set(near.seg, cur);
  }
  return { maxD, rms: Math.sqrt(sum2 / acc.n), worst, worstSeg, perSeg };
}

/** line-position error against the ideal geometry of the commanded G-code point */
export function lineError(machine, acc, polyline) {
  const na = machine.axes.length;
  const max = new Array(na).fill(0);
  const min = new Array(na).fill(0);
  const sum2 = new Array(na).fill(0);
  const atWorst = new Array(na).fill(null);
  for (let i = 0; i < acc.n; i += 1) {
    const p = Array.from(acc.pos.row(i));
    const near = nearestOnPolyline(p, polyline);
    const ideal = idealLines(machine, near.q);
    const row = acc.lines.row(i);
    for (let k = 0; k < na; k += 1) {
      const d = row[k] - ideal[k];
      if (Math.abs(d) > Math.abs(Math.abs(max[k]) > Math.abs(min[k]) ? max[k] : min[k])) {
        atWorst[k] = near.q.map((v) => +v.toFixed(1));
      }
      if (d > max[k]) max[k] = d;
      if (d < min[k]) min[k] = d;
      sum2[k] += d * d;
    }
  }
  return { max, min, rms: sum2.map((v) => Math.sqrt(v / acc.n)), atWorst };
}

export function polylineLength(pts) {
  let s = 0;
  for (let i = 1; i < pts.length; i += 1) {
    s += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1], pts[i][2] - pts[i - 1][2]);
  }
  return s;
}

export { parseMachine, gcodePolyline };
