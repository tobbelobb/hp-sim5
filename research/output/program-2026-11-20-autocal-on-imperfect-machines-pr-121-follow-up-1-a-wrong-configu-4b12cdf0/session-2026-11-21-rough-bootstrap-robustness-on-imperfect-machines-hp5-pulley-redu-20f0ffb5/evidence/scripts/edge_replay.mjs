// Replay logged edge-ramp travel through the old (3aa8cacc) and new findEdgeForce.
// usage (from the repo root; the old module needs its own directory for relative imports):
//   git show 3aa8cacc:autocal/control/behaviors/force_tuning.mjs > autocal/control/behaviors/_before.tmp.mjs
//   node edge_replay.mjs autocal/control/behaviors/_before.tmp.mjs autocal/control/behaviors/force_tuning.mjs tuning.json...
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';
const [oldPath, newPath, ...files] = process.argv.slice(2);
const oldMod = await import(pathToFileURL(resolve(oldPath)));
const newMod = await import(pathToFileURL(resolve(newPath)));
async function run(fn, t) {
  const samples = t.edge_samples.slice();
  const r = await fn(null, { forceStart: t.force_start, capForceLimit: t.force_cap, bracketFactor: 1.4,
    trialFn: async () => { const s = samples.shift(); return s ? { travelDeg: s.travelDeg, moved: s.moved, stalled: s.stalled, fixedDriftDeg: s.fixedDriftDeg } : { travelDeg: NaN }; } });
  return { edge: r.forceEdge, n: r.samples.length, reason: r.reason, exhausted: samples.length === 0 && r.samples.length > t.edge_samples.length };
}
for (const f of files) {
  let t; try { t = JSON.parse(readFileSync(f)).force_tuning; } catch { continue; }
  if (!Array.isArray(t?.edge_samples) || !Number.isFinite(t.force_start)) continue;
  const o = await run(oldMod.findEdgeForce, t), n = await run(newMod.findEdgeForce, t);
  const same = o.edge === n.edge && o.n === n.n;
  console.log(JSON.stringify({ file: f, logged_edge: t.force_edge, start: t.force_start, old: o, new: n, same }));
}
