// Summarize native Node/V8 profiles; load raw files in Chrome DevTools for stacks.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const frameKey = frame => `${frame.functionName || '(anonymous)'} ${frame.url}:${frame.lineNumber + 1}`;

export function summarizeCpu(profile, windows) {
  const nodes = new Map(profile.nodes.map(node => [node.id, node]));
  const parents = new Map();
  for (const node of profile.nodes) for (const child of node.children ?? []) parents.set(child, node.id);
  const rows = new Map();
  let time = profile.startTime, sampledUs = 0;
  for (let i = 0; i < profile.samples.length; i++) {
    const next = time + profile.timeDeltas[i];
    const duration = windows.reduce((sum, window) => sum + Math.max(0, Math.min(next, window.endUs) - Math.max(time, window.startUs)), 0);
    time = next;
    if (!duration) continue;
    sampledUs += duration;
    let id = profile.samples[i], leaf = true;
    const seen = new Set();
    while (id !== undefined) {
      const key = frameKey(nodes.get(id).callFrame);
      if (!seen.has(key)) {
        seen.add(key);
        const row = rows.get(key) ?? { function: key, selfUs: 0, inclusiveUs: 0 };
        row.inclusiveUs += duration;
        if (leaf) row.selfUs += duration;
        rows.set(key, row);
      }
      leaf = false; id = parents.get(id);
    }
  }
  if (!sampledUs) throw new Error('CPU profile does not overlap measurement windows');
  const all = [...rows.values()].map(row => ({ ...row, selfPercent: 100 * row.selfUs / sampledUs }));
  return { sampledMs: sampledUs / 1000,
    bySelf: all.sort((a, b) => b.selfUs - a.selfUs).slice(0, 50),
    byInclusive: [...all].sort((a, b) => b.inclusiveUs - a.inclusiveUs).slice(0, 50) };
}

export function summarizeHeap(profile) {
  const rows = new Map();
  function visit(node) {
    const key = frameKey(node.callFrame);
    const row = rows.get(key) ?? { function: key, selfBytes: 0 };
    row.selfBytes += node.selfSize; rows.set(key, row);
    for (const child of node.children ?? []) visit(child);
  }
  visit(profile.head);
  const all = [...rows.values()].sort((a, b) => b.selfBytes - a.selfBytes);
  return { estimatedAllocatedBytes: all.reduce((sum, row) => sum + row.selfBytes, 0), bySelf: all.slice(0, 50) };
}

if (process.argv[1] && fileURLToPath(import.meta.url) === path.resolve(process.argv[1])) {
  const [profileFile, resultFile] = process.argv.slice(2);
  if (!profileFile) throw new Error('Usage: node scripts/summarize_v8_profile.mjs PROFILE [BENCHMARK.json]');
  const profile = JSON.parse(fs.readFileSync(profileFile));
  const summary = profile.nodes
    ? summarizeCpu(profile, resultFile ? JSON.parse(fs.readFileSync(resultFile)).runs : [{ startUs: profile.startTime, endUs: profile.endTime }])
    : summarizeHeap(profile);
  console.log(JSON.stringify(summary, null, 2));
}
