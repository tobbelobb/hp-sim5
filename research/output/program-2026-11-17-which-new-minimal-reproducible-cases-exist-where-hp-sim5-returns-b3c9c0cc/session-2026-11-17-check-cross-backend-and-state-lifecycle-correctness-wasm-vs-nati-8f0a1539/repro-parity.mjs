// Finding 4: Wasm and native parity on the production pipeline.
//
// Measures, with identical scene text and identical command schedules:
//   * the prepared record difference before any step (must be 0),
//   * per-sample max absolute differences in the full entity record, cable-path
//     records, body position, encoder angle and quaternion angle,
//   * discrete agreement: topology version, cable joint count, missed steps,
//     non-finite state,
//   * a one-step fork (re-synchronize Wasm from native every step) to separate
//     the per-step operator difference from feedback amplification,
//   * a native-only 1e-15/1e-12/1e-9 perturbation series to show whether
//     divergence is linear amplification of infinitesimal noise or a discrete
//     branch decision.
//
// Exit code 1 when a *qualitative* difference appears (topology/joint-count/
// missed-step mismatch, or mismatched non-finite state). Quantitative rounding
// differences are reported in results.json without failing the run.
import fs from 'node:fs';
import { measure, oneStepFork, perturbationScaling } from './parity.mjs';
import { logoCommands } from './logoCommands.mjs';

const here = new URL('./', import.meta.url);

function fitGrowth(rows, key, floor, ceiling) {
  const points = rows.filter(row => Number.isFinite(row[key]) && row[key] > floor && row[key] < ceiling)
    .map(row => [row.step, Math.log(row[key])]);
  if (points.length < 3) return null;
  const n = points.length;
  const mx = points.reduce((s, p) => s + p[0], 0) / n;
  const my = points.reduce((s, p) => s + p[1], 0) / n;
  const cov = points.reduce((s, p) => s + (p[0] - mx) * (p[1] - my), 0);
  const varX = points.reduce((s, p) => s + (p[0] - mx) ** 2, 0);
  const lambda = cov / varX;
  return { lambdaPerStep: lambda, lambdaPerSecond: lambda / 0.002,
    doublingSteps: lambda > 0 ? Math.log(2) / lambda : null, samples: n,
    fromStep: points[0][0], toStep: points.at(-1)[0] };
}

const cases = [];
const qualitative = [];

// ---- Case 1: untouched HP3 settling ---------------------------------------
{
  const result = await measure({ machine: 'hp3', layering: true, steps: 2000, probe: true, probeSteps: 2000, sampleEvery: 50 });
  cases.push({ id: 'hp3-settle-layered', ...result });
}

// ---- Case 2: HP3 logo, layering off (documented drift case) ---------------
{
  const commands = await logoCommands('hp3', { layering: false, limit: 20000 });
  const result = await measure({ machine: 'hp3', layering: false, steps: 20000, commands,
    probe: true, probeSteps: 20000, sampleEvery: 100 });
  cases.push({ id: 'hp3-logo-off', ...result });
}

// ---- Case 3: HP4 logo, layering on (production print) ---------------------
{
  const commands = await logoCommands('hp4', { layering: true, limit: 20000 });
  const result = await measure({ machine: 'hp4', layering: true, steps: 20000, commands,
    probe: true, probeSteps: 20000, sampleEvery: 100 });
  cases.push({ id: 'hp4-logo-on', ...result });
}

// ---- Case 4: HP4 logo with closed-loop motor feedback ---------------------
{
  const commands = await logoCommands('hp4', { layering: true, limit: 20000 });
  const result = await measure({ machine: 'hp4', layering: true, closedLoop: true, steps: 20000, commands,
    probe: true, probeSteps: 20000, sampleEvery: 100 });
  cases.push({ id: 'hp4-logo-closedloop', ...result });
}

// ---- One-step fork: is the per-step operator itself different? ------------
const forks = [];
{
  const commands = await logoCommands('hp4', { layering: true, limit: 500 });
  forks.push(await oneStepFork({ machine: 'hp4', layering: true, steps: 500, commands }));
  forks.push(await oneStepFork({ machine: 'hp3', layering: false, steps: 500,
    commands: await logoCommands('hp3', { layering: false, limit: 500 }) }));
}

// ---- Perturbation scaling on the most sensitive production case -----------
const scaling = await perturbationScaling({ machine: 'hp4', layering: true, steps: 1500,
  commands: await logoCommands('hp4', { layering: true, limit: 1500 }), deltas: [1e-15, 1e-12, 1e-9], sampleEvery: 100 });

for (const item of cases) {
  const last = item.lastRow;
  console.log(`\n${item.id}: ${item.case.steps} steps`);
  console.log(`  prepared records: max diff ${item.initialRecordMaxDifference} (lengths ${item.initialRecordLengths.join('/')})`);
  console.log(`  final entity-record max abs diff ${item.finalMaxAbs?.toExponential(3)} at ${item.finalMaxKey}`);
  console.log(`  final body position diff ${last.dPosition.toExponential(3)} m, encoder ${last.dEncoder.toExponential(3)} rad, quaternion ${last.quatAngle.toExponential(3)} rad`);
  console.log(`  peak body position ${Math.max(...item.rows.map(r => r.dPosition)).toExponential(3)} m, peak encoder ${Math.max(...item.rows.map(r => r.dEncoder)).toExponential(3)} rad, peak quaternion ${Math.max(...item.rows.map(r => r.quatAngle)).toExponential(3)} rad`);
  console.log(`  events: ${JSON.stringify(item.events)}`);
  console.log(`  growth(position): ${JSON.stringify(fitGrowth(item.rows, 'dPosition', 1e-17, 1e-3))}`);
  console.log(`  growth(encoder):  ${JSON.stringify(fitGrowth(item.rows, 'dEncoder', 1e-16, 1e-2))}`);
  if (item.events.lengthMismatch || item.events.topologyFirstDifference ||
      item.events.missedStepDifference || item.events.nonFinite) qualitative.push({ id: item.id, events: item.events });
}

for (const fork of forks) {
  console.log(`\nfork ${fork.machine} layering=${fork.layering}: ${fork.steps} one-step comparisons, ` +
    `max entity ${fork.maxEntity.toExponential(3)}, path ${fork.maxPath.toExponential(3)}, encoder ${fork.maxEncoder.toExponential(3)}`);
  console.log(`  per-sample: ${JSON.stringify(fork.rows)}`);
  if (fork.pathLengthMismatch) qualitative.push({ id: `fork-${fork.machine}`, events: fork.pathLengthMismatch });
}

console.log('\nperturbation scaling (HP4 logo, native control vs native twins):');
for (const row of scaling.rows) {
  console.log(`  step ${row.step}: ` + row.deltas.map(d => `${d.delta.toExponential(0)} -> entity ${d.entity.toExponential(2)}, pos ${d.position.toExponential(2)}, enc ${d.encoder.toExponential(2)}`).join(' | '));
}

const out = { schema: 1, generatedAt: new Date().toISOString(), revision: 'c2450975',
  cases, forks, scaling, qualitative };
fs.writeFileSync(new URL('results.json', here), JSON.stringify(out, null, 1) + '\n');

if (qualitative.length) {
  console.error(`\nFAIL qualitative cross-backend divergence: ${JSON.stringify(qualitative, null, 2)}`);
  process.exit(1);
}
console.log('\nPASS cross-backend: no topology, joint-count, missed-step or finite-state divergence; differences are quantitative only');
