// Extended parity run: 100,000 steps of a bundled logo schedule (the CAN files
// contain ~1,000,000 commands; the first 100,000 are used, one per 2 ms step).
// Default writes long-hp4-100k.json (~43 s); pass "100000 hp3 off" for the
// unlayered HP3 reference (~10 s).
import fs from 'node:fs';
import { measure } from './parity.mjs';
import { logoCommands } from './logoCommands.mjs';

const steps = Number(process.argv[2] ?? 100000);
const machine = process.argv[3] ?? 'hp4';
const layering = (process.argv[4] ?? 'on') !== 'off';
const file = machine === 'hp4' && layering
  ? `long-hp4-${steps / 1000}k.json`
  : `long-${machine}-${layering ? 'on' : 'off'}-${steps / 1000}k.json`;
const commands = await logoCommands(machine, { layering, limit: steps });
const result = await measure({ machine, layering, steps, commands,
  probe: true, probeSteps: steps, sampleEvery: 100 });
result.longRunCommand = `node run-long-hp4.mjs ${steps} ${machine} ${layering ? 'on' : 'off'}`;
fs.writeFileSync(new URL(file, import.meta.url), JSON.stringify(result, null, 1) + '\n');
console.log(`wrote ${file}: ${steps} steps, peak encoder ` +
  `${Math.max(...result.rows.map(r => r.dEncoder)).toExponential(3)} rad, peak position ` +
  `${Math.max(...result.rows.map(r => r.dPosition)).toExponential(3)} m, events ${JSON.stringify(result.events)}`);
