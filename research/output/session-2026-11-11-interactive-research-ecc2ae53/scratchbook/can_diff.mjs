// Decode two CAN streams with one config's axis map; report counts, axes and max per-axis difference.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const [config, fa, fb] = process.argv.slice(2);
async function decode(f) {
  const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
  p.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(config, 'utf8')));
  const out = [], cur = {}; p.sendCommand = async c => { Object.assign(cur, c); out.push({ ...cur }); };
  await p.run(Readable.toWeb(fs.createReadStream(f)), FileFormat.RRF_CAN_BINARY); return out;
}
const [a, b] = await Promise.all([fa, fb].map(decode));
const keys = [...new Set([...Object.keys(a.at(-1)), ...Object.keys(b.at(-1))])].filter(k => k !== 'type');
const max = {};
for (let i = 0; i < Math.min(a.length, b.length); i++) for (const k of keys) max[k] = Math.max(max[k] ?? 0, Math.abs((a[i][k] ?? 0) - (b[i][k] ?? 0)));
console.log(JSON.stringify({ na: a.length, nb: b.length, keys, maxAbsDiff: Object.fromEntries(Object.entries(max).map(([k, v]) => [k, +v.toExponential(2)])) }));
