// Decode two CAN streams with the HP4 axis map and report command count and max angle difference.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
const root = new URL('../../../../', import.meta.url);
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
async function decode(path) {
  const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
  p.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', root), 'utf8')));
  const out = []; p.sendCommand = async c => { out.push(c); };
  await p.run(Readable.toWeb(fs.createReadStream(path)), FileFormat.RRF_CAN_BINARY); return out;
}
const [a, b] = await Promise.all(process.argv.slice(2, 4).map(decode));
let max = 0, at = -1;
for (let i = 0; i < Math.min(a.length, b.length); i++) for (const k of 'ABCD') { const d = Math.abs((a[i][k] ?? 0) - (b[i][k] ?? 0)); if (d > max) { max = d; at = i; } }
console.log(JSON.stringify({ na: a.length, nb: b.length, maxAbsDiffRad: max, atStep: at, first16: [a[15], b[15]] }));
