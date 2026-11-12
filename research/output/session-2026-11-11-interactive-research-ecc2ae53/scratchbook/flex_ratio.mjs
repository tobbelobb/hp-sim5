// Ratio of flex (stream minus no-flex stream) between two flex-enabled streams, per axis, at sampled steps.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const [config, noflex, oldf, newf] = process.argv.slice(2);
async function decode(f) {
  const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
  p.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(config, 'utf8')));
  const out = [], cur = {}; p.sendCommand = async c => { Object.assign(cur, c); out.push({ ...cur }); };
  await p.run(Readable.toWeb(fs.createReadStream(f)), FileFormat.RRF_CAN_BINARY); return out;
}
const [z, a, b] = await Promise.all([noflex, oldf, newf].map(decode));
for (const step of [2000, 10000, 20000, 30000, 40000]) console.log(step, Object.fromEntries('ABCD'.split('').map(k => [k, +((b[step][k] - z[step][k]) / (a[step][k] - z[step][k])).toFixed(3)])));
