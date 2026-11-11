// Print cumulative motor targets at chosen steps for CAN streams (axis map from config).
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const [config, stepsText, ...files] = process.argv.slice(2);
const steps = stepsText.split(',').map(Number);
for (const f of files) {
  const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
  p.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(config, 'utf8')));
  const out = [], cur = {}; p.sendCommand = async c => { if (out.length < Math.max(...steps)) { Object.assign(cur, c); out.push({ ...cur }); } };
  await p.run(Readable.toWeb(fs.createReadStream(f)), FileFormat.RRF_CAN_BINARY);
  for (const s of steps) console.log(f.split('/').pop().padEnd(42), s, 'ABCD'.split('').map(k => (out[s - 1][k] ?? 0).toFixed(4)).join(' '));
}
