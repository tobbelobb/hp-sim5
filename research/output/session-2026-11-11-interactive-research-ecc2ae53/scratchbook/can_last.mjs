// Print command count and last motor targets of CAN streams, decoded with the given RRF config's axis map.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const [config, ...files] = process.argv.slice(2);
for (const f of files) {
  const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
  p.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(config, 'utf8')));
  const last = {}; let n = 0, peak = 0;
  p.sendCommand = async c => { n++; for (const [k, v] of Object.entries(c)) if (typeof v === 'number') { last[k] = v; } };
  await p.run(Readable.toWeb(fs.createReadStream(f)), FileFormat.RRF_CAN_BINARY);
  console.log(f.split('/').pop().padEnd(48), n, JSON.stringify(Object.fromEntries(Object.entries(last).map(([k, v]) => [k, +v.toFixed(4)]))));
}
