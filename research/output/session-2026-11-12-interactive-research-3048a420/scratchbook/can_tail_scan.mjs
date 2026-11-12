// For each .can: the last motion message's spool-axis steps and duration.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const root = new URL('../../../../', import.meta.url);
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
for (const f of fs.readdirSync(new URL('public/RRF_CAN_commands/', root)).filter(f => f.endsWith('.can'))) {
  const p = new RrfCanPlayer(); p.setDt(.002); p.setAsapMode(true);
  const rows = []; p._handleMovement = r => rows.push(r); p.sendCommand = async () => {};
  await p.run(Readable.toWeb(fs.createReadStream(new URL(`public/RRF_CAN_commands/${f}`, root))), FileFormat.RRF_CAN_BINARY);
  const last = rows.at(-1).whenToExecute;
  const tail = rows.filter(r => r.whenToExecute === last);
  const ticks = tail[0].accelTicks + tail[0].steadyTicks + tail[0].decelTicks;
  const maxSteps = Math.max(...tail.filter(r => Number.isInteger(r.steps)).map(r => Math.abs(r.steps)), 0);
  console.log(f.padEnd(52), 'msgs', tail.length, 'ticks', ticks, `(${(ticks/750).toFixed(0)} ms)`, 'max|steps|', maxSteps, 'motors', tail.map(r=>r.motorId+':'+Math.round(r.steps)).join(' '));
}
