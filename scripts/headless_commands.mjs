// Use the browser's production firmware players to schedule native commands.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { detectFileFormat, isRrfFormat, isKlipperFormat } from '../integrations/shared/fileFormatUtils.js';
import { parseRrfMotorAxisMapFromConfigText } from '../integrations/rrf/rrfFirmwareModel.js';

const [input, output, dt = '0.002', config] = process.argv.slice(2);
if (!input || !output || !Number.isFinite(Number(dt)) || Number(dt) <= 0) {
  throw new Error('Usage: node scripts/headless_commands.mjs INPUT OUTPUT.json [DT] [RRF_CONFIG]');
}
globalThis.self = { addEventListener() {} };
globalThis.postMessage = (message) => {
  if (message.type === 'error') throw new Error(message.message);
};
const format = detectFileFormat(input);
let player;
if (isRrfFormat(format)) {
  const { RrfCanPlayer } = await import('../integrations/rrf/rrfCanPlayer.js');
  player = new RrfCanPlayer();
  if (config) player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(config, 'utf8')));
} else if (isKlipperFormat(format)) {
  const { KlipperMcuCommandPlayer } = await import('../integrations/klipper/klipperMcuCommandPlayer.js');
  player = new KlipperMcuCommandPlayer();
} else {
  throw new Error(`Unsupported command format: ${input}`);
}
player.setDt(Number(dt));
player.setAsapMode(true);
const commands = [];
player.sendCommand = async (command) => { commands.push(command); };
await player.run(Readable.toWeb(fs.createReadStream(input)), format);
fs.writeFileSync(output, JSON.stringify(commands) + '\n');
console.log(`${commands.length} commands (${commands.length * Number(dt)} s): ${output}`);
