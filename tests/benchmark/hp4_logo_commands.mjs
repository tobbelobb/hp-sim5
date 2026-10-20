import assert from 'node:assert/strict';
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';

// Schedule the real bundled logo at the firmware's 2 ms cadence. Keep only
// the prefix needed to expose both the immediate and the later missed steps.
globalThis.self = { addEventListener() {} };
globalThis.postMessage = message => {
  if (message.type === 'error') throw new Error(message.message);
};
const { RrfCanPlayer } = await import('../../integrations/rrf/rrfCanPlayer.js');
const player = new RrfCanPlayer();
player.setDt(.002);
player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(
  new URL('../../RRF/run/vsd/sys/config_hp4_w_line_layers.g', import.meta.url), 'utf8')));
export const commands = [];
player.sendCommand = async command => { if (commands.length < 10000) commands.push(command); };
await player.run(Readable.toWeb(fs.createReadStream(
  new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can', import.meta.url))), FileFormat.RRF_CAN_BINARY);
assert.equal(commands.length, 10000);
