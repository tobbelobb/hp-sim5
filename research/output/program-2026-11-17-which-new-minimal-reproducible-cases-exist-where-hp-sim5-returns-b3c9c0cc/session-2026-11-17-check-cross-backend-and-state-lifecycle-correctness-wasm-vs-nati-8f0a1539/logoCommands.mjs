// Real bundled RRF/CAN motion schedules, decoded to the Move commands that the
// production RemoteSpoolSystem consumes one per 2 ms physics step.
// Same approach as tests/benchmark/hp4_logo_commands.mjs, parameterized by
// machine and layering so both backends receive byte-identical command arrays.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';

globalThis.self ??= { addEventListener() {} };
globalThis.postMessage ??= message => {
  if (message.type === 'error') throw new Error(message.message);
};

const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');

const MACHINES = {
  hp3: {
    layered: 'Hangprinter_logo6_hp3_w_line_layers.can',
    unlayered: 'Hangprinter_logo6_hp3_no_buildup.can',
    config: 'RRF/run/vsd/sys/config_hp3_w_line_layers.g',
  },
  hp4: {
    layered: 'Hangprinter_logo6_hp4_w_line_layers.can',
    unlayered: 'Hangprinter_logo6_hp4_no_buildup.can',
    config: 'RRF/run/vsd/sys/config_hp4_w_line_layers.g',
  },
};

export function logoCommands(machine, { layering = true, file: chosen, config, limit = 10000 } = {}) {
  const spec = MACHINES[machine];
  if (!spec) throw new Error(`No bundled logo schedule for ${machine}`);
  const player = new RrfCanPlayer();
  player.setDt(0.002);
  player.setAsapMode(true);
  player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(
    fs.readFileSync(new URL(`../../../../${config ?? spec.config}`, import.meta.url), 'utf8')));
  const commands = [];
  player.sendCommand = async command => { if (commands.length < limit) commands.push(command); };
  const file = chosen ?? (layering ? spec.layered : spec.unlayered);
  return player.run(Readable.toWeb(fs.createReadStream(
    new URL(`../../../../public/RRF_CAN_commands/${file}`, import.meta.url))), FileFormat.RRF_CAN_BINARY)
    .then(() => commands);
}
