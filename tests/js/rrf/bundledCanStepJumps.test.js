import fs from 'node:fs';
import path from 'node:path';
import { Readable } from 'node:stream';
import { FileFormat } from '../../../integrations/shared/fileFormatUtils.js';

// A single motion message that moves a spool motor thousands of steps is a
// capture artifact, not a planned move: the RRF host simulator once reset
// motor endpoints to zero at every standstill, so the bundled logos ended
// with a 63 ms move of every motor back home (3801-14877 steps). Planned
// moves in the bundled files stay below 250 steps per message.
const MAX_SPOOL_STEPS_PER_MESSAGE = 1000;
const CAN_DIR = path.join(__dirname, '../../../public/RRF_CAN_commands');

describe('bundled RRF CAN command files', () => {
  let RrfCanPlayer;

  beforeAll(async () => {
    globalThis.self = { addEventListener() {} };
    globalThis.postMessage = (message) => {
      if (message?.type === 'error') {
        throw new Error(message.message);
      }
    };
    ({ RrfCanPlayer } = await import('../../../integrations/rrf/rrfCanPlayer.js'));
  });

  afterAll(() => {
    delete globalThis.self;
    delete globalThis.postMessage;
  });

  const files = fs.readdirSync(CAN_DIR).filter((name) => name.endsWith('.can'));

  test('exist', () => {
    expect(files.length).toBeGreaterThan(0);
  });

  test.each(files)('%s has no single-message spool motor jump', async (name) => {
    const player = new RrfCanPlayer();
    let worst = 0;
    let messages = 0;
    // Spool motors carry integer steps; extruders carry fractional steps.
    player._handleMovement = (row) => {
      if (!Number.isInteger(row.steps)) {
        return;
      }
      messages += 1;
      worst = Math.max(worst, Math.abs(row.steps));
    };
    player.sendCommand = async () => {};
    await player.run(Readable.toWeb(fs.createReadStream(path.join(CAN_DIR, name))), FileFormat.RRF_CAN_BINARY);
    expect(messages).toBeGreaterThan(0);
    expect(worst).toBeLessThanOrEqual(MAX_SPOOL_STEPS_PER_MESSAGE);
  });
});
