import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { MACHINE_CONFIGS, loadSweepConfigFile } from '../../behaviors/sweep_data_collection.mjs';

async function load(text) {
  const dir = await fs.mkdtemp(path.join(os.tmpdir(), 'sweep-cfg-'));
  const file = path.join(dir, 'cfg.txt');
  await fs.writeFile(file, text);
  return loadSweepConfigFile(file, MACHINE_CONFIGS.hangprinter_4);
}

describe('sweep config files', () => {
  test('per-line held lengths follow the fixed anchors through sorting', async () => {
    const configs = await load('[3,0] 1 2 fixed=-215,430\n[1,3] 0 2\n');
    expect(configs[0]).toMatchObject({ fixedAnchors: [0, 3], fixedTargets: [430, -215] });
    expect(configs[1].fixedTargets).toBeUndefined();
  });

  test('rows that differ only in held lengths are kept', async () => {
    const configs = await load('[0,3] 1 2 fixed=430,0\n[0,3] 1 2 fixed=215,0\n');
    expect(configs.map(cfg => cfg.fixedTargets)).toEqual([[430, 0], [215, 0]]);
  });

  test('held lengths need one value per fixed anchor', async () => {
    await expect(load('[0,3] 1 2 fixed=430\n')).rejects.toThrow('one length per fixed anchor');
  });
});
