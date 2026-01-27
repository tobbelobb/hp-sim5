import { readdirSync } from 'fs';
import path from 'path';
import { spawnSync } from 'child_process';

const scriptsDir = path.resolve(__dirname, '../../scripts');
const scripts = readdirSync(scriptsDir)
  .filter((name) => name.startsWith('test_') && name.endsWith('.mjs'))
  .sort();

describe('scripts/test_*.mjs', () => {
  for (const script of scripts) {
    test(script, () => {
      const scriptPath = path.join(scriptsDir, script);
      const result = spawnSync('node', [scriptPath], { encoding: 'utf8' });
      if (result.status !== 0) {
        const stdout = result.stdout ? `\nstdout:\n${result.stdout}` : '';
        const stderr = result.stderr ? `\nstderr:\n${result.stderr}` : '';
        throw new Error(`script failed: ${scriptPath}${stdout}${stderr}`);
      }
    });
  }
});
