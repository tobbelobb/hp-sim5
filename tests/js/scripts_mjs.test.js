import { readdirSync } from 'fs';
import path from 'path';
import { spawnSync } from 'child_process';

const controlTestsDir = path.resolve(__dirname, '../../autocal/control/tests');
const unitTestDirs = ['behaviors', 'cli', 'primitives'];
const scripts = unitTestDirs
  .flatMap((dir) => {
    const dirPath = path.join(controlTestsDir, dir);
    return readdirSync(dirPath)
      .filter((name) => name.endsWith('.test.mjs'))
      .map((name) => path.join(dirPath, name));
  })
  .sort();

describe('autocal/control/tests/*/*.test.mjs', () => {
  for (const scriptPath of scripts) {
    const scriptName = path.relative(controlTestsDir, scriptPath);
    test(scriptName, () => {
      const result = spawnSync('node', [scriptPath], { encoding: 'utf8' });
      if (result.status !== 0) {
        const stdout = result.stdout ? `\nstdout:\n${result.stdout}` : '';
        const stderr = result.stderr ? `\nstderr:\n${result.stderr}` : '';
        throw new Error(`script failed: ${scriptPath}${stdout}${stderr}`);
      }
    });
  }
});
