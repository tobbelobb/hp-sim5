import { createViteServerOptions, getViteWatchIgnoredGlobs } from '../viteServerOptions.mjs';

describe('Vite server options for integration tests', () => {
  test('ignore list excludes workspace-scale directories before the watcher starts', () => {
    const ignored = createViteServerOptions('/tmp/project').server.watch.ignored;

    expect(ignored).toEqual(
      expect.arrayContaining([
        '**/autocal/**',
        '**/RRF/**',
        '**/.venv/**',
        '**/build/**',
        '**/__pycache__/**',
      ]),
    );
  });

  test('shared watcher ignore globs stay focused on generated and heavy trees', () => {
    expect(getViteWatchIgnoredGlobs()).toEqual(
      expect.arrayContaining([
        '**/node_modules/**',
        '**/dist/**',
        '**/dist-hangprinter-org/**',
        '**/RRF/**',
        '**/.venv/**',
      ]),
    );
  });
});
