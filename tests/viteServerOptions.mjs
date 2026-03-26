export function getViteWatchIgnoredGlobs() {
  return [
    '**/node_modules/**',
    '**/attic/**',
    '**/dist/**',
    '**/dist-hangprinter-org/**',
    '**/RRF/**',
    '**/.venv/**',
    '**/build/**',
    '**/*.o',
    '**/__pycache__/**',
  ];
}

export function createViteServerOptions(projectRoot) {
  return {
    root: projectRoot,
    configFile: false,
    logLevel: 'error',
    server: {
      port: 0,
      host: '127.0.0.1',
      watch: {
        ignored: [
          '**/autocal/**',
          ...getViteWatchIgnoredGlobs(),
        ],
      },
    },
    optimizeDeps: { entries: [] },
  };
}
