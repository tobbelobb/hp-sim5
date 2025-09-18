// vite.config.js
import { defineConfig } from 'vite';
import { resolve } from 'path';
import fg from 'fast-glob';

export default defineConfig(async () => {
  const pages = await fg('**/*.html', {
    ignore: [
      'node_modules/**',
      'attic/**',
      'dist/**',
      'examples/js/flipper/flipper_with_sliding_beads.html',
      'examples/js/flipper/flipper_with_attached_beads.html'
    ],
  });
  const inputs = pages.reduce((o, p) => { o[p] = resolve(__dirname, p); return o; }, {});

  // Switch between 'require-corp' and 'credentialless' if you want to test both behaviors
  const COEP = process.env.VITE_COEP || 'require-corp';
  const commonHeaders = {
    'Cross-Origin-Opener-Policy': 'same-origin',
    'Cross-Origin-Embedder-Policy': COEP,
  };

  const coiHeadersPlugin = () => ({
    name: 'coi-headers',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        for (const [k, v] of Object.entries(commonHeaders)) res.setHeader(k, v);
        next();
      });
    },
    configurePreviewServer(server) {
      server.middlewares.use((req, res, next) => {
        for (const [k, v] of Object.entries(commonHeaders)) res.setHeader(k, v);
        next();
      });
    },
  });

  return {
    base: '/hp-sim5/',
    build: { rollupOptions: { input: inputs } },
    server: {
      headers: commonHeaders, // still keep this
    },
    preview: {
      headers: commonHeaders,
    },
    plugins: [coiHeadersPlugin()],
    assetsInclude: ['**/*.usda', '**/*.usda.txt'],
  };
});
