import { defineConfig } from 'vite';
import { resolve } from 'path';
import fg from 'fast-glob';

export default defineConfig(async () => {

  const pages = await fg('**/*.html', { ignore: ['node_modules/**', 'attic/**', 'examples/js/flipper/flipper_with_sliding_beads.html', 'examples/js/flipper/flipper_with_attached_beads.html'] });
  const inputs = pages.reduce((o, p) => { o[p] = resolve(__dirname, p); return o; }, {});

  return {
    base: '/hp-sim5/',
    build: { rollupOptions: { input: inputs } },
    server: {
      headers: {
        'Cross-Origin-Opener-Policy': 'same-origin',
        'Cross-Origin-Embedder-Policy': 'require-corp'
      }
    },
    assetsInclude: ['**/*.usda', '**/*.usda.txt']
  };
});
