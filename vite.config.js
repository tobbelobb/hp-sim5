import { defineConfig } from 'vite';
import { resolve } from 'path';
import fg from 'fast-glob';

export default defineConfig(async () => {

  const pages = await fg('**/*.html', { ignore: ['node_modules/**', 'attic/**'] });
  const inputs = pages.reduce((o, p) => { o[p] = resolve(__dirname, p); return o; }, {});

  return {
    base: '/hp-sim5/',
    build: { rollupOptions: { input: inputs } },
    assetsInclude: ['**/*.usda']
  };
});
