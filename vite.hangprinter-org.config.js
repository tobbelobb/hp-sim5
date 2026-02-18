// vite.hangprinter-org.config.js
import { defineConfig } from 'vite';
import { resolve, join } from 'path';
import { promises as fs } from 'fs';
import fg from 'fast-glob';

export default defineConfig(async () => {
  const root = resolve(__dirname, 'hangprinter-org');

  // Only HTML inside hangprinter-org (multi-page support)
  const pages = await fg('**/*.html', {
    cwd: root,
    ignore: ['**/dist/**', '**/.vite/**'],
  });
  const inputs = pages.reduce((o, p) => {
    o[p] = resolve(root, p);
    return o;
  }, {});

  // Copies nested ".../media" folders + slideprinter assets (parity with your main config)
  const copyExtrasPlugin = () => ({
    name: 'copy-hangprinter-org-extras',
    closeBundle: async () => {
      const distRoot = resolve(__dirname, 'dist-hangprinter-org');

      // Copy any "<section>/media/**" dirs that aren’t auto-imported by Vite
      const mediaDirs = await fg('**/media', { cwd: root, onlyDirectories: true });
      await Promise.all(mediaDirs.map(async (dir) => {
        const from = join(root, dir);
        const to = join(distRoot, dir);
        await fs.mkdir(to, { recursive: true });
        await fs.cp(from, to, { recursive: true });
      }));

      // Copy slideprinter helper modules that the workers import directly
      const slideprinterSrc = resolve(__dirname, 'examples/js/slideprinter');
      const slideprinterDist = resolve(distRoot, 'assets');
      await fs.mkdir(slideprinterDist, { recursive: true });
      for (const file of ['kinematics.js', 'guessedData.js', 'fileFormatUtils.js', 'rrfMotionUtils.js', 'klipperSerialParser.js']) {
        await fs.copyFile(join(slideprinterSrc, file), join(slideprinterDist, file));
      }
    },
  });

  return {
    // Important bits for a standalone site at a custom domain:
    root,
    base: '/',                   // no '/hp-sim5/' — serve from the domain root
    publicDir: resolve(__dirname, 'public'), // reuse shared static assets (usd scenes etc.)
    build: {
      outDir: resolve(__dirname, 'dist-hangprinter-org'), // separate output
      emptyOutDir: true,
      rollupOptions: { input: inputs },
      assetsDir: 'assets'
    },
    // Keep your special formats working:
    assetsInclude: [
      '**/*.usda',
      '**/*.usda.txt',
      '**/*.usdc',
      '**/*.gcode',
      '**/*.serial',
      '**/*.txt',
    ],
    plugins: [copyExtrasPlugin()],
  };
});
