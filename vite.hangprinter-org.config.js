// vite.hangprinter-org.config.js
import { defineConfig } from 'vite';
import { resolve, join } from 'path';
import { promises as fs } from 'fs';
import fg from 'fast-glob';

export default defineConfig(async () => {
  // Vite only emits pages inside its root, so build from the repo root and
  // lift the hangprinter-org pages to the site root afterwards.
  const root = resolve(__dirname, 'hangprinter-org');
  const pages = await fg('**/*.html', {
    cwd: root,
    ignore: ['**/dist/**', '**/.vite/**'],
  });
  const inputs = pages.reduce((o, p) => {
    o[p] = resolve(root, p);
    return o;
  }, { 'hp-sim-3d/index.html': resolve(__dirname, 'hp-sim-3d/index.html') });

  // Copies nested ".../media" folders + slideprinter assets (parity with your main config)
  const copyExtrasPlugin = () => ({
    name: 'copy-hangprinter-org-extras',
    closeBundle: async () => {
      const distRoot = resolve(__dirname, 'dist-hangprinter-org');
      const builtPages = join(distRoot, 'hangprinter-org');
      await fs.cp(builtPages, distRoot, { recursive: true });
      await fs.rm(builtPages, { recursive: true });

      // Copy any "<section>/media/**" dirs that aren’t auto-imported by Vite
      const mediaDirs = await fg('**/media', { cwd: root, onlyDirectories: true });
      await Promise.all(mediaDirs.map(async (dir) => {
        const from = join(root, dir);
        const to = join(distRoot, dir);
        await fs.mkdir(to, { recursive: true });
        await fs.cp(from, to, { recursive: true });
      }));

      // Copy worker helper modules that the workers import directly
      const klipperSrc = resolve(__dirname, 'integrations/klipper');
      const sharedSrc = resolve(__dirname, 'integrations/shared');
      const slideprinterDist = resolve(distRoot, 'assets');
      await fs.mkdir(slideprinterDist, { recursive: true });
      await fs.copyFile(join(klipperSrc, 'klipperSerialDecoder.js'), join(slideprinterDist, 'klipperSerialDecoder.js'));
      await fs.copyFile(join(sharedSrc, 'fileFormatUtils.js'), join(slideprinterDist, 'fileFormatUtils.js'));
      await fs.copyFile(join(sharedSrc, 'motionUtils.js'), join(slideprinterDist, 'motionUtils.js'));

      // hp-sim-3d fetches these config files at runtime to map RRF drivers to axes.
      const rrfConfigSrc = resolve(__dirname, 'RRF/run/vsd/sys');
      const rrfConfigDist = resolve(distRoot, 'RRF/run/vsd/sys');
      const rrfConfigFiles = await fg('config_*.g', { cwd: rrfConfigSrc, onlyFiles: true });
      await fs.mkdir(rrfConfigDist, { recursive: true });
      await Promise.all(rrfConfigFiles.map((file) => (
        fs.copyFile(join(rrfConfigSrc, file), join(rrfConfigDist, file))
      )));
    },
  });

  return {
    // Important bits for a standalone site at a custom domain:
    root: __dirname,
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
