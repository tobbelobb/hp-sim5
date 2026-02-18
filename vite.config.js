// vite.config.js
import { defineConfig } from 'vite';
import { resolve, join } from 'path';
import { promises as fs } from 'fs';
import fg from 'fast-glob';

export default defineConfig(async () => {
  const pages = await fg('**/*.html', {
    ignore: [
      'node_modules/**',
      'attic/**',
      'dist/**',
      'dist-hangprinter-org/**',
      'examples/js/flipper/flipper_with_sliding_beads.html',
      'examples/js/flipper/flipper_with_attached_beads.html',
      'RRF/**'
    ],
  });
  const inputs = pages.reduce((o, p) => { o[p] = resolve(__dirname, p); return o; }, {});

  // Switch between 'require-corp' and 'credentialless' if you want to test both behaviors
  const COEP = process.env.VITE_COEP || 'require-corp';
  const commonHeaders = {
    'Cross-Origin-Opener-Policy': 'same-origin',
    'Cross-Origin-Embedder-Policy': COEP,
  };
  const skipIsolation = [/^\/hp-sim5\/hangprinter-org\//];
  const applyHeaders = (req, res) => {
    const path = req.url || '';
    if (skipIsolation.some((re) => re.test(path))) return;
    for (const [k, v] of Object.entries(commonHeaders)) res.setHeader(k, v);
  };

  const coiHeadersPlugin = () => ({
    name: 'coi-headers',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        applyHeaders(req, res);
        next();
      });
    },
    configurePreviewServer(server) {
      server.middlewares.use((req, res, next) => {
        applyHeaders(req, res);
        next();
      });
    },
  });

  const copyHangprinterExtrasPlugin = () => ({
    name: 'copy-hangprinter-extras',
    closeBundle: async () => {
      const srcRoot = resolve(__dirname, 'hangprinter-org');
      const distRoot = resolve(__dirname, 'dist/hangprinter-org');
      const mediaDirs = await fg('**/media', {
        cwd: srcRoot,
        onlyDirectories: true,
      });
      await Promise.all(mediaDirs.map(async (dir) => {
        const from = join(srcRoot, dir);
        const to = join(distRoot, dir);
        await fs.mkdir(to, { recursive: true });
        await fs.cp(from, to, { recursive: true });
      }));

      // Copy slideprinter helper modules that the workers import directly
      const slideprinterSrc = resolve(__dirname, 'examples/js/slideprinter');
      const slideprinterDist = resolve(__dirname, 'dist/assets');
      for (const file of ['kinematics.js', 'guessedData.js', 'fileFormatUtils.js', 'rrfMotionUtils.js', 'klipperSerialParser.js']) {
        await fs.copyFile(join(slideprinterSrc, file), join(slideprinterDist, file));
      }
    },
  });

  return {
    base: '/hp-sim5/',
    build: { rollupOptions: { input: inputs } },
    server: {},
    preview: {},
    plugins: [coiHeadersPlugin(), copyHangprinterExtrasPlugin()],
    assetsInclude: [
      '**/*.usda',
      '**/*.usda.txt',
      '**/*.usdc',
      '**/*.gcode',
      '**/*.serial',
      '**/*.txt',
    ],
  };
});
