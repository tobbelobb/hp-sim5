import fs from 'fs';
import path from 'path';

function readWorkspaceFile(filePath) {
  const absolutePath = path.resolve(process.cwd(), filePath);
  return fs.readFileSync(absolutePath, 'utf8');
}

describe('worker helper module copy config', () => {
  test('main vite build copies the shared and klipper helper modules for worker imports', () => {
    const source = readWorkspaceFile('vite.config.js');

    expect(source).toContain("resolve(__dirname, 'integrations/shared')");
    expect(source).toContain("resolve(__dirname, 'integrations/klipper')");
  });

  test('main vite build copies RRF config files used by hp-sim-3d runtime fetches', () => {
    const source = readWorkspaceFile('vite.config.js');

    expect(source).toContain("resolve(__dirname, 'RRF/run/vsd/sys')");
    expect(source).toContain("resolve(__dirname, 'dist/RRF/run/vsd/sys')");
    expect(source).toContain("fg('config_*.g'");
  });

  test('hangprinter-org build copies the shared and klipper helper modules for worker imports', () => {
    const source = readWorkspaceFile('vite.hangprinter-org.config.js');

    expect(source).toContain("resolve(__dirname, 'integrations/shared')");
    expect(source).toContain("resolve(__dirname, 'integrations/klipper')");
  });
});
