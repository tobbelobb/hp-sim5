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

  test('hangprinter-org build copies the shared and klipper helper modules for worker imports', () => {
    const source = readWorkspaceFile('vite.hangprinter-org.config.js');

    expect(source).toContain("resolve(__dirname, 'integrations/shared')");
    expect(source).toContain("resolve(__dirname, 'integrations/klipper')");
  });
});
