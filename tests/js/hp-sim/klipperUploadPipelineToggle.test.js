import fs from 'fs';
import path from 'path';

function readHpSim3dSource() {
  const scriptPath = path.resolve(process.cwd(), 'hp-sim-3d/assets/hp-sim.js');
  return fs.readFileSync(scriptPath, 'utf8');
}

describe('hp-sim-3d Klipper upload pipeline toggle', () => {
  test('defines a top-level Klipper upload pipeline toggle', () => {
    const source = readHpSim3dSource();

    expect(source).toContain("const KLIPPER_UPLOAD_PIPELINE = 'player';");
  });

  test('routes uploaded Klipper files through the selected pipeline', () => {
    const source = readHpSim3dSource();

    expect(source).toContain("const useRawKlipperUploadPipeline = KLIPPER_UPLOAD_PIPELINE === 'raw';");
    expect(source).toContain('worker = useRawKlipperUploadPipeline');
    expect(source).toContain('? createKlipperRawUploadBridge()');
    expect(source).toContain(": ensureKlipperMcuCommandPlayerWorker();");
  });
});
