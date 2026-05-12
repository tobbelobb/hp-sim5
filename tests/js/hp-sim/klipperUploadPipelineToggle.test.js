import fs from 'fs';
import path from 'path';

function readHpSim3dSource() {
  const scriptPath = path.resolve(process.cwd(), 'hp-sim-3d/app/workerController.js');
  return fs.readFileSync(scriptPath, 'utf8');
}

function readHpSim3dCommandJobSource() {
  const scriptPath = path.resolve(process.cwd(), 'hp-sim-3d/app/commandJobController.js');
  return fs.readFileSync(scriptPath, 'utf8');
}

describe('hp-sim-3d Klipper upload pipeline toggle', () => {
  test('defines a top-level Klipper upload pipeline toggle', () => {
    const source = readHpSim3dSource();

    expect(source).toContain("const KLIPPER_UPLOAD_PIPELINE = 'player';");
  });

  test('routes uploaded Klipper files through the selected pipeline', () => {
    const source = readHpSim3dCommandJobSource();

    expect(source).toContain('shouldUseRawKlipperUploadPipeline(KLIPPER_UPLOAD_PIPELINE)');
    expect(source).toContain('? workers.createKlipperRawUploadBridge()');
    expect(source).toContain(": workers.ensureKlipperMcuCommandPlayerWorker();");
  });
});
