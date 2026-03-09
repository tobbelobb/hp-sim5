import { QualityMonitor } from '../../../hp-sim-3d/assets/quality-monitor.js';

describe('QualityMonitor 3D extrusion scoring', () => {
  function createMonitor(referenceSegments) {
    const monitor = new QualityMonitor();
    monitor.setReferenceSegments(referenceSegments);
    return monitor;
  }

  test('normalizes 3D extrusion points and projects them onto 3D reference segments', () => {
    const monitor = createMonitor([
      { start: [0, 0, -0.001], end: [0.1, 0, -0.001] },
    ]);

    const normalized = monitor._normalizeExtrusionEvent({
      pos: [0.05, 0, -0.001],
      length: 0.0005,
    });
    const projection = monitor._projectToPath(0.05, 0, -0.001);

    expect(normalized).toEqual({
      x: 0.05,
      y: 0,
      z: -0.001,
      length: 0.0005,
    });
    expect(projection).not.toBeNull();
    expect(projection.normalError).toBeCloseTo(0, 9);
    expect(projection.point[2]).toBeCloseTo(-0.001, 9);
  });

  test('z offsets affect both distance error and coverage metrics', () => {
    const reference = [
      { start: [0, 0, 0], end: [0.1, 0, 0] },
    ];
    const onPath = createMonitor(reference);
    const offPlane = createMonitor(reference);

    onPath.recordExtrusion({ pos: [0.05, 0, 0], length: 0.0005 });
    onPath.runFinalCheck();

    offPlane.recordExtrusion({ pos: [0.05, 0, 0.002], length: 0.0005 });
    offPlane.runFinalCheck();

    expect(onPath.metrics.rmseStraight).toBeCloseTo(0, 9);
    expect(onPath.metrics.coverage).toBeGreaterThan(0);
    expect(offPlane.metrics.rmseStraight).toBeGreaterThan(0.0019);
    expect(offPlane.metrics.coverage).toBe(0);
    expect(offPlane.metrics.score).toBeLessThan(onPath.metrics.score);
  });
});
