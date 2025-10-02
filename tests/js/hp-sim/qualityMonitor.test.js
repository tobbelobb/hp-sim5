import { QualityMonitor } from '../../../hp-sim/assets/quality-monitor.js';

describe('QualityMonitor _projectToPath', () => {
  function createMonitor() {
    const monitor = new QualityMonitor();
    const segments = [
      { start: [0, 0, 0], end: [0.1, 0, 0] },
      { start: [0.1, 0, 0], end: [0.1, 0.1, 0] },
      { start: [0.1, 0.1, 0], end: [0.2, 0.1, 0] },
      { start: [0.2, 0.1, 0], end: [0.2, 0.2, 0] },
    ];
    monitor.setReferenceSegments(segments);
    return monitor;
  }

  test('projects points onto the nearest straight segment with correct normal error', () => {
    const monitor = createMonitor();

    const projection = monitor._projectToPath(0.05, 0.003);

    expect(projection).not.toBeNull();
    expect(projection.segmentIndex).toBe(0);
    expect(projection.isStraight).toBe(true);
    expect(projection.normalError).toBeCloseTo(0.003, 6);
    expect(projection.tangentialError).toBeCloseTo(0, 6);
    expect(monitor.lastSegmentIndex).toBe(0);
  });

  test('projects near a corner onto the following segment with signed normal error', () => {
    const monitor = createMonitor();

    // First project somewhere else to prime the hint
    monitor._projectToPath(0.02, 0);

    const projection = monitor._projectToPath(0.102, 0.08);

    expect(projection).not.toBeNull();
    expect(projection.segmentIndex).toBe(1);
    expect(projection.isStraight).toBe(false);
    expect(projection.normalError).toBeCloseTo(-0.002, 6);
    expect(projection.tangentialError).toBeCloseTo(0, 6);
    expect(monitor.lastSegmentIndex).toBe(1);
  });

  test('search window finds distant segments when hint is far away', () => {
    const monitor = createMonitor();

    monitor.lastSegmentIndex = 0;
    const farProjection = monitor._projectToPath(0.195, 0.18);

    expect(farProjection.segmentIndex).toBe(3);
    expect(farProjection.normalError).toBeCloseTo(0.005, 6);
    expect(monitor.lastSegmentIndex).toBe(3);

    monitor.lastSegmentIndex = monitor.segmentData.length - 1;
    const backProjection = monitor._projectToPath(0.01, -0.001);

    expect(backProjection.segmentIndex).toBe(0);
    expect(backProjection.normalError).toBeCloseTo(-0.001, 6);
    expect(monitor.lastSegmentIndex).toBe(0);
  });
});
