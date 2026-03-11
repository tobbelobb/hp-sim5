import { QualityMonitor } from '../../../hp-sim/assets/quality-monitor.js';

describe('QualityMonitor _projectToPath', () => {
  function createHudElement() {
    const classes = new Set(['sim-hidden']);
    const listeners = new Map();
    return {
      innerHTML: '',
      textContent: '',
      style: {
        setProperty() {},
      },
      classList: {
        add(...names) {
          for (const name of names) {
            classes.add(name);
          }
        },
        remove(...names) {
          for (const name of names) {
            classes.delete(name);
          }
        },
        contains(name) {
          return classes.has(name);
        },
      },
      addEventListener(type, callback) {
        listeners.set(type, callback);
      },
      removeEventListener(type) {
        listeners.delete(type);
      },
      clickSection(section) {
        const callback = listeners.get('click');
        callback?.({
          target: {
            closest(selector) {
              if (selector === '.quality-hud__toggle') {
                return { dataset: { section } };
              }
              return null;
            },
          },
          preventDefault() {},
        });
      },
    };
  }

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

  test('deferred draining of queued extrusions yields identical metrics', () => {
    const monitor = createMonitor();
    const extrusions = Array.from({ length: 50 }, (_, i) => ({
      pos: [0.002 * i, i < 25 ? 0 : 0.01],
      length: 0.0005,
    }));

    const eagerMonitor = createMonitor();
    for (const extrusion of extrusions) {
      eagerMonitor.recordExtrusion(extrusion);
    }
    eagerMonitor.runFinalCheck();

    monitor.setEnabled(true);
    for (let i = 0; i < extrusions.length; i += 1) {
      if (i === 20) {
        monitor.setEnabled(false);
      }
      monitor.recordExtrusion(extrusions[i]);
    }
    monitor.setEnabled(true);
    monitor.runFinalCheck();

    expect(monitor.metrics).toEqual(eagerMonitor.metrics);
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

  test('renders collapsible quality and missed-step sections in the HUD', () => {
    const hudElement = createHudElement();
    const monitor = new QualityMonitor({ hudElement });
    monitor.setReferenceSegments([
      { start: [0, 0, 0], end: [0.1, 0, 0] },
    ]);
    monitor.setMotorDiagnosticsProvider(() => ({
      totalMissedSteps: 15,
      motors: [
        { axis: 'A', missedSteps: 7 },
        { axis: 'B', missedSteps: 3 },
        { axis: 'C', missedSteps: 3 },
        { axis: 'D', missedSteps: 2 },
      ],
    }));

    monitor.recordExtrusion({ pos: [0.05, 0, 0], length: 0.0005 });
    monitor.runFinalCheck();

    expect(hudElement.innerHTML).toContain('data-section="quality-details"');
    expect(hudElement.innerHTML).toContain('data-section="missed-steps"');
    expect(hudElement.innerHTML).toContain('Missed Steps <strong>15</strong>');
    expect((hudElement.innerHTML.match(/quality-hud__details--hidden/g) || [])).toHaveLength(2);

    hudElement.clickSection('quality-details');
    expect(monitor.qualityDetailsExpanded).toBe(true);
    expect((hudElement.innerHTML.match(/quality-hud__details--hidden/g) || [])).toHaveLength(1);

    hudElement.clickSection('missed-steps');
    expect(monitor.missedStepsExpanded).toBe(true);
    expect((hudElement.innerHTML.match(/quality-hud__details--hidden/g) || [])).toHaveLength(0);
    expect(hudElement.innerHTML).toContain('<span>A</span><span>7</span>');
    expect(hudElement.innerHTML).toContain('<span>D</span><span>2</span>');
  });
});
