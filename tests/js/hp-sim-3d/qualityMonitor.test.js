import { QualityMonitor } from '../../../hp-sim-3d/app/quality-monitor.js';

describe('QualityMonitor 3D extrusion scoring', () => {
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

  // A segment consuming 1 unit of filament per metre.
  function segment(start, end) {
    return { start, end, extrusion: Math.hypot(end[0] - start[0], end[1] - start[1], end[2] - start[2]) };
  }

  // Feed samples the way RemoteSpoolSystem does: the tip at the start of each
  // 0.1 mm step together with that step's extrusion; `offset(point, tangent)`
  // displaces the tip from where it should be.
  function print(monitor, segments, offset = () => [0, 0, 0], step = 0.0001) {
    for (const seg of segments) {
      const d = seg.end.map((value, i) => value - seg.start[i]);
      const length = Math.hypot(...d);
      if (length === 0) {
        const o = offset(seg.start, [0, 0, 0]);
        monitor.recordExtrusion({ pos: seg.start.map((value, i) => value + o[i]), length: seg.extrusion });
        continue;
      }
      const tangent = d.map((value) => value / length);
      const count = Math.round(length / step);
      for (let i = 0; i < count; i += 1) {
        const point = seg.start.map((value, k) => value + d[k] * (i / count));
        const o = offset(point, tangent);
        monitor.recordExtrusion({ pos: point.map((value, k) => value + o[k]), length: seg.extrusion / count });
      }
    }
    monitor.runFinalCheck();
    return monitor.metrics;
  }

  function createMonitor(segments) {
    const monitor = new QualityMonitor();
    monitor.setReferenceSegments(segments);
    return monitor;
  }

  test('a print on the path scores 100 with zero error and full progress', () => {
    const reference = [segment([0, 0, 0], [0.05, 0, 0]), segment([0.05, 0, 0], [0.05, 0.05, 0])];
    const metrics = print(createMonitor(reference), reference);
    expect(metrics.score).toBeCloseTo(100, 9);
    expect(metrics.contourMax).toBeLessThan(1e-9);
    expect(metrics.trackingMax).toBeLessThan(1e-9);
    expect(metrics.progress).toBeCloseTo(1, 9);
    expect(metrics.cornerCount).toBe(1);
  });

  test('a vertical offset is reported as vertical path deviation and mean Z offset', () => {
    const reference = [segment([0, 0, 0], [0.1, 0, 0])];
    const metrics = print(createMonitor(reference), reference, () => [0, 0, 0.002]);
    expect(metrics.contourRms).toBeCloseTo(0.002, 9);
    expect(metrics.contourVerticalRms).toBeCloseTo(0.002, 9);
    expect(metrics.contourHorizontalRms).toBeCloseTo(0, 9);
    expect(metrics.meanOffset[2]).toBeCloseTo(0.002, 7);
    expect(metrics.withinTolerance).toBe(0);
  });

  test('a rigidly shifted but otherwise perfect print still scores high', () => {
    const reference = [
      segment([0, 0, 0], [0.05, 0, 0]),
      segment([0.05, 0, 0], [0.05, 0.05, 0]),
      segment([0.05, 0.05, 0], [0, 0.05, 0]),
    ];
    const metrics = print(createMonitor(reference), reference, () => [0.001, -0.0005, 0.0003]);
    expect(metrics.shapeP95).toBeLessThan(1e-4);
    expect(metrics.cornerDeviationMax).toBeLessThan(1e-4);
    expect(metrics.score).toBeGreaterThan(90);
  });

  test('a layer shift halfway scores well below a rigid shift, a blow-up near zero', () => {
    const reference = [
      segment([0, 0, 0], [0.05, 0, 0]),
      segment([0.05, 0, 0], [0.05, 0.05, 0]),
      segment([0.05, 0.05, 0], [0, 0.05, 0]),
    ];
    const shifted = print(createMonitor(reference), reference, () => [0.0015, 0, 0]);
    const layerShift = print(createMonitor(reference), reference, (point) => (point[1] > 0.02 ? [0.0015, 0, 0] : [0, 0, 0]));
    const blowUp = print(createMonitor(reference), reference, (point) => (point[1] > 0.01 ? [0.02 * Math.sin(point[1] * 500), 0.02, 0.01] : [0, 0, 0]));
    expect(layerShift.score).toBeLessThan(shifted.score - 15);
    expect(blowUp.score).toBeLessThan(20);
  });

  test('missed steps lower the score', () => {
    const reference = [segment([0, 0, 0], [0.05, 0, 0])];
    const monitor = createMonitor(reference);
    const clean = print(monitor, reference).score;
    monitor.setMotorDiagnosticsProvider(() => ({ totalMissedSteps: 40, motors: [] }));
    expect(monitor.getMetrics().score).toBeLessThan(clean - 5);
  });

  test('lag along the path is position error, not path deviation', () => {
    const reference = [segment([0, 0, 0], [0.1, 0, 0])];
    const metrics = print(createMonitor(reference), reference, (point, tangent) => (
      point[0] >= 0.001 ? tangent.map((value) => -0.001 * value) : [0, 0, 0]
    ));
    expect(metrics.meanLag).toBeGreaterThan(0.00098);
    expect(metrics.trackingP95).toBeGreaterThan(0.00098);
    expect(metrics.contourMax).toBeLessThan(1e-9);
    expect(metrics.withinTolerance).toBe(1);
  });

  test('the first sample after a travel belongs to the line it starts', () => {
    const reference = [segment([0, 0, 0], [0.01, 0, 0]), segment([0.2, 0.2, 0], [0.21, 0.2, 0])];
    const metrics = print(createMonitor(reference), reference);
    expect(metrics.trackingMax).toBeLessThan(1e-9);
  });

  test('extrusion without motion (a prime) keeps the stream in sync with the path', () => {
    const reference = [
      segment([0, 0, 0], [0.01, 0, 0]),
      { start: [0.02, 0, 0], end: [0.02, 0, 0], extrusion: 0.005 },
      segment([0.02, 0, 0], [0.03, 0, 0]),
    ];
    const metrics = print(createMonitor(reference), reference);
    expect(metrics.trackingMax).toBeLessThan(1e-9);
    expect(metrics.progress).toBeCloseTo(1, 9);
  });

  test('a deviation toward the neighbouring infill line is not mistaken for that line', () => {
    const reference = [
      segment([0, 0, 0], [0.05, 0, 0]),
      segment([0.05, 0, 0], [0.05, 0.002, 0]),
      segment([0.05, 0.002, 0], [0, 0.002, 0]),
    ];
    const metrics = print(createMonitor(reference), reference, (point) => (
      point[1] === 0 && point[0] < 0.04 ? [0, 0.0015, 0] : [0, 0, 0]
    ));
    expect(metrics.contourMax).toBeCloseTo(0.0015, 6);
  });

  test('corner rounding shows up as corner deviation', () => {
    const reference = [segment([0, 0, 0], [0.05, 0, 0]), segment([0.05, 0, 0], [0.05, 0.05, 0])];
    // Cut the corner: pull samples within 2 mm of it toward the inside.
    const metrics = print(createMonitor(reference), reference, (point) => {
      const distance = Math.hypot(point[0] - 0.05, point[1]);
      return distance < 0.002 ? [-(0.002 - distance) / 2, (0.002 - distance) / 2, 0] : [0, 0, 0];
    });
    expect(metrics.cornerCount).toBe(1);
    expect(metrics.cornerDeviationMax).toBeGreaterThan(0.001);
    expect(metrics.settleDistanceMax).toBeLessThan(0.002);
  });

  test('tints extrusions by their path deviation', () => {
    const monitor = createMonitor([segment([0, 0, 0], [0.1, 0, 0])]);
    const onPath = { pos: [0.01, 0, 0], length: 0.0001 };
    const offPath = { pos: [0.01, 0.003, 0], length: 0.0001 };
    monitor.recordExtrusion(onPath);
    monitor.recordExtrusion(offPath);
    expect(onPath.penalty).toBe(0);
    expect(offPath.penalty).toBe(1);
  });

  test('renders missed-step diagnostics in the 3D HUD card', () => {
    const hudElement = createHudElement();
    const monitor = new QualityMonitor({ hudElement });
    monitor.setReferenceSegments([segment([0, 0, 0], [0.1, 0, 0])]);
    monitor.setMotorDiagnosticsProvider(() => ({
      totalMissedSteps: 20,
      motors: [
        { axis: 'A', closedLoop: false, slips: 4, netSlips: 4, missedSteps: 16 },
        { axis: 'B', closedLoop: false, slips: 1, netSlips: -1, missedSteps: 4 },
      ],
    }));

    monitor.recordExtrusion({ pos: [0, 0, 0], length: 0.0005 });
    monitor.runFinalCheck();

    expect(hudElement.innerHTML).toContain('Missed Steps <strong>20</strong>');

    hudElement.clickSection('missed-steps');
    expect(monitor.missedStepsExpanded).toBe(true);
    expect(hudElement.innerHTML).toContain('<span>A</span><span>16 (4 slips, net +4)</span>');
    expect(hudElement.innerHTML).toContain('<span>B</span><span>4 (1 slips, net -1)</span>');
  });
});
