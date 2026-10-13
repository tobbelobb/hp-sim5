import { jest } from '@jest/globals';
import { PerformanceMonitor } from '../../../hp-sim-3d/app/performance-monitor.js';

test('headless snapshots retain all systems and use actual steps, including partial windows', () => {
  const clock = jest.spyOn(performance, 'now');
  const monitor = new PerformanceMonitor({ enabled: true, logEverySteps: 0, slowStepMs: Infinity });
  for (let i = 0; i < 3; i++) {
    clock.mockReturnValue(i * 10); monitor.beginStep();
    monitor.measureSample('Attachment', 2);
    clock.mockReturnValue(i * 10 + 4); monitor.endStep();
  }
  expect(monitor.snapshot()).toEqual({ steps: 3, avgMs: 4, p50Ms: 4, p95Ms: 4, maxMs: 4,
    systems: [{ system: 'Attachment', calls: 3, totalMs: 6, avgPerCallMs: 2, avgPerStepMs: 2, maxMs: 2 }] });
  monitor.resetWindow();
  expect(monitor.snapshot().steps).toBe(0);
  expect(monitor.snapshot().systems).toEqual([]);
  expect(monitor.stepCount).toBe(3);
  clock.mockRestore();
});

test('timed calls return values and account for thrown errors', () => {
  const monitor = new PerformanceMonitor({ enabled: true });
  expect(monitor.measureSystem('value', () => 42)).toBe(42);
  expect(() => monitor.measureSystem('error', () => { throw new Error('failed'); })).toThrow('failed');
  expect(monitor.systemCounts.get('error')).toBe(1);
});
